/**
  ******************************************************************************
  * @file    bsp_sbus.c
  * @brief   SBUS 接收与解析（USART3 + DMA + 接收至空闲）。
  *
  *  接收链路（对应验收里要讲的那句“串口接收 → DMA 缓冲区 → 数据检查 → 通道解析”）：
  *
  *    ① 串口接收：HAL_UARTEx_ReceiveToIdle_DMA() 一次性挂上 DMA，长度 25。
  *                 此后 CPU 不参与搬运，硬件把每个字节直接写进 sbus_rx_buf。
  *    ② 事件触发：满 25 字节触发 TC（传输完成），或线路空闲一个字节时间
  *                 触发 IDLE。两者都会进 HAL_UARTEx_RxEventCallback()，
  *                 参数 Size 就是“这一轮实际收到了几个字节”。
  *    ③ 数据检查：长度必须 == 25，帧头必须 0x0F，结束字节必须 0x00，
  *                 三者有一个不对就整帧丢弃、绝不拿去更新通道值。
  *    ④ 通道解析：22 字节里按 11 bit 一路的顺序把 16 路拆出来，再减 1024。
  *
  *  注意时序：一定要“先拷快照 → 再检查解析 → 最后才重新挂 DMA”。
  *  如果先挂 DMA，硬件可能在解析到一半时就往同一块内存写下一帧，数据会被撕裂。
  ******************************************************************************
  */

#include "bsp_sbus.h"
#include "usart.h"      /* 取 huart3 的声明 */

/* ======================== 私有变量 ======================== */

/**
 * @brief DMA 的落点。容量必须 >= 接收长度 25，这里是正好 25。
 *        只有 DMA 写它，CPU 只读，所以不用 volatile 修饰元素，
 *        但跨中断/主循环访问，加上更保险。
 */
static uint8_t sbus_rx_buf[SBUS_FRAME_LEN];

/**
 * @brief 本帧快照。回调里把 sbus_rx_buf 整段拷到这里，之后的解析全部基于快照，
 *        这样 DMA 立刻重启也不会影响正在解析的数据。
 */
static uint8_t sbus_frame[SBUS_FRAME_LEN];

/**
 * @brief 遥控器数据本体。
 *        volatile：会被中断修改、被主循环/调试器读取，不加的话 -Og 下
 *        编译器可能把它优化掉，Live Watch 里会显示 optimized out。
 */
volatile rc_ctrl_t rc_data;

/* ======================== 私有函数声明 ======================== */

static void    sbus_start_rx(void);
static uint8_t sbus_check_frame(const uint8_t *frame);
static void    sbus_decode(const uint8_t *frame);

/* ======================== 私有函数实现 ======================== */

/**
 * @brief 启动/重启一轮 DMA 接收。上电、每帧解析完、出错恢复都要调它。
 * @note  这里把“重开失败”也处理了：ORE 等错误后 HAL 可能还处于锁死状态，
 *        直接再调会一直返回 HAL_BUSY，必须先清标志、解锁。
 */
static void sbus_start_rx(void)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3, sbus_rx_buf, SBUS_FRAME_LEN) != HAL_OK)
    {
        /* 上一次异常残留：清溢出标志 + 清错误码 + 解除 HAL 锁，再试一次 */
        __HAL_UART_CLEAR_OREFLAG(&huart3);
        huart3.ErrorCode = HAL_UART_ERROR_NONE;// 清软件错误码不清的话，下次任何代码检查 if (huart3.ErrorCode != HAL_UART_ERROR_NONE) 都会误判成“还在出错”。
        __HAL_UNLOCK(&huart3);//释放 HAL 的互斥锁
        (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart3, sbus_rx_buf, SBUS_FRAME_LEN);
    }

    /* 关掉“半传输”中断。
       25 字节一帧，如果开着 HT，收到第 12/13 字节时也会进一次回调，
       Size 变成 12 之类的小数值，白白多一次中断还容易误判。
       注意每次重新挂 DMA 后 HT 都会重新被使能，所以这行必须跟在后面。 */
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
}

/**
 * @brief 帧合法性检查（对应要求 5：格式不对就不采用）。
 * @param frame 指向 25 字节快照
 * @retval 1 合法；0 丢弃
 */
static uint8_t sbus_check_frame(const uint8_t *frame)
{
    /* 帧头不对：说明 DMA 窗口和真实的帧边界错位了（多半是中间丢过一个字节）。
       丢弃这一帧后，下一帧会重新从 0x0F 开始对齐，这是唯一的自动重同步手段。 */
    if (frame[0] != SBUS_HEAD_BYTE)
    {
        return 0U;
    }

    /* 结束字节不对：同样说明帧不完整或错位。 */
    if (frame[SBUS_FRAME_LEN - 1U] != SBUS_END_BYTE)
    {
        return 0U;
    }

    return 1U;
}

/**
 * @brief 把 25 字节快照解成 16 路原始值，再挑出前 10 路减 1024。
 *
 *  11 bit 一路、地址不完全对齐，所以每路要用 3 个字节拼：
 *      bit 起点 = i * 11
 *      byte     = 1 + (bit / 8)      数据从 frame[1] 开始，frame[0] 是帧头
 *      shift    = bit % 8            该路在 byte 里右移多少位
 *      raw      = (frame[byte] >> shift)
 *               | (frame[byte+1] << (8  - shift))
 *               | (frame[byte+2] << (16 - shift))
 *  最后与 0x07FF 相与，只留低 11 位。
 *
 * @note 中间量必须用 uint32_t。用 int16_t 拼的话，<< 16 那一步直接溢出/未定义。
 */
static void sbus_decode(const uint8_t *frame)
{
    for (uint8_t i = 0U; i < SBUS_CH_NUM; i++)
    {
        uint16_t bit  = (uint16_t)((uint16_t)i * 11U);   /* 该路起始 bit */ //前 i 路一共占用多少 bit	每路等宽 11 bit，直接乘
        uint8_t  idx  = (uint8_t)(1U + (bit >> 3));      /* 落在哪个字节 */ //字节下标	>> 3 就是除以 8；+1 跳过 frame[0] 帧头
        uint8_t  sh   = (uint8_t)(bit & 0x07U);          /* 字节内偏移 */   //字节内偏移（0~7）	& 0x07 就是取除以 8 的余数，等价 bit % 8

        uint32_t raw  = ((uint32_t)frame[idx]     >> sh)
                      | ((uint32_t)frame[idx + 1U] << (8U  - sh))
                      | ((uint32_t)frame[idx + 2U] << (16U - sh));

        /* 原始值 0 ~ 2047 */
        rc_data.ch_raw[i] = (uint16_t)(raw & 0x07FFU);
    }

    /* ---- 前 10 路减 1024，得到以 0 为中心的数据（范围约 −784 ~ +784）---- */
    rc_data.rocker_r_ = (int16_t)((int32_t)rc_data.ch_raw[0]  - SBUS_RAW_CENTER); /* CH1  右摇杆 水平 */
    rc_data.rocker_r1 = (int16_t)((int32_t)rc_data.ch_raw[1]  - SBUS_RAW_CENTER); /* CH2  右摇杆 竖直 */
    rc_data.rocker_l1 = (int16_t)((int32_t)rc_data.ch_raw[2]  - SBUS_RAW_CENTER); /* CH3  左摇杆 竖直 */
    rc_data.rocker_l_ = (int16_t)((int32_t)rc_data.ch_raw[3]  - SBUS_RAW_CENTER); /* CH4  左摇杆 水平 */
    rc_data.SWA       = (int16_t)((int32_t)rc_data.ch_raw[4]  - SBUS_RAW_CENTER); /* CH5  开关 A */
    rc_data.SWB       = (int16_t)((int32_t)rc_data.ch_raw[5]  - SBUS_RAW_CENTER); /* CH6  开关 B */
    rc_data.SWC       = (int16_t)((int32_t)rc_data.ch_raw[6]  - SBUS_RAW_CENTER); /* CH7  //开关 C */
    rc_data.SWD       = (int16_t)((int32_t)rc_data.ch_raw[7]  - SBUS_RAW_CENTER); /* CH8  开关 D */
    rc_data.VRA       = (int16_t)((int32_t)rc_data.ch_raw[8]  - SBUS_RAW_CENTER); /* CH9  旋钮 A */
    rc_data.VRB       = (int16_t)((int32_t)rc_data.ch_raw[9]  - SBUS_RAW_CENTER); /* CH10 旋钮 B */

    /* ---- 状态字节 frame[23] 的位定义 ----
       bit0 = CH17，bit1 = CH18，bit2 = 丢帧标志，bit3 = 失控保护标志
       这里只取后两位，正是要求里要观察的两个状态。 */
    rc_data.byte_lose     = (uint8_t)((frame[23] >> 2) & 0x01U);
    rc_data.error_protect = (uint8_t)((frame[23] >> 3) & 0x01U);
}

/* ======================== HAL 回调 ======================== */

/**
 * @brief DMA 接收事件回调（整帧收满 TC，或线路空闲 IDLE，都会进这里）。
 * @param huart 触发事件的串口句柄
 * @param Size  本轮实际收到的字节数
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART3)
    {
        return;     /* 别的串口的事件，不关我事 */
    }

    /* ① 长度检查：不是整帧就丢掉。线上一个毛刺引起的 3~5 字节 IDLE
          也会进这个回调，如果不判长度，这些垃圾就会被当成通道数据。 */
    if (Size == SBUS_FRAME_LEN)
    {
        /* ② 先拷快照，后面的解析只认 sbus_frame，不再碰 sbus_rx_buf */
        for (uint8_t i = 0U; i < SBUS_FRAME_LEN; i++)
        {
            sbus_frame[i] = sbus_rx_buf[i];
        }

        /* ③ 帧头/结束字节检查通过才允许更新通道值 */
        if (sbus_check_frame(sbus_frame) != 0U)
        {
            sbus_decode(sbus_frame);            /* ④ 通道解析 */
            rc_data.last_tick = HAL_GetTick();  /* 记下“最后一次有效帧” */
            rc_data.frame_ok_cnt++;
        }
        else
        {
            rc_data.frame_err_cnt++;            /* 错位帧，计数但不动通道值 */
        }
    }
    else
    {
        rc_data.frame_err_cnt++;                /* 长度不对，丢弃 */
    }

    /* ⑤ 最后一步才是重新挂 DMA，为下一帧做准备 */
    sbus_start_rx();
}

/**
 * @brief 串口错误回调。ORE（溢出）/ FE（帧错误）/ NE（噪声）发生后，
 *        HAL 会把接收整个 abort 掉；不在这里重开，接收就永久停了。
 *        现象是：遥控器数据突然定住不动，只能按复位。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);    /* F4 上 ORE 必须手动清 */
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        rc_data.frame_err_cnt++;
        sbus_start_rx();                    /* 重新挂上，链路自愈 */
    }
}

/* ======================== 对外函数 ======================== */

/**
 * @brief 初始化：清零数据块 + 启动第一轮 DMA 接收。
 * @note  必须在 MX_USART3_UART_Init()、MX_DMA_Init() 之后调用。
 */
void SBUS_Init(void)
{
    for (uint8_t i = 0U; i < SBUS_CH_NUM; i++)
    {
        rc_data.ch_raw[i] = 0U;
    }

    rc_data.rocker_r_     = 0;
    rc_data.rocker_r1     = 0;
    rc_data.rocker_l1     = 0;
    rc_data.rocker_l_     = 0;
    rc_data.SWA           = 0;
    rc_data.SWB           = 0;
    rc_data.SWC           = 0;
    rc_data.SWD           = 0;
    rc_data.VRA           = 0;
    rc_data.VRB           = 0;

    rc_data.error_protect = 0U;
    rc_data.byte_lose     = 0U;
    rc_data.online        = 0U;
    rc_data.last_tick     = 0U;
    rc_data.frame_ok_cnt  = 0U;
    rc_data.frame_err_cnt = 0U;

    sbus_start_rx();
}

/**
 * @brief 维护在线标志，主循环里反复调用。
 *
 *  为什么要单独判超时：关掉遥控器后接收机有两种表现 ——
 *    ① 继续发“失控保护帧”，此时链路还通，靠 error_protect 标志判断；
 *    ② 直接停止发送，此时回调再也不进，只能靠“最后一次有效帧的时间”判断。
 *  ② 才是必须用时间戳的原因。
 *
 * @note 必须用无符号减法 (now - last)，这样 HAL_GetTick() 在 49 天回绕时
 *       结果依然正确；写成 if (now > last + 100) 会在回绕点误判离线。
 */
void SBUS_Update(void)
{
    if (rc_data.frame_ok_cnt == 0U)
    {
        rc_data.online = 0U;        /* 还没收到过任何有效帧 */
        return;
    }

    rc_data.online = ((HAL_GetTick() - rc_data.last_tick) < SBUS_OFFLINE_MS) ? 1U : 0U;
}

/**
 * @brief 当前链路是否在线。
 */
uint8_t SBUS_IsOnline(void)
{
    return rc_data.online;
}
