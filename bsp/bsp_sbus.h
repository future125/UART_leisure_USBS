/**
  ******************************************************************************
  * @file    bsp_sbus.h
  * @brief   SBUS 接收与解析（USART3 + DMA + 空闲中断）对外接口。
  *
  *  数据流：
  *      接收机 --(SBUS 25 字节)--> USART3(PC11) --DMA--> sbus_rx_buf
  *            --(空闲/传输完成中断)--> 长度检查 --帧头检查--> 快照
  *            --> 11bit 解包 --> rc_data.ch_raw[] --> 减 1024 --> rc_data.rocker_xxx
  *
  *  用法：
  *      main() 里初始化完外设后调用一次 SBUS_Init()；
  *      主循环里反复调用 SBUS_Update() 维护在线标志；
  *      其它模块直接读全局 rc_data 即可（已在调试器中加 volatile，可 Live Watch）。
  ******************************************************************************
  */
#ifndef __BSP_SBUS_H
#define __BSP_SBUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ============================ 协议常量 ============================ */

/** SBUS 一帧的固定长度：1 帧头 + 22 字节数据 + 1 状态 + 1 结束 = 25 字节。
 *  DMA 的接收长度必须与它严格一致，否则每次都会等到超时才进回调。 */
#define SBUS_FRAME_LEN      25U

/** 一帧里打包的通道总数：16 路 × 11 bit = 176 bit = 22 字节。 */
#define SBUS_CH_NUM         16U

/** 本任务实际使用的前 10 路（CH1~CH10）。 */
#define SBUS_CH_USED        10U

/** 帧头 / 结束字节，用于判断数据是否错位（错位说明中间丢过字节）。 */
#define SBUS_HEAD_BYTE      0x0FU
#define SBUS_END_BYTE       0x00U

/** 通道原始值中点：解析出 0~2047 后减去它，得到 −784 ~ +784。 */
#define SBUS_RAW_CENTER     1024

/** 超过这么久没收到有效帧，就认为接收机掉线（遥控器关机 / 线松了）。 */
#define SBUS_OFFLINE_MS     100U

/* ============================ 数据结构 ============================ */

/**
 * @brief 遥控器数据。一份结构体同时保存“解析结果 + 原始值 + 链路状态”，
 *        方便在 VSCode 的 Live Watch 里整块展开观察。
 */
typedef struct
{
    /* ---- 解析结果：已减 1024，零位在中间，范围约 −784 ~ +784 ---- */
    int16_t rocker_r_;      /*!< CH1  右摇杆 水平（右为正） */
    int16_t rocker_r1;      /*!< CH2  右摇杆 竖直（上为正） */
    int16_t rocker_l1;      /*!< CH3  左摇杆 竖直（上为正） */
    int16_t rocker_l_;      /*!< CH4  左摇杆 水平（右为正） */

    int16_t SWA;            /*!< CH5  三档开关 A（−784 / 0 / +784） */
    int16_t SWB;            /*!< CH6  三档开关 B */
    int16_t SWC;            /*!< CH7  三档开关 C */
    int16_t SWD;            /*!< CH8  三档开关 D */

    int16_t VRA;            /*!< CH9  旋钮 A（连续变化） */
    int16_t VRB;            /*!< CH10 旋钮 B（连续变化） */

    /* ---- 原始值：0 ~ 2047，没减 1024，用来对照“减中点前后”的变化 ---- */
    uint16_t ch_raw[SBUS_CH_NUM];

    /* ---- 链路状态：关遥控器时重点看这几个 ---- */
    uint8_t  error_protect; /*!< 1 = 接收机处于失控保护（状态字节 bit3） */
    uint8_t  byte_lose;     /*!< 1 = 上一帧被标记丢帧（状态字节 bit2） */
    uint8_t  online;        /*!< 1 = 最近 SBUS_OFFLINE_MS 内收到过有效帧 */
    uint32_t last_tick;     /*!< 最后一次有效帧的时刻，单位 ms */
    uint32_t frame_ok_cnt;  /*!< 累计有效帧数（在涨说明接收是通的） */
    uint32_t frame_err_cnt; /*!< 累计被丢弃的帧数（长度/帧头/结束字节不对） */
} rc_ctrl_t;

/** 全局唯一的遥控器数据块。中断里写，主循环和调试器里读，所以必须 volatile。 */
extern volatile rc_ctrl_t rc_data;

/* ============================ 对外函数 ============================ */

/**
 * @brief 清空数据并启动第一次 DMA 接收。放在 MX_USART3_UART_Init() 之后调用。
 */
void SBUS_Init(void);

/**
 * @brief 维护 online 标志。必须放在 while(1) 里反复调用（建议每轮都调）。
 */
void SBUS_Update(void);

/**
 * @brief 便捷查询：当前链路是否在线。
 * @retval 1 在线 / 0 离线
 */
uint8_t SBUS_IsOnline(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_SBUS_H */
