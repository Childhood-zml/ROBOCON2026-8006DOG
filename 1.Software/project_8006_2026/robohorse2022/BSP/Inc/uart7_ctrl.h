#ifndef UART7_CTRL_H
#define UART7_CTRL_H

#include "serial.h"

#define PC_FRAME_HEAD1  0xAA
#define PC_FRAME_HEAD2  0x55
#define PC_RX_FIFO_SIZE 128     /* 原 32 扩大为 128，防止任务调度延迟导致溢出 */

typedef struct {
    uint32_t rx_bytes;      /* 总接收字节 */
    uint32_t frame_ok;      /* 校验通过帧数 */
    uint32_t frame_err;     /* 校验失败帧数 */
    uint32_t fifo_ovf;      /* FIFO 溢出次数 */
    uint32_t state_reset;   /* 状态机异常重置次数 */
} UART7_Debug_t;

void UART7_Ctrl_Init(void);
void UART7_Rx_Byte(void);
void UART7_Process_InTask(void);
uint8_t UART7_Get_Ctrl_Data(RC_ctrl_t *rc_ctrl);
void UART7_Get_Debug(UART7_Debug_t *dbg);

#endif
