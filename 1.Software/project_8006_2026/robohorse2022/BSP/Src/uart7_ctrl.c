#include "uart7_ctrl.h"
#include "usart.h"
#include "string.h"
#include "app_ebf_piddebug.h"   /* <-- 新增：第一套原有，PID调参器双路分流 */

static uint8_t  pc_rx_byte;
static uint8_t  pc_rx_fifo[PC_RX_FIFO_SIZE];
static volatile uint32_t pc_rx_head = 0;
static volatile uint32_t pc_rx_tail = 0;

static RC_ctrl_t pc_ctrl_buf[2];
static volatile RC_ctrl_t *pc_ctrl_active = &pc_ctrl_buf[0];
static volatile RC_ctrl_t *pc_ctrl_write  = &pc_ctrl_buf[1];
static volatile uint8_t pc_new_data = 0;

/* 调试统计 */
static volatile UART7_Debug_t pc_dbg = {0};

/* 状态机超时：若一帧内超过 50ms 未收完，强制重置（防止状态机卡死） */
#define FRAME_TIMEOUT_MS 50
static volatile uint32_t last_rx_tick = 0;

void UART7_Ctrl_Init(void)
{
    pc_rx_head = 0;
    pc_rx_tail = 0;
    pc_new_data = 0;
    pc_ctrl_active = &pc_ctrl_buf[0];
    pc_ctrl_write  = &pc_ctrl_buf[1];
    memset((void*)pc_ctrl_active, 0, sizeof(RC_ctrl_t));
    memset((void*)pc_ctrl_write, 0, sizeof(RC_ctrl_t));
    memset((void*)&pc_dbg, 0, sizeof(pc_dbg));

    __HAL_UART_FLUSH_DRREGISTER(&huart7);
    while (__HAL_UART_GET_FLAG(&huart7, UART_FLAG_RXNE))
    {
        (void)(huart7.Instance->DR);
    }

    HAL_UART_Receive_IT(&huart7, &pc_rx_byte, 1);
}

void UART7_Rx_Byte(void)
{
    /* 双路分流: 同时喂 PID 调参器 (EBFPIDDebuger) 和 PC 控制帧 FIFO */
    /* <-- 新增：第一套原有，确保 UART7 数据同时被 PID 调参器接收 */
    EBFPIDDebuger_Data_Receive(&pc_rx_byte, 1);

    uint32_t next = (pc_rx_head + 1) % PC_RX_FIFO_SIZE;
    if(next != pc_rx_tail)
    {
        pc_rx_fifo[pc_rx_head] = pc_rx_byte;
        pc_rx_head = next;
    }
    else
    {
        pc_dbg.fifo_ovf++;          /* FIFO 满，记录溢出 */
    }
    pc_dbg.rx_bytes++;
    last_rx_tick = HAL_GetTick();   /* 更新最后接收时间戳 */

    HAL_UART_Receive_IT(&huart7, &pc_rx_byte, 1);
}

void UART7_Process_InTask(void)
{
    static uint8_t state = 0;
    static uint8_t buf[13];
    static uint8_t cnt = 0;
    uint32_t now = HAL_GetTick();

    while(pc_rx_tail != pc_rx_head)
    {
        uint8_t data = pc_rx_fifo[pc_rx_tail];
        pc_rx_tail = (pc_rx_tail + 1) % PC_RX_FIFO_SIZE;

        /* 超时保护：若状态机卡在非 0 状态超过 FRAME_TIMEOUT_MS，强制重置 */
        if(state != 0 && (now - last_rx_tick) > FRAME_TIMEOUT_MS)
        {
            state = 0;
            cnt = 0;
            pc_dbg.state_reset++;
            break;  /* 退出本次循环，等下次再来 */
        }

        switch(state)
        {
            case 0:
                if(data == PC_FRAME_HEAD1) {
                    state = 1;
                }
                break;
            case 1:
                if(data == PC_FRAME_HEAD2) {
                    state = 2;
                    cnt = 0;
                } else if(data == PC_FRAME_HEAD1) {
                    /* 连续 0xAA 支持 */
                    state = 1;
                } else {
                    state = 0;
                }
                break;
            case 2:
                buf[cnt++] = data;
                if(cnt >= 13)
                {
                    uint8_t sum = 0;
                    for(int i = 0; i < 12; i++) sum += buf[i];

                    if(sum == buf[12])
                    {
                        RC_ctrl_t temp;
                        memset(&temp, 0, sizeof(RC_ctrl_t));
                        temp.rc.ch[0] = (int16_t)(buf[0] | (buf[1] << 8));
                        temp.rc.ch[1] = (int16_t)(buf[2] | (buf[3] << 8));
                        temp.rc.ch[2] = (int16_t)(buf[4] | (buf[5] << 8));
                        temp.rc.ch[3] = (int16_t)(buf[6] | (buf[7] << 8));
                        temp.rc.ch[4] = (int16_t)(buf[8] | (buf[9] << 8));
                        temp.rc.s[0]  = buf[10];
                        temp.rc.s[1]  = buf[11];

                        int all_zero = (temp.rc.ch[0] == 0) && (temp.rc.ch[1] == 0) &&
                                       (temp.rc.ch[2] == 0) && (temp.rc.ch[3] == 0) &&
                                       (temp.rc.ch[4] == 0);
                        if (!all_zero)
                        {
                            *pc_ctrl_write = temp;
                            RC_ctrl_t *tmp = (RC_ctrl_t*)pc_ctrl_active;
                            pc_ctrl_active = pc_ctrl_write;
                            pc_ctrl_write = tmp;
                            pc_new_data = 1;
                            pc_dbg.frame_ok++;
                        }
                    }
                    else
                    {
                        pc_dbg.frame_err++;
                    }
                    state = 0;
                    cnt = 0;
                }
                break;
            default:
                state = 0;
                cnt = 0;
                break;
        }
    }
}

uint8_t UART7_Get_Ctrl_Data(RC_ctrl_t *rc_ctrl)
{
    if(pc_new_data)
    {
        *rc_ctrl = *(RC_ctrl_t*)pc_ctrl_active;
        pc_new_data = 0;
        return 1;
    }
    return 0;
}

void UART7_Get_Debug(UART7_Debug_t *dbg)
{
    if(dbg) {
        /* 逐字段拷贝，避免 volatile 结构体整体 cast 导致的编译错误 */
        dbg->rx_bytes    = pc_dbg.rx_bytes;
        dbg->frame_ok    = pc_dbg.frame_ok;
        dbg->frame_err   = pc_dbg.frame_err;
        dbg->fifo_ovf    = pc_dbg.fifo_ovf;
        dbg->state_reset = pc_dbg.state_reset;
    }
}
