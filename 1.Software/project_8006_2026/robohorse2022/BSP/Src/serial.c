/**
 * 航模遥控器（SBUS协议）及传感器串口通讯
 *
 * 硬件连接：
 *   UART1 (DMA) → 大疆航模遥控器 SBUS，18字节/帧
 *   UART7 (IT)  → PC控制协议（备用遥控通道）
 *   UART8 (DMA) → JY901S 陀螺仪
 */

#include "serial.h"
#include "app_ebf_piddebug.h"

#include "usart.h"
#include "jy901s.h"
#include "uart7_ctrl.h"   /* <-- 新增：UART7 PC控制协议 */

__IO uint8_t ch8[11]; // UART8 DMA接收缓存（陀螺仪数据）

uint8_t rc_data[18];          // SBUS原始数据缓存（18字节一帧）
RC_ctrl_t my_rc_ctrl;         // 解析后的遥控器数据
RC_Key rc_sw_state = RC_NULL; // 当前开关状态

void RemoCtrl_Data_Handle(RC_ctrl_t *rc_ctrl);

/**
 * @brief 启动遥控器和传感器接收
 *        UART1: DMA方式接收SBUS帧
 *        UART7: 由 UART7_Ctrl_Init() 初始化，此处不再启动
 *        UART8: DMA方式接收陀螺仪数据
 */
void RC_Start()
{
    HAL_UART_Receive_DMA(&huart1, rc_data, 18);
    /* UART7 接收已在 UART7_Ctrl_Init() 中启动，此处不再重复初始化 */
}

/**
 * @brief SBUS帧解析：18字节原始数据 → RC_ctrl_t 结构体
 * @param sbus_buf SBUS原始数据缓冲区（18字节）
 * @param rc_ctrl  输出：解析后的遥控器数据结构体
 *
 * SBUS协议通道编码（每通道11位，0~2047）：
 *   ch0: 右摇杆左右  (转向)
 *   ch1: 右摇杆上下
 *   ch2: 左摇杆左右
 *   ch3: 左摇杆上下  (油门/速度)
 *   ch4: 旋钮/拨轮
 *   s0:  左开关 S1（三档：上=1, 中=3, 下=2）← 注意：编码不是123顺序
 *   s1:  右开关 S2
 */
void RC_Transform(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = ((int16_t)sbus_buf[0] | (int16_t)(sbus_buf[1] << 8)) & 0x07ff;
    rc_ctrl->rc.ch[1] = ((int16_t)(sbus_buf[1] >> 3) | (int16_t)(sbus_buf[2] << 5)) & 0x07ff;
    rc_ctrl->rc.ch[2] = ((int16_t)(sbus_buf[2] >> 6) | (int16_t)(sbus_buf[3] << 2) |
                         (int16_t)(sbus_buf[4] << 10)) & 0x07ff;
    rc_ctrl->rc.ch[3] = ((int16_t)(sbus_buf[4] >> 1) | (int16_t)(sbus_buf[5] << 7)) & 0x07ff;

    // 开关状态：2位编码，范围 0~3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;  // 左开关 S1
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x0003);       // 右开关 S2

    // 扩展通道（鼠标/键盘模拟，你的遥控器可能用不到）
    rc_ctrl->mouse.x = (int16_t)sbus_buf[6] | ((int16_t)sbus_buf[7] << 8);
    rc_ctrl->mouse.y = (int16_t)sbus_buf[8] | (int16_t)(sbus_buf[9] << 8);
    rc_ctrl->mouse.z = (int16_t)sbus_buf[10] | (int16_t)(sbus_buf[11] << 8);
    rc_ctrl->mouse.press_l = sbus_buf[12];
    rc_ctrl->mouse.press_r = sbus_buf[13];
    rc_ctrl->key.v = (int16_t)sbus_buf[14] | (int16_t)(sbus_buf[15] << 8);
    rc_ctrl->rc.ch[4] = (int16_t)sbus_buf[16] | (int16_t)(sbus_buf[17] << 8);  // 预留通道

    RemoCtrl_Data_Handle(rc_ctrl);
}

/**
 * @brief 遥控器数据后处理：更新开关状态
 * @note  rc_sw_state 编码 = (S1 << 4) | S2
 *        例如 S1=0x01(上), S2=0x01(上) → RC_11 = 0x11
 */
void RemoCtrl_Data_Handle(RC_ctrl_t *rc_ctrl)
{
    rc_sw_state = (RC_Key)((rc_ctrl->rc.s[0] << 4) | (rc_ctrl->rc.s[1]));
}

/**
 * @brief UART接收完成回调（DMA + 中断）
 *        UART1: SBUS帧收完 → 重启DMA接收下一帧
 *        UART7: PC控制协议单字节入FIFO
 *        UART8: 陀螺仪数据收完 → 解析姿态数据
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        // SBUS帧完整接收：停DMA → 重装接收长度 → 重启DMA
        HAL_UART_DMAStop(&huart1);
        HAL_UART_Receive_DMA(&huart1, (uint8_t *)rc_data, 18);
    }
    else if (huart == &huart7)
    {
        /* <-- 修改：PC控制协议单字节入FIFO */
        UART7_Rx_Byte();
        /* 注意：UART7_Rx_Byte() 内部已重新启动 HAL_UART_Receive_IT */
    }
    else if (huart == &huart8)
    {
        // JY901S 陀螺仪数据解析
        HAL_UART_DMAStop(&huart8);
        CopeSerial2Data((uint8_t*)ch8);
        HAL_UART_Receive_DMA(&huart8, (uint8_t*)ch8, 11);
    }
}
