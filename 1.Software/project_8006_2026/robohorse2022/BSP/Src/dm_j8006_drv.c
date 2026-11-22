/**
  ****************************(C) COPYRIGHT 2024 Zhang****************************
  * @file      dm_j8006_drv.c
  * @brief     达妙电机CAN总线底层驱动 (DM-J8006-2EC)
  * @note      8个达妙DM-J8006电机 + 1个DM-J4340气泵电机
  * @history
  *  Version    Date            Author          Modification
  *   V3.0.0    2024-7-12       R&A战队         张哲元
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Zhang****************************
  */

#include "dm_j8006_drv.h"
#include "motor_config.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* CAN发送缓冲区（需在临界区或单任务中使用以避免竞争） */
CAN_TxHeaderTypeDef  can_tx_message;
uint8_t              can_send_data[8];

DM_J8006_DATA_t dm_j8006_data[9];       // 达妙电机反馈数据 (8腿+1气泵)

/* 12位有符号数符号扩展常量 */
#define DM_12BIT_SIGN_BIT       2047    // 12位有符号数中值 (2^11 - 1)
#define DM_12BIT_FULL_RANGE      4096    // 12位完整范围 (2^12)

/* ======================== 数值转换工具函数 ======================== */

/**
 * @brief  浮点数转无符号整数
 * @param  x_float 待转换的浮点数
 * @param  x_min   范围最小值
 * @param  x_max   范围最大值
 * @param  bits    目标无符号整数的位数
 * @return 无符号整数结果
 * @note   将浮点数 x 在 [x_min, x_max] 内线性映射为指定位数的无符号整数
 */
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    float span   = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)(((unsigned int)1 << bits) - 1)) / span);
}

/**
 * @brief  无符号整数转浮点数
 * @param  x_int 待转换的无符号整数
 * @param  x_min 范围最小值
 * @param  x_max 范围最大值
 * @param  bits  无符号整数的位数
 * @return 浮点数结果
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span   = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)(((unsigned int)1 << bits) - 1)) + offset;
}

/* ======================== 内部辅助函数 ======================== */

/**
 * @brief  发送8字节CAN命令（所有简单命令的公共实现）
 * @param  can      CAN句柄
 * @param  id       电机ID
 * @param  cmd_byte 命令字节 (0xFB=清除错误, 0xFC=使能, 0xFD=失能, 0xFE=保存零点)
 */
static void dm_j8006_send_cmd(CAN_HandleTypeDef can, uint16_t id, uint8_t cmd_byte)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = id;
    can_tx_message.IDE   = CAN_ID_STD;
    can_tx_message.RTR   = CAN_RTR_DATA;
    can_tx_message.DLC   = 0x08;
    can_send_data[0] = 0xFF;
    can_send_data[1] = 0xFF;
    can_send_data[2] = 0xFF;
    can_send_data[3] = 0xFF;
    can_send_data[4] = 0xFF;
    can_send_data[5] = 0xFF;
    can_send_data[6] = 0xFF;
    can_send_data[7] = cmd_byte;

    HAL_CAN_AddTxMessage(&can, &can_tx_message, can_send_data, &send_mail_box);
}

/* ======================== CAN命令公开API ======================== */

void CLEAR_ERROR_DM_J8006(CAN_HandleTypeDef can, uint16_t id)
{
    dm_j8006_send_cmd(can, id, 0xFB);
}

void ENABLE_DM_J8006(CAN_HandleTypeDef can, uint16_t id)
{
    dm_j8006_send_cmd(can, id, 0xFC);
}

void DISABLE_DM_J8006(CAN_HandleTypeDef can, uint16_t id)
{
    dm_j8006_send_cmd(can, id, 0xFD);
}

void SAVE_ZERO_DM_J8006(CAN_HandleTypeDef can, uint16_t id)
{
    dm_j8006_send_cmd(can, id, 0xFE);
}

/* ======================== MIT模式控制 ======================== */

/**
 * @brief  MIT模式控制
 * @param  id    电机ID
 * @param  _pos  位置 (rad)
 * @param  _vel  速度 (rad/s)
 * @param  _KP   位置环比例增益
 * @param  _KD   速度环微分增益
 * @param  _torq 前馈转矩 (N·m)
 * @note   给定位置+KP+KD且其余项为0 → 位置控制
 *         给定速度+KD且其余项为0 → 速度控制
 *         给定扭矩且其余项为0 → 扭矩控制
 */
void CTRL_DM_J8006_MIT(CAN_HandleTypeDef can, uint16_t id,
                        float _pos, float _vel, float _KP, float _KD, float _torq)
{
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

    /* 限制输入范围，防止溢出 */
    _pos  = clamp_float(_pos,  P_MIN,  P_MAX);
    _vel  = clamp_float(_vel,  V_MIN,  V_MAX);
    _KP   = clamp_float(_KP,   KP_MIN, KP_MAX);
    _KD   = clamp_float(_KD,   KD_MIN, KD_MAX);
    _torq = clamp_float(_torq, T_MIN,  T_MAX);

    pos_tmp = float_to_uint(_pos,  P_MIN,  P_MAX,  16);
    vel_tmp = float_to_uint(_vel,  V_MIN,  V_MAX,  12);
    kp_tmp  = float_to_uint(_KP,   KP_MIN, KP_MAX, 12);
    kd_tmp  = float_to_uint(_KD,   KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(_torq, T_MIN,  T_MAX,  12);

    uint32_t send_mail_box;
    can_tx_message.StdId = id;
    can_tx_message.IDE   = CAN_ID_STD;
    can_tx_message.RTR   = CAN_RTR_DATA;
    can_tx_message.DLC   = 0x08;
    can_send_data[0] = (pos_tmp >> 8);
    can_send_data[1] = (pos_tmp & 0xFF);
    can_send_data[2] = (vel_tmp >> 4);
    can_send_data[3] = ((vel_tmp & 0xF) << 4) | ((kp_tmp >> 8) & 0xF);
    can_send_data[4] = (kp_tmp & 0xFF);
    can_send_data[5] = (kd_tmp >> 4);
    can_send_data[6] = ((kd_tmp & 0xF) << 4) | ((tor_tmp >> 8) & 0xF);
    can_send_data[7] = (tor_tmp & 0xFF);

    HAL_CAN_AddTxMessage(&can, &can_tx_message, can_send_data, &send_mail_box);
}

/* ======================== 电机反馈数据解析 ======================== */

/**
 * @brief  解析达妙电机CAN反馈数据帧
 * @param  num  电机序号 (0~8)
 * @param  data 8字节CAN数据
 * @note   修复12位有符号数符号扩展，分离ID和故障码
 */
void FEEDBACK_DATA_DM_J8006(uint16_t num, uint8_t data[])
{
    int16_t p_raw, v_raw, t_raw;

    /* 解析原始数据 */
    p_raw = (int16_t)((data[1] << 8) | data[2]);                    // 16位有符号位置
    v_raw = (int16_t)((data[3] << 4) | (data[4] >> 4));             // 12位速度
    t_raw = (int16_t)(((data[4] & 0xF) << 8) | data[5]);           // 12位扭矩

    /* 12位有符号数符号扩展 */
    if (v_raw > DM_12BIT_SIGN_BIT) v_raw -= DM_12BIT_FULL_RANGE;
    if (t_raw > DM_12BIT_SIGN_BIT) t_raw -= DM_12BIT_FULL_RANGE;

    /* 存入结构体 */
    dm_j8006_data[num].p_int = (int)p_raw;
    dm_j8006_data[num].v_int = (int)v_raw;
    dm_j8006_data[num].t_int = (int)t_raw;

    /* ID和故障码（data[0]高4位=故障码，低4位=ID） */
    dm_j8006_data[num].id = data[0];

    /* 转换为物理量 */
    dm_j8006_data[num].POS    = uint_to_float(dm_j8006_data[num].p_int, P_MIN, P_MAX, 16);
    dm_j8006_data[num].VEL    = uint_to_float(dm_j8006_data[num].v_int, V_MIN, V_MAX, 12);
    dm_j8006_data[num].TORQUE = uint_to_float(dm_j8006_data[num].t_int, T_MIN, T_MAX, 12);
    dm_j8006_data[num].T_MOS   = data[6];
    dm_j8006_data[num].T_Rotor = data[7];
}

/* ======================== DM-J4340 MIT模式控制 ======================== */

/**
 * @brief  DM-J4340-2EC MIT模式控制
 * @note   与 CTRL_DM_J8006_MIT 相同协议, 使用 J4340 专用量程
 *         P_MIN_J4340=-12.5, P_MAX_J4340=12.5 rad
 */
void CTRL_DM_J4340_MIT(CAN_HandleTypeDef can, uint16_t id,
                        float _pos, float _vel, float _KP, float _KD, float _torq)
{
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

    /* 限制输入范围 (J4340 专用量程) */
    _pos  = clamp_float(_pos,  P_MIN_J4340,  P_MAX_J4340);
    _vel  = clamp_float(_vel,  V_MIN_J4340,  V_MAX_J4340);
    _KP   = clamp_float(_KP,   KP_MIN_J4340, KP_MAX_J4340);
    _KD   = clamp_float(_KD,   KD_MIN_J4340, KD_MAX_J4340);
    _torq = clamp_float(_torq, T_MIN_J4340,  T_MAX_J4340);

    pos_tmp = float_to_uint(_pos,  P_MIN_J4340,  P_MAX_J4340,  16);
    vel_tmp = float_to_uint(_vel,  V_MIN_J4340,  V_MAX_J4340,  12);
    kp_tmp  = float_to_uint(_KP,   KP_MIN_J4340, KP_MAX_J4340, 12);
    kd_tmp  = float_to_uint(_KD,   KD_MIN_J4340, KD_MAX_J4340, 12);
    tor_tmp = float_to_uint(_torq, T_MIN_J4340,  T_MAX_J4340,  12);

    uint32_t send_mail_box;
    can_tx_message.StdId = id;
    can_tx_message.IDE   = CAN_ID_STD;
    can_tx_message.RTR   = CAN_RTR_DATA;
    can_tx_message.DLC   = 0x08;
    can_send_data[0] = (pos_tmp >> 8);
    can_send_data[1] = (pos_tmp & 0xFF);
    can_send_data[2] = (vel_tmp >> 4);
    can_send_data[3] = ((vel_tmp & 0xF) << 4) | ((kp_tmp >> 8) & 0xF);
    can_send_data[4] = (kp_tmp & 0xFF);
    can_send_data[5] = (kd_tmp >> 4);
    can_send_data[6] = ((kd_tmp & 0xF) << 4) | ((tor_tmp >> 8) & 0xF);
    can_send_data[7] = (tor_tmp & 0xFF);

    HAL_CAN_AddTxMessage(&can, &can_tx_message, can_send_data, &send_mail_box);
}

/* ======================== 气泵电机 (DM-J4340-2EC) ======================== */

void PumpMotor_Init(void)
{
    ENABLE_DM_J8006(hcan1, PUMP_MOTOR_CAN_ID);
}

/**
 * @brief  气泵电机二位置触发切换 (每10ms调用, 仅在身高测试模式)
 * @param  stick_norm  摇杆归一化值 [-1, 1]
 *         上升沿 (< -0.5) → 在 0° 和 -180° 之间切换
 */
void PumpMotor_Update(float stick_norm)
{
    static float  pump_target = 0.0f;    // 目标位置: 0° 或 -180°
    static float  pump_pos    = 0.0f;    // 当前指令位置 (°)
    static uint8_t stick_was  = 0;       // 上一次摇杆状态
    uint8_t stick_active = (stick_norm < -0.5f);

    /* 上升沿触发: 摇杆从回中推到左边 → 切换目标位置 */
    if (stick_active && !stick_was) {
        pump_target = (pump_target > -90.0f) ? -180.0f : 0.0f;
    }
    stick_was = stick_active;

    /* 平滑过渡: pump_pos → pump_target */
    float diff = pump_target - pump_pos;
    if (diff < 0.0f) diff = -diff;
    if (diff < 0.5f) {
        pump_pos = pump_target;  // 到位, 锁住
    } else {
        pump_pos += (pump_target - pump_pos) * 0.15f;  // 指数平滑
    }

    /* MIT 位置控制 */
    float pos_rad = pump_pos * 3.14159f / 180.0f;
    CTRL_DM_J4340_MIT(hcan1, PUMP_MOTOR_CAN_ID, pos_rad, 0,
                      PUMP_MOTOR_KP, PUMP_MOTOR_KD, 0);
}

float PumpMotor_GetAngle(void)
{
    return dm_j8006_data[8].POS * 180.0f / 3.14159f;
}

void PumpMotor_Disable(void)
{
    DISABLE_DM_J8006(hcan1, PUMP_MOTOR_CAN_ID);
}

/****************************************************达妙电机驱动程序**************************************************************/
// 查阅DM-J8006说明手册
