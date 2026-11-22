/**
 * @file    dm_j8006_ctrl.c
 * @brief   达妙DM-J8006电机控制层（位置环/双环控制）
 * @note    依赖 motor_config.h 中的CAN总线映射表
 */
#include "dm_j8006_ctrl.h"
#include "dm_j8006_drv.h"
#include "dm_j8006.h"
#include "motor_config.h"

#include "can.h"
#include "pid.h"
#include "FreeRTOS.h"
#include "task.h"

// J8009电机反馈数据（8个电机）
J8006_Measure_t J8006[8];

// 速度环PID数组
PID_t PID_Speed[8];

// 关节电机位置限幅（使用motor_config中的统一常量，保留数组接口兼容性）
float pos_limit_min[8] = {
    MOTOR_POS_LIMIT_MIN, MOTOR_POS_LIMIT_MIN, MOTOR_POS_LIMIT_MIN, MOTOR_POS_LIMIT_MIN,
    MOTOR_POS_LIMIT_MIN, MOTOR_POS_LIMIT_MIN, MOTOR_POS_LIMIT_MIN, MOTOR_POS_LIMIT_MIN
};
float pos_limit_max[8] = {
    MOTOR_POS_LIMIT_MAX, MOTOR_POS_LIMIT_MAX, MOTOR_POS_LIMIT_MAX, MOTOR_POS_LIMIT_MAX,
    MOTOR_POS_LIMIT_MAX, MOTOR_POS_LIMIT_MAX, MOTOR_POS_LIMIT_MAX, MOTOR_POS_LIMIT_MAX
};

extern uint8_t DM_J8006_ENABLE_FLAG;
extern uint8_t DM_J8006_ctrl_FLAG;

/* ======================== 内部辅助函数 ======================== */

/**
 * @brief 等待CAN邮箱可用（带超时保护）
 * @param hcan CAN总线句柄
 * @return 0=成功, -1=超时
 */
static int can_wait_mailbox(CAN_HandleTypeDef *hcan)
{
    uint32_t tickstart = HAL_GetTick();
    while (!HAL_CAN_GetTxMailboxesFreeLevel(hcan))
    {
        if ((HAL_GetTick() - tickstart) > CAN_TX_TIMEOUT_MS)
        {
            return -1;  // 超时
        }
    }
    return 0;
}

/* ======================== 公开API ======================== */

/**
 * @brief  达妙电机位置环控制
 * @param  can   CAN句柄
 * @param  id    电机ID (0~7)
 * @param  set_pos 目标位置 (rad)
 * @param  kp    位置环比例增益
 * @param  kd    位置环微分增益
 */
void dm_j8006_pos_ctrl(CAN_HandleTypeDef can, uint8_t id, float set_pos, float kp, float kd)
{
    if (id >= MOTOR_COUNT)
    {
        return;  // 无效ID保护
    }

    // 关节限幅，防止超调卡住损坏电机
    set_pos = clamp_float(set_pos, g_motor_config[id].pos_limit_min, g_motor_config[id].pos_limit_max);

    CTRL_DM_J8006_MIT(can, id, set_pos, 0, kp, kd, 0);
}

void dm_pd_set(PID_t *pid, LegGain state_pid)
{
    pid->p = state_pid.kp_pos;
    pid->d = state_pid.kd_pos;
}

/**
 * @brief 达妙电机双环控制
 * @param SET_Angle 目标角度数组
 * @param PID_POS   位置环PID数组（pos_out=速度目标, p=KP, d=KD）
 * @param PID_SPD   速度环PID数组（pos_out=力矩输出）
 * @note  使用MIT模式：T = kp*(pos - pos_actual) + kd*(vel - vel_actual) + torque
 */
void dm_j8006_all_dual_ctrl(float SET_Angle[8], PID_t PID_POS[8], PID_t PID_SPD[8])
{
    for (uint8_t id = 0; id < MOTOR_COUNT; id++)
    {
        CAN_HandleTypeDef *hcan = g_motor_config[id].hcan;

        // 位置限幅
        float set_pos = clamp_float(SET_Angle[id],
                                    g_motor_config[id].pos_limit_min,
                                    g_motor_config[id].pos_limit_max);

        // 双环参数
        float vel_target = PID_POS[id].pos_out;
        float torque     = PID_SPD[id].pos_out;
        float kp         = PID_POS[id].p;
        float kd         = PID_POS[id].d;

        if (can_wait_mailbox(hcan) == 0)
        {
            CTRL_DM_J8006_MIT(*hcan, id, set_pos, vel_target, kp, kd, torque);
        }
        vTaskDelay(CAN_POST_SEND_DELAY_MS);
        DM_J8006_ctrl_FLAG = 1;
    }
}

/**
 * @brief 达妙电机单环位置控制（保留兼容接口）
 */
void dm_j8006_all_ctrl(float SET_Angle[8], PID_t PID_SET[8])
{
    for (uint8_t id = 0; id < MOTOR_COUNT; id++)
    {
        CAN_HandleTypeDef *hcan = g_motor_config[id].hcan;

        if (can_wait_mailbox(hcan) == 0)
        {
            dm_j8006_pos_ctrl(*hcan, id, SET_Angle[id], PID_SET[id].p, PID_SET[id].d);
        }
        vTaskDelay(CAN_POST_SEND_DELAY_MS);
        DM_J8006_ctrl_FLAG = 1;
    }
}

/**
 * @brief 跳跃专用: MIT 模式 + 速度/力矩前馈
 *
 * 和正常 MIT 帧一样, 但 vel_target 和 torque 不是来自 PID 输出,
 * 而是直接传参, 用于跳跃蹬腿阶段额外助力。
 *
 * 电机公式: T = kp×(pos - actual) + kd×(vel_ff - vel) + torque_ff
 */
void dm_j8006_send_mit_ff(float pos[8], float kp, float kd,
                          float vel_ff, float torque_ff)
{
    for (uint8_t id = 0; id < MOTOR_COUNT; id++)
    {
        CAN_HandleTypeDef *hcan = g_motor_config[id].hcan;

        float set_pos = clamp_float(pos[id],
                                    g_motor_config[id].pos_limit_min,
                                    g_motor_config[id].pos_limit_max);

        if (can_wait_mailbox(hcan) == 0) {
            CTRL_DM_J8006_MIT(*hcan, id, set_pos, vel_ff, kp, kd, torque_ff);
        }
        vTaskDelay(CAN_POST_SEND_DELAY_MS);
        DM_J8006_ctrl_FLAG = 1;
    }
}

/**
 * @brief 达妙电机全部使能
 */
void dm_j8006_all_enable(void)
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        CAN_HandleTypeDef *hcan = g_motor_config[i].hcan;
        if (can_wait_mailbox(hcan) == 0)
        {
            ENABLE_DM_J8006(*hcan, g_motor_config[i].motor_id);
        }
        vTaskDelay(CAN_POST_SEND_DELAY_MS);
    }
    DM_J8006_ENABLE_FLAG = 1;
}

/**
 * @brief 达妙电机全部失能
 */
void dm_j8006_all_disable(void)
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        CAN_HandleTypeDef *hcan = g_motor_config[i].hcan;
        if (can_wait_mailbox(hcan) == 0)
        {
            DISABLE_DM_J8006(*hcan, g_motor_config[i].motor_id);
        }
        vTaskDelay(CAN_POST_SEND_DELAY_MS);
    }
    DM_J8006_ENABLE_FLAG = 0;
}
