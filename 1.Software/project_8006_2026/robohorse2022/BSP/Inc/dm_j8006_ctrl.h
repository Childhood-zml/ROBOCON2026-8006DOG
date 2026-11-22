/**
 * @file    dm_j8006_ctrl.h
 * @brief   达妙DM-J8006电机控制层API
 */
#ifndef DM_J8009_CTRL_H
#define DM_J8009_CTRL_H

#include "main.h"
#include "gait_param.h"
#include "pid.h"
#include "motor_config.h"

// 保留旧定义兼容性（后续版本移除）
#define MOTOR_0_POS_MIN  MOTOR_POS_LIMIT_MIN
#define MOTOR_0_POS_MAX  MOTOR_POS_LIMIT_MAX
#define MOTOR_1_POS_MIN  MOTOR_POS_LIMIT_MIN
#define MOTOR_1_POS_MAX  MOTOR_POS_LIMIT_MAX
#define MOTOR_2_POS_MIN  MOTOR_POS_LIMIT_MIN
#define MOTOR_2_POS_MAX  MOTOR_POS_LIMIT_MAX
#define MOTOR_3_POS_MIN  MOTOR_POS_LIMIT_MIN
#define MOTOR_3_POS_MAX  MOTOR_POS_LIMIT_MAX
#define MOTOR_4_POS_MIN  MOTOR_POS_LIMIT_MIN
#define MOTOR_4_POS_MAX  MOTOR_POS_LIMIT_MAX
#define MOTOR_5_POS_MIN  MOTOR_POS_LIMIT_MIN
#define MOTOR_5_POS_MAX  MOTOR_POS_LIMIT_MAX
#define MOTOR_6_POS_MIN  MOTOR_POS_LIMIT_MIN
#define MOTOR_6_POS_MAX  MOTOR_POS_LIMIT_MAX
#define MOTOR_7_POS_MIN  MOTOR_POS_LIMIT_MIN
#define MOTOR_7_POS_MAX  MOTOR_POS_LIMIT_MAX

// 达妙电机位置环控制
void dm_j8006_pos_ctrl(CAN_HandleTypeDef can, uint8_t id, float set_pos, float kp, float kd);

// 达妙电机双环控制
void dm_j8006_all_dual_ctrl(float SET_Angle[8], PID_t PID_POS[8], PID_t PID_SPD[8]);

// 保留原有位置控制接口
void dm_j8006_all_ctrl(float SET_Angle[8], PID_t PID_SET[8]);

void dm_pd_set(PID_t *pid, LegGain state_pid);

void dm_j8006_all_enable(void);

void dm_j8006_all_disable(void);

/**
 * @brief 跳跃专用: MIT 模式 + 速度/力矩前馈
 * @param pos[8]     目标位置 (rad)
 * @param kp         位置刚性 (0~500)
 * @param kd         速度阻尼 (0~5)
 * @param vel_ff     速度前馈 (rad/s), 让电机主动加速
 * @param torque_ff  力矩前馈 (N·m),  额定 20Nm, 短期可超
 *
 * 电机执行: T = kp×(pos - actual) + kd×(vel_ff - vel) + torque_ff
 */
void dm_j8006_send_mit_ff(float pos[8], float kp, float kd,
                          float vel_ff, float torque_ff);

#endif /* DM_J8009_CTRL_H */
