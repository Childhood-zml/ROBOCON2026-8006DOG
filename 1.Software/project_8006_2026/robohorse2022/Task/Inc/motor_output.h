/**
 * @file    motor_output.h
 * @brief   统一电机输出层 — IK 解算 → 电机角度映射的单一出口
 *
 * 设计目的:
 *   旧代码有两套电机映射 (Target_toMotor 和 Polar_setCoord),
 *   缩放不一致(一个除 MOTOR_ANGLE_SCALE 一个不除), 符号约定不同。
 *   这里统一为一条路径, 所有步态和跳跃都走这里。
 *
 * 数据流:
 *   极坐标(r,θ) / 直角坐标(x,y)
 *     → Polar_toCartesian (极→直转换)
 *     → Stanford_Type_Lite_Inverse_Kinematics (IK 解算, 余弦定理)
 *     → 关节角→电机角: (π/2 ± joint_angle) / MOTOR_ANGLE_SCALE
 *     → 符号映射: 左腿(0,1)取负, 右腿(2,3)取正 (补偿机械镜像)
 *     → Aim_Angle[8] (弧度)
 *     → Is_Aim_Angle_Get = 1 (通知 Motor_Task)
 *
 * 电机映射表 (已验证):
 *   | 腿编号 | 位置 | 内侧电机(hip) | 外侧电机(knee) | CAN |
 *   |--------|------|---------------|----------------|-----|
 *   | Leg 0  | 左前 | Motor 2       | Motor 3        | CAN2|
 *   | Leg 1  | 左后 | Motor 4       | Motor 5        | CAN2|
 *   | Leg 2  | 右后 | Motor 6       | Motor 7        | CAN1|
 *   | Leg 3  | 右前 | Motor 1       | Motor 0        | CAN1|
 */
#ifndef MOTOR_OUTPUT_H
#define MOTOR_OUTPUT_H

#include <stdint.h>
#include "stanford_type.h"
#include "gait_param.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 腿编号常量 ======================== */
#define LEG_LF  0   // 左前腿
#define LEG_LR  1   // 左后腿
#define LEG_RR  2   // 右后腿
#define LEG_RF  3   // 右前腿

/* ======================== 电机索引常量 ======================== */
#define MOTOR_LF_INNER  2
#define MOTOR_LF_OUTER  3
#define MOTOR_LR_INNER  4
#define MOTOR_LR_OUTER  5
#define MOTOR_RR_INNER  6
#define MOTOR_RR_OUTER  7
#define MOTOR_RF_INNER  1
#define MOTOR_RF_OUTER  0

/* ======================== API ======================== */

/** 单腿: 直角坐标 → IK → 电机角度 */
void motor_output_set_cartesian(uint8_t leg_id, float x, float y);

/** 单腿: 极坐标 → 直角 → IK → 电机角度 */
void motor_output_set_polar(uint8_t leg_id, float radius, float theta);

/** 四腿批量: 极坐标 → 电机角度, 设置 Is_Aim_Angle_Get */
void motor_output_set_all_polar(Polar_Coord_Data_t polar[4]);

/** 四腿批量: 直角坐标 → 电机角度, 设置 Is_Aim_Angle_Get (行走步态用) */
void motor_output_set_all_cartesian(float x[4], float y[4]);

/** 通知 Motor_Task 数据就绪 */
static inline void motor_output_commit(void)
{
    Is_Aim_Angle_Get = 1;
}

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_OUTPUT_H */
