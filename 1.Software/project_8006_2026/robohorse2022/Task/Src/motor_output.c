/**
 * @file    motor_output.c
 * @brief   统一电机输出层实现
 *
 * 这是所有步态和跳跃的电机映射唯一出口。
 * 替换了旧的 Target_toMotor() (posture_task.c) 和 Polar_setCoord() (jump_task.c)。
 *
 * 核心公式:
 *   IK 输出 joint_angle (髋/膝 rad)
 *   inner_motor = (π/2 + hip_angle) / MOTOR_ANGLE_SCALE   // 内侧=髋关节
 *   outer_motor = (π/2 - knee_angle) / MOTOR_ANGLE_SCALE   // 外侧=膝关节
 *   左腿取负, 右腿取正 (补偿机械镜像安装)
 *
 * MOTOR_ANGLE_SCALE = 0.5: 电机到关节减速比, 即电机转 2 圈 = 关节转 1 圈
 */
#include "motor_output.h"
#include "gait_param.h"
#include "motor_config.h"
#include "debug_output.h"
#include <stddef.h>

/* ======================== 电机映射表 ======================== */
/* [leg_id][0]=inner_motor, [1]=outer_motor */
static const uint8_t leg_motor_map[4][2] = {
    {MOTOR_LF_INNER, MOTOR_LF_OUTER},  // leg 0 左前: 电机2,3
    {MOTOR_LR_INNER, MOTOR_LR_OUTER},  // leg 1 左后: 电机4,5
    {MOTOR_RR_INNER, MOTOR_RR_OUTER},  // leg 2 右后: 电机6,7
    {MOTOR_RF_INNER, MOTOR_RF_OUTER},  // leg 3 右前: 电机1,0
};

/** 左侧腿取负 (补偿机械镜像), 右侧取正 */
static inline float leg_sign(uint8_t leg_id)
{
    return (leg_id < 2) ? -1.0f : 1.0f;   // leg 0,1=左, leg 2,3=右
}

/* ======================== 核心: IK + 电机映射 ======================== */

/**
 * @brief 单腿 IK 解算 → 电机角度
 * @param leg_id        腿编号 0~3
 * @param x             足端 x (cm, 相对髋关节)
 * @param y             足端 y (cm, 相对髋关节)
 * @param sign_override 0=自动左右符号, ±1=强制覆写 (未使用)
 */
static void motor_output_resolve(uint8_t leg_id, float x, float y, float sign_override)
{
    if (leg_id >= 4) return;

    /* ① 直角坐标 → IK → 关节角 (stanford_type.c, 余弦定理) */
    Stanford_Type_Lite_SetPosition(&Gait_Data[leg_id], x, y);
    Stanford_Type_Lite_Inverse_Kinematics(&Gait_Data[leg_id]);

    /* ② 关节角 → 电机角 (含减速比 MOTOR_ANGLE_SCALE=0.5) */
    float angle_inner = (pi / 2.0f + Gait_Data[leg_id].angle[0]) / MOTOR_ANGLE_SCALE;
    float angle_outer = (pi / 2.0f - Gait_Data[leg_id].angle[1]) / MOTOR_ANGLE_SCALE;

    /* ③ 符号: 左腿取反 (机械镜像补偿) */
    float sign = (sign_override != 0.0f) ? sign_override : leg_sign(leg_id);
    angle_inner *= sign;
    angle_outer *= sign;

    /* ④ 写入全局 Aim_Angle[] */
    uint8_t m_inner = leg_motor_map[leg_id][0];
    uint8_t m_outer = leg_motor_map[leg_id][1];
    Aim_Angle[m_inner] = angle_inner;
    Aim_Angle[m_outer] = angle_outer;

    DEBUG_VAL("ik_inner", angle_inner);   // 调试: 编译时 -DDEBUG_ENABLE 开启
    DEBUG_VAL("ik_outer", angle_outer);
}

/* ======================== 公开 API ======================== */

void motor_output_set_cartesian(uint8_t leg_id, float x, float y)
{
    motor_output_resolve(leg_id, x, y, 0.0f);
}

void motor_output_set_polar(uint8_t leg_id, float radius, float theta)
{
    if (leg_id >= 4) return;

    Cart_Coord_Data_t cart;
    Polar_Coord_Data_t polar = {radius, theta};
    Polar_toCartesian(&polar, &cart);

    motor_output_resolve(leg_id, cart.cx, cart.cy, 0.0f);
}

/** 跳跃和 NORMAL 模式用: 四腿极坐标 → 电机, 含 Is_Aim_Angle_Get */
void motor_output_set_all_polar(Polar_Coord_Data_t polar[4])
{
    if (polar == NULL) return;

    for (uint8_t i = 0; i < 4; i++) {
        Cart_Coord_Data_t cart;
        Polar_toCartesian(&polar[i], &cart);
        motor_output_resolve(i, cart.cx, cart.cy, 0.0f);
    }
    Is_Aim_Angle_Get = 1;
}

/** 行走步态用: 四腿直角坐标 → 电机, 含 Is_Aim_Angle_Get */
void motor_output_set_all_cartesian(float x[4], float y[4])
{
    if (x == NULL || y == NULL) return;

    for (uint8_t i = 0; i < 4; i++) {
        motor_output_resolve(i, x[i], y[i], 0.0f);
    }
    Is_Aim_Angle_Get = 1;
}
