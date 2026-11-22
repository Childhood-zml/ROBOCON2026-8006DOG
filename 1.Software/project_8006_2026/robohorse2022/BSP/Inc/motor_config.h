/**
 * @file    motor_config.h
 * @brief   电机硬件配置 — CAN 总线映射 + 位置限制 + 统一常量
 *
 * 8 个 DM-J8006 电机:
 *   CAN1 (右侧): Motor 0,1 (右前外/内), 6,7 (右后外/内)
 *   CAN2 (左侧): Motor 2,3 (左前外/内), 4,5 (左后外/内)
 *
 * MIT 模式控制:
 *   set_pos (rad) 被钳位到 [MOTOR_POS_LIMIT_MIN, MAX]
 *   电机硬件范围: P_MIN=-3.125, P_MAX=+3.125 rad
 *   软件安全限:   ±3.0 rad (覆盖 π/2±1rad 全关节行程)
 *
 * MOTOR_ANGLE_SCALE = 0.5: 电机到关节减速比
 *   电机转 2 圈 → 关节转 1 圈
 *   电机命令 = (π/2 ± 关节角) / 0.5 = (π/2 ± 关节角) × 2
 */
#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 电机常量定义 ======================== */

#define MOTOR_COUNT             9       // 电机总数
#define MOTORS_PER_LEG           2       // 每条腿电机数
#define LEG_COUNT                4       // 腿数

#define MOTOR_POS_LIMIT_MIN     (-3.0f) // 位置下限 (rad) — 覆盖完整关节行程
#define MOTOR_POS_LIMIT_MAX     3.0f    // 位置上限 (rad) — 电机硬件限 ±3.125

#define ENCODER_RESOLUTION      8191.0f // 编码器分辨率 (13-bit = 0~8191)
#define DEGREES_PER_REVOLUTION  360.0f  // 每圈角度
#define ENCODER_TO_DEG          0.043945f // 360/8191 预计算

#define VEL_LIMIT_MAX           720.0f  // 速度环输出上限 (RPM)
#define VEL_LIMIT_MIN          -720.0f  // 速度环输出下限 (RPM)
#define TORQUE_LIMIT_MAX        18.0f   // 力矩输出上限 (N·m)
#define TORQUE_LIMIT_MIN       -18.0f   // 力矩输出下限 (N·m)

#define MOTOR_ANGLE_SCALE       0.5f    // 电机角度缩放因子

#define CAN_TX_TIMEOUT_MS       10      // CAN发送超时 (ms)
#define CAN_POST_SEND_DELAY_MS  1       // CAN发送后延时 (ms) — DM-J8006 命令间隔

#define MOTOR_TEMP_HIGH         100     // 高温警告阈值 (℃)
#define MOTOR_TEMP_RECOVERY     80      // 温度恢复阈值 (℃)

/* ======================== 电机配置结构体 ======================= */

typedef struct {
    uint8_t             motor_id;       // CAN ID (0x00~0x07)
    CAN_HandleTypeDef   *hcan;          // 所属CAN总线
    uint8_t             leg_index;      // 腿编号 (0=左前, 1=左后, 2=右后, 3=右前)
    uint8_t             is_outer;       // 0=内侧关节, 1=外侧关节
    float               pos_limit_min;  // 位置下限 (rad)
    float               pos_limit_max;  // 位置上限 (rad)
} MotorConfig_t;

/* ======================== 全局电机配置表 ======================== */

extern const MotorConfig_t g_motor_config[MOTOR_COUNT];

/* ======================== 辅助内联函数 ======================== */

/**
 * @brief 根据电机ID获取CAN总线句柄
 * @param id 电机ID (0~7)
 * @return CAN句柄指针，无效ID返回NULL
 */
static inline CAN_HandleTypeDef* motor_id_to_hcan(uint8_t id)
{
    return (id < MOTOR_COUNT) ? g_motor_config[id].hcan : NULL;
}

/**
 * @brief 检查电机ID是否在CAN1上
 */
static inline uint8_t motor_is_on_can1(uint8_t id)
{
    return (id < MOTOR_COUNT) ? (g_motor_config[id].hcan->Instance == CAN1) : 0;
}

/**
 * @brief 限制浮点数到指定范围
 */
static inline float clamp_float(float val, float min, float max)
{
    return (val < min) ? min : ((val > max) ? max : val);
}

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONFIG_H */
