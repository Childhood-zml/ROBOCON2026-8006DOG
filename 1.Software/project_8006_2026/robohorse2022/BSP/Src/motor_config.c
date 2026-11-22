/**
 * @file    motor_config.c
 * @brief   电机硬件配置表实现
 */
#include "motor_config.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/**
 * @brief 全局电机配置表
 * @note  索引即电机ID，共9个电机（8腿 + 1气泵）分布在两条CAN总线上
 *        每条腿有两个关节电机：外侧(outer)和内侧(inner)
 *
 *        腿编号:    Leg 0 (左前)   Leg 1 (左后)   Leg 2 (右后)   Leg 3 (右前)
 *        CAN1:                                          ID 6,7         ID 0,1
 *        CAN2:      ID 2,3         ID 4,5
 */
const MotorConfig_t g_motor_config[MOTOR_COUNT] = {
    /* id, hcan,   leg, outer, pos_min, pos_max */
    {0x00, &hcan1, 3,   1,    MOTOR_POS_LIMIT_MIN, MOTOR_POS_LIMIT_MAX}, // 右前腿-外侧
    {0x01, &hcan1, 3,   0,    MOTOR_POS_LIMIT_MIN, MOTOR_POS_LIMIT_MAX}, // 右前腿-内侧
    {0x02, &hcan2, 0,   1,    MOTOR_POS_LIMIT_MIN, MOTOR_POS_LIMIT_MAX}, // 左前腿-外侧
    {0x03, &hcan2, 0,   0,    MOTOR_POS_LIMIT_MIN, MOTOR_POS_LIMIT_MAX}, // 左前腿-内侧
    {0x04, &hcan2, 1,   1,    MOTOR_POS_LIMIT_MIN, MOTOR_POS_LIMIT_MAX}, // 左后腿-外侧
    {0x05, &hcan2, 1,   0,    MOTOR_POS_LIMIT_MIN, MOTOR_POS_LIMIT_MAX}, // 左后腿-内侧
    {0x06, &hcan1, 2,   1,    MOTOR_POS_LIMIT_MIN, MOTOR_POS_LIMIT_MAX}, // 右后腿-外侧
    {0x07, &hcan1, 2,   0,    MOTOR_POS_LIMIT_MIN, MOTOR_POS_LIMIT_MAX}, // 右后腿-内侧
    {0x08, &hcan1, 0xFF, 0,    -3.14159f,           3.14159f           }, // 气泵电机 (CAN1)
};
