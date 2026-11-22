/**
 * @file    motor_task.c
 * @brief   电机驱动任务 (500Hz 目标)
 */
#include <stdarg.h>
#include <stdint.h>

#include "motor_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "dm_j8006.h"
#include "pid.h"
#include "gait_param.h"
#include "usart.h"

#include "standup_task.h"
#include "dm_j8006_ctrl.h"
#include "debug_output.h"

extern PID_t PID_Position[8];
extern PID_t PID_Speed[8];

extern float imu_Leg_offset;
extern float Standup_LegR_Offset;
extern float Standup_LegL_Offset;

float PID_Aim_Angle[8];
uint8_t DM_J8006_ENABLE_FLAG = 1;
uint8_t DM_J8006_ctrl_FLAG = 0;

static const struct {
    int8_t standup_r_sign;
    int8_t standup_l_sign;
    int8_t imu_sign;
} motor_offset_map[8] = {
    { -1,  0, -1 },
    {  0, +1, +1 },
    {  0, -1, +1 },
    { +1,  0, -1 },
    {  0, -1, +1 },
    { +1,  0, -1 },
    {  0, +1, -1 },
    { -1,  0, +1 },
};

static void apply_motor_offsets(void)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        float standup = motor_offset_map[i].standup_r_sign * Standup_LegR_Offset
                      + motor_offset_map[i].standup_l_sign * Standup_LegL_Offset;
        float imu     = motor_offset_map[i].imu_sign * imu_Leg_offset;
        PID_Aim_Angle[i] = Aim_Angle[i] + standup + imu;
    }
}

void Motor_Task(void *argument)
{
    static portTickType xLastWakeTime;
    static const portTickType xFrequency = pdMS_TO_TICKS(2);
    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        /* TROT 低趴高速腿撑不住: 仅TROT模式提高MIT刚度 KP 60→80 / KD 2.0→2.5,
           其余模式保持原始 60/2.0 (TROT转弯瞬间NowState=TURN_*会短暂回60, 转弯载荷小无影响) */
        {
            const float kp = (NowState == TROT) ? 80.0f : 60.0f;
            const float kd = (NowState == TROT) ? 2.5f : 2.0f;
            for (uint8_t i = 0; i < 8; i++)
            {
                PID_Position[i].p = kp;
                PID_Position[i].d = kd;
            }
        }

        if (Is_Aim_Angle_Get == 1) {
            apply_motor_offsets();
            Is_Aim_Angle_Get = 0;
        }

        for (uint8_t i = 0; i < 8; i++)
        {
            PID_Calc(&PID_Position[i],
                    J8006[i].Total_Angle / ReductionAndAngleRatio,
                    PID_Aim_Angle[i]);
            PID_Calc(&PID_Speed[i],
                    J8006[i].Speed_RPM,
                    PID_Position[i].pos_out);
        }

        dm_j8006_all_dual_ctrl(PID_Aim_Angle, PID_Position, PID_Speed);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
