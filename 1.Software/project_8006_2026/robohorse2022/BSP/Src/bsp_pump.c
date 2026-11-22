#include "main.h"
#include "bsp_pump.h"

// #define PUMP_PWM_CHANNEL1 1
// #define PUMP_PWM_CHANNEL2 2

#define PUMP_MAX_PWM     10000
#define PUMP_MIN_PWM     0

extern TIM_HandleTypeDef htim5;

// static void servo_pwm_set(uint16_t pwm, uint8_t i);
static void pump_pwm_set(uint8_t i, uint16_t pwm);

BspTim bsp_pump = {
    .pump_ctrl = PwmCmdPump,
    // .servo_ctrl = pump_pwm_set
};

/*-------------------- Public functions --------------------*/

/**
 * @brief          通过PWM发送PWM信号控制气泵
 * @param[in]      pump_id 气泵ID，从PWM口开始为1
 * @param[in]      pwm pwm信号占空比
 * @retval         none
 */
void PwmCmdPump(uint8_t pump_id, uint16_t pwm)
{
    if (pwm > PUMP_MAX_PWM) {
        pwm = PUMP_MAX_PWM;
    }
    pump_pwm_set(pump_id, pwm);
}

/*-------------------- Private functions --------------------*/

/**
 * @brief           通过改变ccr的值来改变占空比
 * @param           pwm 
 * @param           i 
 * @retval          none
 */
void pump_pwm_set(uint8_t i, uint16_t pwm)
{
    switch(i)
    {
        case 1:
        {
            __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, pwm);
        }break;
        case 2:
        {
            __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, pwm);
        }break;
        case 3:
        {
            __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, pwm);
        }break;
        case 4:
        {
            __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, pwm);
        }break;
    }
}

// void pump_pwm_set(uint16_t pwm, uint8_t i)
// {
//     switch(i)
//     {
//         case 1:
//         {
//             __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, pwm);
//         }break;
//         case 2:
//         {
//             __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, pwm);
//         }break;
//         case 3:
//         {
//             __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, pwm);
//         }break;
//         case 4:
//         {
//             __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, pwm);
//         }break;
//     }
// }