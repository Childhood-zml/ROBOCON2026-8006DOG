#include "detect_task.h"
#include "drv_led.h"
#include "dm_j8006_drv.h"
#include "can.h"
#include "motor_config.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"


uint8_t High_Temp_Warning = 0; //高温警报标志

/**
 * 系统检测任务，用于检测电机温度
 * 流程：检测电机温度->状态灯亮
 * */
void Detect_Task(void *argument)
{
    while (1)
    {
        if (High_Temp_Warning == 0)
        {
            // 检测8个电机温度
            for (int i = 0; i < 8; i++)
            {
                // 跳过未使用的电机
                if (dm_j8006_data[i].id == 0)
                    continue;
                
                // 检查电机线圈温度（或 MOS 温度）
                if (dm_j8006_data[i].T_Rotor >= MOTOR_TEMP_HIGH)  // 温度达到目标时高温警告
                {
                    High_Temp_Warning = 1;
                    // 失能电机，防止烧坏
                    // 使用电机配置表获取正确的CAN总线（CAN2电机不能发到CAN1）
                    DISABLE_DM_J8006(*g_motor_config[i].hcan, dm_j8006_data[i].id);
                }
            }
            
            LED_On(LED_GREEN);//温度正常绿灯闪烁
            osDelay(500);
            LED_Off(LED_GREEN);
            osDelay(500);
        }
        else
        {
            // 检查温度是否恢复（低于80度解除警告）
            uint8_t all_normal = 1;
            for (int i = 0; i < 8; i++)
            {
                if (dm_j8006_data[i].id == 0)
                    continue;
                if (dm_j8006_data[i].T_Rotor >= MOTOR_TEMP_RECOVERY)
                {
                    all_normal = 0;
                    break;
                }
            }
            if (all_normal)
                High_Temp_Warning = 0;
            
            LED_On(LED_RED);//高温警告红灯闪烁
            osDelay(500);
            LED_Off(LED_RED);
            osDelay(500);
        }
    }
}
