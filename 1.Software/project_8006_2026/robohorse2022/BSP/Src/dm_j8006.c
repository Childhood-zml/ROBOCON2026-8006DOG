/**
 * @file    dm_j8006.c
 * @brief   电机初始化 — CAN 过滤器 + PID 初始化 + IK 腿长设置 (DM-J8006-2EC)
 *
 * PID 初始化 (DM_J8006_Init):
 *   PID_Position[8]: p=60, i=0.0008, d=2.0, MaxOutput=0
 *     p→电机 MIT KP, d→电机 MIT KD, MaxOutput=0 禁用软件 PID 输出
 *   PID_Speed[8]:    p=3.5, i=0, d=0, MaxOutput=0
 *     速度环同样禁用输出
 *
 *   所有步态共用这套默认 PID, 只有跳跃时 jump_set_pid() 临时改、
 *   跳跃结束后恢复。
 *
 * CAN 映射:
 *   CAN1 (右侧): 电机 0,1,6,7
 *   CAN2 (左侧): 电机 2,3,4,5
 */
#include "dm_j8006.h"
#include "main.h"
#include "pid.h"
#include "gait_param.h"
#include "can.h"
#include "dm_j8006_drv.h"

PID_t PID_Position[8];  //位置PID
extern PID_t PID_Speed[8];  //速度PID (定义在 dm_j8006_ctrl.c)
PID_t PID_IMU_Climbing; //爬坡用PID
PID_t PID_IMU_Line;     //直线修正pid

extern DM_J8006_DATA_t dm_j8006_data[9];		//达妙电机反馈数据

// 配置并启动CAN，并设置PID数据
void DM_J8006_Init()
{
    //初始化CAN过滤器
    CAN_FilterTypeDef can_filter =
    {
        0x0000,
        0x0000,
        0x0000,
        0x0000,
        CAN_FILTER_FIFO0,
        0,
        CAN_FILTERMODE_IDMASK,
        CAN_FILTERSCALE_32BIT,
        CAN_FILTER_ENABLE,
        14
    };
    HAL_CAN_ConfigFilter(&hcan1, &can_filter); //设置CAN1过滤器
    can_filter.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter); //设置CAN2过滤器
    
    // 修改电机ID映射关系
    // CAN1: ID1-4 -> 右前腿(0,1), 右后腿(2,3)
    // CAN2: ID5-8 -> 左前腿(4,5), 左后腿(6,7)
    
    /* --- 位置环 PID 初始化 (8个电机) --- */
    /* p/d 即电机 MIT 模式 KP/KD, 保持原始值确保跟踪刚性 */
    /* 诊断: MaxOutput=0 禁用软件PID输出, 纯靠电机MIT内部控制 */
    PID_Set_t pid_set =
        {
            .pid_mode = POSITION_PID,
            .p = 60.0f,        // 位置环 Kp → 电机 MIT KP (原始值)
            .i = 0.0008f,      // 微量积分 (原始值)
            .d = 2.0f,         // 位置环 Kd → 电机 MIT KD (原始值)
            .MaxOutput = 0,    // 0=禁用软件PID速度前馈 (诊断用)
            .IntegralLimit = 0,
            .max_err = 0,
            .deadband = 0,
        };
    for (uint8_t i = 0; i < 8; i++)
    {
        PID_Init(&PID_Position[i], &pid_set);
    }

    /* --- 速度环 PID 初始化 (8个电机) --- */
    pid_set.p = 3.5f;         // 速度环 Kp (原始值)
    pid_set.i = 0.0f;
    pid_set.d = 0.0f;         // 速度环 Kd (原始值)
    pid_set.MaxOutput = 0;    // 0=禁用软件PID力矩前馈 (诊断用)
    pid_set.IntegralLimit = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        PID_Init(&PID_Speed[i], &pid_set);
    }

    /* --- IMU 校正 PID --- */
    pid_set.p = 3.5f;
    pid_set.i = 0.0f;
    pid_set.d = 0.0f;
    pid_set.MaxOutput = 0;    // IMU PID 不限幅
    PID_Init(&PID_IMU_Climbing, &pid_set);

    pid_set.MaxOutput = 16;
    pid_set.p = 1.5f;
    pid_set.i = 0.0f;
    pid_set.d = 0.01f;
    PID_Init(&PID_IMU_Line, &pid_set);

    for (uint8_t i = 0; i < 4; i++)
    {
        Stanford_Type_Lite_Init(&Gait_Data[i], 12.0f, 20.0f); //设置腿部长度参数
    }

    HAL_CAN_Start(&hcan1);                                             // CAN1开始运行
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // CAN1开始接受数据
    HAL_CAN_Start(&hcan2);                                             // CAN2开始运行
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); // CAN2开始接受数据
}


/*接收数据*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef can_header;
    uint8_t rx_data[8];
	
	if(hcan->Instance == CAN1 || hcan->Instance == CAN2)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can_header, rx_data);
	    if (can_header.StdId <= 0x07)			
		{	
			//达妙电机反馈数据
			FEEDBACK_DATA_DM_J8006(can_header.StdId , rx_data);
		}
	}
}
