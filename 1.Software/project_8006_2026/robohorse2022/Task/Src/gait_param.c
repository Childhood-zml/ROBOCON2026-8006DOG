/*步态参数设置*/
#include "gait_param.h"

//当前生效PID参数
LegGain Gait_Gains = {15.0f, 0.3f, 20.0f, 0.05f};

//各状态PID参数表（明确大小END+1）
LegGain state_gait_gains[END+1] = {
    //{kp_pos, kd_pos， kp_spd, kd_spd}，
    {15.0f, 0.3f, 20.0f, 0.05f},  // NORMAL
    {18.0f, 0.5f, 16.0f, 0.08f},  // CLIMBING
    {12.0f, 0.2f, 22.0f, 0.03f},  // TURN_LEFT
    {12.0f, 0.2f, 22.0f, 0.03f},  // TURN_RIGHT
    {14.0f, 0.4f, 22.0f, 0.06f},  // WALK
    {10.0f, 0.15f, 25.0f, 0.02f},  // TROT
    {10.0f, 0.15f, 25.0f, 0.02f},  // BRIDGE (套用TROT参数,后续独立调优)
    {22.0f, 1.0f, 10.0f, 0.15f},  // STOP
    {15.0f, 0.3f, 20.0f, 0.05f},  // RELEASE
    {16.0f, 0.4f, 18.0f, 0.06f},  // JUMP_GROUND
    {16.0f, 0.4f, 18.0f, 0.06f},  // JUMP_BRIDGE
    {16.0f, 0.4f, 18.0f, 0.06f},  // JUMP_UPHILL
    {25.0f, 1.0f, 10.0f, 0.15f},  // HEIGHT_TEST (硬锁位置)
    {18.0f, 0.5f, 15.0f, 0.08f},  // START
    {15.0f, 0.3f, 20.0f, 0.05f},  // END
};

//当前生效轨迹参数
Cycloid_Generator_Param_t gait_params = {14.3f, 0.0f, 3.0f, 0.00f, 0.25f, 0.3f};

Cycloid_Generator_Param_t state_gait_params[END + 1] = {
      // NORMAL
    {14.3f, 0.0f, 0.00f, 0.00f, 0.0f, 0.1f},

            // CLIMBING 爬坡
    {FORWARD_hight, 7.30f, 6.50f, 0.1f, 0.3f, 3.5f},  // CLIMBING

             // TURN_LEFT
    {15.0f, 6.0f, 3.0f, 1.5f, 0.30f, 3.0f},

            // TURN_RIGHT
    {15.0f, 6.0f, 3.0f, 1.5f, 0.30f, 3.0f},

            // WALK
     {14.3f, 6.0f, 4.0f, 0.1f, 0.50f, 4.5f},  // 机身高度12.0→14.3恢复原版标准站立高度; 提速freq 4.0→4.5 (108cm/s)

            // TROT（低趴）
    {11.99f, 6.00f, 3.0f, 0.1f, 0.5f, 5.0f},  // 低趴机身保持11.99; 提速freq 3.5→5.0 (120cm/s), 抬腿2.5→3.0防蹭


        // BRIDGE (与WALK保持一致，低姿态)
    {14.3f, 6.0f, 3.0f, 0.1f, 0.5f, 3.5f},

            // STOP
    {14.3f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},

    {NAN, NAN, NAN, NAN, NAN, NAN},           // RELASE
    {18.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // JUMP_GROUND (防NAN污染高度过渡)
    {18.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // JUMP_BRIDGE (防NAN污染高度过渡)
    {NAN, NAN, NAN, NAN, NAN, NAN},           // JUMP_UPHILL
    {18.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // HEIGHT_TEST (身体高度会动态改)
    {NAN, NAN, NAN, NAN, NAN, NAN},           // START
    {NAN, NAN, NAN, NAN, NAN, NAN},           // END
};

//四腿独立参数（运行时）
DetachedParam detached_params;

//四腿独立参数预设表
DetachedParam state_detached_params[4] = {
    //TROT（索引0）
    {                                           // TROT 对角小跑
     {14.3f, 5.0f, 2.50f, 1.50f, 0.25f, 4.0f},
     {14.3f, 5.0f, 2.50f, 1.50f, 0.25f, 4.0f},
     {14.3f, 5.0f, 2.50f, 1.50f, 0.25f, 4.0f},
     {14.3f, 5.0f, 2.50f, 1.50f, 0.25f, 4.0f}},

    {                                          // CLIMBING	爬坡
     {15.0f, 8.0f, 3.00f, 1.00f, 0.25f, 3.6f}, //前腿
     {15.0f, 8.0f, 3.00f, 1.00f, 0.25f, 3.6f},
     {17.0f, 8.0f, 3.00f, 1.00f, 0.25f, 3.6f}, //后腿
     {17.0f, 8.0f, 3.00f, 1.00f, 0.25f, 3.6f}},
    {
        {15.0f, 6.0f, 3.5f, 2.00f, 0.28f, 3.0f}, // TURN_LEFT
        {15.0f, 6.0f, 3.5f, 2.00f, 0.28f, 3.0f},
        {15.0f, 6.0f, 3.5f, 2.00f, 0.28f, 3.0f},
        {15.0f, 6.0f, 3.5f, 2.00f, 0.28f, 3.0f}
    },
    {
        {15.0f, 6.0f, 3.5f, 2.00f, 0.28f, 3.0f}, // TURN_RIGHT
        {15.0f, 6.0f, 3.5f, 2.00f, 0.28f, 3.0f},
        {15.0f, 6.0f, 3.5f, 2.00f, 0.28f, 3.0f},
        {15.0f, 6.0f, 3.5f, 2.00f, 0.28f, 3.0f}
    },

};

//遥控调参默认值
DetachedParam RcDetachedParam = {
    {14.3f, 0.0f, 3.00f, 2.00f, 0.25f, 3.2f},
    {14.3f, 0.0f, 3.00f, 2.00f, 0.25f, 3.2f},
    {14.3f, 0.0f, 3.00f, 2.00f, 0.25f, 3.2f},
    {14.3f, 0.0f, 3.00f, 2.00f, 0.25f, 3.2f}};
//系统状态
Robohorse_State NowState = STOP; //当前运行状态
Robohorse_Direction NowDir = POSITIVE;

Stanford_Type_Lite_Data_t Gait_Data[4]; // 步态数据
Cycloid_Generator_t Cycloid_Gen[4]; // 摆线生成器
float Aim_Angle[8]; //目标角度，单位为弧度rad
volatile uint8_t Is_Aim_Angle_Get = 0; //是否已经获得到目标角度
float Last_Time = 0.0f; //上次计算时间
uint8_t Mode_Change_Flag = 1; //模式切换标志
	