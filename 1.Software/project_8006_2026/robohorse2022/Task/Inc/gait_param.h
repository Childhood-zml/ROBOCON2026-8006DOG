/**
 * @file    gait_param.h
 * @brief   步态参数 — 运动状态枚举, PID/轨迹参数表, 全局变量声明
 *
 * 数据流核心:
 *   遥控器开关 → Change_NowState(状态) → Posture_Task 主循环
 *     → 装载 state_gait_params[状态] → Cycloid_Gen[4]
 *     → Ctrl_byRemoctrl 覆写摇杆参数
 *     → Calculate_Gait: 摆线轨迹 → aim_x[4], aim_y[4]
 *     → Target_toMotor → motor_output_set_all_cartesian
 *     → IK → Aim_Angle[8] → Motor_Task → CAN → 电机
 *
 * 注意: state_gait_gains[] (各状态 PID) 当前为死代码,
 *       所有状态共用 DM_J8006_Init() 的默认 PID (p=60,d=2.0,MaxOutput=0)
 */
#ifndef GAIT_PARAM_H
#define GAIT_PARAM_H

#include <math.h>
#include "stanford_type.h"
#include "cycloid_generator.h"

#define pi (3.14159265f) // 圆周率

/*
 * 编码器 → 弧度:
 *   编码器 0~8191 对应 360°×减速比9 = 3240°
 *   = 56.55 rad (电机输出轴)
 *   所以 1 rad = 8191/(2π×9) ≈ 515.66 个 tick
 */
#define ReductionAndAngleRatio (515.6620162069706f)

/*
 * 机械尺寸 (单位: cm)
 *   L1: 前后髋间距 (身体长度)
 */
#define L1 (39.6f)          // 前后腿髋关节间距
#define init_h 14.3f        // 站立时的身体离地高度
#define FORWARD_hight 15.0f // 前进时前腿身体高度
#define BACK_hight 15.0f    // 前进时后腿身体高度
#define shortK_hight 18.0f  // 短腿模式（屈膝，身体较高）
#define longK_hight   14.0f // 长腿模式（伸展，身体较低）

/* ======================== 运动状态枚举 ======================== */
/*
 * 遥控器映射:
 *   S1上 S2上 = 跳跃(推上)/过桥跳(拉下)  S1上 S2下 = 身高测试   S1上 S2中 = (空)
 *   S1下 S2上 = 爬坡   S1下 S2下 = 急停     S1下 S2中 = 小跑
 *   S1中 S2上 = 过台阶桥  S1中 S2下 = (空)  S1中 S2中 = 走路
 *
 * 新增状态必须放在 END 之前 (数组大小=END+1)
 */
typedef enum
{
    NORMAL = 0,  // 手动调试 (非遥控器触发, 摇杆直接控腿)

    CLIMBING,    // 爬坡 (S1下 S2上)
    TURN_LEFT,   // 左转 (摇杆左推自动)
    TURN_RIGHT,  // 右转 (摇杆右推自动)
    WALK,        // 走路 (S1中 S2中)
    TROT,        // 小跑 (S1下 S2中)
    BRIDGE,      // 过台阶桥 (S1中 S2上)

    STOP,        // 急停/站立 (S1下 S2下)
    RELEASE,     // 释放 (断电保护)
    JUMP_GROUND, // 平台跳跃 (S1↑ S2↑ + 双杆推上)
    JUMP_BRIDGE, // 过桥水平跳 (S1↑ S2↑ + 双杆拉下)
    JUMP_UPHILL, // 上坡跳跃 (已废弃)
    HEIGHT_TEST, // 身高测试 (S1上 S2下, 摇杆调高度)
    START,       // 启动 (数组占位)
    END,         // 数组末尾标记 (自动计算数组长度)
} Robohorse_State;

typedef enum
{
    POSITIVE,
    NEGATIVE
} Robohorse_Direction;

/* ======================== 腿部 PID 增益 ======================== */
/*
 * kp_pos: 位置环 P → 电机 MIT KP
 * kd_pos: 位置环 D → 电机 MIT KD
 * kp_spd: 速度环 P (当前未使用, MaxOutput=0)
 * kd_spd: 速度环 D (当前未使用, MaxOutput=0)
 */
typedef struct
{
    float kp_pos;
    float kd_pos;
    float kp_spd;
    float kd_spd;
} LegGain;

/* ======================== 四腿独立参数 ======================== */
typedef struct
{
    Cycloid_Generator_Param_t detached_params_0;
    Cycloid_Generator_Param_t detached_params_1;
    Cycloid_Generator_Param_t detached_params_2;
    Cycloid_Generator_Param_t detached_params_3;
} DetachedParam;

/* ======================== 全局变量声明 ======================== */
/*
 * 当前生效的 PID (死代码, 从未加载到 PID_Position)
 * 实际生效的是 DM_J8006_Init() 默认值
 */
extern LegGain Gait_Gains;
extern LegGain state_gait_gains[END+1];                     // 各状态 PID 参数表

/* 当前生效的轨迹参数 / 各状态轨迹参数表 */
extern Cycloid_Generator_Param_t gait_params;
extern Cycloid_Generator_Param_t state_gait_params[END+1];  // 跳跃类填 NAN (不走摆线)

/* 四腿独立参数 (死代码, 未使用) */
extern DetachedParam detached_params;
extern DetachedParam state_detached_params[4];
extern DetachedParam RcDetachedParam;

/* 核心全局状态 */
extern Robohorse_State NowState;              // 当前运行状态 (Posture_Task 每周期读)
extern Robohorse_Direction NowDir;            // 当前方向 (前进/后退)
extern Stanford_Type_Lite_Data_t Gait_Data[4]; // 四条腿 IK 数据
extern Cycloid_Generator_t Cycloid_Gen[4];    // 四条腿摆线生成器实例
extern float Aim_Angle[8];                    // 目标电机角度 rad (IK 输出)
extern volatile uint8_t Is_Aim_Angle_Get;     // 角度就绪标志 (Motor_Task 读后清零)
extern float Last_Time;                       // 上次轨迹计算时间(秒)
extern uint8_t Mode_Change_Flag;              // 状态切换标志 (Posture_Task 检测)

#endif /* GAIT_PARAM_H */
