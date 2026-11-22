/**
 * @file    jump_task.h
 * @brief   跳跃任务 — 状态机驱动的平台跳跃
 *
 * 数据流:
 *   遥控器 S1↑S2↑ + 双杆推上 → Change_NowState(JUMP_GROUND)
 *     → Posture_Task 唤醒 Jump_Task → Jump_OnlyOnce() 状态机
 *     → 每阶段设极坐标 + PID → motor_output_set_all_polar()
 *     → IK 解算 → Aim_Angle[8] → Motor_Task → CAN → 电机
 *
 * 跳跃时序:
 *   prep(下蹲) → launch(蹬腿) → air(缩腿) → stretch(伸腿) → land(软着陆) → STOP
 *
 * 电机控制:
 *   jump_set_pid(kp, kd) → PID_Reset() → pid->p = kp → 电机 MIT KP
 *                                      → pid->d = kd → 电机 MIT KD
 *   电机内部: T = KP×(pos_set-pos) + KD×(0-速度)
 *   软件 PID 前馈被 MaxOutput=0 禁用, 纯靠电机内部 MIT 位置环
 */
#ifndef __JUMP_TASK_H__
#define __JUMP_TASK_H__

#include "stanford_type.h"
#include "gait_param.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 跳跃阶段 ======================== */
typedef enum {
    JUMP_PHASE_PREP = 0,   // 储能下蹲 (1.2s)
    JUMP_PHASE_LAUNCH,     // 爆发蹬腿 (0.18s)
    JUMP_PHASE_AIR,        // 空中缩腿 (0.15s)
    JUMP_PHASE_STRETCH,    // 伸腿够台 (0.04s)
    JUMP_PHASE_LAND,       // 落地缓冲 (1.2s, 腿从 stretch 渐变到 land)
    JUMP_PHASE_DONE        // 完成, 恢复 PID, 切 STOP
} JumpPhase_t;

/* ======================== 跳跃参数结构体 ======================== */
typedef struct {
    /*
     * 极坐标 — 每条腿的姿态
     *   r: 腿长 cm, 范围 6(全缩) ~ 26(全伸), 站立约 17.5
     *   θ: 角度 °,  0=腿垂直, 负=腿在后, 正=腿在前
     */
    float prep_r,      prep_theta;      // 下蹲姿态
    float launch_r,    launch_theta;    // 蹬腿姿态
    float air_r,       air_theta;       // 空中缩腿姿态
    float stretch_r,   stretch_theta;   // 伸腿够台姿态
    float land_r,      land_theta;      // 落地目标姿态 (渐变终点)

    /*
     * 阶段时长 (秒)
     *   时间 × kp 决定追到位速度: 时间短+kp高=爆发, 时间长+kp低=柔和
     *   ≤0.05s 电机可能来不及响应
     */
    float prep_time;       // 下蹲窗口
    float launch_time;     // 蹬腿窗口 (关键: 太短蹬不到位)
    float air_time;        // 缩腿窗口
    float stretch_time;    // 伸腿窗口
    float land_time;       // 落地缓冲窗口 (渐变过渡)

    /*
     * 电机 PID — 每阶段独立, 阶段切换时写一次
     *   kp: 位置刚性 → 电机 MIT KP, 范围 0~500
     *       高=用力追目标, 低=柔和, 配合时间使用
     *   kd: 速度阻尼 → 电机 MIT KD, 范围 0~5 (硬件上限)
     *       高=刹车强防抖, 低=动作快可能过冲
     *
     *   电机内部: 力矩 = KP×(目标角−当前角) + KD×(0−当前速度)
     */
    float prep_kp,     prep_kd;        // 下蹲 PID
    float launch_kp,   launch_kd;      // 蹬腿 PID
    float air_kp,      air_kd;         // 缩腿 PID
    float stretch_kp,  stretch_kd;     // 伸腿 PID
    float land_kp,     land_kd;        // 落地 PID (kp低+kd高=软着陆)
    float end_kp,      end_kd;         // 已废弃, 跳跃结束恢复固定值

    /*
     * MIT 前馈 — 跳跃专用, 绕过位置环直接加力
     *   vel_ff:    目标速度 (rad/s), 让电机主动往目标冲
     *   torque_ff: 前馈力矩 (N·m), 额定20Nm, 跳跃短期可超
     *   电机公式: T = KP×Δpos + KD×(vel_ff-vel) + torque_ff
     */
    float launch_vel_ff;     // 蹬腿速度前馈 (rad/s)
    float launch_torque_ff;  // 蹬腿力矩前馈 (N·m)
} JumpParams_t;

/* ======================== 全局跳跃参数 (jump_task.c 中定义) ======================== */
extern JumpParams_t jump_params;
extern JumpParams_t bridge_jump_params;   // 过桥水平跳参数

/* ======================== API ======================== */
void Jump_Task(void *argument);                                // FreeRTOS 任务入口
void Polar_setCoord(Polar_Coord_Data_t *Action_Polar_Buffer);  // 极坐标→电机 (兼容接口)

#ifdef __cplusplus
}
#endif

#endif /* __JUMP_TASK_H__ */
