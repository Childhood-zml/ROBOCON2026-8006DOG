/**
 * @file    jump_task.c
 * @brief   跳跃任务 — 状态机 + 参数化 + 落地缓冲渐变
 *
 * 架构:
 *   jump_params → Jump_Task() → Jump_OnlyOnce() → motor_output_set_all_polar()
 *   → IK → Aim_Angle[8] → Motor_Task → CAN → 电机
 *
 * 关键细节:
 *   - jump_set_pid() 每阶段只调一次 (pid_dirty), 防和 Motor_Task 抢 PID
 *   - vTaskDelay(1) 让 CAN 喘息, Jump(25)>Posture(24) 保证姿态不干扰
 *   - 跳跃结束后恢复默认 PID, 否则步态变软
 *   - 落地阶段腿从 stretch 渐变到 land, 模拟弹簧缓震
 */
#include "jump_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "gait_param.h"
#include "posture_task.h"
#include "motor_output.h"
#include "dm_j8006_ctrl.h"          /* [8009] 8009 驱动头文件 */

extern osThreadId_t JumpHandle;
extern PID_t PID_Position[8];

/* ======================== 平台跳跃参数 ======================== */
/*
 * 极坐标: r=腿长cm(6~26, 站立~17.5), θ=角度°(负=腿在后, 0=垂直)
 * 时间: 决定电机追速, 短+高=爆发, 长+低=柔和
 * PID: kp=MIT KP(0~500, 位置刚性), kd=MIT KD(0~5, 速度阻尼)
 *       只在阶段切换时写一次, 避免和 Motor_Task 冲突
 */
JumpParams_t jump_params = {

    /* ===== 极坐标 — 跳远: θ越负=越往后蹬=越远 ===== */
    /* prep_r: 准备腿长(cm) | [8009] 11.0
     * 8009 电机高61.7mm(vs 8006 40mm), 蹲太深易触地, 适度抬高 */
    .prep_r    = 11.0f,   
    /* prep_theta: 准备角度(°) | [8009] -10.0
     * 电机更重, 前倾略收, 防重心压过头 */
    .prep_theta    = -10.0f,   
    /* launch_r: 蹬腿腿长(cm) | [8009] 20.0
     * 减速比9:1(vs 6:1), 输出轴转速降44%, 留速度余量 */
    .launch_r  = 20.0f,  
    /* launch_theta: 蹬腿角度(°) | [8009] -10.0
     * 配合缩短 launch_r, 后蹬略收, 防拖地 */
    .launch_theta  = -10.0f,    
    /* air_r: 腾空腿长(cm) | [8009] 12.0
     * 收腿减小, 8009 转速慢, 收太紧会抖动 */
    .air_r     = 12.0f,  
    /* air_theta: 腾空角度(°) | [8009] 8.0
     * 正值前摆, 角度保守, 配合较慢伸腿速度 */
    .air_theta     =  8.0f,
    /* stretch_r: 伸腿腿长(cm) | [8009] 14.0
     * 伸腿适度, 转速限制下优先落点精准 */
    .stretch_r = 14.0f,  
    /* stretch_theta: 伸腿角度(°) | [8009] 4.0
     * 略收, 防9:1减速器末端高速冲击 */
    .stretch_theta =   4.0f,
    /* land_r: 落地腿长(cm) | [8009] 11.0
     * 修正: 从 stretch=14.0 降到 11.0, 主动屈膝吸能, 模拟弹簧压缩 */
    .land_r    = 11.0f,  
    /* land_theta: 落地角度(°) | [8009] 5.0
     * 修正: 从 2.0 改到 5.0, 略前倾, 重心前移配合屈膝缓冲 */
    .land_theta    =   5.0f,


    /* ===== 阶段时长(秒) ===== */
    /* prep_time: 下蹲蓄力(s) | [8009] 1.8
     * 输出轴速度降44%, 下蹲更慢更稳; 电机重, 惯性大, 猛蹲易振荡 */
    .prep_time    = 1.8f,
    /* launch_time: 蹬腿发力窗口(s) | [8009] 0.20
     * 蹬地延长, 补转速不足; 超0.5s 腿未离地就伸直, 效率低 */
    .launch_time  = 0.15f,     
    /* air_time: 腾空收腿(s) | [8009] 0.20
     * 收腿幅度减小(air_r 12.0), 时间翻倍, 防电机饱和 */
    .air_time     = 0.20f,
    /* stretch_time: 伸腿够台(s) | [8009] 0.25
     * 伸腿放缓, 确保9:1减速器末端精准到位, 不甩腿 */
    .stretch_time = 0.25f,
    /* land_time: 软着陆缓冲(s) | [8009] 2.0
     * 电机自重+机身, 落地动能大, 延长缓冲, 像"重坦克"着陆 */
    .land_time    = 2.0f,

    /* ===== PID ===== */
    /* prep_kp: 准备位置刚性(0~500) | [8009] 70.0
     * 8009 相电感61uH(vs 215uH), 电机极硬, KP 下调, 防下蹲高频啸叫 */
    .prep_kp    = 70.0f,   
    /* prep_kd: 准备速度阻尼(0~5) | [8009] 2.5
     * 低电感电机需更大阻尼抑制振荡 */
    .prep_kd    = 2.5f,
    /* launch_kp: 蹬腿位置刚性(0~500) | [8009] 180.0
     * 爆发力阶段要高 KP 保方向, 但比8006的280必须降, 防过冲 */
    .launch_kp  = 180.0f,  
    /* launch_kd: 蹬腿速度阻尼(0~5) | [8009] 1.8
     * 增加 KD, 抑制蹬腿末端低电感电流过冲 */
    .launch_kd  = 1.8f,
    /* air_kp: 腾空位置刚性(0~500) | [8009] 250.0
     * 空中锁姿态, 8009 刚性过强, KP 从350降至250, 配合更高 KD */
    .air_kp     = 250.0f,  
    /* air_kd: 腾空速度阻尼(0~5) | [8009] 4.5
     * 收腿需强阻尼, 防9:1减速器背隙+低电感引发抖动 */
    .air_kd     = 4.5f,
    /* stretch_kp: 伸腿位置刚性(0~500) | [8009] 250.0
     * 精准落点需刚度, 8009 输出轴绝对编码器直接反馈, 无需过高 KP */
    .stretch_kp = 250.0f,  
    /* stretch_kd: 伸腿速度阻尼(0~5) | [8009] 5.0
     * 最大阻尼, 伸腿末端像"刹车"钉住, 防甩过目标 */
    .stretch_kd  = 5.0f,
    /* land_kp: 落地位置刚性(0~500) | [8009] 12.0
     * 修正: 电机更重, 落地必须更软! 从18压到12, 像弹簧吸能, 防机身弹起 */
    .land_kp    = 12.0f,   
    /* land_kd: 落地速度阻尼(0~5) | [8009] 5.0
     * 最大阻尼, 快速吸收落地速度, 防像皮球弹跳 */
    .land_kd    = 5.0f,
    /* end_kp: 结束位置刚性(0~500) | [8009] 12.0
     * 恢复站立需一定刚性, 8009 低电感, 起步要缓 */
    .end_kp     = 12.0f,   
    /* end_kd: 结束速度阻尼(0~5) | [8009] 2.5
     * 增加阻尼, 消除残余抖动 */
    .end_kd     = 2.5f,

    /* ===== MIT 前馈: 跳远需要力量 ===== */
    /* launch_vel_ff: 蹬腿速度前馈 | [8009] 32.0
     * 按转速比例: 60*(100/120)*(6/9)≈33.3, 取32, 防超速报警 */
    .launch_vel_ff    = 32.0f,
    /* launch_torque_ff: 蹬腿力矩前馈 | [8009] 16.0
     * 8009 额定扭矩20NM, 取16NM(80%额定), 用大扭矩优势, 留20%余量防过流 */
    .launch_torque_ff = 16.0f,
};

/* ======================== 过桥水平跳参数 ======================== */
/*
 * 简化流程: 下蹲 → 后蹬 → 伸腿上平台(无落地缓冲)
 * 目标: COM 水平移动~30cm, 越过15cm间隙
 */
JumpParams_t bridge_jump_params = {

    /* ===== 极坐标 ===== */
    /* prep_r: 准备腿长(cm) | [8009] 13.0
     * 过桥浅蹲, 电机大, 重心不宜压太低 */
    .prep_r    = 13.0f,   
    /* prep_theta: 准备角度(°) | [8009] -8.0
     * 略直立, 防前倾磕头 */
    .prep_theta    = -8.0f,   
    /* launch_r: 蹬腿长度(cm) | [8009] 19.0
     * 水平跳不需腿完全伸直, 适度发力 */
    .launch_r  = 19.0f,   
    /* launch_theta: 蹬腿角度(°) | [8009] -8.0
     * 负值保水平后蹬, 角度保守防后仰 */
    .launch_theta  = -8.0f,   
    /* air_r: 腾空收腿长度(cm) | [8009] 13.0
     * 间隙小, 收腿紧, 但比8006略松(转速限制) */
    .air_r     = 13.0f,   
    /* air_theta: 腾空角度(°) | [8009] 6.0
     * 正值前摆, 准备伸腿, 角度保守 */
    .air_theta     =   6.0f,   
    /* stretch_r: 伸腿长度(cm) | [8009] 18.0
     * 必须够到对面, 保持原值, 伸腿时间延长 */
    .stretch_r = 18.0f,   
    /* stretch_theta: 伸腿角度(°) | [8009] 16.0
     * 正值很大, 但比8006的19°收3°, 防9:1末端超伸报警 */
    .stretch_theta =   16.0f,   
    /* land_r: 落地长度(cm) | [8009] 18.0
     * 与 stretch 相同, 代码中 stretch 后直接结束, 占位 */
    .land_r    = 18.0f,   
    /* land_theta: 落地角度(°) | [8009] 16.0
     * 与 stretch 相同, 占位 */
    .land_theta    =  16.0f,   

    /* ===== 阶段时长 ===== */
    /* prep_time: 蓄力时间(s) | [8009] 1.2
     * 比平台跳短, 但比8006的0.8s长, 给大电机到位时间 */
    .prep_time    = 1.2f,      
    /* launch_time: 蹬腿窗口(s) | [8009] 5.0
     * 水平跳需速度累积, 8009 输出轴慢, 大幅延长蹬地时间建水平速度 */
    .launch_time  = 5.0f,     
    /* air_time: 收腿时间(s) | [8009] 0.15
     * 快速过渡, 但比8006略长 */
    .air_time     = 0.15f,     
    /* stretch_time: 伸腿时间(s) | [8009] 0.18
     * 伸腿够台, 时间略增, 保证到位 */
    .stretch_time = 0.18f,     
    /* land_time: 落地缓冲时间(s) | [8009] 3.0
     * 代码中 stretch 后直接结束, 保持占位 */
    .land_time    = 3.0f,      

    /* ===== PID ===== */
    /* prep_kp: 下蹲刚性(0~500) | [8009] 70.0 */
    .prep_kp    = 70.0f,   
    /* prep_kd: 下蹲阻尼(0~5) | [8009] 2.5 */
    .prep_kd    = 2.5f,
    /* launch_kp: 蹬腿刚性(0~500) | [8009] 200.0
     * 水平跳方向必须准, 8009 低电感, KP 从300降至200 */
    .launch_kp  = 200.0f,  
    /* launch_kd: 蹬腿阻尼(0~5) | [8009] 1.8 */
    .launch_kd  = 1.8f,    
    /* air_kp: 腾空刚性(0~500) | [8009] 250.0 */
    .air_kp     = 250.0f,  
    /* air_kd: 腾空阻尼(0~5) | [8009] 4.5 */
    .air_kd     = 4.5f,
    /* stretch_kp: 伸腿刚性(0~500) | [8009] 250.0 */
    .stretch_kp = 250.0f,  
    /* stretch_kd: 伸腿阻尼(0~5) | [8009] 5.0 */
    .stretch_kd  = 5.0f,
    /* land_kp: 落地刚性(0~500) | [8009] 18.0 */
    .land_kp    = 18.0f,   
    /* land_kd: 落地阻尼(0~5) | [8009] 5.0 */
    .land_kd    = 5.0f,     
    /* end_kp: 结束刚性(0~500) | [8009] 50.0
     * 过桥结束要迅速站稳, 8009 刚性过强, 从70降至50 */
    .end_kp     = 50.0f,   
    /* end_kd: 结束阻尼(0~5) | [8009] 2.5 */
    .end_kd     = 2.5f,

    /* ===== MIT 前馈 ===== */
    /* launch_vel_ff: 蹬腿速度前馈 | [8009] 28.0
     * 按转速比例: 55*(100/120)*(6/9)≈30.6, 取28, 保守防水平冲过头 */
    .launch_vel_ff    = 28.0f,
    /* launch_torque_ff: 蹬腿力矩前馈 | [8009] 14.0
     * 8009 额定20NM, 取14(70%), 水平跳摩擦大, 力矩需求略低于平台跳 */
    .launch_torque_ff = 14.0f,
};

/* ======================== 静态辅助 ======================== */

/**
 * @brief 设全部 8 个电机 MIT KP/KD, 每阶段调用一次
 * @param kp  位置刚性 → pid->p → 电机 MIT KP
 * @param kd  速度阻尼 → pid->d → 电机 MIT KD
 *
 * PID_Reset(p, i, d): p→MIT KP, i→清积分, d→MIT KD
 */
static void jump_set_pid(float kp, float kd)
{
    for (uint8_t i = 0; i < 8; i++)
        PID_Reset(&PID_Position[i], kp, 0.0f, kd);
}

/** 设四条腿到同一极坐标 */
static void jump_set_polar(Polar_Coord_Data_t *p, float r, float theta)
{
    for (int i = 0; i < 4; i++) { p[i].radius = r; p[i].theta = theta; }
}

/* ======================== 跳跃状态机 ======================== */

void Jump_OnlyOnce(void)
{
    const JumpParams_t *p = (NowState == JUMP_BRIDGE) ? &bridge_jump_params : &jump_params;
    JumpPhase_t phase = JUMP_PHASE_PREP;
    TickType_t tick = xTaskGetTickCount();
    Polar_Coord_Data_t polar[4];
    uint8_t pid_dirty = 1;  // 新阶段置1, PID写完清0, 每阶段只写一次

    /* 初始姿态: 下蹲 */
    jump_set_polar(polar, p->prep_r, p->prep_theta);

    while (phase != JUMP_PHASE_DONE)
    {
        float t = (float)(xTaskGetTickCount() - tick)
                * portTICK_PERIOD_MS / 1000.0f;    // 本阶段已过时间(秒)

        switch (phase)
        {

        /* ========== ① 下蹲: 保持姿态, 等时间到 ========== */
        case JUMP_PHASE_PREP:
            if (t >= p->prep_time) {
                phase = JUMP_PHASE_LAUNCH; tick = xTaskGetTickCount();
                jump_set_polar(polar, p->launch_r, p->launch_theta);
                pid_dirty = 1;
                break;
            }
            if (pid_dirty) { jump_set_pid(p->prep_kp, p->prep_kd); pid_dirty = 0; }
            motor_output_set_all_polar(polar);
            break;

        /* ========== ② 蹬腿: 梯形速度前馈, 爆发推离地面 ========== */
        case JUMP_PHASE_LAUNCH:
            if (t >= p->launch_time) {
                phase = JUMP_PHASE_AIR; tick = xTaskGetTickCount();
                jump_set_polar(polar, p->air_r, p->air_theta);
                pid_dirty = 1;
                break;
            }
            if (pid_dirty) { jump_set_pid(p->launch_kp, p->launch_kd); pid_dirty = 0; }
            motor_output_set_all_polar(polar);
            {
                /* 速度前馈递减: 先猛冲(加速) → 逐渐收力(减速靠近目标)
                   模拟梯形速度曲线, 避免过冲到目标位置 */
                float ratio = 1.0f - (t / p->launch_time);  // 1→0
                float vel_ff = p->launch_vel_ff * ratio * ratio;  // 二次减速
                dm_j8006_send_mit_ff(Aim_Angle,                   /* [8009] 8009 发送接口 */
                                    p->launch_kp, p->launch_kd,
                                    vel_ff, p->launch_torque_ff);
            }
            break;

        /* ========== ③ 缩腿: 快速收腿, 越过台沿 ========== */
        case JUMP_PHASE_AIR:
            if (t >= p->air_time) {
                phase = JUMP_PHASE_STRETCH; tick = xTaskGetTickCount();
                jump_set_polar(polar, p->stretch_r, p->stretch_theta);
                pid_dirty = 1;
                break;
            }
            if (pid_dirty) { jump_set_pid(p->air_kp, p->air_kd); pid_dirty = 0; }
            motor_output_set_all_polar(polar);
            break;

        /* ========== ④ 伸腿: 腿伸出够台面 ========== */
        case JUMP_PHASE_STRETCH:
            if (t >= p->stretch_time) {
                phase = JUMP_PHASE_LAND; tick = xTaskGetTickCount();
                pid_dirty = 1;
                break;
            }
            if (pid_dirty) { jump_set_pid(p->stretch_kp, p->stretch_kd); pid_dirty = 0; }
            motor_output_set_all_polar(polar);
            break;

        /* ========== ⑤ 软着陆: 腿从 stretch 渐变到 land, 弹簧缓震 ========== */
        case JUMP_PHASE_LAND:
            if (t >= p->land_time) { phase = JUMP_PHASE_DONE; break; }
            {
                /* s: 归一化时间 0→1 */
                float s = t / p->land_time;          // 0 → 1
                /* 修正: 弹簧曲线 — 二次缓入
                 * 触地瞬间(s≈0)变化最快, 快速屈膝让位;
                 * 压到底(s≈1)变化最慢, 柔和减速.
                 * 模拟弹簧压缩: 1 - (1-s)^2 */
                float spring = 1.0f - (1.0f - s) * (1.0f - s);
                /* 修正: 从 stretch 姿态向 land 姿态插值
                 * stretch_r=14.0 → land_r=11.0, 腿主动弯曲吸能 */
                float r = p->stretch_r + (p->land_r - p->stretch_r) * spring;
                /* stretch_theta=4.0 → land_theta=5.0, 略前倾配合缓冲 */
                float th = p->stretch_theta + (p->land_theta - p->stretch_theta) * spring;
                jump_set_polar(polar, r, th);        // 每周期重算, 渐变过渡
            }
            if (pid_dirty) { jump_set_pid(p->land_kp, p->land_kd); pid_dirty = 0; }
            motor_output_set_all_polar(polar);
            break;

        default: break;
        }
        vTaskDelay(1);   // 1ms: CAN 不堵, Jump(25)>Posture(24) 姿态不干扰
    }

    /* 平滑过渡到站立姿态 (防落地后突变回弹) */
    {
        /* relax_time: 站立过渡时间(s) | [8009] 0.8
         * 电机更重, 从落地姿态过渡到站立需更长时间, 防"硬拉"起身 */
        const float relax_time = 0.8f;
        /* stand_r: 站立腿长(cm) | [8009] 17.5
         * 标准站立姿态, 四条腿伸直支撑 */
        const float stand_r = 17.5f, 
        /* stand_theta: 站立角度(°) | [8009] 0.0
         * 0=垂直, 腿直上直下, 重心在支撑面中心 */
        stand_theta = 0.0f;
        Polar_Coord_Data_t relax_polar[4];
        TickType_t relax_tick = xTaskGetTickCount();
        float t = 0.0f;

        /* 修正: 过渡阶段PID调更软 | [8009] kp=25, kd=3.0
         * 从落地缓冲姿态恢复站立, 8009 大重量+低电感, 必须更软更阻尼 */
        jump_set_pid(25.0f, 3.0f);

        while (t < 1.0f) {
            t = (float)(xTaskGetTickCount() - relax_tick)
                * portTICK_PERIOD_MS / 1000.0f / relax_time;
            if (t > 1.0f) t = 1.0f;
            /* 修正: 从 land 姿态插值到 stand 姿态
             * 原来从 stretch 开始, 完全跳过 land 缓冲姿态, 导致硬拉 */
            float r = p->land_r + (stand_r - p->land_r) * t;
            float th = p->land_theta + (stand_theta - p->land_theta) * t;
            jump_set_polar(relax_polar, r, th);
            motor_output_set_all_polar(relax_polar);
            vTaskDelay(1);
        }
    }

    /* 恢复默认 PID */
    for (uint8_t i = 0; i < 8; i++) {
        /* 默认位置PID: [8009] kp=50, ki=0.0008, kd=2.5
         * kp=50: 8009 低电感, 站立刚性比8006的60略低, 更稳
         * ki=0.0008: 极小积分, 消除静差, 防长期偏移
         * kd=2.5: 增加阻尼, 抑制8009高频响应 */
        PID_Reset(&PID_Position[i], 50.0f, 0.0008f, 2.5f);
    }
    Change_NowState(STOP);
}

void Jump_Task(void *argument)
{
    while (1)
    {
        static uint8_t once = 0;

        if (once == 0) {
            once++;                         // 上电首次跳过 (防误触发)
        } else {
            Jump_OnlyOnce();                // 执行一次跳跃
        }
        osThreadSuspend(JumpHandle);        // 挂起, 等 Posture_Task 唤醒
    }
}

/* ======================== 兼容接口 (posture_task IKUN_State 调用) ======================== */

void Polar_setCoord(Polar_Coord_Data_t *buf)
{
    motor_output_set_all_polar(buf);
}