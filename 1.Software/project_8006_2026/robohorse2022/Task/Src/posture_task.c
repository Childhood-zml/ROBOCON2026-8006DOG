/**
 * @file    posture_task.c
 * @brief   步态控制主任务 (100Hz) — 状态机分发 + 摆线轨迹 + 高度过渡
 *
 * 这是整个运动控制的核心循环 (优先级 24, 低于 Motor(26) 和 Jump(25)):
 *
 *   每个周期 (10ms):
 *     ① 装载 state_gait_params[NowState] → Cycloid_Gen[4]
 *     ② Ctrl_byRemoctrl(): 开关切换状态, 摇杆覆写步态参数
 *     ③ Mode_Change_Flag 处理: 相位重置 + 800ms 高度平滑过渡
 *     ④ 状态机分发 → 行走态: Calculate_Gait + Target_toMotor
 *                   → 跳跃态: 唤醒 Jump_Task
 *                   → 其他态: 各自处理
 *
 * 高度过渡 (HEIGHT_TRANSITION_MS=800ms):
 *   切换步态时, body_height 从旧值线性渐变到新值,
 *   期间 step_length=0, freq=0 (不走动)。
 *   通过 Cycloid_Gen[].param.body_height 完成, 和正常步态走同一路径。
 *
 * 摆线轨迹 (Calculate_Gait):
 *   对角步态: 腿0+2 相位0.51, 腿1+3 相位0.01
 *   摆动相: x 线性前移 + y 正弦弧线抬腿
 *   支撑相: x 线性后推 + y 轻微正弦下压
 *
 * 状态切换 (Change_NowState):
 *   remoctrl_task / jump_task 调用 → 设 NowState + Mode_Change_Flag=1
 *   Posture_Task 下一周期检测 → 重置相位 + 启动高度过渡
 */
#include "posture_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "hwt906.h"
#include "serial.h"
#include "ahrs_task.h"
#include "jump_task.h"
#include "gait_param.h"
#include "motor_output.h"

// 常量定义
#define CONTROL_PERIOD_MS       10
#define TRANSITION_DELAY_MS     500
#define HEIGHT_TRANSITION_MS    800     // 步态切换时身体高度过渡时间 (ms)
#define STAND_RETURN_MS         400     // 摇杆回中后腿复位到站立的时间 (ms)
#define AIM_MAX_STEP            0.5f    // aim每tick最大变化量 (cm) —— 兜底防跳变
#define RC_CH_SCALE             1320.0f

#define LEG_RADIUS_BASE         9.0f
#define LEG_RADIUS_VAR          15.0f
#define LEG_RADIUS_MIN          6.0f    // 最小半径限幅 (cm)
#define LEG_RADIUS_MAX          26.0f   // 最大半径限幅 (cm)

#define CLIMBING_X_OFFSET       2.0f
#define STEP_LENGTH_MAX         20.0f
#define STEP_LENGTH_MIN         0.0f

// 全局变量
float aim_x[4] = {0.0f};
float aim_y[4] = {0.0f};
float now_time = 0.0f;
int time_cnt = 0;

extern float revise_LegLenth[4];
extern osThreadId_t JumpHandle;
extern uint8_t flag_Standup;
extern uint8_t flag_imu_line;

extern float CH0_Offset, CH1_Offset, CH2_Offset, CH3_Offset;

/* ==================== 函数声明 ==================== */
static void Calculate_Gait(Robohorse_State state);
static void Target_toMotor(void);
static void IKUN_State(Robohorse_State target, RC_ctrl_t *pRCData);
static inline float Limit_StepLength(float val);
static void Update_PhaseOnModeChange(void);

extern void Ctrl_byRemoctrl(Cycloid_Generator_t *pCycGenerato, RC_ctrl_t *pRCData);

/**
 * @brief 步长限幅，防止负数或过大
 */
static inline float Limit_StepLength(float val)
{
    if (val > STEP_LENGTH_MAX) return STEP_LENGTH_MAX;
    if (val < STEP_LENGTH_MIN) return STEP_LENGTH_MIN;
    return val;
}

/**
 * @brief 模式切换时重置所有相位基准
 * @note  避免状态切换瞬间腿的不连续跳动
 */
static void Update_PhaseOnModeChange(void)
{
    Last_Time = HAL_GetTick() / 1000.0f;
    for (uint8_t i = 0; i < 4; i++) {
        Cycloid_Gen[i].last_time = Last_Time;
    }
}

/**
 * @brief 步态控制主任务（100Hz, 10ms周期）
 */
void Posture_Task(void *argument)
{
    static portTickType xLastWakeTime = 0;
    const portTickType xFrequency = pdMS_TO_TICKS(CONTROL_PERIOD_MS);

    /* 身体高度过渡状态（切换步态时平滑过渡） */
    static uint8_t  height_transition_active = 0;
    static float    height_transition_start  = init_h;
    static float    height_transition_target = init_h;
    static uint32_t height_transition_tick   = 0;

    /* aim变化率限制（兜底防跳变） */
    static float    prev_aim_x[4] = {0};
    static float    prev_aim_y[4] = {0};
    static uint8_t  limiter_ready = 0;

    (void)argument;

    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        /* 状态到独立参数表的映射（-1 表示该状态无独立参数表） */
        static const int8_t detached_idx_map[END + 1] = {
            -1,  /* NORMAL */
             1,  /* CLIMBING */
             2,  /* TURN_LEFT */
             3,  /* TURN_RIGHT */
            -1,  /* WALK */
             0,  /* TROT */
            -1,  /* BRIDGE */
            -1,  /* STOP */
            -1,  /* RELEASE */
            -1,  /* JUMP_GROUND */
            -1,  /* JUMP_BRIDGE */
            -1,  /* JUMP_UPHILL */
            -1,  /* HEIGHT_TEST */
            -1,  /* START */
            -1,  /* END */
        };

        // 从参数表装载当前状态的轨迹参数
        gait_params = state_gait_params[NowState];

        /* [修改1] 转弯状态继承当前运动模式的高度，避免从 WALK/TROT 切过来时突然抬头 */
        if (NowState == TURN_LEFT || NowState == TURN_RIGHT) {
            gait_params.body_height = Cycloid_Gen[0].param.body_height;
        }

        int8_t d_idx = detached_idx_map[NowState];
        if (d_idx >= 0) {
            RcDetachedParam = state_detached_params[d_idx];
        }

        // 模式切换时重置相位（防止轨迹跳变），并启动高度过渡
        if (Mode_Change_Flag) {
            Update_PhaseOnModeChange();
            height_transition_active  = 1;
            height_transition_target  = state_gait_params[NowState].body_height;
            height_transition_start   = Cycloid_Gen[0].param.body_height;
            height_transition_tick    = xTaskGetTickCount();

            /* [修改2] 转弯状态保持原模式高度，不触发高度过渡 */
            if (NowState == TURN_LEFT || NowState == TURN_RIGHT) {
                height_transition_target = height_transition_start;
            }

            Mode_Change_Flag = 0;
        }

        // 统一装载轨迹参数到四个腿的生成器
        for (uint8_t i = 0; i < 4; i++) {
            Cycloid_Gen[i].param = gait_params;
        }

        // 遥控器实时调节步态参数
        Ctrl_byRemoctrl(Cycloid_Gen, &my_rc_ctrl);

        // 身体高度平滑过渡：切换步态时先调整高度再开始走
        if (height_transition_active) {
            uint32_t elapsed = xTaskGetTickCount() - height_transition_tick;
            float t = (float)elapsed / pdMS_TO_TICKS(HEIGHT_TRANSITION_MS);
            if (t >= 1.0f) {
                t = 1.0f;
                height_transition_active = 0;
            }
            float cur_h = height_transition_start
                        + (height_transition_target - height_transition_start) * t;
            for (int i = 0; i < 4; i++) {
                Cycloid_Gen[i].param.body_height = cur_h;
                Cycloid_Gen[i].param.step_length  = 0.0f;  // 过渡期间不走动
                Cycloid_Gen[i].param.freq         = 0.0f;  // 过渡期间不累积相位
                Cycloid_Gen[i].param.up_amp       = 0.0f;  // 过渡期间不上下摆动
            }
        }

        // 状态机分发
        switch (NowState)
        {
        case STOP:
            // 站立：四腿固定在初始高度
            for (uint8_t i = 0; i < 4; i++) {
                aim_x[i] = 0.0f;
                aim_y[i] = init_h;
                Cycloid_Gen[i].param.body_height = init_h;
            }
            Target_toMotor();
            break;

        case NORMAL:
            // 手动调试模式：遥控器摇杆直接控制腿的极坐标位置
            IKUN_State(NORMAL, &my_rc_ctrl);
            break;

        case CLIMBING:
        case WALK:
        case TROT:
        case BRIDGE:
        case TURN_LEFT:
        case TURN_RIGHT:
        {
            /* 静止复位：摇杆回中时平滑回到站立姿态 */
            static uint8_t  stand_active = 0;
            static float    stand_x0[4];
            static float    stand_y0[4];
            static uint32_t stand_tick  = 0;

            uint8_t stationary = 1;
            float   target_y   = state_gait_params[NowState].body_height;
            for (int i = 0; i < 4; i++) {
                if (Cycloid_Gen[i].param.step_length != 0.0f ||
                    Cycloid_Gen[i].param.freq        != 0.0f) {
                    stationary = 0;
                    break;
                }
            }

            /* 始终运行cycloid保持相位（高度过渡时参数已被主循环修改） */
            Calculate_Gait(NowState);

            if (stationary && !height_transition_active) {
                /* 静止：在cycloid输出基础上叠加，平滑拉回站立位 */
                if (!stand_active) {
                    stand_active = 1;
                    stand_tick   = xTaskGetTickCount();
                    for (int i = 0; i < 4; i++) {
                        stand_x0[i] = aim_x[i];
                        stand_y0[i] = aim_y[i];
                    }
                }
                uint32_t elapsed = xTaskGetTickCount() - stand_tick;
                float    t       = (float)elapsed / pdMS_TO_TICKS(STAND_RETURN_MS);
                if (t > 1.0f) t = 1.0f;
                for (int i = 0; i < 4; i++) {
                    aim_x[i] = stand_x0[i] * (1.0f - t);
                    aim_y[i] = stand_y0[i] + (target_y - stand_y0[i]) * t;
                }
            } else {
                /* 运动中（或高度过渡中）：复位静止状态 */
                if (stand_active) {
                    stand_active = 0;
                    /* 刚从静止切回运动，同步cycloid时间防跳变 */
                    now_time = HAL_GetTick() / 1000.0f;
                    for (int i = 0; i < 4; i++) {
                        Cycloid_Gen[i].last_time = now_time;
                    }
                }
            }

            /* aim变化率限制：每tick最多变AIM_MAX_STEP cm */
            if (limiter_ready) {
                for (int i = 0; i < 4; i++) {
                    float dx = aim_x[i] - prev_aim_x[i];
                    float dy = aim_y[i] - prev_aim_y[i];
                    if (dx >  AIM_MAX_STEP) dx =  AIM_MAX_STEP;
                    if (dx < -AIM_MAX_STEP) dx = -AIM_MAX_STEP;
                    if (dy >  AIM_MAX_STEP) dy =  AIM_MAX_STEP;
                    if (dy < -AIM_MAX_STEP) dy = -AIM_MAX_STEP;
                    aim_x[i] = prev_aim_x[i] + dx;
                    aim_y[i] = prev_aim_y[i] + dy;
                }
            }
            for (int i = 0; i < 4; i++) {
                prev_aim_x[i] = aim_x[i];
                prev_aim_y[i] = aim_y[i];
            }
            limiter_ready = 1;
            Target_toMotor();
            break;
        }

        case HEIGHT_TEST:
        {
            /* 身高测试: CH1(右摇杆上下) 控制身体高度,
               通过 Cycloid_Gen 参数渐变, 和高度过渡一样稳 */
            static float cur_h = init_h;
            static Robohorse_State prev = END;

            if (prev != HEIGHT_TEST) {
                /* 继承当前高度, 防趴地 */
                float h = Cycloid_Gen[0].param.body_height;
                cur_h = (h >= 10.0f) ? h : init_h;
                prev = HEIGHT_TEST;
            }

            float ch1 = (float)(my_rc_ctrl.rc.ch[1] - 1024) / 660.0f;
            float target = init_h + ch1 * 8.0f;
            if (target > 26.0f) target = 26.0f;
            if (target < 10.0f) target = 10.0f;
            cur_h += (target - cur_h) * 0.08f;

            for (uint8_t i = 0; i < 4; i++) {
                Cycloid_Gen[i].param.body_height = cur_h;
                Cycloid_Gen[i].param.step_length  = 0.0f;
                Cycloid_Gen[i].param.freq         = 0.0f;
                Cycloid_Gen[i].param.up_amp       = 0.0f;
                Cycloid_Gen[i].param.down_amp     = 0.0f;
            }
            Calculate_Gait(HEIGHT_TEST);
            Target_toMotor();
            break;
        }

        case JUMP_GROUND:
        case JUMP_BRIDGE:
            // 跳跃走独立线程（Jump_Task），这里只负责唤醒
            osThreadResume(JumpHandle);
            break;

        case RELEASE:
        default:
            osDelay(TRANSITION_DELAY_MS);
            break;
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * @brief NORMAL 模式：遥控器摇杆直接控制四条腿的极坐标位置
 */
void IKUN_State(Robohorse_State target, RC_ctrl_t *pRCData)
{
    (void)target;

    static Polar_Coord_Data_t Action_Polar[4];
    uint8_t i;

    const float ch1_offset = pRCData->rc.ch[1] / RC_CH_SCALE;
    const float ch3_offset = pRCData->rc.ch[3] / RC_CH_SCALE;

    CH1_Offset = ch1_offset;
    CH3_Offset = ch3_offset;

    for (i = 0; i < 4; i++) {
        // 前腿(0,1)用ch1，后腿(2,3)用ch3
        const float offset = (i < 2) ? ch1_offset : ch3_offset;

        float raw_radius = LEG_RADIUS_BASE + LEG_RADIUS_VAR * offset;

        if (raw_radius < 0.0f) raw_radius = -raw_radius;
        if (raw_radius > LEG_RADIUS_MAX) raw_radius = LEG_RADIUS_MAX;
        if (raw_radius < LEG_RADIUS_MIN) raw_radius = LEG_RADIUS_MIN;

        Action_Polar[i].radius = raw_radius;
        Action_Polar[i].theta = 0.0f;  // NORMAL模式下腿不前后摆动
    }

    Polar_setCoord(Action_Polar);
}

/**
 * @brief 切换机器人运动状态
 */
void Change_NowState(Robohorse_State target)
{
    if (!flag_Standup) {
        return;
    }

    if (NowState != target) {
        /* 运动状态与转弯状态互切时不触发 Mode_Change_Flag，
         * 避免左摇杆原地转弯时有 800ms 卡顿 */
        uint8_t is_locomotion = (NowState == WALK || NowState == TROT || 
                                 NowState == CLIMBING || NowState == BRIDGE);
        uint8_t is_turn = (NowState == TURN_LEFT || NowState == TURN_RIGHT);
        uint8_t target_is_locomotion = (target == WALK || target == TROT || 
                                        target == CLIMBING || target == BRIDGE);
        uint8_t target_is_turn = (target == TURN_LEFT || target == TURN_RIGHT);

        NowState = target;

        if (!((is_locomotion && target_is_turn) || (is_turn && target_is_locomotion))) {
            Mode_Change_Flag = 1;
        }
    }
}

// 摆线计算结果暂存区
static Cycloid_Generator_Solution_t cyc_solution[4];

/**
 * @brief IMU自适应步长调整（前后腿反向补偿维持平衡）
 */
static inline void Apply_IMU_Correction(uint8_t leg_idx)
{
    if (flag_imu_line != 1) {
        return;
    }

    const float correction = (leg_idx < 2) ? (-step_len_dev) : step_len_dev;
    Cycloid_Gen[leg_idx].param.step_length += correction;
    Cycloid_Gen[leg_idx].param.step_length =
        Limit_StepLength(Cycloid_Gen[leg_idx].param.step_length);
}

/**
 * @brief 步态计算：用摆线生成器为四条腿生成足端轨迹
 */
static void Calculate_Gait(Robohorse_State state)
{
    now_time = HAL_GetTick() / 1000.0f;
    uint8_t i;

    switch (state)
    {
    case WALK:
        for (i = 0; i < 4; i++) {
            Apply_IMU_Correction(i);
        }
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[0], 0.51f, &cyc_solution[0]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[1], 0.01f, &cyc_solution[1]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[2], 0.51f, &cyc_solution[2]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[3], 0.01f, &cyc_solution[3]);
        break;

    case TROT:
        for (i = 0; i < 4; i++) {
            Apply_IMU_Correction(i);
        }
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[0], 0.51f, &cyc_solution[0]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[1], 0.01f, &cyc_solution[1]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[2], 0.51f, &cyc_solution[2]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[3], 0.01f, &cyc_solution[3]);
        break;

    case TURN_LEFT:
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[0], 0.51f, &cyc_solution[0]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[1], 0.76f, &cyc_solution[1]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[2], 0.26f, &cyc_solution[2]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[3], 0.01f, &cyc_solution[3]);
        break;

    case TURN_RIGHT:
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[0], 0.01f, &cyc_solution[0]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[1], 0.26f, &cyc_solution[1]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[2], 0.76f, &cyc_solution[2]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[3], 0.51f, &cyc_solution[3]);
        break;

    case CLIMBING:
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[0], 0.51f, &cyc_solution[0]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[1], 0.01f, &cyc_solution[1]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[2], 0.51f, &cyc_solution[2]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[3], 0.01f, &cyc_solution[3]);
        break;

    case BRIDGE:
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[0], 0.51f, &cyc_solution[0]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[1], 0.01f, &cyc_solution[1]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[2], 0.51f, &cyc_solution[2]);
        Cycloid_Generator_Calc(now_time, &Cycloid_Gen[3], 0.01f, &cyc_solution[3]);
        break;

    case HEIGHT_TEST:
        /* 四条腿原地站立, y=身体高度, x=0 */
        for (i = 0; i < 4; i++) {
            cyc_solution[i].x = 0.0f;
            cyc_solution[i].y = Cycloid_Gen[i].param.body_height;
        }
        break;

    default:
        break;
    }

    // 爬坡模式额外加x偏移（适应斜面）
    const float x_offset = (state == CLIMBING) ? CLIMBING_X_OFFSET : 0.0f;
    for (i = 0; i < 4; i++) {
        aim_x[i] = cyc_solution[i].x - x_offset;
        aim_y[i] = cyc_solution[i].y;
    }
}

/**
 * @brief 将足端坐标 (aim_x[], aim_y[]) 通过统一电机输出层转换成电机角度
 * @note  委托给 motor_output 统一处理 IK + 电机映射 + 符号约定
 */
static void Target_toMotor(void)
{
    motor_output_set_all_cartesian(aim_x, aim_y);
}
