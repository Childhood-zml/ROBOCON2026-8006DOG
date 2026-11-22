/**
 * @file    remoctrl_task.c
 * @brief   遥控器任务 — SBUS 解析 + 开关/摇杆 → 步态参数
 */

#include "remoctrl_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "pid.h"
#include "app_btdebug.h"
#include "serial.h"
#include "posture_task.h"
#include "hwt906.h"

#define TX_DATAF32_SIZE         (8U)
#define TX_DATA8_SIZE           (TX_DATAF32_SIZE * 4U)
#define RC_CH_HIGH_THRESH       (1600U)
#define RC_CH_LOW_THRESH        (400U)
#define RC_CH_DEADZONE          (10U)
#define SWITCH_DEBOUNCE_MS      (50U)
#define TURN_THRESHOLD          (0.01f)
#define BRIDGE_CH0_THRESHOLD    (0.05f)
#define AMP_BASE_UP             (2.8f)
#define AMP_BASE_DOWN           (1.5f)
#define AMP_TURN_UP_GAIN        (1.2f)
#define AMP_TURN_DOWN_GAIN      (0.7f)
#define AMP_WALK_UP_BASE        (3.0f)  // 恢复原版
#define AMP_WALK_UP_GAIN        (1.2f)  // 1.0→1.2 配合4Hz提速, 满杆抬腿4.2防蹭地
#define AMP_WALK_DOWN_BASE      (0.8f)  // 恢复原版
#define AMP_WALK_DOWN_GAIN      (0.3f)  // 恢复原版
#define AMP_WALK_TURN_UP_BASE   (2.8f)  // 恢复原版值 (=AMP_BASE_UP)
#define AMP_WALK_TURN_UP_GAIN   (1.2f)  // 恢复原版值 (=AMP_TURN_UP_GAIN)
#define AMP_WALK_TURN_DOWN_BASE (1.5f)  // 恢复原版值 (=AMP_BASE_DOWN)
#define AMP_WALK_TURN_DOWN_GAIN (0.7f)  // 恢复原版值 (=AMP_TURN_DOWN_GAIN)

typedef union {
    uint8_t  data8[TX_DATA8_SIZE];
    float    dataf32[TX_DATAF32_SIZE];
} BtTxData_t;

extern float Target_Angle;
extern AHRS_Angle_t Now_Angle;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern uint8_t rc_data[18];
extern RC_ctrl_t my_rc_ctrl;
extern RC_Key rc_sw_state;

float CH0_Offset = 0.0f;
float CH1_Offset = 0.0f;
float CH2_Offset = 0.0f;
float CH3_Offset = 0.0f;
uint32_t get_num = 0;
uint8_t flag_imu_line = 0;

static BtTxData_t bt_tx_data;
static BTDebug_Operate_t btdbg_opt;

/* ===================== 前置声明 (static 辅助函数) ===================== */
static void UART_Transmit_Bytes(uint8_t *pData, uint32_t Size);
static void UART_Receive_Bytes(uint8_t *pData, uint32_t Size);
static inline float NormalizeChannel(uint16_t ch_value);
static inline uint8_t IsChannelNeutral(uint16_t ch_value);
static inline float LimitFloat(float value, float min, float max);
static uint8_t IsRcNeutral(const RC_ctrl_t *pRCData);
static void UpdateChOffset(const RC_ctrl_t *pRCData);
static void SetAllLegStepLength(Cycloid_Generator_t *pCycGenerato, float step_length);
static void SetAllLegFreq(Cycloid_Generator_t *pCycGenerato, float freq_norm, float freq_max);
static void SetTurnStepLength(Cycloid_Generator_t *pCycGenerato, float ch3_norm, float ch2_norm, float max_length);
static void UpdateTurnAmplitude(Cycloid_Generator_t *pCycGenerato, Robohorse_State state, float turn_intensity);
static void UpdateWalkAmplitude(Cycloid_Generator_t *pCycGenerato, float speed_intensity);
static void HandleTurningOrStraight(Cycloid_Generator_t *pCycGenerato, Robohorse_State state, float ch1_norm, float ch2_norm, float ch3_norm);
static void HandleNormalLocomotion(Cycloid_Generator_t *pCycGenerato, Robohorse_State state, float ch1_norm, float ch2_norm, float ch3_norm);
static void HandleLocomotionMode(Cycloid_Generator_t *pCycGenerato, const RC_ctrl_t *pRCData, Robohorse_State state, float ch0_norm, float ch1_norm, float ch2_norm, float ch3_norm);
static void HandleBridgeMode(Cycloid_Generator_t *pCycGenerato, float ch0_norm, float ch3_norm);
static void UpdateClimbingHeight(Cycloid_Generator_t *pCycGenerato);
static void UpdateBridgeHeight(Cycloid_Generator_t *pCycGenerato, uint8_t idx);

/* ===================== UART操作函数 ===================== */
static void UART_Transmit_Bytes(uint8_t *pData, uint32_t Size)
{
    HAL_UART_Transmit(&huart6, pData, Size, 100);
}

static void UART_Receive_Bytes(uint8_t *pData, uint32_t Size)
{
    HAL_UART_Receive(&huart6, pData, Size, 100);
}

/* ===================== 内联辅助函数 ===================== */
static inline float NormalizeChannel(uint16_t ch_value)
{
    return ((float)ch_value - RC_CH_MID) / RC_CH_RANGE;
}

static inline uint8_t IsChannelNeutral(uint16_t ch_value)
{
    return (ch_value <= (RC_CH_MID + RC_CH_DEADZONE)) &&
           (ch_value >= (RC_CH_MID - RC_CH_DEADZONE));
}

static inline float LimitFloat(float value, float min, float max)
{
    return (value < min) ? min : ((value > max) ? max : value);
}

/* ===================== 主任务 ===================== */
void RemoCtrl_Task(void *argument)
{
    btdbg_opt.ReceiveBytes = UART_Receive_Bytes;
    btdbg_opt.TransmitBytes = UART_Transmit_Bytes;

    (void)argument;

    for (;;)
    {
        /* SBUS 解析 */
        taskENTER_CRITICAL();
        RC_Transform(rc_data, &my_rc_ctrl);
        taskEXIT_CRITICAL();

        /* 蓝牙调试数据发送（USART6）*/
        BTDebug_TransmitPackage(&btdbg_opt, bt_tx_data.data8, TX_DATA8_SIZE);

        osDelay(10);
    }
}

/* ===================== 遥控器数据处理 ===================== */
void Ctrl_byRemoctrl(Cycloid_Generator_t *pCycGenerato, RC_ctrl_t *pRCData)
{
    static RC_Key now_sw_state = RC_NULL;
    static RC_Key last_sw_state;
    static uint32_t last_sw_time = 0;

    if (pCycGenerato == NULL || pRCData == NULL)
    {
        return;
    }

    /* 急停处理 - 最高优先级 */
    if (rc_sw_state == RC_22)
    {
        if (now_sw_state != RC_22)
        {
            now_sw_state = RC_22;
            Change_NowState(STOP);
            dm_j8006_all_disable();
            flag_imu_line = 0;
        }
        return;
    }

    dm_j8006_all_enable();

    /* 开关状态消抖处理 */
    if (now_sw_state != rc_sw_state)
    {
        last_sw_state = rc_sw_state;
        uint32_t current_time = osKernelGetTickCount();

        if ((current_time - last_sw_time) >= SWITCH_DEBOUNCE_MS)
        {
            if (rc_sw_state == last_sw_state)
            {
                now_sw_state = rc_sw_state;
                last_sw_time = current_time;
            }
        }
    }

    /* 根据开关状态先切换模式（姿态立即变化） */
    switch (now_sw_state)
    {
        case RC_33: Change_NowState(WALK);   break;
        case RC_23: Change_NowState(TROT);   break;
        case RC_11:
        {
            uint8_t ch1_high = (pRCData->rc.ch[1] > RC_CH_HIGH_THRESH);
            uint8_t ch3_high = (pRCData->rc.ch[3] > RC_CH_HIGH_THRESH);
            uint8_t ch1_low  = (pRCData->rc.ch[1] < RC_CH_LOW_THRESH);
            uint8_t ch3_low  = (pRCData->rc.ch[3] < RC_CH_LOW_THRESH);
            if (ch1_high && ch3_high)
                Change_NowState(JUMP_GROUND);
            else if (ch1_low && ch3_low)
                Change_NowState(JUMP_BRIDGE);
            else
                for (int i = 0; i < 4; i++) {
                    pCycGenerato[i].param.step_length = 0.0f;
                    pCycGenerato[i].param.freq = 0.0f;
                }
            break;
        }
        case RC_21: Change_NowState(CLIMBING); break;
        case RC_13: Change_NowState(HEIGHT_TEST); break;
        case RC_31: Change_NowState(BRIDGE);    break;
        case RC_32:
            for (int i = 0; i < 4; i++) {
                pCycGenerato[i].param.step_length = 0.0f;
                pCycGenerato[i].param.freq = 0.0f;
            }
            break;
    }

    /* 摇杆回中：停止运动，但保持当前模式 */
    if (IsRcNeutral(pRCData))
    {
        osDelay(5);
        if (IsRcNeutral(pRCData))
        {
            for (int i = 0; i < 4; i++) {
                pCycGenerato[i].param.step_length = 0.0f;
                pCycGenerato[i].param.freq = 0.0f;
            }
            return;
        }
    }

    /* 摇杆激活：更新运动参数 */
    taskENTER_CRITICAL();
    UpdateChOffset(pRCData);

    switch (now_sw_state)
    {
        case RC_33: Change_RmVAL(pCycGenerato, pRCData, WALK);     break;
        case RC_23: Change_RmVAL(pCycGenerato, pRCData, TROT);     break;
        case RC_21: Change_RmVAL(pCycGenerato, pRCData, CLIMBING); break;
        case RC_31: Change_RmVAL(pCycGenerato, pRCData, BRIDGE);   break;
        default: break;
    }
    taskEXIT_CRITICAL();
}

/* ===================== 步态参数更新 ===================== */
void Change_RmVAL(Cycloid_Generator_t *pCycGenerato, RC_ctrl_t *pRCData, Robohorse_State state)
{
    if (pCycGenerato == NULL || pRCData == NULL)
    {
        return;
    }

    switch (state)
    {
        case TROT:
        case WALK:
        case CLIMBING:
        case BRIDGE:
            HandleLocomotionMode(pCycGenerato, pRCData, state,
                                CH0_Offset, CH1_Offset, CH2_Offset, CH3_Offset);
            break;
        default:
            Change_NowState(state);
            break;
    }
}

/* ===================== 运动模式处理 ===================== */
static void HandleLocomotionMode(Cycloid_Generator_t *pCycGenerato, 
                                 const RC_ctrl_t *pRCData, 
                                 Robohorse_State state, 
                                 float ch0_norm, float ch1_norm, 
                                 float ch2_norm, float ch3_norm)
{
    const uint8_t ch4_lock = (pRCData->rc.ch[4] >= 1200);
    float ch2_eff = ch4_lock ? 0.0f : ch2_norm;

    /* 右摇杆是否近似回中（宽死区，避免微偏导致无法原地转弯） */
    uint8_t right_stick_centered = 
        (pRCData->rc.ch[0] <= (RC_CH_MID + 50) && pRCData->rc.ch[0] >= (RC_CH_MID - 50)) &&
        (pRCData->rc.ch[1] <= (RC_CH_MID + 50) && pRCData->rc.ch[1] >= (RC_CH_MID - 50));

    /* 拨了右摇杆: WALK直走/转弯都用右摇杆数值(频率右杆比例, 状态表幅度);
       右摇杆回中: 用左摇杆调好的固定数值;
       其他模式保持左摇杆转向优先 */
    if (right_stick_centered || (state != WALK && fabsf(ch2_eff) > TURN_THRESHOLD))
    {
        HandleTurningOrStraight(pCycGenerato, state, ch1_norm, ch2_eff, ch3_norm);
    }
    else
    {
        HandleNormalLocomotion(pCycGenerato, state, ch1_norm, ch2_eff, ch3_norm);
    }
}

static void HandleTurningOrStraight(Cycloid_Generator_t *pCycGenerato, 
                                    Robohorse_State state, 
                                    float ch1_norm, float ch2_norm, float ch3_norm)
{
    const float turn_intensity = fabsf(ch2_norm);
    const float abs_ch3 = fabsf(ch3_norm);

    /* ===== 各模式原地转弯参数（只拨左摇杆左右时的合适数值）===== */
    float turn_freq;
    float turn_step_max;
    switch (state) {
        case WALK:     turn_freq = 2.0f;  turn_step_max = 8.0f;  break;
        case TROT:     turn_freq = 2.5f;  turn_step_max = 10.0f; break;
        case BRIDGE:   turn_freq = 2.0f;  turn_step_max = 7.0f;  break;
        case CLIMBING: turn_freq = 1.5f;  turn_step_max = 6.0f;  break;
        default:       turn_freq = 2.0f;  turn_step_max = 8.0f;  break;
    }

    /* WALK/TROT转弯频率与直走一致: 跟随前进杆量 (同各自直走公式, TROT用RC_TROT_FREQ_MAX);
       原地转弯(ch1≈0)时保底turn_freq; 其他模式保持原turn_freq不变 */
    float walk_turn_freq = turn_freq;
    if (state == WALK || state == TROT) {
        const float straight_freq = ch1_norm *
            ((state == TROT) ? RC_TROT_FREQ_MAX : RC_CPLTCTRL_FREQ_MAX);
        if (straight_freq > turn_freq) {
            walk_turn_freq = straight_freq;
        }
    }

    if (ch2_norm > TURN_THRESHOLD)
    {
        Change_NowState(TURN_RIGHT);
        float max_len = (abs_ch3 > TURN_THRESHOLD) ? RC_CPLTCTRL_STEPLENTH_MAX : turn_step_max;
        SetTurnStepLength(pCycGenerato, ch3_norm, ch2_norm, max_len);
        for (int i = 0; i < 4; i++) {
            pCycGenerato[i].param.freq = walk_turn_freq;
        }
    }
    else if (ch2_norm < -TURN_THRESHOLD)
    {
        Change_NowState(TURN_LEFT);
        float max_len = (abs_ch3 > TURN_THRESHOLD) ? RC_CPLTCTRL_STEPLENTH_MAX : turn_step_max;
        SetTurnStepLength(pCycGenerato, ch3_norm, ch2_norm, max_len);
        for (int i = 0; i < 4; i++) {
            pCycGenerato[i].param.freq = walk_turn_freq;
        }
    }
    else
    {
        Change_NowState(state);
        SetAllLegStepLength(pCycGenerato, RC_CPLTCTRL_STEPLENTH_MAX * ch3_norm);
        if (state == WALK && abs_ch3 > TURN_THRESHOLD)
        {
            UpdateWalkAmplitude(pCycGenerato, abs_ch3);
        }
    }

    if (turn_intensity > TURN_THRESHOLD)
    {
        UpdateTurnAmplitude(pCycGenerato, state, turn_intensity);
    }
}

static void HandleNormalLocomotion(Cycloid_Generator_t *pCycGenerato, 
                                   Robohorse_State state, 
                                   float ch1_norm, float ch2_norm, float ch3_norm)
{
    if (state == CLIMBING)
    {
        UpdateClimbingHeight(pCycGenerato);
    }

    Change_NowState(state);
    SetTurnStepLength(pCycGenerato, ch3_norm, ch2_norm, RC_CPLTCTRL_STEPLENTH_MAX);

    float freq_max = (state == TROT)   ? RC_TROT_FREQ_MAX
                   : (state == BRIDGE) ? RC_BRIDGE_FREQ_MAX
                                       : RC_CPLTCTRL_FREQ_MAX;
    SetAllLegFreq(pCycGenerato, ch1_norm, freq_max);
}

static void HandleBridgeMode(Cycloid_Generator_t *pCycGenerato, 
                             float ch0_norm, float ch3_norm)
{
    uint8_t idx;
    if (ch0_norm > BRIDGE_CH0_THRESHOLD)
    {
        idx = 0;
    }
    else if (ch0_norm < -BRIDGE_CH0_THRESHOLD)
    {
        idx = 1;
    }
    else
    {
        return;
    }

    UpdateBridgeHeight(pCycGenerato, idx);
    SetAllLegStepLength(pCycGenerato, RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * ch3_norm);
    SetAllLegFreq(pCycGenerato, ch0_norm, RC_TROT_FREQ_MAX);
}

/* ===================== 辅助函数 ===================== */
static void UpdateChOffset(const RC_ctrl_t *pRCData)
{
    CH0_Offset = NormalizeChannel(pRCData->rc.ch[0]);

    float ch1_norm = NormalizeChannel(pRCData->rc.ch[1]);
    CH1_Offset = (ch1_norm > 0.0f) ? ch1_norm : 0.0f;

    CH2_Offset = NormalizeChannel(pRCData->rc.ch[2]);
    CH3_Offset = NormalizeChannel(pRCData->rc.ch[3]);
}

static uint8_t IsRcNeutral(const RC_ctrl_t *pRCData)
{
    for (int i = 0; i < 4; i++)
    {
        if (!IsChannelNeutral(pRCData->rc.ch[i]))
        {
            return 0;
        }
    }
    return 1;
}

static void SetAllLegStepLength(Cycloid_Generator_t *pCycGenerato, float step_length)
{
    float limited = LimitFloat(step_length, -RC_CPLTCTRL_STEPLENTH_MAX, RC_CPLTCTRL_STEPLENTH_MAX);
    for (int i = 0; i < 4; i++)
    {
        pCycGenerato[i].param.step_length = limited;
    }
}

static void SetAllLegFreq(Cycloid_Generator_t *pCycGenerato, float freq_norm, float freq_max)
{
    float freq = LimitFloat(freq_max * freq_norm, -freq_max, freq_max);
    for (int i = 0; i < 4; i++)
    {
        pCycGenerato[i].param.freq = freq;
    }
}

static void SetTurnStepLength(Cycloid_Generator_t *pCycGenerato, 
                              float ch3_norm, float ch2_norm, float max_length)
{
    const float left_len = LimitFloat(max_length * (ch3_norm + ch2_norm), 
                                      -max_length, max_length);
    const float right_len = LimitFloat(max_length * (ch3_norm - ch2_norm), 
                                       -max_length, max_length);

    pCycGenerato[0].param.step_length = left_len;
    pCycGenerato[1].param.step_length = left_len;
    pCycGenerato[2].param.step_length = right_len;
    pCycGenerato[3].param.step_length = right_len;
}

static void UpdateTurnAmplitude(Cycloid_Generator_t *pCycGenerato, Robohorse_State state, float turn_intensity)
{
    float up_amp;
    float down_amp;
    if (state == WALK)
    {
        up_amp   = AMP_WALK_TURN_UP_BASE + turn_intensity * AMP_WALK_TURN_UP_GAIN;
        down_amp = AMP_WALK_TURN_DOWN_BASE - turn_intensity * AMP_WALK_TURN_DOWN_GAIN;
    }
    else
    {
        up_amp   = AMP_BASE_UP + turn_intensity * AMP_TURN_UP_GAIN;
        down_amp = AMP_BASE_DOWN - turn_intensity * AMP_TURN_DOWN_GAIN;
    }

    for (int i = 0; i < 4; i++)
    {
        pCycGenerato[i].param.up_amp = up_amp;
        pCycGenerato[i].param.down_amp = down_amp;
    }
}

static void UpdateWalkAmplitude(Cycloid_Generator_t *pCycGenerato, float speed_intensity)
{
    const float up_amp = AMP_WALK_UP_BASE + speed_intensity * AMP_WALK_UP_GAIN;
    const float down_amp = AMP_WALK_DOWN_BASE - speed_intensity * AMP_WALK_DOWN_GAIN;

    for (int i = 0; i < 4; i++)
    {
        pCycGenerato[i].param.up_amp = up_amp;
        pCycGenerato[i].param.down_amp = down_amp;
    }
}

static void UpdateClimbingHeight(Cycloid_Generator_t *pCycGenerato)
{
    pCycGenerato[0].param.body_height = FORWARD_hight;
    pCycGenerato[1].param.body_height = BACK_hight;
    pCycGenerato[2].param.body_height = FORWARD_hight;
    pCycGenerato[3].param.body_height = BACK_hight;
}

static void UpdateBridgeHeight(Cycloid_Generator_t *pCycGenerato, uint8_t idx)
{
    if (idx > 1) 
    {
        return;
    }

    static const float body_h[2][4] = {
        {longK_hight, longK_hight, shortK_hight, shortK_hight},
        {shortK_hight, shortK_hight, longK_hight, longK_hight}
    };

    for (int i = 0; i < 4; i++)
    {
        pCycGenerato[i].param.body_height = body_h[idx][i];
    }
}
