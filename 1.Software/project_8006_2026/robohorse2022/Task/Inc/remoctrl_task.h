#ifndef REMOCTRL_TASK_H
#define REMOCTRL_TASK_H

#include "gait_param.h"
#include "serial.h"

//步态参数限制
#define STEPLENTH_MAX                      (14.0f) //步长最大值 (参考宏, 实际限幅见RC_CPLTCTRL_STEPLENTH_MAX)
#define RC_CPLTCTRL_FREQ_MAX                (5.0f) // 通用模式步频上限 (Hz)
#define RC_TROT_FREQ_MAX                   (5.0f) // TROT小跑步频上限 (5.5实测摆动91ms蹭地右偏+机身晃, 回退5.0保步幅13=130cm/s)
#define RC_BRIDGE_FREQ_MAX                 (3.5f) // BRIDGE右杆频率上限 (保持3.5, 过桥求稳)
#define RC_WALK_FREQ_MAX                   (2.5f) // WALK走路步频上限 (Hz) [死宏, 未使用]
#define RC_CPLTCTRL_STEPLENTH_MAX          (13.0f)//遥控器控制步幅最大值 (12→13冲143cm/s, KP80刚度兜底)
#define RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX (5.0f) //遥控器微调步幅最大值
#define RC_CPLTCTRL_FLIGHTPERCENT_MAX      (0.2f) //遥控器控制摆动相占比最大值

//通道参数
#define RC_CH_MID           (1024U)
#define RC_CH_RANGE         (660.0f)
#define RC_CH_MIN           (364U)   // 手册定义最小值
#define RC_CH_MAX           (1684U)  // 手册定义最大值


extern float CH0_Offset;
extern float CH1_Offset;
extern float CH2_Offset;
extern float CH3_Offset;

void RemoCtrl_Task(void *argument);

/* 内部函数声明（供调试或外部调用） */
void Ctrl_byRemoctrl(Cycloid_Generator_t *pCycGenerato, RC_ctrl_t *pRCData);
void Change_RmVAL(Cycloid_Generator_t *pCycGenerato, RC_ctrl_t *pRCData, Robohorse_State state);

#endif /*REMOCTRL_TASK_H*/
