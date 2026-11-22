#ifndef DM_J8006_H
#define DM_J8006_H

#include <stdint.h>

typedef struct
{
    int16_t Rotor_Angle;  //转子角度 0~8192 0~360°
    int16_t Speed_RPM;    //每分钟电机转速(RPM)
    int16_t Real_Current; //实际电机电流值 -16384~+16384
    uint16_t Temp;        //电机温度
    int16_t Last_Angle;   //上次读取到的角度
    int16_t Offset_Angle; //初始位置偏移角度
    int32_t Round_Cnt;    //转子旋转圈数
    int32_t Total_Angle;  //总角度
    uint32_t Msg_Cnt;     //消息计数
} J8006_Measure_t;   //电机测量数据结构体

// 外部声明8个电机的测量数据
extern J8006_Measure_t J8006[8];

void DM_J8006_Init(void);

#endif
