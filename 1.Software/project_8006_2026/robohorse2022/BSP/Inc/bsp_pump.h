#ifndef BSP_PWM_H
#define BSP_PWM_H

typedef struct __BspTim {
    void (*servo_ctrl)(uint8_t id, uint16_t pwm);
    void (*pump_ctrl)(uint8_t id, uint16_t pwm);
} BspTim;

/*-------------------- Public definition --------------------*/

#define PUMP_ON      10000
#define PUMP_OFF     0

extern BspTim bsp_pump;

extern void PwmCmdPump(uint8_t pump_id, uint16_t pwm);

#endif
