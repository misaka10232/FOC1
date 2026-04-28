#ifndef __BLDC_H
#define __BLDC_H

#include "tim.h"
#include "gpio.h"
#include "usart.h"
void set_phase_voltage(float Uq, float Ud, float Angle);
void BLDC_Init(void);
float velocity_open_loop(float target_velocity);
float _electricalangle(void);
void Set_TarAngle(float Target_Angle);
float angle_nomalize(float angle);




#endif /* __BLDC_H */
