#ifndef _MOTOR_H_
#define _MOTOR_H_
#include "main.h"
#include "stm32f7xx_hal.h"
#include "motor_control.h"
extern void SetPWM(int32_t value);
extern void AngleControlEN(void);
extern float InvertCotrol(PIDTypeDef* pidstr,uint32_t value);
extern float MotorControl(PIDTypeDef* pidstr,uint32_t value);
#endif