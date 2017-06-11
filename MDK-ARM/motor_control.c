#include "main.h"
#include "stm32f7xx_hal.h"
#include "motor_control.h"
int32_t dead = 0;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;
extern float angle;
extern char  angel_control_flag;


void SetTIM3CH4_PWM(uint32_t pwm_value)
{
	htim3.Instance->CCR4 =pwm_value; 
}
void SetTIM3CH3_PWM(uint32_t pwm_value)
{
	htim3.Instance->CCR3 =pwm_value; 
}
void AngleControlEN(void)
{
	    while(1)
    {
        if(angle>900&&angle<1100)break;
    }
    while(angle>890&&angle<1110);
    angel_control_flag=1;   
}

void SetPWM(int32_t value)
{
	if(value >=0)
	{
		value +=dead;
		if(value>=1000)value = 1000;		
		SetTIM3CH3_PWM(value);
		SetTIM3CH4_PWM(0);
	}
	else
	{
		value -=dead;
		if(value<=-1000)value = -1000;	
		SetTIM3CH3_PWM(0);
		SetTIM3CH4_PWM(-value);
	}
}

float InvertCotrol(PIDTypeDef* pidstr,uint32_t value)
{
	return LocPIDCalc(pidstr, value);
}
float MotorControl(PIDTypeDef* pidstr,uint32_t value)
{
	return LocPIDCalc(pidstr, value);
}