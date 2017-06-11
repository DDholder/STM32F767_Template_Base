#include "main.h"
#include "stm32f7xx_hal.h"
#include "motor_control.h"
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;
extern float angle;
extern char  angel_control_flag;
void SetTIM3CH4_PWM(uint32_t pwm_value)
{
	htim3.Instance->CCR4 =pwm_value; 
}

void angle_control_en(void)
{
	    while(1)
    {
        if(angle>900&&angle<1100)break;
    }
    while(angle>890&&angle<1110);
    angel_control_flag=1;   
}

