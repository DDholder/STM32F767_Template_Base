#ifndef _PID_H_
#define _PID_H_


/*
 定义PID结构体
*/

typedef struct PIDTypeDef
{
    float  SetPoint;     //设定目标 Desired Value
    float SumError;                //误差累计
    float  Proportion;         //比例常数 Proportional Cons
    float  Integral;           //积分常数 Integral Const
    float  Derivative;         //微分常数 Derivative Const
    float LastError;               //Error[-1]
    float PrevError;               //Error[-2]
} PIDTypeDef;
extern PIDTypeDef PID_angle;
extern PIDTypeDef PID_angle_site;
extern float IncPIDCalc(PIDTypeDef* pidstr, float CurValue);
extern float LocPIDCalc(PIDTypeDef* pidstr, float CurValue);
#endif
