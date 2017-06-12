#ifndef _PID_H_
#define _PID_H_


/*
 ����PID�ṹ��
*/

typedef struct PIDTypeDef
{
    float  SetPoint;     //�趨Ŀ�� Desired Value
    float SumError;                //����ۼ�
    float  Proportion;         //�������� Proportional Cons
    float  Integral;           //���ֳ��� Integral Const
    float  Derivative;         //΢�ֳ��� Derivative Const
    float LastError;               //Error[-1]
    float PrevError;               //Error[-2]
} PIDTypeDef;
extern PIDTypeDef PID_angle;
extern PIDTypeDef PID_angle_site;
extern float IncPIDCalc(PIDTypeDef* pidstr, float CurValue);
extern float LocPIDCalc(PIDTypeDef* pidstr, float CurValue);
#endif
