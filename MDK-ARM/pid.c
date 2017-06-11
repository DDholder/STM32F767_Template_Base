#include "main.h"
#include "stm32f7xx_hal.h"
#include "pid.h"

/*******************************************************************************
* �������� : IncPIDCalc
* �������� : ����ʽ PID ���Ƽ���
* �������� : int ��ǰλ��
* ������� : ��
* �������� : ����ʽPID���
*******************************************************************************/
float IncPIDCalc(PIDTypeDef* pidstr, float CurValue)
{
    float iError, iIncpid;
    //��ǰ���
    iError = pidstr->SetPoint - CurValue;
    //��������
    iIncpid = pidstr->Proportion * iError               //E[k]��
              - pidstr->Integral   * pidstr->LastError     //E[k��1]��
              + pidstr->Derivative * pidstr->PrevError;   //E[k��2]��
    //�洢�������´μ���
    pidstr->PrevError = pidstr->LastError;
    pidstr->LastError = iError;
    //��������ֵ
    return(iIncpid);
}
/*******************************************************************************
* �������� : LocPIDCalc
* �������� : λ��ʽ PID ���Ƽ���
* �������� : int ��ǰλ��
* ������� : ��
* �������� : λ��ʽPID���
*******************************************************************************/
float LocPIDCalc(PIDTypeDef* pidstr, float CurValue)
{
    float  iError,dError;
    iError = pidstr->SetPoint - CurValue;       //ƫ��
    pidstr->SumError += iError;       //����
    dError = iError - pidstr->LastError;     //΢��
    pidstr->LastError = iError;
    return(pidstr->Proportion * iError            //������
           + pidstr->Integral * pidstr->SumError   //������
           + pidstr->Derivative * dError);        //΢����
}