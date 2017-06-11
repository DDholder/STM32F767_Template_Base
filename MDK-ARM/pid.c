#include "main.h"
#include "stm32f7xx_hal.h"
#include "pid.h"

/*******************************************************************************
* 函数名称 : IncPIDCalc
* 函数描述 : 增量式 PID 控制计算
* 函数输入 : int 当前位置
* 函数输出 : 无
* 函数返回 : 增量式PID结果
*******************************************************************************/
float IncPIDCalc(PIDTypeDef* pidstr, float CurValue)
{
    float iError, iIncpid;
    //当前误差
    iError = pidstr->SetPoint - CurValue;
    //增量计算
    iIncpid = pidstr->Proportion * iError               //E[k]项
              - pidstr->Integral   * pidstr->LastError     //E[k－1]项
              + pidstr->Derivative * pidstr->PrevError;   //E[k－2]项
    //存储误差，用于下次计算
    pidstr->PrevError = pidstr->LastError;
    pidstr->LastError = iError;
    //返回增量值
    return(iIncpid);
}
/*******************************************************************************
* 函数名称 : LocPIDCalc
* 函数描述 : 位置式 PID 控制计算
* 函数输入 : int 当前位置
* 函数输出 : 无
* 函数返回 : 位置式PID结果
*******************************************************************************/
float LocPIDCalc(PIDTypeDef* pidstr, float CurValue)
{
    float  iError,dError;
    iError = pidstr->SetPoint - CurValue;       //偏差
    pidstr->SumError += iError;       //积分
    dError = iError - pidstr->LastError;     //微分
    pidstr->LastError = iError;
    return(pidstr->Proportion * iError            //比例项
           + pidstr->Integral * pidstr->SumError   //积分项
           + pidstr->Derivative * dError);        //微分项
}