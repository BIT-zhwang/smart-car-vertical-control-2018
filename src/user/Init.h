#ifndef __INIT_H__
#define __INIT_H__

#include "include.h"
#include "8700_2100.h"
#include "Parameter.h"

void Init_Hardwares(void); //全硬件初始化

void Sample_Static_87002100(void);        //加陀板静态采集
void Get_AccANDGyro_Data(void);//获取加陀板数据
short Get_Gyro_X(void);//获取陀螺仪X轴角速度
short Get_Gyro_Y(void);//获取陀螺仪Y轴角速度
short Get_Gyro_Z(void);//获取陀螺仪Z轴角速度
uint16_t Get_Acc_X(void);//获取加速度计X轴角度
uint16_t Get_Acc_Y(void);//获取加速度计Y轴角度
uint16_t Get_Acc_Z(void);//获取加速度计Z轴角度

void PWM_Iint(void);//PWM通道初始化
void ECLK_Capture_Init(void);//编码器通道初始化

void Inductance_ADC_Init(void);//电感AD值采集初始化

void UART_Init(void);//串口通信初始化

void GHG_Init(void);//停车干簧管初始化
void Boma_Init(void);//拨码开关初始化
void Key_Init(void);//  按键控制初始化
void Buzzer_Init(void);//蜂鸣器初始化b
void Read_Boma(void);//读取拨码开关状态

void ShutDown_DiDi(void);//关闭蜂鸣器
void DiDi(void);//蜂鸣器响

void Read_GHG(void);//停车干簧管

#endif