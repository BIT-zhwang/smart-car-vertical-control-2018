#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "include.h"
#include "Parameter.h"

/***            电感采集控制部分                ***/
void Inductanc_Inertia_Filter(void);   //进行惯性滤波
void Inductance_Ave_Filter(int N_Times); //对电感值进行均值滤波 变量为样本容量
void Get_Inductor_Data(void);    //采集电感值并进行处理
void CurrentDetect(void);    //赛道电流检测
void Normalization(void);//电感归一化

/***            done            ***/

/***            直立控制部分          ***/
void Get_Angle_Data(void);       //获取车身角度、角速度信息
void Angle_Output(float Current_Angle);  //角度环计算输出
/***            done            ***/


/***            速度计算部分          ***/

extern float Circle_Ratio;
//圆周参数  从脉冲单位换算成实际距离单位 1个脉冲对应xx米
//【CircleRatio】 = 【(PI*65/1000)/x】; 
//理论计算:周长/1000(米换算) * PI=周长 / 一圈脉冲量程-->x线


void Get_ArcLength(Disdance_ArcLength *dis, float Unit, float circle_ratio);     //将编码器脉冲数转换成实际距离
void Get_Speed_Data(void);  //获取车辆速度信息
void Judge_Left_Wheel_Dir(void);        //判断左车轮方向
void Judge_Right_Wheel_Dir(void);       //判断右车轮方向

void Car_Speed_Control_Output(void);     //车身速度计算输出
/***            done            ***/

/***            读取并处理传感器数据      ***/
void Get_Sensors_Data(void);    
/***            done             ***/

/***            方向计算部分          ***/
void Car_Direction_Control_Output(void); //方向环计算输出
void Car_Direction_Control_Output_Plus(void); //加速转向补偿
void Circle_Test(void);//圆环检测程序
void Car_Protect(void);//保护程序
void Straight_Judge(void);
/***            done            ***/

/***            电机输出部分          ***/
void Set_Motor_Voltage(float f_Left_Voltage, float f_Right_Voltage); 	//设置电机旋转方向和速度
void Set_PWM(int PWM_n,int value);       //设置PWM输出
void Motor_Output(void);        //计算电机输出量
/***            done            ***/



void Auto_Normalization(void);
void Auto_Inductance_Get(int Left_Voltage,int Right_Voltage);



void Set_Fussy_PTable(void);
void Set_Fussy_DTable(void);
void Calc_Membership_Without_CenterZone_7Gear(float value, float* membership, float* dividePoints,int NUM);

void Go(void);

#endif