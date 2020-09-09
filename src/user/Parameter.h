#ifndef __Parameter_H__
#define __Parameter_H__

#include "include.h"


/**********加陀板参数定义*****/
//加速度传感器原始数据
extern short int accle_x;		
extern short int accle_y;
extern short int accle_z;
//陀螺仪原始数据
extern short int gyro_x;
extern short int gyro_y;
extern short int gyro_z;
//IIC读取数据
extern uint8_t IIC_bit8_data[6];
extern short  IIC_bit16_data16[3];
/***      done      **/

/***            上位机发送数据              ***/
extern float Send_Data[8];
/***            done            ***/

/***            编码器脉冲计数         ***/
extern int Right_Wheel_Pulse;
extern int Left_Wheel_Pulse;
extern int Right_Wheel_Pulse_temp;
extern int Left_Wheel_Pulse_temp;
/***            done            ***/

/***            电感ADCc采集数据              ***/
extern uint16_t Test_Adc[5];    //供上位机观察电感值
typedef struct
{
  uint16_t Middle_Voltage;      //中间电感值
  uint16_t Left_Voltage;        //左边电感值
  uint16_t Right_Voltage;       //右边电感值
  uint16_t Middle_Voltage_Temp; //temp值用于惯性滤波
  uint16_t Left_Voltage_Temp;
  uint16_t Right_Voltage_Temp;
  uint16_t Left_Rigth_Delta;
  uint16_t Middle_Voltage_Filter; //Filter值为惯性滤波之后的数据
  uint16_t Left_Voltage_Filter;
  uint16_t Right_Voltage_Filter;
  uint16_t Middle_Voltage_Ave; //Ave值为均值滤波之后的数据
  uint16_t Left_Voltage_Ave;
  uint16_t Right_Voltage_Ave;
  uint16_t Left_Ring_Voltage;
  uint16_t Right_Ring_Voltage;
  uint16_t Left_Ring_Voltage_Filter;
  uint16_t Right_Ring_Voltage_Filter;
  uint16_t Left_Ring_Voltage_Temp;
  uint16_t Right_Ring_Voltage_Temp;
} Inductance_Data;
extern Inductance_Data L;
/***            done            ***/

/***            角度计算使用数据                ***/
//角度数据结构体
typedef struct
{       
  float Balance_Zero;  //车辆机械零点的加速度计读数
  float Final_Integral_Angle;   //互补滤波后车身角度
  float Accle_Z_Data;   //加速度计Z轴读数
  float Z;      //加速度计读出的Z轴角度
  float Angular_Velocity_Z;     //车身Z轴角速度
  float Gyro_X_Offset;  //陀螺仪X轴偏差
  float Gyro_Y_Offset;  //陀螺仪Y轴偏差
  float Gyro_Z_Offset;  //陀螺仪Z轴偏差
  float Car_Z;  //车身实际Z轴角度
  float Output; //角度环输出
  
} Angle_Data;

extern Angle_Data Angle; //车身角度、角速度结构体

extern float Acc_Z_Angle;  //互补滤波中使用
extern float Gyro_Data;  //互补滤波中使用
extern float Delta;
extern short Gyro_X_Staic_Sum;
/***            done            ***/

/***            编码器数据采集&&速度、距离参数                ***/
typedef struct
{
  float arcLength;//弧长
  float arcLengthBuffer;//弧长缓冲区
  float meter; //超过1米处理
  uint16 PulseCount;//脉冲数
}Disdance_ArcLength;//计算真实距离时用到
extern Disdance_ArcLength Left_Arc;
extern Disdance_ArcLength Right_Arc;
extern Disdance_ArcLength Left_Spd;
extern Disdance_ArcLength Right_Spd;

typedef struct
{ 
  float arcLength; //弧长
  float arcLengthBuffer; //弧长缓冲区
  float One_meter;  //1米超量程换算
}Circle;//有关圆周的结构体变量
//计算 真实速度时用到

//extern  float Circle_Ratio ;//圆周参数  从脉冲单位换算成实际距离单位 1个脉冲对应xx米
//【CircleRatio】 = 【(PI*65/1000)/100】; //理论计算:65mm直径/1000(米换算) * PI=周长 / 一圈脉冲量程-->100线


typedef struct
{
  float _Left_Wheel;//左轮速度
  float _Right_Wheel;//右轮速度
  float _Car;//左右中间速度
  float _2ms_True;//2ms测速的真实速度
  float _GrowLimit; //测速速度控制变化率
  float _ReduceLimit;
  float _TrueDistance;
  
  //下面是速度滤波所用变量
  float Info_TrueSpeed;
  float Info_TrueSpeedL;
  float Info_TrueSpeedR;
  float Info_TSpeedL1;
  float Info_TSpeedR1;
}_Speed; //计算速度数据时使用的结构体
extern _Speed Speed;

extern int Left_Wheel_Dir;   //左车轮旋转方向 1为正 0 为负
extern int Right_Wheel_Dir;   //右车轮旋转方向 1为正 0 为负
extern float Left_Distance;
extern float Right_Distance;
/***            done            ***/

/***            车身角度计算输出参数              ***/
extern float Car_Angle_Temp; 
extern float Angle_Kp;  //-800
extern float Angle_Kd;         //-10
/***            done            ***/

/***            车身速度计算输出参数              ***/
extern float Car_Speed_Set;
extern float Speed_Kp;
extern float Speed_Ki;
extern float Car_Speed_Integral;
extern float Car_Speed_Diff;
extern float SpeedControlOld;
extern float SpeedControlNew;
extern float g_fSpeedControlOutNew;
extern float g_fSpeedControlOutOld;
extern float g_fSpeedControlOut;
/***            done            ***/
extern float left,right,middle;
extern float leftOld,rightOld;
extern float qulv;
extern float qulvOld;
extern float pianyi;
extern float pianyiOld;
/*转向环*/
extern float DIR_CLOSED_P;
extern float DIR_CLOSED_D;
extern float g_fDirectionControlOutOld;
extern float g_fDirectionControlOutNew;
extern float fClosedValueOld;
extern float fClosedValueNew;
extern float fDClosedValueOld;
extern float fDClosedValueNew;
extern float g_fCaljiaosuduOld;
extern float g_fCaljiaosuduNew;
extern float g_fTruejiaosuduOld;
extern float g_fTruejiaosuduNew;
extern float ExDir_Control_Output;
extern float Gyro_Dir_Data;
extern float Gyro_Y_Offset;

/***            计算电机输出使用参数              ***/
extern float Angle_Control_Output;
extern float Speed_Control_Output;
extern float Dir_Control_Output;
/***       done     ***/
//检测圆环
extern int T_count;
extern int Ex_count;//强制进入圆环计数
extern int DirectionJudge;
//extern int DirectionJudgeEnd;
extern int CircleCount;//连续符合计数条件检测
extern int CircleEndCount;//连续十次符合结束圆环计数条件
extern int CircleStartFlag;//开始检测圆环
extern int CircleEndFlag;//结束检测圆环
extern int CircleDoFlag;
extern float CircleMidLimit;//检测圆环中间电感阈值
//保护程序
extern int ProtectFlag;
extern int ProtectCount;
extern int ProtectCount2;
extern int ProtectEnableSPD;
extern int ProtectEnableSGN;
//赛道检测
extern int StraightFlag;
extern int StraightFlagOld;
extern int StraightCount;
extern int CurveFlag;
extern int CurveCount;
//拨码开关状态
extern int Boma_Num_Run;
extern int Boma_Num_Protect;
extern int SpeedUpEnable;
//extern float Angles[21];
//extern int Counts[21];
//extern float Left_Norm_Inductances[21];
//extern float Right_Norm_Inductances[21];
extern int NormCount;
extern float NormLeft;
extern float NormRight;
extern int WriteNormFlag;
extern int WriteEndFlag;
extern int WriteFlag;
extern int ReadNormFlag;

extern float TrueLength;
extern float LengthDelta;
extern float TrueLengthOld;

extern float Left_Ave_Diangan;
extern float Right_Ave_Diangan;

extern float FuzzyKd_P[7][7];
extern float FuzzyKd_D[7][7];

extern int GoFlag;
extern int GoCount;
 
extern int RingFlag;
extern int BomaRingFlag;
extern int ZaoFlag;
extern int WanFlag;

extern float IK;

extern int GHGFlag;

extern int LCD_Show[16];
#endif