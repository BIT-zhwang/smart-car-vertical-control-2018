#include "include.h"
#include "Parameter.h"
#include "common.h"

/***            加速度传感器原始数据              ***/
short int accle_x =0;		
short int accle_y =0;
short int accle_z =0;
/***    done            ***/

/***            陀螺仪原始数据         ***/
short int gyro_x =0;
short int gyro_y =0;
short int gyro_z =0;
/***            done            ***/    

/***            IIC读取数据             ***/
uint8_t IIC_bit8_data[6];
short  IIC_bit16_data16[3];
/***            done            ***/

/***            上位机发送数据         ***/
float Send_Data[8];
/***            done            ***/

/***            编码器脉冲计数         ***/
int Right_Wheel_Pulse =0;
int Left_Wheel_Pulse =0;
int Right_Wheel_Pulse_temp =0;
int Left_Wheel_Pulse_temp =0;
/***            done            ***/

/***            电感ADC采集数据               ***/
uint16_t Test_Adc[5] =0;
Inductance_Data L;
/***            done            ***/

/***            角度计算使用数据                ***/
Angle_Data Angle;       //角度数据结构体

float Acc_Z_Angle = 0;  //互补滤波中使用
float Gyro_Data = 0;     //互补滤波中使用
float Delta= 0;
short Gyro_X_Staic_Sum = 0;
/***            done            ***/

/***编码器数据采集&&速度、距离参数            ***/
Disdance_ArcLength Left_Arc;
Disdance_ArcLength Right_Arc;
Disdance_ArcLength Left_Spd;
Disdance_ArcLength Right_Spd;

//float Circle_Ratio  = 0.0000870795f;//圆周参数  从脉冲单位换算成实际距离单位 1个脉冲对应xx米
//【CircleRatio】 = 【(PI*65/1000)/100】; //理论计算:周长/1000(米换算) * PI=周长 / 一圈脉冲量程-->100线


_Speed Speed;
int Left_Wheel_Dir = 1;   //左车轮旋转方向 1为正 0 为负
int Right_Wheel_Dir = 1;   //右车轮旋转方向 1为正 0 为负
float Left_Distance = 0;
float Right_Distance = 0;
/***            done            ***/


/***            计算电机输出使用参数              ***/
float Angle_Control_Output = 0;
float Speed_Control_Output = 0;
float Dir_Control_Output = 0;
/***       done     ***/

/***            车身角度计算输出参数              ***/
float Car_Angle_Temp = 0;
float Angle_Kp = 0;  //-800
float Angle_Kd = 0;         //-10


/***            done            ***/

/***            车身速度计算输出参数              ***/
float Car_Speed_Set = 0.0f;
float Speed_Kp = 0;//2.0-->25
float Speed_Ki = 0;//2.0-->0.3
float Car_Speed_Integral = 0;
float Car_Speed_Diff=0;
float SpeedControlOld=0;
float SpeedControlNew=0;
float g_fSpeedControlOutNew=0;
float g_fSpeedControlOutOld=0;
float g_fSpeedControlOut=0;
/***            done            ***/
float left,right,middle;
float leftOld,rightOld;
float qulv;
float qulvOld;
float pianyi;
float pianyiOld;
/*转向环计算数据*/
float DIR_CLOSED_P=0;
float DIR_CLOSED_D=0;
float g_fDirectionControlOutOld=0;
float g_fDirectionControlOutNew=0;
float fClosedValueOld=0;
float fClosedValueNew=0;
float fDClosedValueOld=0;
float fDClosedValueNew=0;
float g_fCaljiaosuduOld=0;
float g_fCaljiaosuduNew=0;
float g_fTruejiaosuduOld=0;
float g_fTruejiaosuduNew=0;

float ExDir_Control_Output=0;

float Gyro_Dir_Data = 0;
float Gyro_Y_Offset = 0;

//检测圆环
int T_count = 0;
int Ex_count;//强制进入圆环计数
int DirectionJudge=0;
//int DirectionJudgeEnd=0;
int CircleStartFlag=0;//开始检测圆环
int CircleEndFlag=0;//结束检测圆环
int CircleCount=0;
int CircleEndCount=0;
int CircleDoFlag=0;
float CircleMidLimit=0;//检测圆环中间电感阈值
//保护程序标志位
int ProtectFlag=0;
int ProtectCount=0;
int ProtectCount2=0;
int ProtectEnableSPD=0;
int ProtectEnableSGN=0;
//直道检测标志位
int StraightFlag=0;
int StraightFlagOld=0;
int StraightCount=0;
int CurveFlag=0;
int CurveCount=0;

//拨码开关状态
int Boma_Num_Run=0;
int Boma_Num_Protect=0;
int SpeedUpEnable=0;
//自动归一化
//float Angles[21]={-4,-3.5,-3,-2.5,-2,-1.5,-1,-0.5,0,0.5,1,1.5,2,2.5,3,3.5,4,4.5,5,5.5,6};
//int Counts[21]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//float Left_Norm_Inductances[21];
//float Right_Norm_Inductances[21];
int NormCount=0;
float NormLeft=0;
float NormRight=0;
int WriteNormFlag=0;
int WriteFlag=0;
int WriteEndFlag=0;
int ReadNormFlag=0;

float TrueLength=0;
float LengthDelta=0;
float TrueLengthOld=0;

float Left_Ave_Diangan=0;
float Right_Ave_Diangan=0;

float FuzzyKd_P[7][7];
float FuzzyKd_D[7][7];

int GoFlag=0;
int GoCount=0;

int RingFlag=0;
int BomaRingFlag=0;
int ZaoFlag=0;
int WanFlag=0;
float IK=1.0;

int GHGFlag=0;

int LCD_Show[16]={0};