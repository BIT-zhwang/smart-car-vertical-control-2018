#ifndef __Parameter_H__
#define __Parameter_H__

#include "include.h"


/**********���Ӱ��������*****/
//���ٶȴ�����ԭʼ����
extern short int accle_x;		
extern short int accle_y;
extern short int accle_z;
//������ԭʼ����
extern short int gyro_x;
extern short int gyro_y;
extern short int gyro_z;
//IIC��ȡ����
extern uint8_t IIC_bit8_data[6];
extern short  IIC_bit16_data16[3];
/***      done      **/

/***            ��λ����������              ***/
extern float Send_Data[8];
/***            done            ***/

/***            �������������         ***/
extern int Right_Wheel_Pulse;
extern int Left_Wheel_Pulse;
extern int Right_Wheel_Pulse_temp;
extern int Left_Wheel_Pulse_temp;
/***            done            ***/

/***            ���ADCc�ɼ�����              ***/
extern uint16_t Test_Adc[5];    //����λ���۲���ֵ
typedef struct
{
  uint16_t Middle_Voltage;      //�м���ֵ
  uint16_t Left_Voltage;        //��ߵ��ֵ
  uint16_t Right_Voltage;       //�ұߵ��ֵ
  uint16_t Middle_Voltage_Temp; //tempֵ���ڹ����˲�
  uint16_t Left_Voltage_Temp;
  uint16_t Right_Voltage_Temp;
  uint16_t Left_Rigth_Delta;
  uint16_t Middle_Voltage_Filter; //FilterֵΪ�����˲�֮�������
  uint16_t Left_Voltage_Filter;
  uint16_t Right_Voltage_Filter;
  uint16_t Middle_Voltage_Ave; //AveֵΪ��ֵ�˲�֮�������
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

/***            �Ƕȼ���ʹ������                ***/
//�Ƕ����ݽṹ��
typedef struct
{       
  float Balance_Zero;  //������е���ļ��ٶȼƶ���
  float Final_Integral_Angle;   //�����˲�����Ƕ�
  float Accle_Z_Data;   //���ٶȼ�Z�����
  float Z;      //���ٶȼƶ�����Z��Ƕ�
  float Angular_Velocity_Z;     //����Z����ٶ�
  float Gyro_X_Offset;  //������X��ƫ��
  float Gyro_Y_Offset;  //������Y��ƫ��
  float Gyro_Z_Offset;  //������Z��ƫ��
  float Car_Z;  //����ʵ��Z��Ƕ�
  float Output; //�ǶȻ����
  
} Angle_Data;

extern Angle_Data Angle; //����Ƕȡ����ٶȽṹ��

extern float Acc_Z_Angle;  //�����˲���ʹ��
extern float Gyro_Data;  //�����˲���ʹ��
extern float Delta;
extern short Gyro_X_Staic_Sum;
/***            done            ***/

/***            ���������ݲɼ�&&�ٶȡ��������                ***/
typedef struct
{
  float arcLength;//����
  float arcLengthBuffer;//����������
  float meter; //����1�״���
  uint16 PulseCount;//������
}Disdance_ArcLength;//������ʵ����ʱ�õ�
extern Disdance_ArcLength Left_Arc;
extern Disdance_ArcLength Right_Arc;
extern Disdance_ArcLength Left_Spd;
extern Disdance_ArcLength Right_Spd;

typedef struct
{ 
  float arcLength; //����
  float arcLengthBuffer; //����������
  float One_meter;  //1�׳����̻���
}Circle;//�й�Բ�ܵĽṹ�����
//���� ��ʵ�ٶ�ʱ�õ�

//extern  float Circle_Ratio ;//Բ�ܲ���  �����嵥λ�����ʵ�ʾ��뵥λ 1�������Ӧxx��
//��CircleRatio�� = ��(PI*65/1000)/100��; //���ۼ���:65mmֱ��/1000(�׻���) * PI=�ܳ� / һȦ��������-->100��


typedef struct
{
  float _Left_Wheel;//�����ٶ�
  float _Right_Wheel;//�����ٶ�
  float _Car;//�����м��ٶ�
  float _2ms_True;//2ms���ٵ���ʵ�ٶ�
  float _GrowLimit; //�����ٶȿ��Ʊ仯��
  float _ReduceLimit;
  float _TrueDistance;
  
  //�������ٶ��˲����ñ���
  float Info_TrueSpeed;
  float Info_TrueSpeedL;
  float Info_TrueSpeedR;
  float Info_TSpeedL1;
  float Info_TSpeedR1;
}_Speed; //�����ٶ�����ʱʹ�õĽṹ��
extern _Speed Speed;

extern int Left_Wheel_Dir;   //������ת���� 1Ϊ�� 0 Ϊ��
extern int Right_Wheel_Dir;   //�ҳ�����ת���� 1Ϊ�� 0 Ϊ��
extern float Left_Distance;
extern float Right_Distance;
/***            done            ***/

/***            ����Ƕȼ����������              ***/
extern float Car_Angle_Temp; 
extern float Angle_Kp;  //-800
extern float Angle_Kd;         //-10
/***            done            ***/

/***            �����ٶȼ����������              ***/
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
/*ת��*/
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

/***            ���������ʹ�ò���              ***/
extern float Angle_Control_Output;
extern float Speed_Control_Output;
extern float Dir_Control_Output;
/***       done     ***/
//���Բ��
extern int T_count;
extern int Ex_count;//ǿ�ƽ���Բ������
extern int DirectionJudge;
//extern int DirectionJudgeEnd;
extern int CircleCount;//�������ϼ����������
extern int CircleEndCount;//����ʮ�η��Ͻ���Բ����������
extern int CircleStartFlag;//��ʼ���Բ��
extern int CircleEndFlag;//�������Բ��
extern int CircleDoFlag;
extern float CircleMidLimit;//���Բ���м�����ֵ
//��������
extern int ProtectFlag;
extern int ProtectCount;
extern int ProtectCount2;
extern int ProtectEnableSPD;
extern int ProtectEnableSGN;
//�������
extern int StraightFlag;
extern int StraightFlagOld;
extern int StraightCount;
extern int CurveFlag;
extern int CurveCount;
//���뿪��״̬
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