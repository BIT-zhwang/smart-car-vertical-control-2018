#include "include.h"
#include "Parameter.h"
#include "common.h"

/***            ���ٶȴ�����ԭʼ����              ***/
short int accle_x =0;		
short int accle_y =0;
short int accle_z =0;
/***    done            ***/

/***            ������ԭʼ����         ***/
short int gyro_x =0;
short int gyro_y =0;
short int gyro_z =0;
/***            done            ***/    

/***            IIC��ȡ����             ***/
uint8_t IIC_bit8_data[6];
short  IIC_bit16_data16[3];
/***            done            ***/

/***            ��λ����������         ***/
float Send_Data[8];
/***            done            ***/

/***            �������������         ***/
int Right_Wheel_Pulse =0;
int Left_Wheel_Pulse =0;
int Right_Wheel_Pulse_temp =0;
int Left_Wheel_Pulse_temp =0;
/***            done            ***/

/***            ���ADC�ɼ�����               ***/
uint16_t Test_Adc[5] =0;
Inductance_Data L;
/***            done            ***/

/***            �Ƕȼ���ʹ������                ***/
Angle_Data Angle;       //�Ƕ����ݽṹ��

float Acc_Z_Angle = 0;  //�����˲���ʹ��
float Gyro_Data = 0;     //�����˲���ʹ��
float Delta= 0;
short Gyro_X_Staic_Sum = 0;
/***            done            ***/

/***���������ݲɼ�&&�ٶȡ��������            ***/
Disdance_ArcLength Left_Arc;
Disdance_ArcLength Right_Arc;
Disdance_ArcLength Left_Spd;
Disdance_ArcLength Right_Spd;

//float Circle_Ratio  = 0.0000870795f;//Բ�ܲ���  �����嵥λ�����ʵ�ʾ��뵥λ 1�������Ӧxx��
//��CircleRatio�� = ��(PI*65/1000)/100��; //���ۼ���:�ܳ�/1000(�׻���) * PI=�ܳ� / һȦ��������-->100��


_Speed Speed;
int Left_Wheel_Dir = 1;   //������ת���� 1Ϊ�� 0 Ϊ��
int Right_Wheel_Dir = 1;   //�ҳ�����ת���� 1Ϊ�� 0 Ϊ��
float Left_Distance = 0;
float Right_Distance = 0;
/***            done            ***/


/***            ���������ʹ�ò���              ***/
float Angle_Control_Output = 0;
float Speed_Control_Output = 0;
float Dir_Control_Output = 0;
/***       done     ***/

/***            ����Ƕȼ����������              ***/
float Car_Angle_Temp = 0;
float Angle_Kp = 0;  //-800
float Angle_Kd = 0;         //-10


/***            done            ***/

/***            �����ٶȼ����������              ***/
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
/*ת�򻷼�������*/
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

//���Բ��
int T_count = 0;
int Ex_count;//ǿ�ƽ���Բ������
int DirectionJudge=0;
//int DirectionJudgeEnd=0;
int CircleStartFlag=0;//��ʼ���Բ��
int CircleEndFlag=0;//�������Բ��
int CircleCount=0;
int CircleEndCount=0;
int CircleDoFlag=0;
float CircleMidLimit=0;//���Բ���м�����ֵ
//���������־λ
int ProtectFlag=0;
int ProtectCount=0;
int ProtectCount2=0;
int ProtectEnableSPD=0;
int ProtectEnableSGN=0;
//ֱ������־λ
int StraightFlag=0;
int StraightFlagOld=0;
int StraightCount=0;
int CurveFlag=0;
int CurveCount=0;

//���뿪��״̬
int Boma_Num_Run=0;
int Boma_Num_Protect=0;
int SpeedUpEnable=0;
//�Զ���һ��
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