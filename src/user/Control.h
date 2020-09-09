#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "include.h"
#include "Parameter.h"

/***            ��вɼ����Ʋ���                ***/
void Inductanc_Inertia_Filter(void);   //���й����˲�
void Inductance_Ave_Filter(int N_Times); //�Ե��ֵ���о�ֵ�˲� ����Ϊ��������
void Get_Inductor_Data(void);    //�ɼ����ֵ�����д���
void CurrentDetect(void);    //�����������
void Normalization(void);//��й�һ��

/***            done            ***/

/***            ֱ�����Ʋ���          ***/
void Get_Angle_Data(void);       //��ȡ����Ƕȡ����ٶ���Ϣ
void Angle_Output(float Current_Angle);  //�ǶȻ��������
/***            done            ***/


/***            �ٶȼ��㲿��          ***/

extern float Circle_Ratio;
//Բ�ܲ���  �����嵥λ�����ʵ�ʾ��뵥λ 1�������Ӧxx��
//��CircleRatio�� = ��(PI*65/1000)/x��; 
//���ۼ���:�ܳ�/1000(�׻���) * PI=�ܳ� / һȦ��������-->x��


void Get_ArcLength(Disdance_ArcLength *dis, float Unit, float circle_ratio);     //��������������ת����ʵ�ʾ���
void Get_Speed_Data(void);  //��ȡ�����ٶ���Ϣ
void Judge_Left_Wheel_Dir(void);        //�ж����ַ���
void Judge_Right_Wheel_Dir(void);       //�ж��ҳ��ַ���

void Car_Speed_Control_Output(void);     //�����ٶȼ������
/***            done            ***/

/***            ��ȡ��������������      ***/
void Get_Sensors_Data(void);    
/***            done             ***/

/***            ������㲿��          ***/
void Car_Direction_Control_Output(void); //���򻷼������
void Car_Direction_Control_Output_Plus(void); //����ת�򲹳�
void Circle_Test(void);//Բ��������
void Car_Protect(void);//��������
void Straight_Judge(void);
/***            done            ***/

/***            ����������          ***/
void Set_Motor_Voltage(float f_Left_Voltage, float f_Right_Voltage); 	//���õ����ת������ٶ�
void Set_PWM(int PWM_n,int value);       //����PWM���
void Motor_Output(void);        //�����������
/***            done            ***/



void Auto_Normalization(void);
void Auto_Inductance_Get(int Left_Voltage,int Right_Voltage);



void Set_Fussy_PTable(void);
void Set_Fussy_DTable(void);
void Calc_Membership_Without_CenterZone_7Gear(float value, float* membership, float* dividePoints,int NUM);

void Go(void);

#endif