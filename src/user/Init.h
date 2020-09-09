#ifndef __INIT_H__
#define __INIT_H__

#include "include.h"
#include "8700_2100.h"
#include "Parameter.h"

void Init_Hardwares(void); //ȫӲ����ʼ��

void Sample_Static_87002100(void);        //���Ӱ徲̬�ɼ�
void Get_AccANDGyro_Data(void);//��ȡ���Ӱ�����
short Get_Gyro_X(void);//��ȡ������X����ٶ�
short Get_Gyro_Y(void);//��ȡ������Y����ٶ�
short Get_Gyro_Z(void);//��ȡ������Z����ٶ�
uint16_t Get_Acc_X(void);//��ȡ���ٶȼ�X��Ƕ�
uint16_t Get_Acc_Y(void);//��ȡ���ٶȼ�Y��Ƕ�
uint16_t Get_Acc_Z(void);//��ȡ���ٶȼ�Z��Ƕ�

void PWM_Iint(void);//PWMͨ����ʼ��
void ECLK_Capture_Init(void);//������ͨ����ʼ��

void Inductance_ADC_Init(void);//���ADֵ�ɼ���ʼ��

void UART_Init(void);//����ͨ�ų�ʼ��

void GHG_Init(void);//ͣ���ɻɹܳ�ʼ��
void Boma_Init(void);//���뿪�س�ʼ��
void Key_Init(void);//  �������Ƴ�ʼ��
void Buzzer_Init(void);//��������ʼ��b
void Read_Boma(void);//��ȡ���뿪��״̬

void ShutDown_DiDi(void);//�رշ�����
void DiDi(void);//��������

void Read_GHG(void);//ͣ���ɻɹ�

#endif