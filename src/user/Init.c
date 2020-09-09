#include "init.h"
#include "include.h"

//��ʼ����Ӳ���ܽ�
void Init_Hardwares(void)
{
  //��ʼ��8700_2100���Ӱ�
  Init_8700_2100();
  //��ʱ�ȴ�1000MS
  time_delay_ms(2000);
  //��̬���������˲�
  Sample_Static_87002100();
  //ʹ��PWM
  PWM_Iint(); 
  //ʹ��ECLK
  ECLK_Capture_Init();
  //ʹ�ܴ���
   UART_Init();
   //ʹ��ADC
   Inductance_ADC_Init(); 
 //  gpio_init (PTD2, GPO,HIGH);
   //��ʼ��ͣ���ɻɹ�
   GHG_Init();
   //��ʼ�����뿪��
   Boma_Init();
   //��ʼ����������
   Key_Init();
   //��ʼ��OLED��ʾ��
   LCD_Init();
   //��ʼ��������
   Buzzer_Init();
   //��ȡ���뿪��״̬
   Read_Boma();
   //��ʼ��Flash
   //FLASH_Init();
   FLASH_Init1();
   //��ʼ��LEDָʾ��
   gpio_init (PTA1, GPO,HIGH);
   //��ʱ�ȴ�1000MS
 //  time_delay_ms(1000);
}


/***            ��ȡ���Ӱ�����          ***/
void Get_AccANDGyro_Data(void)
{    /*   
   uint8_t bit8_data[6];
   short bit16_data16[3];
   Read_2100_8700(FXOS_8700_ADDR,bit8_data,bit16_data16);
   accle_x= bit16_data16[0];
   accle_y= bit16_data16[1];
   accle_z= bit16_data16[2];
   //�ɼ�2100����
   Read_2100_8700(FX_2100_ADDR,bit8_data,bit16_data16);//16λ����
   gyro_x= bit16_data16[0];
   gyro_y= bit16_data16[1];
   gyro_z= bit16_data16[2];*/
  Read_8700(FXOS_8700_ADDR,IIC_bit8_data,IIC_bit16_data16);//14λ����
//  accle_x= IIC_bit16_data16[0];
//  accle_y= IIC_bit16_data16[1];
  accle_z= IIC_bit16_data16[2];
  //�ɼ�2100����
  Read_2100(FX_2100_ADDR,IIC_bit8_data,IIC_bit16_data16);//16λ����
  gyro_x= IIC_bit16_data16[0];
  gyro_y= IIC_bit16_data16[1];
//  gyro_z= IIC_bit16_data16[2];
}

short Get_Gyro_X(void)       //��ȡ������X������
{
  short temp = 0;
  Get_AccANDGyro_Data();
  temp = gyro_x;
  return temp;
}

short Get_Gyro_Y(void)       //��ȡ������Y������
{
  short temp = 0;
  Get_AccANDGyro_Data();
  temp = gyro_y;
  return temp;
}

short Get_Gyro_Z(void)       //��ȡ������Z������
{
  short temp = 0;
  Get_AccANDGyro_Data();
  temp = gyro_z;
  return temp;
}

uint16_t Get_Acc_X(void)        //��ȡ���ٶȼ�X������    
{
  uint16_t temp = 0;
  Get_AccANDGyro_Data();
  temp = accle_x;
  return temp;
}

uint16_t Get_Acc_Y(void)        //��ȡ���ٶȼ�Y������
{
  uint16_t temp = 0;
  Get_AccANDGyro_Data();
  temp = accle_y;
  return temp;
}

uint16_t Get_Acc_Z(void)        //��ȡ���ٶȼ�Z������
{
  uint16_t temp = 0;
  Get_AccANDGyro_Data();
  temp = accle_z;
  return temp;
}

void Sample_Static_87002100(void)       //��ʼ����Լ��Ӱ���о�̬�����ѱ����ȶ�
{
  int i;
  int Static_Sum = 0;
  for(i=0; i<100; i++)
  {
     Gyro_X_Staic_Sum += Get_Gyro_X();   //���ڵ�·�Ͱ�װ�����ǵ�X����ٶ�Ϊ����Z����ٶ�
     time_delay_ms(2);
  }
  /*
  if(Gyro_X_Staic_Sum >= 0)
    Static_Sum = (int)Gyro_X_Staic_Sum;
  else
  {
    Gyro_X_Staic_Sum = -Gyro_X_Staic_Sum;
    Static_Sum = (int)Gyro_X_Staic_Sum;
    Static_Sum = -Static_Sum;
  }
  */
  //Send_Data[4] = (float)Gyro_X_Staic_Sum;
  
 // Send_Data[4] = (float)Static_Sum;
  //Angle.Gyro_X_Offset = (float)(Static_Sum/100.0f)*0.0625f;//��ȡʵ�ʽ��ٶ�

  Angle.Gyro_X_Offset = (float)(Gyro_X_Staic_Sum/100.0f)*0.0625f;//��ȡʵ�ʽ��ٶ�
  Gyro_X_Staic_Sum = 0;
  for(i=0; i<100; i++)
  {
     Gyro_X_Staic_Sum += Get_Gyro_Y();   //���ڵ�·�Ͱ�װ�����ǵ�X����ٶ�Ϊ����Z����ٶ�
     time_delay_ms(2);
  }
  Gyro_Y_Offset = (float)(Gyro_X_Staic_Sum/100.0f)*0.0625f;
  //Send_Data[4] = Angle.Gyro_X_Offset;
}

/***            done            ***/


/***            FTMģ���ʼ��                ***/

//      PWM��ʼ��
//      ���ܣ�ʹ��PWM���ͨ��
//      �ܽ���Ϣ�� ʹ��FTM2ģ���CH0, CH1, CH2, CH3�ĸ�ͨ��
//                 ��Ӧ�ܽ�Ϊ   PTC0,PTC1,PTC2,PTC3
//      ռ�ձȾ���Ϊ5000������ͨ���޸ĵײ�PRECISE���������޸�
//      ��ע����Ӧ���ֿ�����Ҫʵ�ʱ궨
void PWM_Iint(void)
{
  FTM_PWM_init(CFTM2, FTM_CH0,FTM_PTC0, 10*1000, 0);//PWM2 PTH0
  FTM_PWM_init(CFTM2, FTM_CH1,FTM_PTC1, 10*1000, 0);//PWM2 PTH1
  FTM_PWM_init(CFTM2, FTM_CH2,FTM_PTC2, 10*1000, 0);//PWM2 PTD0
  FTM_PWM_init(CFTM2, FTM_CH3,FTM_PTC3, 10*1000, 0);//PWM2 PTD1
}

//      ECLK�ⲿʱ�ӳ�ʼ��
//      ���ܣ�ʹ�ܱ������ⲿʱ��
//      �ܽ���Ϣ��ʹ���� FTM0��ECLK0�� FTM1��ECLK1
//                ��Ӧ�ܽ�Ϊ   PTE0          PTE7 
//      ��ע��������3�ܽŷ������壬4�ܽŷ��͸ߵ͵�ƽ���ߵ�ƽ��ʾ��������ת���͵�ƽ��ʾ��������ת 
void ECLK_Capture_Init(void)
{
  FTM_count_init(CFTM1);//�ұ�����������E7
  FTM_count_init(CFTM0);//�������������E0
  
  //gpio_init (PTH6, GPI,LOW);    //�Ҳ�����������ж�
  //gpio_init (PTC6, GPI,LOW);    //�������������ж�
  
}

/***            done            ***/


/***            AD�ӿڳ�ʼ��                ***/

//      ADת���ӿڳ�ʼ��
//      ���ܣ�ʹ�ܶ�ȡ���ֵ����Ҫ��ADC�ܽ�
//      �ܽ���Ϣ:ʹ����ADC0ģ��� 5��   12��  13�� 14��  15���ͨ��
//                   ��Ӧ�ܽ�Ϊ  PTB1  PTF4  PTF5  PTF6  PTF7
void Inductance_ADC_Init(void)     
{
  /*�ɳ�
  ADC_Init(ADC_CHANNEL_AD15,ADC_12BIT); //��ߵ��
  ADC_Init(ADC_CHANNEL_AD14,ADC_12BIT);
  ADC_Init(ADC_CHANNEL_AD13,ADC_12BIT); //�ұߵ��
  ADC_Init(ADC_CHANNEL_AD12,ADC_12BIT); //�м���
  ADC_Init(ADC_CHANNEL_AD5,ADC_12BIT);
  */
  //�³�1#
  ADC_Init(ADC_CHANNEL_AD6,ADC_12BIT);  //�м���
  ADC_Init(ADC_CHANNEL_AD7,ADC_12BIT);  
  ADC_Init(ADC_CHANNEL_AD12,ADC_12BIT);  //��ߵ��
  ADC_Init(ADC_CHANNEL_AD13,ADC_12BIT);  //�Ҽ���
//  ADC_Init(ADC_CHANNEL_AD12,ADC_12BIT);
  //12λ�ֱ���
}

/***            done            ***/


/***            UART0���ڳ�ʼ��                ***/

//      UART0����ͨ�ų�ʼ��
//      ���ܣ�ʹ��UART0ͨ�Źܽ�
//      �ܽ���Ϣ��ʹ����UART0ģ���RX,  TX ͨ��
//                   ��Ӧ�Ĺܽ�Ϊ PTA2 PTA3 (REMAP)
//      �����ʣ�115200

void UART_Init(void)
{
  uart_init(UARTR2,Remap,115200);
}

/***            done            ***/

/***            ͣ���ɻɹܽӿڳ�ʼ��                ***/

//      ���ݸɻɹܱպ�����ж��Ƿ�ͣ��
//      ���ܣ�ʹ�ܸɻɹ��źż��ܽ�
//      �ܽ���Ϣ��
//                   ��Ӧ�Ĺܽ�Ϊ PTE6
//                   �͵�ƽΪ�������ߵ�ƽΪͣ����

void GHG_Init(void)
{
  gpio_init (PTC6, GPI,LOW);    //����Ϊ�����ȡģʽ
}

void Boma_Init(void)
{
  gpio_init(PTG6,GPI,LOW);      //����Ϊ�����ȡģʽ 1
  gpio_init(PTG7,GPI,LOW);      //����Ϊ�����ȡģʽ 2
  gpio_init(PTG4,GPI,LOW);      //����Ϊ�����ȡģʽ 3
  gpio_init(PTG5,GPI,LOW);      //����Ϊ�����ȡģʽ 4
  gpio_init(PTF0,GPI,LOW);      //����Ϊ�����ȡģʽ 5
  gpio_init(PTF1,GPI,LOW);      //����Ϊ�����ȡģʽ 6
  gpio_init(PTD3,GPI,LOW);      //����Ϊ�����ȡģʽ 7
  gpio_init(PTD4,GPI,LOW);      //����Ϊ�����ȡģʽ 8
}

void Key_Init(void)//  �������Ƴ�ʼ��
{
  //���򿪹�
  gpio_init(PTH1,GPI,LOW);      //����Ϊ�����ȡģʽ
  gpio_init(PTI0,GPI,LOW);      //����Ϊ�����ȡģʽ
  gpio_init(PTI1,GPI,LOW);      //����Ϊ�����ȡģʽ
  gpio_init(PTI4,GPI,LOW);      //����Ϊ�����ȡģʽ
  gpio_init(PTH2,GPI,LOW);      //����Ϊ�����ȡģʽ
  
  //��������      
  gpio_init(PTH5,GPI,LOW);      //����Ϊ�����ȡģʽ  
  
}
void Buzzer_Init(void)
{
  gpio_init(PTI3,GPO,LOW);
}
/***            done            ***/

/***            AD�ӿڳ�ʼ��                ***/

//      ���뿪�ض�ȡ
//      ���ܣ�ʶ���뿪�صĿ��ر���
//      �ܽ���Ϣ:ʹ����
//  PTG6      //����Ϊ�����ȡģʽ 1
//  PTG7      //����Ϊ�����ȡģʽ 2
//  PTG4      //����Ϊ�����ȡģʽ 3
//  PTG5      //����Ϊ�����ȡģʽ 4
//  PTF0      //����Ϊ�����ȡģʽ 5
//  PTF1      //����Ϊ�����ȡģʽ 6
//  PTD3      //����Ϊ�����ȡģʽ 7
//  PTD4      //����Ϊ�����ȡģʽ 8
void Read_Boma(void)     
{       
  int D0=0,D1=0,D2=0,D3=0,D4=0,D5=0,D6=0,D7=0;
  //��ȡ��������״̬
  if(gpio_get(PTG6))    D0=1;//ǰ4λ1234
  if(gpio_get(PTG7))    D1=2;
  if(gpio_get(PTG4))    D2=4;
  if(gpio_get(PTG5))    D3=8;
  
  //if(gpio_get(PTF0))    D4=1;//��4λ5678
  if(gpio_get(PTF1))    D5=1;
  if(gpio_get(PTD3))    D6=2;
  if(gpio_get(PTD4))    D7=4;
  //������״̬ת����ʮ����
  Boma_Num_Run=D0+D1+D2+D3;
  Boma_Num_Protect=D4+D5+D6+D7;
}

//��������
void DiDi(void)
{
  gpio_set(PTI3,1);
}

//�ط�����
void ShutDown_DiDi(void)
{
  gpio_set(PTI3,0);
}
/***            done            ***/

//��ȡ�ɻɹܵ�ƽ
void Read_GHG(void)
{
  GHGFlag=gpio_get(PTC6);
}

