
/*****************************************************************
��dev.env.��IAR7.80.4
��Target  ��S9KEAZ128
��Crystal ��16.000Mhz
��busclock��40.000MHz
��pllclock��40.000MHz
******************************************************************/
/*************************����˵��******************************/
/****************************************************************
  ����ʲô���ܾ�ȥ����Ӧ����ǰ�ġ�//��
  ���������mini���İ��Լ�����ĸ���д�������汾���İ��޸�����
  TF��&OLED�汾���İ���OLED�ӿ���ADC�����е�ADC���ų�ͻ��ע���޸�
  PWM�ͱ������ɼ���������ͬʱ���ã���ΪFTMģ���ͻ
  ÿ�����Ժ�����д�� while��1����ͬʱʹ���������Ժ�����ִ�еĺ�����Ч
*******************************************************************/
/******************************************************************
�ӿڶ��壺
-------------------------------------------------------------
LED         ��Ƭ���ӿ�
//���İ���RGB����
LED0           PTC2
LED1           PTB4
LED2           PTE5
//ĸ����LED����
LED3           PTI4
LED4           PTH6
===============================================================
KEY        ��Ƭ���ӿ�
//���İ��ϰ���
KEY0          PTA0
//ĸ�尴��
KEY1          PTB5
KEY2          PTH2
KEY3          PTE6
===============================================================
���뿪��    ��Ƭ���ӿ�
KEY0          PTI2
KEY1          PTI3
KEY2          PTE2
KEY3          PTE3
===============================================================
TSL1401ģ��     ��Ƭ���ӿ�
VCC             5V
GND             GND
SI              I5/D5
SCK             I6/D6
ADC             AD8 C0/AD9 C1
===============================================================
��·���ģ��    ��Ƭ���ӿ�
VCC             5V
GND             GND
ADCͨ��         �ܽŹ�ϵ     
ADC12           F4      
ADC13           F5      
ADC14           F6      
ADC15           F7       
ADC4            B0      
ADC5            B1   
ADC6            B2       
ADC7            B3
-------------------------------------------------------------
��Դ��ػ�������    
ADC11           C3              
-------------------------------------------------------------
MPU6050        ��Ƭ���ӿ�   FLEXCOMM8
VCC             5V
GND             GND
SDA1            H3
SCL1            H4
//////////////////////////////////////////////////////////ͨ�ò��ֹ���//////////
�������        ��Ƭ���ӿ�   
VCC             5V
PWM1            FTM2-CH0  H0
PWM2            FTM2-CH1  H1
PWM3            FTM2-CH2  D0
PWM4            FTM2-CH3  D1
-------------------------------------------------------------
����ӿ�        ��Ƭ���ӿ�
VCC             �ɵ�
GND             GND
PWM1            FTM1-CH1  E7
-------------------------------------------------------------
����512������   ��Ƭ���ӿ�   
VCC             5V
GND             GND      
LSB/A           FTM0 E0    
DIR/B           H7     
LSB/A           FTM1 E7     
DIR/B           H5     
-------------------------------------------------------------
OLEDģ��        ��Ƭ���ӿ�
VCC             5V
GND             GND
SCK             F3
SDA             F2
RST             A7
DC              A6
-------------------------------------------------------------
����/USBTTL    ��Ƭ���ӿ�   FLEXCOMM0
VCC             5V
GND             GND
UART2_RX        I1    
UART2_TX        I0
=============================================================*/
//��������


#include "include.h"
#include "Parameter.h"
#include "Init.h"
#include "debug.h"


void main(void)
{  
  //2#
  DisableInterrupts ;                  //��ֹ�ж�
  time_delay_ms(2000);
  /***          ��ʼ������Ӳ���ܽ�               ***/
  Init_Hardwares();
  //���Ӵ��ڻ�е����ʱ����ٶȼƶ��������ڲ�����
  Angle.Balance_Zero =5900.0f;    
  //�ٶ��趨
  Car_Speed_Set = 2.5f;
  //����
  Set_Fussy_PTable();
  Set_Fussy_DTable();
  DIR_CLOSED_P=18.0;//1.5-->10.     2.0-->10.0
  DIR_CLOSED_D=12;//1.5-->15       2.0-->15
  //�ǶȻ�
  Angle_Kp = -900;//1.5-->-1000   2.0-->-1000  -700   -800
  Angle_Kd = -8.0;//1.5-->-9         2.0-->-9    -6    -7
  //�ٶȻ�
  Speed_Kp = 30;//1.5-->30         2.0-->25
  Speed_Ki = 0.25;//1.5-->0.25      2.0-->0.2
  //time_delay_ms(2000);
  /*************��ʼ��LED***************/ 
  //gpio_init (PTD3, GPO,HIGH);
  //gpio_init (PTD2, GPO,HIGH);
   /*************��ʼ��FTM�������***************/
  //FTM_count_init(CFTM1);//������PTH2,FTM1_CH0
  //FTM_count_init(CFTM0);//������PTB2,FTM0_CH0
  /************��ʼ��8700_2100***********/
  //Init_8700_2100();
  /************��ʼ��UART2***************/
  //uart_init(UARTR1,Remap,9600);
    /*************��ʼ��ADC********/
  //ADC_Init(ADC_CHANNEL_AD15,ADC_12BIT);
  //ADC_Init(ADC_CHANNEL_AD14,ADC_12BIT);
  //ADC_Init(ADC_CHANNEL_AD13,ADC_12BIT);
  //ADC_Init(ADC_CHANNEL_AD12,ADC_12BIT);
  // ADC_Init(ADC_CHANNEL_AD5,ADC_12BIT);
  
  //ͨ�����뿪��״̬ת������
  switch (Boma_Num_Run)        //1234λ���ڿ����ܳ�����
  {
    case 0: Car_Speed_Set = 2.0f;  break;//1234OFF
    case 1: Car_Speed_Set = 2.0f;  RingFlag=1;  break;//1ON
    case 2: Car_Speed_Set = 2.5f;  break;//2ON
    case 3: Car_Speed_Set = 2.5f;  RingFlag=1;   break;//12ON
    case 4: Car_Speed_Set = 2.7f;  break;//3ON
    case 5: Car_Speed_Set = 2.7f;  RingFlag=1;   break;//31ON
    case 6: Car_Speed_Set = 2.0f;  BomaRingFlag=1;  DirectionJudge=1;  break;//32ON  ��ת2.0
    case 7: Car_Speed_Set = 2.0f;  BomaRingFlag=1;  DirectionJudge=-1;  break;//123ON  ��ת2.0
    case 8: Car_Speed_Set = 2.5f;  BomaRingFlag=1;  DirectionJudge=1;   break;//4On  2.5��ת
    case 9: Car_Speed_Set = 2.5f;  BomaRingFlag=1;  DirectionJudge=-1;   break;//14ON  2.5��ת
    case 10: Car_Speed_Set = 2.7;  BomaRingFlag=1;  DirectionJudge=1;   break;//24ON  2.7��
    case 11: Car_Speed_Set = 2.7;  BomaRingFlag=1;  DirectionJudge=-1;   break;//124ON  2.7��
    case 12: Car_Speed_Set = 2.5f;  BomaRingFlag=1;  DirectionJudge=1; ZaoFlag=1;  break;//34ON 2.5Բ����ת�ӳٽ��뻷��
    case 13: Car_Speed_Set = 2.5f;  BomaRingFlag=1;  DirectionJudge=1; WanFlag=1;  break;//134ON  2.5Բ����ת��ǰ���뻷��
    case 14: Car_Speed_Set = 2.5f;  BomaRingFlag=1;  DirectionJudge=-1; ZaoFlag=1;   break;//234ON 2.5Բ����ת�ӳٽ��뻷��
    case 15: Car_Speed_Set = 2.5f;  BomaRingFlag=1;  DirectionJudge=-1; WanFlag=1;   break;//1234ON 2.5Բ����ת��ǰ���뻷��
  }
  switch (Boma_Num_Protect)     //5678λ���ڿ��Ʊ�������
  {
    //case 0: WriteNormFlag=1;  FLASH_EraseSector1(FLASH_SECTOR_NUM - 1);   break;//678 OFF���ж�̬��һ��
    case 1: ProtectEnableSGN=1,ProtectEnableSPD=1,ReadNormFlag=0;   break;//6 ON����������ʹ�ù�һ������
    case 2: ProtectEnableSGN=1;ProtectEnableSPD=1;SpeedUpEnable=1;ReadNormFlag=0;   break;//7 ON���������ܼ��١���ʹ�ù�һ��
    case 3: ProtectEnableSGN=1,ProtectEnableSPD=1,ReadNormFlag=1;   break;//67 ON����ʹ�ù�һ��
    case 4: ProtectEnableSGN=1,ProtectEnableSPD=1,SpeedUpEnable=1,ReadNormFlag=1,IK=0.8;   break;//8on��������м��������ܼ���
    case 5: ProtectEnableSGN=1,ProtectEnableSPD=1,SpeedUpEnable=1,ReadNormFlag=1,IK=0.9;   break;//68ON���������ܼ��١���м���
    case 6: ProtectEnableSGN=1,ProtectEnableSPD=1,SpeedUpEnable=1,ReadNormFlag=1,IK=1.1;   break;//78ON���������ܼ��١������ǿ
    case 7: ProtectEnableSGN=1,ProtectEnableSPD=1,SpeedUpEnable=1,ReadNormFlag=1,IK=1.2;   break;//678ON���������ܼ��١������ǿ
  }
  //gpio_set(PTI3,1);
  /*************��ʼ��PIT0��ʱ��********/
  PIT_Init(PIT_CHANNEL0,2);          //�жϼ��2ms
  /*
  FTM_PWM_Duty(CFTM2, FTM_CH0, 5000);   //ͨ���ߵ�ѹʱ������ǰ
  FTM_PWM_Duty(CFTM2, FTM_CH1, 0);   //ͨ���ߵ�ѹʱ�������
  FTM_PWM_Duty(CFTM2, FTM_CH2, 5000);   //ͨ���ߵ�ѹʱ������ǰ
  FTM_PWM_Duty(CFTM2, FTM_CH3, 0);   //ͨ���ߵ�ѹʱ�������
  */
  EnableInterrupts;     //�������ж�
  //TestLED();//����GPIOmini���İ�RGB������ɫ��˸
  //Test_KEY(); //����GPIO��������
  //Test_KEY1(); //����GPIO���뿪�ع���
  //TestADC();//����ADCת����ͨ��OLED��ʾ��UART����
  //TestKBI();//����GPIO�ⲿ�ж� ĸ����K0��K1������Ч
  //TestPIT();//����PIT��ʱ�ж�
  //TestFTM_PWM();//����PWM/OLED��������ƹ���
  //TestUART012();//����UART012���ͼ������ж�
  //TestFTM_InputCapture();//���Ա���������ɼ�����
  //TestMPU6050();
  //Test_2100_8700();//����������ᴫ����
  //TestRTC();//LED��˸��ͬʱOLED����ʾʱ�䣬��λ��
  

 while(1)
 {
   //Data_Send(UARTR0,Send_Data);
   //vcan_sendware(uint8 *wareaddr, uint32 waresize);
   /*
   int flag=0;
   flag=gpio_get(PTE6);
   if(flag) DiDi();
   else ShutDown_DiDi();
  */
   //vcan_sendware((uint8 *) Send_Data, sizeof(Send_Data), UARTR2);
   TestADC();
 }
  
}



