
/*****************************************************************
【dev.env.】IAR7.80.4
【Target  】S9KEAZ128
【Crystal 】16.000Mhz
【busclock】40.000MHz
【pllclock】40.000MHz
******************************************************************/
/*************************测试说明******************************/
/****************************************************************
  测试什么功能就去掉对应函数前的“//”
  该例程针对mini核心板以及配套母版编写，其他版本核心板修改引脚
  TF卡&OLED版本核心板上OLED接口与ADC例程中的ADC引脚冲突，注意修改
  PWM和编码器采集函数不能同时启用，因为FTM模块冲突
  每个测试函数都写有 while（1），同时使能两个测试函数先执行的函数有效
*******************************************************************/
/******************************************************************
接口定义：
-------------------------------------------------------------
LED         单片机接口
//核心板上RGB灯珠
LED0           PTC2
LED1           PTB4
LED2           PTE5
//母板上LED灯珠
LED3           PTI4
LED4           PTH6
===============================================================
KEY        单片机接口
//核心板上按键
KEY0          PTA0
//母板按键
KEY1          PTB5
KEY2          PTH2
KEY3          PTE6
===============================================================
拨码开关    单片机接口
KEY0          PTI2
KEY1          PTI3
KEY2          PTE2
KEY3          PTE3
===============================================================
TSL1401模块     单片机接口
VCC             5V
GND             GND
SI              I5/D5
SCK             I6/D6
ADC             AD8 C0/AD9 C1
===============================================================
多路电感模块    单片机接口
VCC             5V
GND             GND
ADC通道         管脚关系     
ADC12           F4      
ADC13           F5      
ADC14           F6      
ADC15           F7       
ADC4            B0      
ADC5            B1   
ADC6            B2       
ADC7            B3
-------------------------------------------------------------
电源监控或者它用    
ADC11           C3              
-------------------------------------------------------------
MPU6050        单片机接口   FLEXCOMM8
VCC             5V
GND             GND
SDA1            H3
SCL1            H4
//////////////////////////////////////////////////////////通用部分功能//////////
电机驱动        单片机接口   
VCC             5V
PWM1            FTM2-CH0  H0
PWM2            FTM2-CH1  H1
PWM3            FTM2-CH2  D0
PWM4            FTM2-CH3  D1
-------------------------------------------------------------
舵机接口        单片机接口
VCC             可调
GND             GND
PWM1            FTM1-CH1  E7
-------------------------------------------------------------
龙邱512编码器   单片机接口   
VCC             5V
GND             GND      
LSB/A           FTM0 E0    
DIR/B           H7     
LSB/A           FTM1 E7     
DIR/B           H5     
-------------------------------------------------------------
OLED模块        单片机接口
VCC             5V
GND             GND
SCK             F3
SDA             F2
RST             A7
DC              A6
-------------------------------------------------------------
蓝牙/USBTTL    单片机接口   FLEXCOMM0
VCC             5V
GND             GND
UART2_RX        I1    
UART2_TX        I0
=============================================================*/
//编译运行


#include "include.h"
#include "Parameter.h"
#include "Init.h"
#include "debug.h"


void main(void)
{  
  //2#
  DisableInterrupts ;                  //禁止中断
  time_delay_ms(2000);
  /***          初始化所有硬件管脚               ***/
  Init_Hardwares();
  //车子处于机械零点的时候加速度计读数（用于补偿）
  Angle.Balance_Zero =5900.0f;    
  //速度设定
  Car_Speed_Set = 2.5f;
  //方向环
  Set_Fussy_PTable();
  Set_Fussy_DTable();
  DIR_CLOSED_P=18.0;//1.5-->10.     2.0-->10.0
  DIR_CLOSED_D=12;//1.5-->15       2.0-->15
  //角度环
  Angle_Kp = -900;//1.5-->-1000   2.0-->-1000  -700   -800
  Angle_Kd = -8.0;//1.5-->-9         2.0-->-9    -6    -7
  //速度环
  Speed_Kp = 30;//1.5-->30         2.0-->25
  Speed_Ki = 0.25;//1.5-->0.25      2.0-->0.2
  //time_delay_ms(2000);
  /*************初始化LED***************/ 
  //gpio_init (PTD3, GPO,HIGH);
  //gpio_init (PTD2, GPO,HIGH);
   /*************初始化FTM脉冲计数***************/
  //FTM_count_init(CFTM1);//输入用PTH2,FTM1_CH0
  //FTM_count_init(CFTM0);//输入用PTB2,FTM0_CH0
  /************初始化8700_2100***********/
  //Init_8700_2100();
  /************初始化UART2***************/
  //uart_init(UARTR1,Remap,9600);
    /*************初始化ADC********/
  //ADC_Init(ADC_CHANNEL_AD15,ADC_12BIT);
  //ADC_Init(ADC_CHANNEL_AD14,ADC_12BIT);
  //ADC_Init(ADC_CHANNEL_AD13,ADC_12BIT);
  //ADC_Init(ADC_CHANNEL_AD12,ADC_12BIT);
  // ADC_Init(ADC_CHANNEL_AD5,ADC_12BIT);
  
  //通过拨码开关状态转换参数
  switch (Boma_Num_Run)        //1234位用于控制跑车参数
  {
    case 0: Car_Speed_Set = 2.0f;  break;//1234OFF
    case 1: Car_Speed_Set = 2.0f;  RingFlag=1;  break;//1ON
    case 2: Car_Speed_Set = 2.5f;  break;//2ON
    case 3: Car_Speed_Set = 2.5f;  RingFlag=1;   break;//12ON
    case 4: Car_Speed_Set = 2.7f;  break;//3ON
    case 5: Car_Speed_Set = 2.7f;  RingFlag=1;   break;//31ON
    case 6: Car_Speed_Set = 2.0f;  BomaRingFlag=1;  DirectionJudge=1;  break;//32ON  左转2.0
    case 7: Car_Speed_Set = 2.0f;  BomaRingFlag=1;  DirectionJudge=-1;  break;//123ON  右转2.0
    case 8: Car_Speed_Set = 2.5f;  BomaRingFlag=1;  DirectionJudge=1;   break;//4On  2.5左转
    case 9: Car_Speed_Set = 2.5f;  BomaRingFlag=1;  DirectionJudge=-1;   break;//14ON  2.5右转
    case 10: Car_Speed_Set = 2.7;  BomaRingFlag=1;  DirectionJudge=1;   break;//24ON  2.7左
    case 11: Car_Speed_Set = 2.7;  BomaRingFlag=1;  DirectionJudge=-1;   break;//124ON  2.7右
    case 12: Car_Speed_Set = 2.5f;  BomaRingFlag=1;  DirectionJudge=1; ZaoFlag=1;  break;//34ON 2.5圆环左转延迟进入环岛
    case 13: Car_Speed_Set = 2.5f;  BomaRingFlag=1;  DirectionJudge=1; WanFlag=1;  break;//134ON  2.5圆环左转提前进入环岛
    case 14: Car_Speed_Set = 2.5f;  BomaRingFlag=1;  DirectionJudge=-1; ZaoFlag=1;   break;//234ON 2.5圆环右转延迟进入环岛
    case 15: Car_Speed_Set = 2.5f;  BomaRingFlag=1;  DirectionJudge=-1; WanFlag=1;   break;//1234ON 2.5圆环右转提前进入环岛
  }
  switch (Boma_Num_Protect)     //5678位用于控制保护程序
  {
    //case 0: WriteNormFlag=1;  FLASH_EraseSector1(FLASH_SECTOR_NUM - 1);   break;//678 OFF进行动态归一化
    case 1: ProtectEnableSGN=1,ProtectEnableSPD=1,ReadNormFlag=0;   break;//6 ON开保护，不使用归一化参数
    case 2: ProtectEnableSGN=1;ProtectEnableSPD=1;SpeedUpEnable=1;ReadNormFlag=0;   break;//7 ON保护、起跑加速、不使用归一化
    case 3: ProtectEnableSGN=1,ProtectEnableSPD=1,ReadNormFlag=1;   break;//67 ON护、使用归一化
    case 4: ProtectEnableSGN=1,ProtectEnableSPD=1,SpeedUpEnable=1,ReadNormFlag=1,IK=0.8;   break;//8on保护、电感减弱、起跑加速
    case 5: ProtectEnableSGN=1,ProtectEnableSPD=1,SpeedUpEnable=1,ReadNormFlag=1,IK=0.9;   break;//68ON保护、起跑加速、电感减弱
    case 6: ProtectEnableSGN=1,ProtectEnableSPD=1,SpeedUpEnable=1,ReadNormFlag=1,IK=1.1;   break;//78ON保护、起跑加速、电感增强
    case 7: ProtectEnableSGN=1,ProtectEnableSPD=1,SpeedUpEnable=1,ReadNormFlag=1,IK=1.2;   break;//678ON保护、起跑加速、电感增强
  }
  //gpio_set(PTI3,1);
  /*************初始化PIT0定时器********/
  PIT_Init(PIT_CHANNEL0,2);          //中断间隔2ms
  /*
  FTM_PWM_Duty(CFTM2, FTM_CH0, 5000);   //通道高电压时右轮向前
  FTM_PWM_Duty(CFTM2, FTM_CH1, 0);   //通道高电压时右轮向后
  FTM_PWM_Duty(CFTM2, FTM_CH2, 5000);   //通道高电压时左轮向前
  FTM_PWM_Duty(CFTM2, FTM_CH3, 0);   //通道高电压时左轮向后
  */
  EnableInterrupts;     //开启总中断
  //TestLED();//测试GPIOmini核心板RGB灯珠颜色闪烁
  //Test_KEY(); //测试GPIO按键功能
  //Test_KEY1(); //测试GPIO拨码开关功能
  //TestADC();//测试ADC转换并通过OLED显示和UART发出
  //TestKBI();//测试GPIO外部中断 母版上K0，K1按键有效
  //TestPIT();//测试PIT定时中断
  //TestFTM_PWM();//测试PWM/OLED及电机控制功能
  //TestUART012();//测试UART012发送及接收中断
  //TestFTM_InputCapture();//测试编码器脉冲采集功能
  //TestMPU6050();
  //Test_2100_8700();//测试龙邱九轴传感器
  //TestRTC();//LED闪烁，同时OLED上显示时间，单位秒
  

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



