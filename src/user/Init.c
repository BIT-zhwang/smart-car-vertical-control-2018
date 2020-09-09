#include "init.h"
#include "include.h"

//初始化各硬件管脚
void Init_Hardwares(void)
{
  //初始化8700_2100加陀板
  Init_8700_2100();
  //延时等待1000MS
  time_delay_ms(2000);
  //静态采样进行滤波
  Sample_Static_87002100();
  //使能PWM
  PWM_Iint(); 
  //使能ECLK
  ECLK_Capture_Init();
  //使能串口
   UART_Init();
   //使能ADC
   Inductance_ADC_Init(); 
 //  gpio_init (PTD2, GPO,HIGH);
   //初始化停车干簧管
   GHG_Init();
   //初始化拨码开关
   Boma_Init();
   //初始化按键开关
   Key_Init();
   //初始化OLED显示屏
   LCD_Init();
   //初始化蜂鸣器
   Buzzer_Init();
   //读取拨码开关状态
   Read_Boma();
   //初始化Flash
   //FLASH_Init();
   FLASH_Init1();
   //初始化LED指示灯
   gpio_init (PTA1, GPO,HIGH);
   //延时等待1000MS
 //  time_delay_ms(1000);
}


/***            获取加陀板数据          ***/
void Get_AccANDGyro_Data(void)
{    /*   
   uint8_t bit8_data[6];
   short bit16_data16[3];
   Read_2100_8700(FXOS_8700_ADDR,bit8_data,bit16_data16);
   accle_x= bit16_data16[0];
   accle_y= bit16_data16[1];
   accle_z= bit16_data16[2];
   //采集2100数据
   Read_2100_8700(FX_2100_ADDR,bit8_data,bit16_data16);//16位精度
   gyro_x= bit16_data16[0];
   gyro_y= bit16_data16[1];
   gyro_z= bit16_data16[2];*/
  Read_8700(FXOS_8700_ADDR,IIC_bit8_data,IIC_bit16_data16);//14位精度
//  accle_x= IIC_bit16_data16[0];
//  accle_y= IIC_bit16_data16[1];
  accle_z= IIC_bit16_data16[2];
  //采集2100数据
  Read_2100(FX_2100_ADDR,IIC_bit8_data,IIC_bit16_data16);//16位精度
  gyro_x= IIC_bit16_data16[0];
  gyro_y= IIC_bit16_data16[1];
//  gyro_z= IIC_bit16_data16[2];
}

short Get_Gyro_X(void)       //获取陀螺仪X轴数据
{
  short temp = 0;
  Get_AccANDGyro_Data();
  temp = gyro_x;
  return temp;
}

short Get_Gyro_Y(void)       //获取陀螺仪Y轴数据
{
  short temp = 0;
  Get_AccANDGyro_Data();
  temp = gyro_y;
  return temp;
}

short Get_Gyro_Z(void)       //获取陀螺仪Z轴数据
{
  short temp = 0;
  Get_AccANDGyro_Data();
  temp = gyro_z;
  return temp;
}

uint16_t Get_Acc_X(void)        //获取加速度计X轴数据    
{
  uint16_t temp = 0;
  Get_AccANDGyro_Data();
  temp = accle_x;
  return temp;
}

uint16_t Get_Acc_Y(void)        //获取加速度计Y轴数据
{
  uint16_t temp = 0;
  Get_AccANDGyro_Data();
  temp = accle_y;
  return temp;
}

uint16_t Get_Acc_Z(void)        //获取加速度计Z轴数据
{
  uint16_t temp = 0;
  Get_AccANDGyro_Data();
  temp = accle_z;
  return temp;
}

void Sample_Static_87002100(void)       //初始化后对加陀板进行静态采样已保持稳定
{
  int i;
  int Static_Sum = 0;
  for(i=0; i<100; i++)
  {
     Gyro_X_Staic_Sum += Get_Gyro_X();   //由于电路和安装陀螺仪的X轴角速度为车身Z轴角速度
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
  //Angle.Gyro_X_Offset = (float)(Static_Sum/100.0f)*0.0625f;//获取实际角速度

  Angle.Gyro_X_Offset = (float)(Gyro_X_Staic_Sum/100.0f)*0.0625f;//获取实际角速度
  Gyro_X_Staic_Sum = 0;
  for(i=0; i<100; i++)
  {
     Gyro_X_Staic_Sum += Get_Gyro_Y();   //由于电路和安装陀螺仪的X轴角速度为车身Z轴角速度
     time_delay_ms(2);
  }
  Gyro_Y_Offset = (float)(Gyro_X_Staic_Sum/100.0f)*0.0625f;
  //Send_Data[4] = Angle.Gyro_X_Offset;
}

/***            done            ***/


/***            FTM模块初始化                ***/

//      PWM初始化
//      功能：使能PWM输出通道
//      管脚信息： 使用FTM2模块的CH0, CH1, CH2, CH3四个通道
//                 对应管脚为   PTC0,PTC1,PTC2,PTC3
//      占空比精度为5000，可以通过修改底层PRECISE参数进行修改
//      备注：对应车轮控制需要实际标定
void PWM_Iint(void)
{
  FTM_PWM_init(CFTM2, FTM_CH0,FTM_PTC0, 10*1000, 0);//PWM2 PTH0
  FTM_PWM_init(CFTM2, FTM_CH1,FTM_PTC1, 10*1000, 0);//PWM2 PTH1
  FTM_PWM_init(CFTM2, FTM_CH2,FTM_PTC2, 10*1000, 0);//PWM2 PTD0
  FTM_PWM_init(CFTM2, FTM_CH3,FTM_PTC3, 10*1000, 0);//PWM2 PTD1
}

//      ECLK外部时钟初始化
//      功能：使能编码器外部时钟
//      管脚信息：使用了 FTM0的ECLK0， FTM1的ECLK1
//                对应管脚为   PTE0          PTE7 
//      备注：编码器3管脚发送脉冲，4管脚发送高低电平，高电平表示编码器正转，低电平表示编码器反转 
void ECLK_Capture_Init(void)
{
  FTM_count_init(CFTM1);//右编码器输入用E7
  FTM_count_init(CFTM0);//左编码器输入用E0
  
  //gpio_init (PTH6, GPI,LOW);    //右侧编码器正反判断
  //gpio_init (PTC6, GPI,LOW);    //左侧编码器正反判断
  
}

/***            done            ***/


/***            AD接口初始化                ***/

//      AD转换接口初始化
//      功能：使能读取电感值所需要的ADC管脚
//      管脚信息:使用了ADC0模块的 5、   12、  13、 14、  15五个通道
//                   对应管脚为  PTB1  PTF4  PTF5  PTF6  PTF7
void Inductance_ADC_Init(void)     
{
  /*旧车
  ADC_Init(ADC_CHANNEL_AD15,ADC_12BIT); //左边电感
  ADC_Init(ADC_CHANNEL_AD14,ADC_12BIT);
  ADC_Init(ADC_CHANNEL_AD13,ADC_12BIT); //右边电感
  ADC_Init(ADC_CHANNEL_AD12,ADC_12BIT); //中间电感
  ADC_Init(ADC_CHANNEL_AD5,ADC_12BIT);
  */
  //新车1#
  ADC_Init(ADC_CHANNEL_AD6,ADC_12BIT);  //中间电感
  ADC_Init(ADC_CHANNEL_AD7,ADC_12BIT);  
  ADC_Init(ADC_CHANNEL_AD12,ADC_12BIT);  //左边电感
  ADC_Init(ADC_CHANNEL_AD13,ADC_12BIT);  //右间电感
//  ADC_Init(ADC_CHANNEL_AD12,ADC_12BIT);
  //12位分辨率
}

/***            done            ***/


/***            UART0串口初始化                ***/

//      UART0串口通信初始化
//      功能：使能UART0通信管脚
//      管脚信息：使用了UART0模块的RX,  TX 通道
//                   对应的管脚为 PTA2 PTA3 (REMAP)
//      波特率：115200

void UART_Init(void)
{
  uart_init(UARTR2,Remap,115200);
}

/***            done            ***/

/***            停车干簧管接口初始化                ***/

//      根据干簧管闭合与否判断是否停车
//      功能：使能干簧管信号检测管脚
//      管脚信息：
//                   对应的管脚为 PTE6
//                   低电平为正常，高电平为停车线

void GHG_Init(void)
{
  gpio_init (PTC6, GPI,LOW);    //设置为输入读取模式
}

void Boma_Init(void)
{
  gpio_init(PTG6,GPI,LOW);      //设置为输入读取模式 1
  gpio_init(PTG7,GPI,LOW);      //设置为输入读取模式 2
  gpio_init(PTG4,GPI,LOW);      //设置为输入读取模式 3
  gpio_init(PTG5,GPI,LOW);      //设置为输入读取模式 4
  gpio_init(PTF0,GPI,LOW);      //设置为输入读取模式 5
  gpio_init(PTF1,GPI,LOW);      //设置为输入读取模式 6
  gpio_init(PTD3,GPI,LOW);      //设置为输入读取模式 7
  gpio_init(PTD4,GPI,LOW);      //设置为输入读取模式 8
}

void Key_Init(void)//  按键控制初始化
{
  //五向开关
  gpio_init(PTH1,GPI,LOW);      //设置为输入读取模式
  gpio_init(PTI0,GPI,LOW);      //设置为输入读取模式
  gpio_init(PTI1,GPI,LOW);      //设置为输入读取模式
  gpio_init(PTI4,GPI,LOW);      //设置为输入读取模式
  gpio_init(PTH2,GPI,LOW);      //设置为输入读取模式
  
  //按键开关      
  gpio_init(PTH5,GPI,LOW);      //设置为输入读取模式  
  
}
void Buzzer_Init(void)
{
  gpio_init(PTI3,GPO,LOW);
}
/***            done            ***/

/***            AD接口初始化                ***/

//      拨码开关读取
//      功能：识别拨码开关的开关编码
//      管脚信息:使用了
//  PTG6      //设置为输入读取模式 1
//  PTG7      //设置为输入读取模式 2
//  PTG4      //设置为输入读取模式 3
//  PTG5      //设置为输入读取模式 4
//  PTF0      //设置为输入读取模式 5
//  PTF1      //设置为输入读取模式 6
//  PTD3      //设置为输入读取模式 7
//  PTD4      //设置为输入读取模式 8
void Read_Boma(void)     
{       
  int D0=0,D1=0,D2=0,D3=0,D4=0,D5=0,D6=0,D7=0;
  //读取各个开关状态
  if(gpio_get(PTG6))    D0=1;//前4位1234
  if(gpio_get(PTG7))    D1=2;
  if(gpio_get(PTG4))    D2=4;
  if(gpio_get(PTG5))    D3=8;
  
  //if(gpio_get(PTF0))    D4=1;//后4位5678
  if(gpio_get(PTF1))    D5=1;
  if(gpio_get(PTD3))    D6=2;
  if(gpio_get(PTD4))    D7=4;
  //将开关状态转换成十进制
  Boma_Num_Run=D0+D1+D2+D3;
  Boma_Num_Protect=D4+D5+D6+D7;
}

//蜂鸣器响
void DiDi(void)
{
  gpio_set(PTI3,1);
}

//关蜂鸣器
void ShutDown_DiDi(void)
{
  gpio_set(PTI3,0);
}
/***            done            ***/

//读取干簧管电平
void Read_GHG(void)
{
  GHGFlag=gpio_get(PTC6);
}

