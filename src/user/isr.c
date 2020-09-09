#include "isr.h"
#include "Parameter.h"
#include "Init.h"
#include "debug.h"
#include "Control.h"
//串口0接收中断服务例程
void UART0_ISR(void)
{
  uint8_t ReData;
  uint8_t txt[20];
  
  DisableInterrupts ;//关总中断
  
  ReData = Uart_GetChar(UARTR0);
  sprintf((char*)txt,"UART0_RX: %c \n",ReData);  
  Uart_SendString(UARTR0,txt);
  
  EnableInterrupts;   //开总中断
}


//串口1接收中断服务例程
void UART1_ISR(void)
{
  uint8_t ReData;
  uint8_t txt[20];
  
  DisableInterrupts ;//关总中断
  
  ReData = Uart_GetChar(UARTR1);
  sprintf((char*)txt,"UART1_RX: %c \n",ReData);  
  Uart_SendString(UARTR1,txt);
  
  EnableInterrupts;   //开总中断
}

//串口2接收中断服务例程

void UART2_ISR(void)
{
  uint8_t ReData;
  uint8_t txt[20];
  
  DisableInterrupts ;//关总中断
  
  ReData = Uart_GetChar(UARTR2);
  sprintf((char*)txt,"UART2_RX: %c \n",ReData);  
  Uart_SendString(UARTR2,txt);
  
  EnableInterrupts;   //开总中断
  
  ;
}

//定时器0中断函数

int Speed_Control_Count=0;
int DirectionControlCount=0;
void PIT0_ISR(void)
{
    PIT->CHANNEL[0].TFLG |= PIT_TFLG_TIF_MASK;//清楚中断标志位
    float fSpeedValue;
    float fDirectionValue;
    Speed_Control_Count++;
    DirectionControlCount++;
    Get_Sensors_Data();
    Get_Angle_Data();
    Get_Speed_Data();
    Get_Inductor_Data();
    //gpio_set(PTI3, 1);
    if(Car_Speed_Set < 2.4)
    {
      DIR_CLOSED_P=10.0;
      DIR_CLOSED_D=5;
      //角度环
      Angle_Kp = -1000;//1.5-->-1000   2.0-->-1000  -700   -800
      Angle_Kd = -9.0;//1.5-->-9         2.0-->-9    -6    -7
      //速度环
      Speed_Kp = 35;//1.5-->30         2.0-->25
      Speed_Ki = 0.25;//1.5-->0.25      2.0-->0.2
    }
    
    
    if(WriteNormFlag==1)
    {
      if(WriteFlag==0)
      {
        WriteFlag=1;
        Auto_Inductance_Get(L.Left_Voltage_Filter,L.Right_Voltage_Filter);
        if(NormCount<50) WriteFlag=0;
      }
      if(WriteFlag==1 && WriteEndFlag==0)
      {
        WriteEndFlag=1;
        Auto_Normalization();
      }
    }
    //电感数据采集
    if(WriteEndFlag==1) gpio_set(PTI3, 1);
    else gpio_set(PTI3, 0);
    
    Normalization();
    if(RingFlag==1) Circle_Test();//圆环检测
    if(BomaRingFlag==1) Boma_Circle_Test();//拨码圆环配置
    //方向控制4ms一次
    if(DirectionControlCount==2)
    {
      Car_Direction_Control_Output();
      //在这里进行赵琳的方法
      //Car_Direction_Control_Output_Plus();
      fDirectionValue=g_fDirectionControlOutNew-g_fDirectionControlOutOld;
      DirectionControlCount=0;
    }
    Dir_Control_Output=fDirectionValue * (DirectionControlCount + 1.0) / 2.0 + g_fDirectionControlOutOld;
    
    if(Speed_Control_Count==25)//速度控制50ms一次 20
    { 
      Car_Speed_Control_Output();
      fSpeedValue = g_fSpeedControlOutNew - g_fSpeedControlOutOld;
      Speed_Control_Count=0;
    }
    g_fSpeedControlOut = fSpeedValue * (Speed_Control_Count + 1.0) / 25.0 + g_fSpeedControlOutOld; 
    
    Go();
    if(TrueLength>3.0&&GHGFlag==0) 
    {
      Read_GHG();
      if(GHGFlag==1) TrueLengthOld=TrueLength;
    }
    //if(GHGFlag==1) Car_Speed_Set=0;
    Send_Data[6] = TrueLength;
    Angle_Output(Angle.Car_Z);
    Car_Protect();
    Motor_Output();
    T_count++;
    LCD_Show[4]=(int)(Angle.Final_Integral_Angle);
    LCD_Show[5]=(int)(pianyi);
    LCD_Show[6]=(int)(left*1000);
    LCD_Show[7]=(int)(right*1000);
    //LCD_Show[12]=
}

//定时器1中断函数
void PIT1_ISR(void)
{
  PIT->CHANNEL[1].TFLG |= PIT_TFLG_TIF_MASK;//清楚中断标志位
 
  Get_Sensors_Data();
  Get_Angle_Data();
  Angle_Output(Angle.Car_Z);
}



//KBI0中断函数
void KBI0_Isr(void)	
{  
  KBI0->SC |= KBI_SC_KBACK_MASK;       /* clear interrupt flag */
  uint16_t n = PTn(KBI_PTB5) ;   //PTA0引脚触发中断 
  if(KBI0->SP &(1<<n))
  {
    //用户代码 
    LED_Ctrl(LED0, LEDRVS);             
  } 
}

//KBI1中断函数
void KBI1_Isr(void)	
{  
  KBI1->SC |= KBI_SC_KBACK_MASK;                /* clear interrupt flag */
  
  uint16_t n = PTn(KBI_PTH2) ;   //PTH2引脚触发中断 
  if(KBI1->SP &(1<<n))
  {
    //用户代码 
    LED_Ctrl(LED1, LEDRVS);             
  }
}


/*****************************************************************************//*!
*
* @brief  FTM0_Isr interrupt service routine.
*        
* @param  none.
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*****************************************************************************/
void FTM0_IRQHandler(void)
{
  /* clear the flag */
  FTMx[CFTM0]->SC &= ~FTM_SC_TOF_MASK;
  
}

/*****************************************************************************//*!
*
* @brief  FTM1_Isr interrupt service routine.
*        
* @param  none. 
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*****************************************************************************/
void FTM1_IRQHandler(void)
{
  /* clear the flag */
  FTMx[CFTM1]->SC &= ~FTM_SC_TOF_MASK;
}

/*****************************************************************************//*!
*
* @brief  FTM2_Isr interrupt service routine.
*        
* @param  none. 
*
* @return none.
*
* @ Pass/ Fail criteria: none.
*
*****************************************************************************/

void FTM2_IRQHandler(void)
{
  /* clear the flag */
  FTMx[CFTM2]->SC &= ~FTM_SC_TOF_MASK;
}
