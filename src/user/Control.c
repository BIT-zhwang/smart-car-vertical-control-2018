#include "control.h"
#include "Parameter.h"
#include "Init.h"
/***            关于控制的一些定义             ***/

float DirP_Field[7]=
{
    -30,-20,-10,0,10,20,30
};

float DirD_Field[7]=
{
    -1.5,-1,-0.5,0,0.5,1,1.5
};//位置偏差微分范围

float DirP_Membership[7];//位置偏差隶属度,Kd_P
float DirD_Membership[7];//位置偏差微分隶属度,kd_D

float T_Int = 2.0; //控制周期，单位为毫秒

float Circle_Ratio  = 0.000150943f;//圆周参数  从脉冲单位换算成实际距离单位 1个脉冲对应xx米
//【CircleRatio】 = 【(PI*65/1000)/X】; //理论计算:周长/1000(米换算) * PI=周长 / 一圈脉冲量程-->X线  0.00017
//D车 0.000174
//E车 0.00015042
//电机输出限幅
int Motor_Output_Max = 5000;
int Motor_Output_Min = -5000;
/***            done            ***/

/***            车身角度控制部分                ***/
void Get_Angle_Data(void)       //获取车身角度、角速度信息
{
  //互补滤波
  float Delta_Angular_Speed_Z = 0;
  //Angle.Balance_Zero = 3000.0f;    //车子处于机械零点的时候加速度计读数（用于补偿）
  
  //由于加速度计采值正负不均匀 【0 ~  90】对应【0 ~ -15500】 
  //                         【0 ~ -90】对应【0 ~ 17000】
  //处理如下：
  //            将32500均分到180°内 对两侧均进行补偿
  if(Acc_Z_Angle < 0)
    Angle.Z = (Acc_Z_Angle - Angle.Balance_Zero) * -0.0058064f;
  else
    Angle.Z = (Acc_Z_Angle - Angle.Balance_Zero) * -0.0052941f;//-0.0052941f
  //Angle.Accle_Z_Data = Acc_Z_Angle - 750 ; //获取车身Z轴角度
  //Angle.Z = (Angle.Accle_Z_Data - Angle.Balance_Zero ) * -0.005538461f;   //将加速度计Z的数据转换成车辆实际Z轴角度
                                                              /* 180° / 32500 */                            

  Angle.Angular_Velocity_Z = (Gyro_Data - Angle.Gyro_X_Offset) * 1.4f;//1# 1.2
  //Angle.Angular_Velocity_Z = Gyro_Data * 5.1f;
  Angle.Car_Z = Angle.Final_Integral_Angle;     //车身角度及为积分后的加陀板角度
  
  Delta_Angular_Speed_Z = (Angle.Z - Angle.Car_Z) / 1;       //0.07
  //Delta = Delta_Angular_Speed_Z;
  
  Angle.Final_Integral_Angle += (Angle.Angular_Velocity_Z + Delta_Angular_Speed_Z)/500;
  //互补滤波的在积分后所得到的最终结果
  //uint16_t a[8];
  //a[1] = (uint16_t)Angle.Final_Integral_Angle;
  
 Send_Data[0] = Angle.Final_Integral_Angle;//发送至上位机查看数据
 //Send_Data[1] = Angle.Z;
 //Send_Data[2] = (float)Angle.Angular_Velocity_Z;
}

/***            done            ***/


/***            电感值获取及处理                ***
*       功能：读取赛道电感值并进行滤波、归一化     */

void Inductance_Inertia_Filter(void)    //对电感值进行惯性滤波
{
  //中间电感滤波
  L.Middle_Voltage_Filter  = (uint16_t) ((0.8 * L.Middle_Voltage_Filter) + (0.2 * L.Middle_Voltage_Temp));
  L.Middle_Voltage_Temp = L.Middle_Voltage;
  //左边电感滤波
  L.Left_Voltage_Filter = (uint16_t) ((0.8 * L.Left_Voltage_Filter) + (0.2 * L.Left_Voltage_Temp));
  L.Left_Voltage_Temp = L.Left_Voltage;
  //右边电感滤波
  L.Right_Voltage_Filter = (uint16_t) ((0.8 * L.Right_Voltage_Filter) + (0.2 * L.Right_Voltage_Temp));
  L.Right_Voltage_Temp = L.Right_Voltage;
  //左边圆环电感滤波
  L.Left_Ring_Voltage_Filter = (uint16_t) ((0.8 * L.Left_Ring_Voltage_Filter) + (0.2 * L.Left_Ring_Voltage_Temp));
  L.Left_Ring_Voltage_Temp = L.Left_Ring_Voltage;
  //右边圆环电感滤波
  L.Right_Ring_Voltage_Filter = (uint16_t) ((0.8 * L.Right_Ring_Voltage_Filter) + (0.2 * L.Right_Ring_Voltage_Temp));
  L.Right_Ring_Voltage_Temp = L.Right_Ring_Voltage;
}


void Inductance_Ave_Filter(int N_Times) //对电感值进行均值滤波 变量为样本容量
{
    int Sum_Middle = 0, Sum_Left = 0, Sum_Right = 0;
    for(int i = 0; i < N_Times; i++)
    {
        Sum_Middle += adc_once(ADC_CHANNEL_AD15,ADC_12BIT);
        Sum_Left += adc_once(ADC_CHANNEL_AD14,ADC_12BIT);
        Sum_Right += adc_once(ADC_CHANNEL_AD13,ADC_12BIT);
    }
    L.Middle_Voltage_Ave = (uint16_t) Sum_Middle / N_Times;
    L.Left_Voltage_Ave = (uint16_t) Sum_Left / N_Times;
    L.Right_Voltage_Ave = (uint16_t) Sum_Right / N_Times;
}

void Get_Inductor_Data(void)    //采集电感值并进行处理
{
  L.Left_Ring_Voltage = adc_once(ADC_CHANNEL_AD12,ADC_12BIT);
  L.Right_Ring_Voltage = adc_once(ADC_CHANNEL_AD13,ADC_12BIT);
  L.Left_Voltage = adc_once(ADC_CHANNEL_AD6,ADC_12BIT);
  L.Right_Voltage = adc_once(ADC_CHANNEL_AD7,ADC_12BIT);
  //Inductance_Ave_Filter(10);    //进行均值滤波
  Inductance_Inertia_Filter();   //进行惯性滤波
  Send_Data[4] = L.Left_Voltage_Filter;
  Send_Data[5] = L.Right_Voltage_Filter;
  //Send_Data[4] = L.Left_Ring_Voltage_Filter;
  //Send_Data[5] = L.Right_Ring_Voltage_Filter;
}

/***            done            ***/

/***            赛道电流计算部分          ***/
void CurrentDetect(void)
{
  /*
  float MidCurrent[10];//检测-4-14度之间的电流
  MidCurrent[0]=0;   //-4度电感
  MidCurrent[1]=0;   //-2度电感
  MidCurrent[2]=0;    //0度电感
  MidCurrent[3]=0;    //2度电感
  MidCurrent[4]=0;   //4度电感
  MidCurrent[5]=0;   //6度电感
  MidCurrent[6]=0;   //8度电感
  MidCurrent[7]=0;   //10度电感
  MidCurrent[8]=0;   //12度电感
  MidCurrent[9]=0;   //14度电感
  float Test[10];
  */
  
}

/***            速度计算部分          ***/
void Get_ArcLength(Disdance_ArcLength *dis, float Unit, float circle_ratio)     //将编码器脉冲数转换成实际距离
{
  dis->arcLengthBuffer = (float)dis->PulseCount * Unit * circle_ratio;  
  if(dis->arcLengthBuffer > 1)
  {     
    dis->arcLengthBuffer = 0.0f;
    dis->PulseCount = 0.0f;
    dis->meter += Unit;
  }
  dis->arcLength = dis->meter + dis->arcLengthBuffer;
  
  return;
}
/*
void Judge_Left_Wheel_Dir(void)
{
  Left_Wheel_Dir = gpio_get(PTC6);       //读取左轮方向
}


void Judge_Right_Wheel_Dir(void)
{
  Right_Wheel_Dir = gpio_get(PTH6);       //读取右轮方向
}
*/
void Get_Speed_Data(void)  //获取车辆速度信息   ！！！左轮右轮需要具体标定 ！！！
{
  //float Spd_Temp = 0;
  //测距验证部分
  Left_Arc.PulseCount += FTM_count_get(CFTM0);
  Right_Arc.PulseCount += FTM_count_get(CFTM1);
  Get_ArcLength(&Left_Arc, 1.0, Circle_Ratio);
  Get_ArcLength(&Right_Arc, 1.0, Circle_Ratio);
  Left_Distance = Left_Arc.arcLength;
  Right_Distance = Right_Arc.arcLength;
  TrueLength=(Left_Distance + Right_Distance)/2;
  /*
  if(Left_Wheel_Dir != 0)       
    Left_Spd.PulseCount = FTM_count_get(CFTM0);  
  else
    Left_Spd.PulseCount = - FTM_count_get(CFTM0);
  */
  Left_Spd.PulseCount = FTM_count_get(CFTM0); 
  
  //获取右轮在2ms控制周期内的脉冲数
  /*
  if(Right_Wheel_Dir != 0)
    Right_Spd.PulseCount = FTM_count_get(CFTM1); 
  else
    Right_Spd.PulseCount = FTM_count_get(CFTM1);
  */
  Right_Spd.PulseCount = FTM_count_get(CFTM1); 
  
  //获取编码器读数后就将其清空
  FTM_count_clean(CFTM0);       
  FTM_count_clean(CFTM1);
  
  //1.获取一个控制周期内走过的实际距离
  Get_ArcLength(&Left_Spd, 1.0, Circle_Ratio);   //获取左轮在2ms控制周期内走过的实际距离
  
  //2.计算该距离对应的速度（左轮）
  Speed._Left_Wheel = (float)Left_Spd.arcLength * 1000 / T_Int;
  
  //3.清空缓存数据（编码器计数等）
  //Left_Spd.arcLength = 0;
  //Left_Spd.arcLengthBuffer = 0;
  Left_Spd.meter = 0;
  Left_Spd.PulseCount = 0;
  
  //1.获取一个控制周期内走过的实际距离
  Get_ArcLength(&Right_Spd, 1.0, Circle_Ratio);   //获取右轮在2ms控制周期内走过的实际距离
  //2.计算该距离对应都速度（右轮）
  Speed._Right_Wheel = (float) Right_Spd.arcLength * 1000 / T_Int;
  
  //3.清空缓存数据（编码器计数等）
  //Right_Spd.arcLength = 0;
  //Right_Spd.arcLengthBuffer = 0;
  Right_Spd.meter = 0;
  Right_Spd.PulseCount = 0;
  //空转保护
  if(Speed._Left_Wheel >5.0 && Speed._Right_Wheel>5.0 && ProtectEnableSPD)   //加入了空转保护使能ProtectEnableSPD
  {
    ProtectCount2++;
  }
  else
  {
    ProtectCount2=0;
  }
  if(ProtectCount2>200) ProtectFlag=1;
  //通过左右轮子速度的中值来代表车辆速度值
  Speed._Car = (Speed._Left_Wheel + Speed._Right_Wheel) / 2.0f;
  
  /***          对车辆速度变化率进行限幅滤波          ***/
  if(Car_Speed_Set<2.4) Speed._GrowLimit = (2.0f/1000.0f) * T_Int;
  else Speed._GrowLimit = (3.0f/1000.0f) * T_Int;      //限制幅度 = 期望速度 / 1000 * 控制周期（ms）
  //Speed._GrowLimit = 1  ;
  if((Speed._Car - Speed._2ms_True) > Speed._GrowLimit)
      Speed._2ms_True += Speed._GrowLimit;
  else if((Speed._Car - Speed._2ms_True) < -Speed._GrowLimit)
      Speed._2ms_True -= Speed._GrowLimit;
  else
      Speed._2ms_True = Speed._Car;
  
  //Send_Data[2] = Speed._Car;
  Send_Data[1] = Speed._2ms_True*100;
  //Send_Data[4] = Speed._Right_Wheel;
  /***          done            ***/
}



/***            done            ***/


/***            读取并处理传感器数据      ***/

void Get_Sensors_Data(void)     
{
   Get_AccANDGyro_Data();       //获取加陀板中所有数据
   Acc_Z_Angle = accle_z;       //获取此时加速度计Z轴读数
   Gyro_Data = gyro_x * 0.0625f;     //获取此时陀螺仪X轴角速度
   Gyro_Dir_Data= gyro_y*0.0625f - Gyro_Y_Offset;  
}

/***            done            ***/


/***            直立车角度计算输出部分             ***/
void Angle_Output(float Current_Angle)  //角度环计算输出
{
  float Car_Angle,Car_Angle_Speed,Car_Angle_Set = 0;
  float Car_Angle_Speed_Set = 0;
  float Angle_Delta,Angle_Speed_Delta;
  float AngUpLimit = -6.0;       //减速时的角度变化限幅
  float AngDownLimit = 3.0;      //加速时的角度变化限幅
  float AngMax=3;
  float AngMin=-3;
  if(Car_Speed_Set<2.4) AngUpLimit=-3;
    
  /*
  //角度变化率限幅
  float CAR_ANGLE_Growlimit=0.002 ;
  
  if( ( Car_Angle_Temp - Speed_Control_Output ) > CAR_ANGLE_Growlimit )
   {
     Speed_Control_Output += CAR_ANGLE_Growlimit;
     Car_Angle_Temp = Speed_Control_Output;
     
   }
  else if( (Speed_Control_Output - Car_Angle_Temp) <  CAR_ANGLE_Growlimit )
    {   
      Speed_Control_Output -= CAR_ANGLE_Growlimit;
      Car_Angle_Temp = Speed_Control_Output;
    }
  else 
    {
      Car_Angle_Temp = Speed_Control_Output;
    }
  */
  
  //变量赋值
  Car_Angle = Current_Angle;
  //Car_Angle_Set = 0;
  Car_Angle_Set = g_fSpeedControlOut;
  Car_Angle_Speed = Angle.Angular_Velocity_Z;
  /******               起跑加速策略          ******/
  if(Left_Distance<1.0 && Right_Distance<1.0 && Speed._2ms_True<2.0&&SpeedUpEnable==1)
  {
    AngMax=15;
  }
  //角度限幅
  if(Car_Angle_Set > AngMax)
    Car_Angle_Set = AngMax;
  if(Car_Angle_Set < AngMin)
    Car_Angle_Set = AngMin;
  
  if(GHGFlag==1) Car_Angle_Set=-20;
  /******               done            ******/
  
  //         误差计算            
  Angle_Speed_Delta = Car_Angle_Speed_Set - Car_Angle_Speed;
  Angle_Delta = Car_Angle_Set - Car_Angle; 
  
  
  //角度变化限幅
  if(Angle_Delta > AngDownLimit)
    Angle_Delta = AngDownLimit;
  if(Angle_Delta < AngUpLimit)
    Angle_Delta = AngUpLimit;
  
 //Send_Data[1]= Angle_Delta;
 /*  
 float Angle_Delta_Temp = 0;
 if((Angle_Delta - Angle_Delta_Temp) > CAR_ANGLE_Growlimit)
    Angle_Delta += CAR_ANGLE_Growlimit;
 if((Angle_Delta - Angle_Delta_Temp) < CAR_ANGLE_Growlimit)
    Angle_Delta -= CAR_ANGLE_Growlimit;
  */
      
  
  //角度输出
  Angle.Output = Angle_Kp * Angle_Delta + Angle_Kd * Angle_Speed_Delta;
  //Send_Data[3]= Angle_Kp * Angle_Delta;
  //Send_Data[4] = Angle_Kd * Angle_Speed_Delta;
  //Send_Data[5] = Angle.Output;
}

/***            done            ***/

/***            车身速度环计算输出部分             ***/
void Car_Speed_Control_Output(void)     //车身速度计算输出
{
  float Car_Speed = 0;
  float Car_Speed_Delta = 0;
  float alpha=1.0;
  float absDelta;
  Car_Speed = Speed._2ms_True;
  /******               起跑加速策略          ******/
  //误差计算
  Car_Speed_Delta = Car_Speed_Set - Car_Speed;
  
  
  //变速积分
  //求取误差绝对值
  if(Car_Speed_Delta>0) absDelta = Car_Speed_Delta;
  else absDelta = -Car_Speed_Delta;
  //根据不同误差等级，调整积分权值
  if(absDelta>0.5) alpha = 0;
  if(absDelta<0.1) alpha = 1;
  if(absDelta>0.1 && absDelta<0.5) alpha=(0.5-absDelta)/(0.5-0.1);
  
  
  Car_Speed_Integral += Car_Speed_Delta * Speed_Ki * alpha ;
  //速度积分限幅
  if(Car_Speed_Integral > 3)
    Car_Speed_Integral = 3;
  if(Car_Speed_Integral < -3)
    Car_Speed_Integral = -3;
  
  //速度输出计算
  Speed_Control_Output = Speed_Kp * Car_Speed_Delta + Car_Speed_Integral;
  g_fSpeedControlOutOld=g_fSpeedControlOutNew;
  g_fSpeedControlOutNew=Speed_Control_Output;
  
}
/***            done            ***/

/***                归一化部分                  ***/
void Normalization(void)
{
    float rMax=0,lMax=0;
    
    int kr=60,br=2200;//左右归一到0.5
    int kl=60,bl=2400;
    //int km=30,bm=3700;
    //int bl2=544,br2=512;
    //uint8 sector = FLASH_SECTOR_NUM - 1;
    
    rMax=Angle.Car_Z*kr+br;//43000
    if(rMax>8000) rMax=8000;
    if(rMax<200) rMax=200;
    
    lMax=Angle.Car_Z*kl+bl;
    if(lMax>8000) lMax=8000;
    if(lMax<200) lMax=200;
    /*
    mMax=Angle.Car_Z*km+bm;
    if(mMax>4400) mMax=4400;
    if(mMax<1000) mMax=1000;
    */
    leftOld=left;
    rightOld=right;
    left = (L.Left_Voltage_Filter/lMax);
    right = (L.Right_Voltage_Filter/rMax);
    
    if(ReadNormFlag==1)
    {
      /*
      //kl = flash_read(sector,0,uint16);                         //读取数据
      //bl2 = flash_read(sector,0,uint16);
      //kr = flash_read(sector,8,uint16);                         //读取数据
      //br2 = flash_read(sector,4,uint16);
      //IK=((float)bl2/544.0+(float)br2/512.0)/2.0;
      //if(IK>1.2) IK=1.2;
      //if(IK<0.8) IK=0.8;
      L.Right_Ring_Voltage_Filter /= IK;
      L.Left_Ring_Voltage_Filter /= IK;
      L.Right_Voltage_Filter /= IK;
      L.Left_Voltage_Filter /= IK;
      */
      left=left*IK;
      right=right*IK;
    }
    
    Send_Data[2] =left*1000;
    Send_Data[3] =right*1000;
    
    
    //Send_Data[2] = kl ;
    //Send_Data[3] = kr;
    //Send_Data[4] = bl;
    //Send_Data[5] = br;
}
/***            done            ***/

/***                圆环检测部分                  ***/
void Circle_Test(void)
{
    //CircleMidLimit = 0.9;
    float CircleUpLeftLimit=0.7;//0.7
    float CircleUpRightLimit=0.70;//0.7
    float CircleDownLeftLimit=1.1;//
    float CircleDownRightLimit=1.1;
    //动态阈值
    if(Car_Speed_Set > 2.4&&Car_Speed_Set < 2.6)
    {
      CircleDownLeftLimit=1.1;//1.1晚了zuozhuan完了，右转早（但是圆环很不好）
      CircleDownRightLimit=1.1;
    }
    if(ZaoFlag==1) 
    {
      CircleDownLeftLimit-=0.1;//2.5-->1.1 2.0-->1.0
      CircleDownRightLimit-=0.1;
    }
    if(WanFlag==1)
    {
      CircleDownLeftLimit+=0.1;//2.5-->1.1 2.0-->1.0
      CircleDownRightLimit+=0.1;
    }
    //如果出现圆环初始条件，开启圆环检测计时
    if(left > CircleUpLeftLimit && right > CircleUpRightLimit)
    {
      CircleCount++;
    }
    else
    {
      if(CircleCount<10) CircleCount=0;
    }
    //结束条件
    if(CircleCount>=10 && CircleEndFlag==0 && (left-leftOld)<0 && (right-rightOld)<0 && left<CircleDownLeftLimit && right<CircleDownRightLimit)
    {
      CircleEndCount++;
      if(CircleEndCount==3)//2.5速度之前
      {
         CircleEndFlag=1;//电感信号消失，开启结束Flag
         LengthDelta=TrueLength-TrueLengthOld;//圆环结束，采集到圆环的半径信息
      }
    }
    else
      CircleEndCount=0;
    
    //在圆环Flag开启时，确定一下开始的Length
    if(CircleCount==10)
    {
      TrueLengthOld=TrueLength;
    }
    //确定开启圆环Flag
    if(CircleCount>=10) CircleStartFlag=1;
    //采集左右电感平均值
    if(CircleStartFlag==1 && CircleEndFlag==0 && DirectionJudge==0)
    {
      if(L.Left_Ring_Voltage_Filter>3300) DirectionJudge=1;
      if(L.Right_Ring_Voltage_Filter>3300) DirectionJudge=-1;
      //Left_Ave_Diangan=(CircleCount-10)*Left_Ave_Diangan/(CircleCount-9)+L.Left_Ring_Voltage_Filter/(CircleCount-9);
      //Right_Ave_Diangan=(CircleCount-10)*Right_Ave_Diangan/(CircleCount-9)+L.Right_Ring_Voltage_Filter/(CircleCount-9);
    }
    //执行打弯操作
    if(CircleStartFlag==1 && CircleEndFlag==1 && (TrueLength-TrueLengthOld)<(LengthDelta + 0.5))//0.5指代执行时间
      CircleDoFlag=1;
    else
      CircleDoFlag=0;
    
      
    if(CircleEndFlag==1&&CircleStartFlag==1)
    {
      if(TrueLength > (TrueLengthOld + LengthDelta * 10 + 2))//6
      {
        CircleEndFlag=0;
        CircleStartFlag=0;
        CircleCount=0;
        TrueLengthOld=0;
        LengthDelta = 0;
        Left_Ave_Diangan=0;
        Right_Ave_Diangan=0;
        DirectionJudge=0;
        CircleEndCount=0;
      } 
    }
    
    if((TrueLength-TrueLengthOld)>8&&CircleStartFlag==1)
    {
      CircleEndFlag=0;
      CircleStartFlag=0;
      CircleCount=0;
      TrueLengthOld=0;
      LengthDelta = 0;
      Left_Ave_Diangan=0;
      Right_Ave_Diangan=0;
      DirectionJudge=0;
      CircleEndCount=0;
    }
    Send_Data[7] =LengthDelta*1000*DirectionJudge; //上位机检测圆环距离
        
       
}

void Boma_Circle_Test(void)
{
    float CircleUpLimit=0.8;//0.65
    
    //如果出现圆环初始条件，开启圆环检测计时
    if(left > CircleUpLimit && right > CircleUpLimit)
    {
      CircleCount++;
    }
    else
    {
      if(CircleCount<10) CircleCount=0;
    }
    
    //在圆环Flag开启时，确定一下开始的Length
    if(CircleCount==10)
    {
      TrueLengthOld=TrueLength;
      //20180717修改
      if(ZaoFlag==1)
      {
          TrueLengthOld+=0.07;//进早了延迟入环
      }
      if(WanFlag==1)
      {
          TrueLengthOld-=0.07;//进晚了提前入弯道
      }
    }
    
    //确定开启圆环Flag
    if(CircleCount>=10) CircleStartFlag=1;
    //执行打弯操作
    if(Car_Speed_Set<2.4)
    {
      if(CircleStartFlag==1 && (TrueLength-TrueLengthOld)< 0.82 &&(TrueLength-TrueLengthOld)> 0.32)//0.5指代执行时间
        CircleDoFlag=1;
      else
        CircleDoFlag=0;
    }
    
    if(Car_Speed_Set>2.4&&Car_Speed_Set<2.6)
    {
      if(CircleStartFlag==1 && (TrueLength-TrueLengthOld)< 0.85 &&(TrueLength-TrueLengthOld)> 0.35)//0.5指代执行时间
        CircleDoFlag=1;
      else
        CircleDoFlag=0;
    } 
    
    if(Car_Speed_Set > 2.6)
    {
      //CircleUpLimit=0.65;
      if(CircleStartFlag==1 && (TrueLength-TrueLengthOld)< 0.85 &&(TrueLength-TrueLengthOld)> 0.35)//0.5指代执行时间
        CircleDoFlag=1;
      else
        CircleDoFlag=0;
    } 
    
    if(CircleStartFlag==1)
    {
      if(TrueLength > (TrueLengthOld + 10))//4m
      {
        CircleStartFlag=0;
        CircleCount=0;
        TrueLengthOld=0;
      } 
    }
}
/***                    done                     ***/


void Calc_Membership_Without_CenterZone_7Gear(float value, float* membership, float* dividePoints,int NUM)
{
    int i;
    for (i = 0; i < NUM; i++)
        membership[i] = 0;
    if (value <= dividePoints[0])//现在值比最小还小
    {	    
        membership[0] = 1;
        //return 0;
    }
    else
    {
        if (value >= dividePoints[NUM-1]) //现在的值比最大还大
        { 		    
            membership[NUM-1] = 1;
        }
        else
        {
            for (i = 1; i < NUM; i++)
            {
                if (value <= dividePoints[i])//找到论域中小于当前值的点
                {
                    membership[i-1] = (dividePoints[i]-value) / (dividePoints[i] - dividePoints[i - 1]);
                    //当前点到右边点的距离/左右2点的总距离
                    membership[i] = 1 - membership[i-1];
                    //return i;
                    break;
                }
            }
        }
    }
}


/***            车身转向环计算输出部分             ***/
void Car_Direction_Control_Output(void) //方向环计算输出
{
    float fValue=0;
    float DIR_CONTROL_P=1000;//1000
    float DIR_CONTROL_K1=2000;//线性化方案1中间补偿500//线性化方案2中间补偿2000//改变补线方式
    float DIR_CONTROL_K2=2000;
    float buxian=0,buxian1=0,buxian2=0;
    float middle0=0;
    
    float angle;
    angle=Angle.Car_Z;//
    if(angle>15) angle=15;
    
    //闭环参数
    float qianzhan=33.0;//经测量38,为了能够提前入弯，人为修改前瞻35
    float xiebian2;
    float g_fTruejiaosudu,g_fCaljiaosudu;
    float fClosedValue,fDClosedValue;
    float DIR_CLOSED_K=50.5+0.9*angle;
    
    //middle0=middle;//补线方案1
    //if(middle0>1.0) middle0=1;
    
    int i,j;
    float tempP=0,tempD=0;
    
    middle0=right+left;
    //Send_Data[2] = middle0*1000;
    if(middle0>1.1) middle0=1.1;
    //if(middle0<0.3) middle0=0.3;
    //0.3是因为0.7×0.7=0.5，用于限制最大的转向，防止出赛道后狂甩
    fValue=left-right;
    fValue*=DIR_CONTROL_P;
    //Send_Data[4] = fValue;
    
    buxian1=DIR_CONTROL_K1*(1-middle0);
    buxian2=DIR_CONTROL_K2*(1.1-middle0)*(1.1-middle0);
    if(buxian1>=buxian2) buxian=buxian1;
    else buxian=buxian2;
    
    
    //使用左右电感做补线方案
    if(Angle.Car_Z<15&&Angle.Car_Z>-10)
    {
       if(fValue>0) fValue=fValue+buxian2;
       if(fValue<0) fValue=fValue-buxian2;
     }
    
     //Send_Data[5] = fValue;//补线后的电感偏移值
     
     //计算偏移量
     pianyiOld=pianyi;
     pianyi=fValue/DIR_CLOSED_K;
     if(pianyi>30) pianyi=30;
     if(pianyi<-30) pianyi=-30;
     //Car_Direction_Control_Output_Plus();
     
     if(Car_Speed_Set>2.4)
     {
       DIR_CLOSED_P=0;
       DIR_CLOSED_D=0;
       //模糊控制
       Calc_Membership_Without_CenterZone_7Gear(pianyi, DirP_Membership, DirP_Field,7);
       Calc_Membership_Without_CenterZone_7Gear((pianyi-pianyiOld), DirD_Membership, DirD_Field,7);
       for (i = 0; i < 7; i++)
        {
            if (DirD_Membership[i] != 0)//DirD为行，搜索的是列号
            {
                for (j = 0; j < 7; j++)
                {
                    if (DirP_Membership[j] != 0)//DirP为列，搜索的是行号
                    {
                        tempP = FuzzyKd_P[j][i] * DirP_Membership[j] * DirD_Membership[i];
                        tempD = FuzzyKd_D[j][i] * DirP_Membership[j] * DirD_Membership[i];
                        DIR_CLOSED_P += tempP;
                        DIR_CLOSED_D += tempD;
                     }
                }
            }
        }
     }
    
    //Send_Data[1] = DirD_Membership[0];
    //Send_Data[2] = DirD_Membership[1];
    //Send_Data[3] = DirD_Membership[2];
    //Send_Data[4] = DirD_Membership[3];
    //Send_Data[5] = DirD_Membership[4];
    //Send_Data[6] = DirD_Membership[5];
    //Send_Data[7] = DirD_Membership[6];
     
     //Send_Data[2] = pianyi ;
     //Send_Data[3] = (pianyi - pianyiOld);
     //Send_Data[4] =  DIR_CLOSED_P;
     //Send_Data[5] =  DIR_CLOSED_D;
     
     qulvOld=qulv;  
     xiebian2 = pianyi * pianyi + qianzhan * qianzhan;
     qulv=2*pianyi/xiebian2;
     if(pianyi>11.1) qulv=0.0013*pianyi+0.004;
     if(pianyi<-11.1) qulv=0.0013*pianyi-0.004;
     
     
     if(CircleDoFlag==1&&TrueLength>1.0)//执行时间1.5m->50  1m不检测圆环
     {
         if(DirectionJudge==1)
         {
           qulv=0.025;
           //qulv=0.015/LengthDelta;//0.14
           //if(qulv<0.015) qulv=0.015;//0.012
           //if(qulv>0.025) qulv=0.025;
         }
         
         if(DirectionJudge==-1)
         {
           qulv=-0.025;
           //qulv=-0.015/LengthDelta;
           //if(qulv>-0.015) qulv=-0.015;
           //if(qulv<-0.025) qulv=-0.025;
         }
         
         //DIR_CLOSED_P=30;
         //DIR_CLOSED_D=0;
     }
     
     
     //Send_Data[4] = qulv * 1000;
     
     /***         开环控制           ***/
     //g_fDirectionControlOutOld=g_fDirectionControlOutNew;
     //g_fDirectionControlOutNew=20*pianyi+100*(pianyi-pianyiOld); 
     //if(qulv<0.015&&qulv>-0.015&&Speed._2ms_True>2.0) qulv=0; 
     
     g_fCaljiaosuduOld = g_fCaljiaosuduNew;
     g_fCaljiaosudu = Speed._Car * qulv * 100 * 180/3.1416;
     g_fCaljiaosuduNew = g_fCaljiaosudu * 0.3 + g_fCaljiaosuduOld * 0.7;
     //100是m和cm的转换
     //60是角度和弧度的转换，其实应该是180/PI
     g_fTruejiaosuduOld = g_fTruejiaosuduNew;
     g_fTruejiaosudu=Gyro_Dir_Data;
     g_fTruejiaosuduNew = g_fTruejiaosudu * 0.3 + g_fTruejiaosuduOld * 0.7;
     //Send_Data[4] = g_fCaljiaosudu;
     //Send_Data[5] = g_fTruejiaosudu;
     //Send_Data[7] = g_fTruejiaosuduNew;
     //计算误差和误差微分
     fClosedValueOld = fClosedValueNew;
     //fClosedValue = g_fCaljiaosuduNew - g_fTruejiaosuduNew;  //惯性滤波算法
     fClosedValue = g_fCaljiaosudu - g_fTruejiaosudu;
     fClosedValueNew = fClosedValue;
     
     fDClosedValueOld = fDClosedValueNew;
     fDClosedValue = fClosedValueNew - fClosedValueOld;
     fDClosedValueNew = fDClosedValue;
     
     g_fDirectionControlOutOld = g_fDirectionControlOutNew;
     g_fDirectionControlOutNew = DIR_CLOSED_P * fClosedValue;//P控制
     //Send_Data[2] = DIR_CLOSED_P * fClosedValue;
     g_fDirectionControlOutNew += DIR_CLOSED_D * fDClosedValue;//D控制 
     //Send_Data[3] = DIR_CLOSED_D * fDClosedValue;
     g_fDirectionControlOutNew += 0.1 * DIR_CLOSED_D * fDClosedValueOld * DIR_CLOSED_D / DIR_CLOSED_P;//不完全微分
     //参数范围0.03-0.1，这个参数越小，持续时间越长；参数大，持续时间短
     //Send_Data[4]=DIR_CLOSED_P * fClosedValue;
     //Send_Data[5]=DIR_CLOSED_D * fDClosedValue;
     
     
}
/***            done            ***/



/***         提速策略           ***/
void Car_Direction_Control_Output_Plus(void)
{
  //赵琳的方法，由于转向闭环
  float v = 2.3;
  if(Speed._2ms_True>v&&(pianyi>10||pianyi<-10)) pianyi=pianyi*(1+(Speed._2ms_True-v)*0.5);
}
/***            done            ***/


/***        直道检测            ***/

void Straight_Judge(void)
{
    StraightFlagOld=StraightFlag;
    StraightFlag=0;
    if(qulv<0.011&&qulv>-0.011) StraightCount++;
    else StraightCount=0;
    if(StraightCount>50)
      StraightFlag=1;
    if(StraightFlagOld-StraightFlag==1) CurveFlag=1;
    //Send_Data[4] = StraightFlag;
}
/***            done            ***/



/***        入弯处理            ***/


/***            电机PWM输出部分               ***/
//      功能：发出PWM波给电机输出
//      管脚信息： 使用FTM2模块的CH0, CH1, CH2, CH3四个通道
//                 对应管脚为   PTH0,PTH1,PTD0,PTD1
//      占空比精度为5000，可以通过修改底层PRECISE参数进行修改
//      备注：对应车轮控制需要实际标定
//      PTH0、PTH1为一组    PTD0、PTD1为一组
//      每组之间的PWM应当互补
//      


void Set_PWM(int PWM_n,int value)       //设置PWM输出
{
  uint32 Value;
  Value =(uint32)(5000-value);

  if(Value>5000)//5000  3/15 16:24
  Value=5000;
    switch(PWM_n) 
    {
        case 1:   FTM_PWM_Duty(CFTM2, FTM_CH0, Value); break;    //右轮向前      
        case 2:   FTM_PWM_Duty(CFTM2, FTM_CH1, Value); break;    //右轮向前      
        case 3:   FTM_PWM_Duty(CFTM2, FTM_CH2, Value); break;    //左轮向后    
        case 4:   FTM_PWM_Duty(CFTM2, FTM_CH3, Value); break;    //左轮向前      
    }  
}

//Voltage > 0 : 正传;Voltage < 0，反转
void Set_Motor_Voltage(float f_Left_Voltage, float f_Right_Voltage) 	//设置电机旋转方向和速度	
{
  int nOut;
  if(f_Left_Voltage >5000) 		
    f_Left_Voltage =5000;
  else if(f_Left_Voltage < -5000) 	
    f_Left_Voltage = -5000;
	
  if(f_Right_Voltage > 5000) 	
    f_Right_Voltage = 5000;
  else if(f_Right_Voltage < -5000)	
    f_Right_Voltage = -5000;
            
  if(f_Left_Voltage > 0) //左前
  {
    Set_PWM(3, 0);
    nOut = (int)(f_Left_Voltage);
    Set_PWM(4, nOut);
  } 
  else             //左后
  {
    Set_PWM(4, 0);
    f_Left_Voltage = -f_Left_Voltage;
    nOut = (int)(f_Left_Voltage);
    Set_PWM(3, nOut);
  }                                     
  if(f_Right_Voltage > 0)   //右前
  {
    Set_PWM(1, 0);       //HIGH
    nOut = (int)(f_Right_Voltage);
    Set_PWM(2, nOut);
  } 
  else             //右后
  {
    Set_PWM(2, 0);
    f_Right_Voltage = -f_Right_Voltage;
    nOut = (int)(f_Right_Voltage);
    Set_PWM(1, nOut);
  }
        
}

void Car_Protect(void)
{
  if(L.Left_Voltage_Filter<10 && L.Right_Voltage_Filter<10 && ProtectCount<100 && ProtectEnableSGN && TrueLength>3.0) //加入了信号保护使能ProtectEnableSGN
    {
      ProtectCount++;
    }
    else
    {
      ProtectCount=0;
    }
    if(ProtectCount>50) ProtectFlag=1;
}

void Motor_Output(void) //计算电机输出量
{               
  float f_Left_Motor = 0;
  float f_Right_Motor = 0;
    //电机控制量计算
  if(GoFlag==1)
  {
    f_Left_Motor = Angle.Output -  Dir_Control_Output ;
    f_Right_Motor = Angle.Output +  Dir_Control_Output ; 
    //右拐是左-右+
    //f_Left_Motor = Angle.Output;
    //f_Right_Motor = Angle.Output;
    if(ProtectFlag==1)
    {
      f_Left_Motor = 0;
      f_Right_Motor = 0;
    }
    //if(GHGFlag==1&&((TrueLength-TrueLengthOld)>1.0||Speed._Car<2.0))
    if(GHGFlag==1)
    {
      f_Left_Motor = 0;
      f_Right_Motor = 0;
    }
  }
  //电机输出限幅 PWM满占空比是分子为5000
  if(f_Left_Motor > Motor_Output_Max)   //左电机
    f_Left_Motor = Motor_Output_Max;
  if(f_Left_Motor < Motor_Output_Min)
    f_Left_Motor = Motor_Output_Min;
  if(f_Right_Motor > Motor_Output_Max)   //右电机
    f_Right_Motor = Motor_Output_Max;
  if(f_Right_Motor < Motor_Output_Min)  
    f_Right_Motor = Motor_Output_Min;
  
  //电机死区补偿
  
  if(f_Left_Motor > 0)
    f_Left_Motor += 0;        //      左轮正转死区待标定
  if(f_Left_Motor < 0)
    f_Left_Motor -= 0;         //      左轮反转死区待标定
  if(f_Right_Motor > 0)
    f_Right_Motor += 270;        //      右轮正转死区待标定
  if(f_Right_Motor < 0)
    f_Right_Motor -= 0;         //      右轮反转死区待标定
  Set_Motor_Voltage(f_Left_Motor, f_Right_Motor);
}








/***            done            ***/


/*****************************************************************************************************/


/***           旧自动归一化         ***/


//float Angles[30]={-4,-3.5,-3,-2.5,-2,-1.5,-1,-0.5,0,0.5,1,1.5,2,2.5,3,3.5,4,4.5,5,5.5,6};
//float Inductances[30];
void Auto_Normalization(void)
{
  uint8 sector = FLASH_SECTOR_NUM - 1;
  /*
  uint16 x_bar,y_bar,xy_bar,x2_bar,x_bar2;
  uint16 x,y,xy,x2;
  uint16 kl,bl;
  uint16 kr,br;
  
  uint8 sector = FLASH_SECTOR_NUM - 1;
  
  for(int j=0;j<21;j++)
  {
    x+=Angles[j];
    y+=Left_Norm_Inductances[j];
    xy+=Angles[j]*Left_Norm_Inductances[j];
    x2+=Angles[j]*Angles[j];
  }
  x_bar=x/21;
  y_bar=y/21;
  xy_bar=xy/21;
  x2_bar=x2/21;
  x_bar2=x_bar*x_bar;
  kl=(xy_bar-x_bar*y_bar)/(x2_bar-x_bar2);
  bl=y_bar-kl*x_bar;
  
  x=0;
  y=0;
  xy=0;
  x2=0;
  
  for(int k=0;k<21;k++)
  {
    x+=Angles[k];
    y+=Right_Norm_Inductances[k];
    xy+=Angles[k]*Right_Norm_Inductances[k];
    x2+=Angles[k]*Angles[k];
  }
  x_bar=x/21;
  y_bar=y/21;
  xy_bar=xy/21;
  x2_bar=x2/21;
  x_bar2=x_bar*x_bar;
  kr=(xy_bar-x_bar*y_bar)/(x2_bar-x_bar2);
  br=y_bar-kr*x_bar;
  */
  uint16 bl,br;
  bl=(int)(NormLeft);
  br=(int)(NormRight);
  //FLASH_WriteSector(sector,(const uint8 *)&kl,2,0);
  FLASH_WriteSector(sector,(const uint8 *)&bl,2,0);
  //FLASH_WriteSector(sector,(const uint8 *)&kr,2,8);
  FLASH_WriteSector(sector,(const uint8 *)&br,2,4);
}

//将小车放置在赛道中央，同时采集左右两路电感，对左右电感进行归一化
void Auto_Inductance_Get(int Left_Voltage,int Right_Voltage)
{
  /*
  for(int i=0;i<21;i++)
  {
    if(Angle.Car_Z>(Angles[i]-0.1) && Angle.Car_Z<(Angles[i]+0.1))
    {
      if(Left_Voltage<4100&&Left_Voltage>0&&Right_Voltage<4100&&Right_Voltage>0)
      {
        Left_Norm_Inductances[i] = Left_Norm_Inductances[i] * Counts[i] /(Counts[i] + 1.0) + Left_Voltage /(Counts[i] + 1.0);
        Right_Norm_Inductances[i] = Right_Norm_Inductances[i] * Counts[i] /(Counts[i] + 1.0) + Right_Voltage /(Counts[i] + 1.0);
        Counts[i]++;
      }
    }
  }
  */
  if(Angle.Car_Z>-56.5 && Angle.Car_Z<-55.5)
  {
    if(Left_Voltage<4100&&Left_Voltage>0&&Right_Voltage<4100&&Right_Voltage>0)
    {
      NormLeft=(NormLeft * NormCount /(NormCount + 1.0) + Left_Voltage /(NormCount + 1.0));
      NormRight=(NormRight * NormCount /(NormCount + 1.0) + Right_Voltage /(NormCount + 1.0));
      NormCount++;
    }
  }   
}




/*********************************************************************/




//2m
void Set_Fussy_PTable(void)
{
//舵机P模糊参数uyt
  int m,n;
  
  for(m=0;m<7;m++)
  {
    if(m==0)//模糊表第1行
    {
      for(n=0;n<7;n++)
      {
        if(n==0) FuzzyKd_P[m][n]=13;
        if(n==1) FuzzyKd_P[m][n]=13;
        if(n==2) FuzzyKd_P[m][n]=13;
        if(n==3) FuzzyKd_P[m][n]=13;
        if(n==4) FuzzyKd_P[m][n]=13;
        if(n==5) FuzzyKd_P[m][n]=13;
        if(n==6) FuzzyKd_P[m][n]=13;
      }
    }
    
    if(m==1)//模糊表第2行
    {
      for(n=0;n<7;n++)
      {
        if(n==0) FuzzyKd_P[m][n]=13;
        if(n==1) FuzzyKd_P[m][n]=15;
        if(n==2) FuzzyKd_P[m][n]=13;
        if(n==3) FuzzyKd_P[m][n]=13;
        if(n==4) FuzzyKd_P[m][n]=13;
        if(n==5) FuzzyKd_P[m][n]=15;
        if(n==6) FuzzyKd_P[m][n]=13;
      }
    }
    
    if(m==2)//模糊表第3行
    {
      for(n=0;n<7;n++)
      {
        if(n==0) FuzzyKd_P[m][n]=13;
        if(n==1) FuzzyKd_P[m][n]=13;
        if(n==2) FuzzyKd_P[m][n]=17;
        if(n==3) FuzzyKd_P[m][n]=17;
        if(n==4) FuzzyKd_P[m][n]=17;
        if(n==5) FuzzyKd_P[m][n]=13;
        if(n==6) FuzzyKd_P[m][n]=13;
      }
    }
    
    if(m==3)//模糊表第4行
    {
      for(n=0;n<7;n++)
      {
        if(n==0) FuzzyKd_P[m][n]=13;
        if(n==1) FuzzyKd_P[m][n]=13;
        if(n==2) FuzzyKd_P[m][n]=13;
        if(n==3) FuzzyKd_P[m][n]=13;
        if(n==4) FuzzyKd_P[m][n]=13;
        if(n==5) FuzzyKd_P[m][n]=13;
        if(n==6) FuzzyKd_P[m][n]=13;
      }
    }
    
    if(m==4)//模糊表第5行
    {
      for(n=0;n<7;n++)
      {
        if(n==0) FuzzyKd_P[m][n]=13;
        if(n==1) FuzzyKd_P[m][n]=13;
        if(n==2) FuzzyKd_P[m][n]=17;
        if(n==3) FuzzyKd_P[m][n]=17;
        if(n==4) FuzzyKd_P[m][n]=17;
        if(n==5) FuzzyKd_P[m][n]=13;
        if(n==6) FuzzyKd_P[m][n]=13;
      }
    }
    
    if(m==5)//模糊表第6行
    {
      for(n=0;n<7;n++)
      {
        if(n==0) FuzzyKd_P[m][n]=13;
        if(n==1) FuzzyKd_P[m][n]=15;
        if(n==2) FuzzyKd_P[m][n]=13;
        if(n==3) FuzzyKd_P[m][n]=13;
        if(n==4) FuzzyKd_P[m][n]=13;
        if(n==5) FuzzyKd_P[m][n]=15;
        if(n==6) FuzzyKd_P[m][n]=13;
      }
    }
    
    if(m==6)//模糊表第7行
    {
      for(n=0;n<7;n++)
      {
        if(n==0) FuzzyKd_P[m][n]=13;
        if(n==1) FuzzyKd_P[m][n]=13;
        if(n==2) FuzzyKd_P[m][n]=13;
        if(n==3) FuzzyKd_P[m][n]=13;
        if(n==4) FuzzyKd_P[m][n]=13;
        if(n==5) FuzzyKd_P[m][n]=13;
        if(n==6) FuzzyKd_P[m][n]=13;
      }
    }
  }
}

void Set_Fussy_DTable(void)
{
//舵机P模糊参数uyt
  int m,n;
  for(m=0;m<7;m++)
  {
    if(m==0)
    {
      for(n=0;n<7;n++)
      {
        if(n==0) FuzzyKd_D[n][m]=5;
        if(n==1) FuzzyKd_D[n][m]=7;
        if(n==2) FuzzyKd_D[n][m]=5;
        if(n==3) FuzzyKd_D[n][m]=5;
        if(n==4) FuzzyKd_D[n][m]=5;
        if(n==5) FuzzyKd_D[n][m]=7;
        if(n==6) FuzzyKd_D[n][m]=5;
      }
    }
    
    if(m==1)
    {
      for(n=0;n<7;n++)
      {
        if(n==0) FuzzyKd_D[n][m]=5;
        if(n==1) FuzzyKd_D[n][m]=5;
        if(n==2) FuzzyKd_D[n][m]=5;
        if(n==3) FuzzyKd_D[n][m]=5;
        if(n==4) FuzzyKd_D[n][m]=5;
        if(n==5) FuzzyKd_D[n][m]=5;
        if(n==6) FuzzyKd_D[n][m]=5;
      }
    }
    
    if(m==2)
    {
      for(n=0;n<7;n++)
      {
        if(n==0) FuzzyKd_D[n][m]=5;
        if(n==1) FuzzyKd_D[n][m]=5;
        if(n==2) FuzzyKd_D[n][m]=5;
        if(n==3) FuzzyKd_D[n][m]=5;
        if(n==4) FuzzyKd_D[n][m]=5;
        if(n==5) FuzzyKd_D[n][m]=5;
        if(n==6) FuzzyKd_D[n][m]=5;
      }
    }
    
    if(m==3)
    {
      for(n=0;n<7;n++)
      {
        if(n==0) FuzzyKd_D[n][m]=5;
        if(n==1) FuzzyKd_D[n][m]=5;
        if(n==2) FuzzyKd_D[n][m]=5;
        if(n==3) FuzzyKd_D[n][m]=5;
        if(n==4) FuzzyKd_D[n][m]=5;
        if(n==5) FuzzyKd_D[n][m]=5;
        if(n==6) FuzzyKd_D[n][m]=5;
      }
    }
    
    if(m==4)
    {
      for(n=0;n<7;n++)
      {
        if(n==0) FuzzyKd_D[n][m]=5;
        if(n==1) FuzzyKd_D[n][m]=5;
        if(n==2) FuzzyKd_D[n][m]=5;
        if(n==3) FuzzyKd_D[n][m]=5;
        if(n==4) FuzzyKd_D[n][m]=5;
        if(n==5) FuzzyKd_D[n][m]=5;
        if(n==6) FuzzyKd_D[n][m]=5;
      }
    }
    
    if(m==5)
    {
      for(n=0;n<7;n++)
      {
        if(n==0) FuzzyKd_D[n][m]=5;
        if(n==1) FuzzyKd_D[n][m]=5;
        if(n==2) FuzzyKd_D[n][m]=5;
        if(n==3) FuzzyKd_D[n][m]=5;
        if(n==4) FuzzyKd_D[n][m]=5;
        if(n==5) FuzzyKd_D[n][m]=5;
        if(n==6) FuzzyKd_D[n][m]=5;
      }
    }
    
    if(m==6)
    {
      for(n=0;n<7;n++)
      {
        if(n==0) FuzzyKd_D[n][m]=5;
        if(n==1) FuzzyKd_D[n][m]=7;
        if(n==2) FuzzyKd_D[n][m]=5;
        if(n==3) FuzzyKd_D[n][m]=5;
        if(n==4) FuzzyKd_D[n][m]=5;
        if(n==5) FuzzyKd_D[n][m]=7;
        if(n==6) FuzzyKd_D[n][m]=5;
      }
    }
  }
}






/******************************************************************************************************/


//起跑
void Go(void)
{
  if(TrueLength<0.5)
  {
    if(L.Left_Voltage_Filter>5&&L.Right_Voltage_Filter>5&&L.Right_Ring_Voltage_Filter>5&&L.Left_Ring_Voltage_Filter>5)
      GoCount++;
    else
      GoCount=0;
  }
  if(GoCount>10) GoFlag=1;
}
