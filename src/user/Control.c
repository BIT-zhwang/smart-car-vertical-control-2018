#include "control.h"
#include "Parameter.h"
#include "Init.h"
/***            ���ڿ��Ƶ�һЩ����             ***/

float DirP_Field[7]=
{
    -30,-20,-10,0,10,20,30
};

float DirD_Field[7]=
{
    -1.5,-1,-0.5,0,0.5,1,1.5
};//λ��ƫ��΢�ַ�Χ

float DirP_Membership[7];//λ��ƫ��������,Kd_P
float DirD_Membership[7];//λ��ƫ��΢��������,kd_D

float T_Int = 2.0; //�������ڣ���λΪ����

float Circle_Ratio  = 0.000150943f;//Բ�ܲ���  �����嵥λ�����ʵ�ʾ��뵥λ 1�������Ӧxx��
//��CircleRatio�� = ��(PI*65/1000)/X��; //���ۼ���:�ܳ�/1000(�׻���) * PI=�ܳ� / һȦ��������-->X��  0.00017
//D�� 0.000174
//E�� 0.00015042
//�������޷�
int Motor_Output_Max = 5000;
int Motor_Output_Min = -5000;
/***            done            ***/

/***            ����Ƕȿ��Ʋ���                ***/
void Get_Angle_Data(void)       //��ȡ����Ƕȡ����ٶ���Ϣ
{
  //�����˲�
  float Delta_Angular_Speed_Z = 0;
  //Angle.Balance_Zero = 3000.0f;    //���Ӵ��ڻ�е����ʱ����ٶȼƶ��������ڲ�����
  
  //���ڼ��ٶȼƲ�ֵ���������� ��0 ~  90����Ӧ��0 ~ -15500�� 
  //                         ��0 ~ -90����Ӧ��0 ~ 17000��
  //�������£�
  //            ��32500���ֵ�180���� ����������в���
  if(Acc_Z_Angle < 0)
    Angle.Z = (Acc_Z_Angle - Angle.Balance_Zero) * -0.0058064f;
  else
    Angle.Z = (Acc_Z_Angle - Angle.Balance_Zero) * -0.0052941f;//-0.0052941f
  //Angle.Accle_Z_Data = Acc_Z_Angle - 750 ; //��ȡ����Z��Ƕ�
  //Angle.Z = (Angle.Accle_Z_Data - Angle.Balance_Zero ) * -0.005538461f;   //�����ٶȼ�Z������ת���ɳ���ʵ��Z��Ƕ�
                                                              /* 180�� / 32500 */                            

  Angle.Angular_Velocity_Z = (Gyro_Data - Angle.Gyro_X_Offset) * 1.4f;//1# 1.2
  //Angle.Angular_Velocity_Z = Gyro_Data * 5.1f;
  Angle.Car_Z = Angle.Final_Integral_Angle;     //����Ƕȼ�Ϊ���ֺ�ļ��Ӱ�Ƕ�
  
  Delta_Angular_Speed_Z = (Angle.Z - Angle.Car_Z) / 1;       //0.07
  //Delta = Delta_Angular_Speed_Z;
  
  Angle.Final_Integral_Angle += (Angle.Angular_Velocity_Z + Delta_Angular_Speed_Z)/500;
  //�����˲����ڻ��ֺ����õ������ս��
  //uint16_t a[8];
  //a[1] = (uint16_t)Angle.Final_Integral_Angle;
  
 Send_Data[0] = Angle.Final_Integral_Angle;//��������λ���鿴����
 //Send_Data[1] = Angle.Z;
 //Send_Data[2] = (float)Angle.Angular_Velocity_Z;
}

/***            done            ***/


/***            ���ֵ��ȡ������                ***
*       ���ܣ���ȡ�������ֵ�������˲�����һ��     */

void Inductance_Inertia_Filter(void)    //�Ե��ֵ���й����˲�
{
  //�м����˲�
  L.Middle_Voltage_Filter  = (uint16_t) ((0.8 * L.Middle_Voltage_Filter) + (0.2 * L.Middle_Voltage_Temp));
  L.Middle_Voltage_Temp = L.Middle_Voltage;
  //��ߵ���˲�
  L.Left_Voltage_Filter = (uint16_t) ((0.8 * L.Left_Voltage_Filter) + (0.2 * L.Left_Voltage_Temp));
  L.Left_Voltage_Temp = L.Left_Voltage;
  //�ұߵ���˲�
  L.Right_Voltage_Filter = (uint16_t) ((0.8 * L.Right_Voltage_Filter) + (0.2 * L.Right_Voltage_Temp));
  L.Right_Voltage_Temp = L.Right_Voltage;
  //���Բ������˲�
  L.Left_Ring_Voltage_Filter = (uint16_t) ((0.8 * L.Left_Ring_Voltage_Filter) + (0.2 * L.Left_Ring_Voltage_Temp));
  L.Left_Ring_Voltage_Temp = L.Left_Ring_Voltage;
  //�ұ�Բ������˲�
  L.Right_Ring_Voltage_Filter = (uint16_t) ((0.8 * L.Right_Ring_Voltage_Filter) + (0.2 * L.Right_Ring_Voltage_Temp));
  L.Right_Ring_Voltage_Temp = L.Right_Ring_Voltage;
}


void Inductance_Ave_Filter(int N_Times) //�Ե��ֵ���о�ֵ�˲� ����Ϊ��������
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

void Get_Inductor_Data(void)    //�ɼ����ֵ�����д���
{
  L.Left_Ring_Voltage = adc_once(ADC_CHANNEL_AD12,ADC_12BIT);
  L.Right_Ring_Voltage = adc_once(ADC_CHANNEL_AD13,ADC_12BIT);
  L.Left_Voltage = adc_once(ADC_CHANNEL_AD6,ADC_12BIT);
  L.Right_Voltage = adc_once(ADC_CHANNEL_AD7,ADC_12BIT);
  //Inductance_Ave_Filter(10);    //���о�ֵ�˲�
  Inductance_Inertia_Filter();   //���й����˲�
  Send_Data[4] = L.Left_Voltage_Filter;
  Send_Data[5] = L.Right_Voltage_Filter;
  //Send_Data[4] = L.Left_Ring_Voltage_Filter;
  //Send_Data[5] = L.Right_Ring_Voltage_Filter;
}

/***            done            ***/

/***            �����������㲿��          ***/
void CurrentDetect(void)
{
  /*
  float MidCurrent[10];//���-4-14��֮��ĵ���
  MidCurrent[0]=0;   //-4�ȵ��
  MidCurrent[1]=0;   //-2�ȵ��
  MidCurrent[2]=0;    //0�ȵ��
  MidCurrent[3]=0;    //2�ȵ��
  MidCurrent[4]=0;   //4�ȵ��
  MidCurrent[5]=0;   //6�ȵ��
  MidCurrent[6]=0;   //8�ȵ��
  MidCurrent[7]=0;   //10�ȵ��
  MidCurrent[8]=0;   //12�ȵ��
  MidCurrent[9]=0;   //14�ȵ��
  float Test[10];
  */
  
}

/***            �ٶȼ��㲿��          ***/
void Get_ArcLength(Disdance_ArcLength *dis, float Unit, float circle_ratio)     //��������������ת����ʵ�ʾ���
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
  Left_Wheel_Dir = gpio_get(PTC6);       //��ȡ���ַ���
}


void Judge_Right_Wheel_Dir(void)
{
  Right_Wheel_Dir = gpio_get(PTH6);       //��ȡ���ַ���
}
*/
void Get_Speed_Data(void)  //��ȡ�����ٶ���Ϣ   ����������������Ҫ����궨 ������
{
  //float Spd_Temp = 0;
  //�����֤����
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
  
  //��ȡ������2ms���������ڵ�������
  /*
  if(Right_Wheel_Dir != 0)
    Right_Spd.PulseCount = FTM_count_get(CFTM1); 
  else
    Right_Spd.PulseCount = FTM_count_get(CFTM1);
  */
  Right_Spd.PulseCount = FTM_count_get(CFTM1); 
  
  //��ȡ������������ͽ������
  FTM_count_clean(CFTM0);       
  FTM_count_clean(CFTM1);
  
  //1.��ȡһ�������������߹���ʵ�ʾ���
  Get_ArcLength(&Left_Spd, 1.0, Circle_Ratio);   //��ȡ������2ms�����������߹���ʵ�ʾ���
  
  //2.����þ����Ӧ���ٶȣ����֣�
  Speed._Left_Wheel = (float)Left_Spd.arcLength * 1000 / T_Int;
  
  //3.��ջ������ݣ������������ȣ�
  //Left_Spd.arcLength = 0;
  //Left_Spd.arcLengthBuffer = 0;
  Left_Spd.meter = 0;
  Left_Spd.PulseCount = 0;
  
  //1.��ȡһ�������������߹���ʵ�ʾ���
  Get_ArcLength(&Right_Spd, 1.0, Circle_Ratio);   //��ȡ������2ms�����������߹���ʵ�ʾ���
  //2.����þ����Ӧ���ٶȣ����֣�
  Speed._Right_Wheel = (float) Right_Spd.arcLength * 1000 / T_Int;
  
  //3.��ջ������ݣ������������ȣ�
  //Right_Spd.arcLength = 0;
  //Right_Spd.arcLengthBuffer = 0;
  Right_Spd.meter = 0;
  Right_Spd.PulseCount = 0;
  //��ת����
  if(Speed._Left_Wheel >5.0 && Speed._Right_Wheel>5.0 && ProtectEnableSPD)   //�����˿�ת����ʹ��ProtectEnableSPD
  {
    ProtectCount2++;
  }
  else
  {
    ProtectCount2=0;
  }
  if(ProtectCount2>200) ProtectFlag=1;
  //ͨ�����������ٶȵ���ֵ���������ٶ�ֵ
  Speed._Car = (Speed._Left_Wheel + Speed._Right_Wheel) / 2.0f;
  
  /***          �Գ����ٶȱ仯�ʽ����޷��˲�          ***/
  if(Car_Speed_Set<2.4) Speed._GrowLimit = (2.0f/1000.0f) * T_Int;
  else Speed._GrowLimit = (3.0f/1000.0f) * T_Int;      //���Ʒ��� = �����ٶ� / 1000 * �������ڣ�ms��
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


/***            ��ȡ��������������      ***/

void Get_Sensors_Data(void)     
{
   Get_AccANDGyro_Data();       //��ȡ���Ӱ�����������
   Acc_Z_Angle = accle_z;       //��ȡ��ʱ���ٶȼ�Z�����
   Gyro_Data = gyro_x * 0.0625f;     //��ȡ��ʱ������X����ٶ�
   Gyro_Dir_Data= gyro_y*0.0625f - Gyro_Y_Offset;  
}

/***            done            ***/


/***            ֱ�����Ƕȼ����������             ***/
void Angle_Output(float Current_Angle)  //�ǶȻ��������
{
  float Car_Angle,Car_Angle_Speed,Car_Angle_Set = 0;
  float Car_Angle_Speed_Set = 0;
  float Angle_Delta,Angle_Speed_Delta;
  float AngUpLimit = -6.0;       //����ʱ�ĽǶȱ仯�޷�
  float AngDownLimit = 3.0;      //����ʱ�ĽǶȱ仯�޷�
  float AngMax=3;
  float AngMin=-3;
  if(Car_Speed_Set<2.4) AngUpLimit=-3;
    
  /*
  //�Ƕȱ仯���޷�
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
  
  //������ֵ
  Car_Angle = Current_Angle;
  //Car_Angle_Set = 0;
  Car_Angle_Set = g_fSpeedControlOut;
  Car_Angle_Speed = Angle.Angular_Velocity_Z;
  /******               ���ܼ��ٲ���          ******/
  if(Left_Distance<1.0 && Right_Distance<1.0 && Speed._2ms_True<2.0&&SpeedUpEnable==1)
  {
    AngMax=15;
  }
  //�Ƕ��޷�
  if(Car_Angle_Set > AngMax)
    Car_Angle_Set = AngMax;
  if(Car_Angle_Set < AngMin)
    Car_Angle_Set = AngMin;
  
  if(GHGFlag==1) Car_Angle_Set=-20;
  /******               done            ******/
  
  //         ������            
  Angle_Speed_Delta = Car_Angle_Speed_Set - Car_Angle_Speed;
  Angle_Delta = Car_Angle_Set - Car_Angle; 
  
  
  //�Ƕȱ仯�޷�
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
      
  
  //�Ƕ����
  Angle.Output = Angle_Kp * Angle_Delta + Angle_Kd * Angle_Speed_Delta;
  //Send_Data[3]= Angle_Kp * Angle_Delta;
  //Send_Data[4] = Angle_Kd * Angle_Speed_Delta;
  //Send_Data[5] = Angle.Output;
}

/***            done            ***/

/***            �����ٶȻ������������             ***/
void Car_Speed_Control_Output(void)     //�����ٶȼ������
{
  float Car_Speed = 0;
  float Car_Speed_Delta = 0;
  float alpha=1.0;
  float absDelta;
  Car_Speed = Speed._2ms_True;
  /******               ���ܼ��ٲ���          ******/
  //������
  Car_Speed_Delta = Car_Speed_Set - Car_Speed;
  
  
  //���ٻ���
  //��ȡ������ֵ
  if(Car_Speed_Delta>0) absDelta = Car_Speed_Delta;
  else absDelta = -Car_Speed_Delta;
  //���ݲ�ͬ���ȼ�����������Ȩֵ
  if(absDelta>0.5) alpha = 0;
  if(absDelta<0.1) alpha = 1;
  if(absDelta>0.1 && absDelta<0.5) alpha=(0.5-absDelta)/(0.5-0.1);
  
  
  Car_Speed_Integral += Car_Speed_Delta * Speed_Ki * alpha ;
  //�ٶȻ����޷�
  if(Car_Speed_Integral > 3)
    Car_Speed_Integral = 3;
  if(Car_Speed_Integral < -3)
    Car_Speed_Integral = -3;
  
  //�ٶ��������
  Speed_Control_Output = Speed_Kp * Car_Speed_Delta + Car_Speed_Integral;
  g_fSpeedControlOutOld=g_fSpeedControlOutNew;
  g_fSpeedControlOutNew=Speed_Control_Output;
  
}
/***            done            ***/

/***                ��һ������                  ***/
void Normalization(void)
{
    float rMax=0,lMax=0;
    
    int kr=60,br=2200;//���ҹ�һ��0.5
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
      //kl = flash_read(sector,0,uint16);                         //��ȡ����
      //bl2 = flash_read(sector,0,uint16);
      //kr = flash_read(sector,8,uint16);                         //��ȡ����
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

/***                Բ����ⲿ��                  ***/
void Circle_Test(void)
{
    //CircleMidLimit = 0.9;
    float CircleUpLeftLimit=0.7;//0.7
    float CircleUpRightLimit=0.70;//0.7
    float CircleDownLeftLimit=1.1;//
    float CircleDownRightLimit=1.1;
    //��̬��ֵ
    if(Car_Speed_Set > 2.4&&Car_Speed_Set < 2.6)
    {
      CircleDownLeftLimit=1.1;//1.1����zuozhuan���ˣ���ת�磨����Բ���ܲ��ã�
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
    //�������Բ����ʼ����������Բ������ʱ
    if(left > CircleUpLeftLimit && right > CircleUpRightLimit)
    {
      CircleCount++;
    }
    else
    {
      if(CircleCount<10) CircleCount=0;
    }
    //��������
    if(CircleCount>=10 && CircleEndFlag==0 && (left-leftOld)<0 && (right-rightOld)<0 && left<CircleDownLeftLimit && right<CircleDownRightLimit)
    {
      CircleEndCount++;
      if(CircleEndCount==3)//2.5�ٶ�֮ǰ
      {
         CircleEndFlag=1;//����ź���ʧ����������Flag
         LengthDelta=TrueLength-TrueLengthOld;//Բ���������ɼ���Բ���İ뾶��Ϣ
      }
    }
    else
      CircleEndCount=0;
    
    //��Բ��Flag����ʱ��ȷ��һ�¿�ʼ��Length
    if(CircleCount==10)
    {
      TrueLengthOld=TrueLength;
    }
    //ȷ������Բ��Flag
    if(CircleCount>=10) CircleStartFlag=1;
    //�ɼ����ҵ��ƽ��ֵ
    if(CircleStartFlag==1 && CircleEndFlag==0 && DirectionJudge==0)
    {
      if(L.Left_Ring_Voltage_Filter>3300) DirectionJudge=1;
      if(L.Right_Ring_Voltage_Filter>3300) DirectionJudge=-1;
      //Left_Ave_Diangan=(CircleCount-10)*Left_Ave_Diangan/(CircleCount-9)+L.Left_Ring_Voltage_Filter/(CircleCount-9);
      //Right_Ave_Diangan=(CircleCount-10)*Right_Ave_Diangan/(CircleCount-9)+L.Right_Ring_Voltage_Filter/(CircleCount-9);
    }
    //ִ�д������
    if(CircleStartFlag==1 && CircleEndFlag==1 && (TrueLength-TrueLengthOld)<(LengthDelta + 0.5))//0.5ָ��ִ��ʱ��
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
    Send_Data[7] =LengthDelta*1000*DirectionJudge; //��λ�����Բ������
        
       
}

void Boma_Circle_Test(void)
{
    float CircleUpLimit=0.8;//0.65
    
    //�������Բ����ʼ����������Բ������ʱ
    if(left > CircleUpLimit && right > CircleUpLimit)
    {
      CircleCount++;
    }
    else
    {
      if(CircleCount<10) CircleCount=0;
    }
    
    //��Բ��Flag����ʱ��ȷ��һ�¿�ʼ��Length
    if(CircleCount==10)
    {
      TrueLengthOld=TrueLength;
      //20180717�޸�
      if(ZaoFlag==1)
      {
          TrueLengthOld+=0.07;//�������ӳ��뻷
      }
      if(WanFlag==1)
      {
          TrueLengthOld-=0.07;//��������ǰ�����
      }
    }
    
    //ȷ������Բ��Flag
    if(CircleCount>=10) CircleStartFlag=1;
    //ִ�д������
    if(Car_Speed_Set<2.4)
    {
      if(CircleStartFlag==1 && (TrueLength-TrueLengthOld)< 0.82 &&(TrueLength-TrueLengthOld)> 0.32)//0.5ָ��ִ��ʱ��
        CircleDoFlag=1;
      else
        CircleDoFlag=0;
    }
    
    if(Car_Speed_Set>2.4&&Car_Speed_Set<2.6)
    {
      if(CircleStartFlag==1 && (TrueLength-TrueLengthOld)< 0.85 &&(TrueLength-TrueLengthOld)> 0.35)//0.5ָ��ִ��ʱ��
        CircleDoFlag=1;
      else
        CircleDoFlag=0;
    } 
    
    if(Car_Speed_Set > 2.6)
    {
      //CircleUpLimit=0.65;
      if(CircleStartFlag==1 && (TrueLength-TrueLengthOld)< 0.85 &&(TrueLength-TrueLengthOld)> 0.35)//0.5ָ��ִ��ʱ��
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
    if (value <= dividePoints[0])//����ֵ����С��С
    {	    
        membership[0] = 1;
        //return 0;
    }
    else
    {
        if (value >= dividePoints[NUM-1]) //���ڵ�ֵ����󻹴�
        { 		    
            membership[NUM-1] = 1;
        }
        else
        {
            for (i = 1; i < NUM; i++)
            {
                if (value <= dividePoints[i])//�ҵ�������С�ڵ�ǰֵ�ĵ�
                {
                    membership[i-1] = (dividePoints[i]-value) / (dividePoints[i] - dividePoints[i - 1]);
                    //��ǰ�㵽�ұߵ�ľ���/����2����ܾ���
                    membership[i] = 1 - membership[i-1];
                    //return i;
                    break;
                }
            }
        }
    }
}


/***            ����ת�򻷼����������             ***/
void Car_Direction_Control_Output(void) //���򻷼������
{
    float fValue=0;
    float DIR_CONTROL_P=1000;//1000
    float DIR_CONTROL_K1=2000;//���Ի�����1�м䲹��500//���Ի�����2�м䲹��2000//�ı䲹�߷�ʽ
    float DIR_CONTROL_K2=2000;
    float buxian=0,buxian1=0,buxian2=0;
    float middle0=0;
    
    float angle;
    angle=Angle.Car_Z;//
    if(angle>15) angle=15;
    
    //�ջ�����
    float qianzhan=33.0;//������38,Ϊ���ܹ���ǰ���䣬��Ϊ�޸�ǰհ35
    float xiebian2;
    float g_fTruejiaosudu,g_fCaljiaosudu;
    float fClosedValue,fDClosedValue;
    float DIR_CLOSED_K=50.5+0.9*angle;
    
    //middle0=middle;//���߷���1
    //if(middle0>1.0) middle0=1;
    
    int i,j;
    float tempP=0,tempD=0;
    
    middle0=right+left;
    //Send_Data[2] = middle0*1000;
    if(middle0>1.1) middle0=1.1;
    //if(middle0<0.3) middle0=0.3;
    //0.3����Ϊ0.7��0.7=0.5��������������ת�򣬷�ֹ���������˦
    fValue=left-right;
    fValue*=DIR_CONTROL_P;
    //Send_Data[4] = fValue;
    
    buxian1=DIR_CONTROL_K1*(1-middle0);
    buxian2=DIR_CONTROL_K2*(1.1-middle0)*(1.1-middle0);
    if(buxian1>=buxian2) buxian=buxian1;
    else buxian=buxian2;
    
    
    //ʹ�����ҵ�������߷���
    if(Angle.Car_Z<15&&Angle.Car_Z>-10)
    {
       if(fValue>0) fValue=fValue+buxian2;
       if(fValue<0) fValue=fValue-buxian2;
     }
    
     //Send_Data[5] = fValue;//���ߺ�ĵ��ƫ��ֵ
     
     //����ƫ����
     pianyiOld=pianyi;
     pianyi=fValue/DIR_CLOSED_K;
     if(pianyi>30) pianyi=30;
     if(pianyi<-30) pianyi=-30;
     //Car_Direction_Control_Output_Plus();
     
     if(Car_Speed_Set>2.4)
     {
       DIR_CLOSED_P=0;
       DIR_CLOSED_D=0;
       //ģ������
       Calc_Membership_Without_CenterZone_7Gear(pianyi, DirP_Membership, DirP_Field,7);
       Calc_Membership_Without_CenterZone_7Gear((pianyi-pianyiOld), DirD_Membership, DirD_Field,7);
       for (i = 0; i < 7; i++)
        {
            if (DirD_Membership[i] != 0)//DirDΪ�У����������к�
            {
                for (j = 0; j < 7; j++)
                {
                    if (DirP_Membership[j] != 0)//DirPΪ�У����������к�
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
     
     
     if(CircleDoFlag==1&&TrueLength>1.0)//ִ��ʱ��1.5m->50  1m�����Բ��
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
     
     /***         ��������           ***/
     //g_fDirectionControlOutOld=g_fDirectionControlOutNew;
     //g_fDirectionControlOutNew=20*pianyi+100*(pianyi-pianyiOld); 
     //if(qulv<0.015&&qulv>-0.015&&Speed._2ms_True>2.0) qulv=0; 
     
     g_fCaljiaosuduOld = g_fCaljiaosuduNew;
     g_fCaljiaosudu = Speed._Car * qulv * 100 * 180/3.1416;
     g_fCaljiaosuduNew = g_fCaljiaosudu * 0.3 + g_fCaljiaosuduOld * 0.7;
     //100��m��cm��ת��
     //60�ǽǶȺͻ��ȵ�ת������ʵӦ����180/PI
     g_fTruejiaosuduOld = g_fTruejiaosuduNew;
     g_fTruejiaosudu=Gyro_Dir_Data;
     g_fTruejiaosuduNew = g_fTruejiaosudu * 0.3 + g_fTruejiaosuduOld * 0.7;
     //Send_Data[4] = g_fCaljiaosudu;
     //Send_Data[5] = g_fTruejiaosudu;
     //Send_Data[7] = g_fTruejiaosuduNew;
     //�����������΢��
     fClosedValueOld = fClosedValueNew;
     //fClosedValue = g_fCaljiaosuduNew - g_fTruejiaosuduNew;  //�����˲��㷨
     fClosedValue = g_fCaljiaosudu - g_fTruejiaosudu;
     fClosedValueNew = fClosedValue;
     
     fDClosedValueOld = fDClosedValueNew;
     fDClosedValue = fClosedValueNew - fClosedValueOld;
     fDClosedValueNew = fDClosedValue;
     
     g_fDirectionControlOutOld = g_fDirectionControlOutNew;
     g_fDirectionControlOutNew = DIR_CLOSED_P * fClosedValue;//P����
     //Send_Data[2] = DIR_CLOSED_P * fClosedValue;
     g_fDirectionControlOutNew += DIR_CLOSED_D * fDClosedValue;//D���� 
     //Send_Data[3] = DIR_CLOSED_D * fDClosedValue;
     g_fDirectionControlOutNew += 0.1 * DIR_CLOSED_D * fDClosedValueOld * DIR_CLOSED_D / DIR_CLOSED_P;//����ȫ΢��
     //������Χ0.03-0.1���������ԽС������ʱ��Խ���������󣬳���ʱ���
     //Send_Data[4]=DIR_CLOSED_P * fClosedValue;
     //Send_Data[5]=DIR_CLOSED_D * fDClosedValue;
     
     
}
/***            done            ***/



/***         ���ٲ���           ***/
void Car_Direction_Control_Output_Plus(void)
{
  //���յķ���������ת��ջ�
  float v = 2.3;
  if(Speed._2ms_True>v&&(pianyi>10||pianyi<-10)) pianyi=pianyi*(1+(Speed._2ms_True-v)*0.5);
}
/***            done            ***/


/***        ֱ�����            ***/

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



/***        ���䴦��            ***/


/***            ���PWM�������               ***/
//      ���ܣ�����PWM����������
//      �ܽ���Ϣ�� ʹ��FTM2ģ���CH0, CH1, CH2, CH3�ĸ�ͨ��
//                 ��Ӧ�ܽ�Ϊ   PTH0,PTH1,PTD0,PTD1
//      ռ�ձȾ���Ϊ5000������ͨ���޸ĵײ�PRECISE���������޸�
//      ��ע����Ӧ���ֿ�����Ҫʵ�ʱ궨
//      PTH0��PTH1Ϊһ��    PTD0��PTD1Ϊһ��
//      ÿ��֮���PWMӦ������
//      


void Set_PWM(int PWM_n,int value)       //����PWM���
{
  uint32 Value;
  Value =(uint32)(5000-value);

  if(Value>5000)//5000  3/15 16:24
  Value=5000;
    switch(PWM_n) 
    {
        case 1:   FTM_PWM_Duty(CFTM2, FTM_CH0, Value); break;    //������ǰ      
        case 2:   FTM_PWM_Duty(CFTM2, FTM_CH1, Value); break;    //������ǰ      
        case 3:   FTM_PWM_Duty(CFTM2, FTM_CH2, Value); break;    //�������    
        case 4:   FTM_PWM_Duty(CFTM2, FTM_CH3, Value); break;    //������ǰ      
    }  
}

//Voltage > 0 : ����;Voltage < 0����ת
void Set_Motor_Voltage(float f_Left_Voltage, float f_Right_Voltage) 	//���õ����ת������ٶ�	
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
            
  if(f_Left_Voltage > 0) //��ǰ
  {
    Set_PWM(3, 0);
    nOut = (int)(f_Left_Voltage);
    Set_PWM(4, nOut);
  } 
  else             //���
  {
    Set_PWM(4, 0);
    f_Left_Voltage = -f_Left_Voltage;
    nOut = (int)(f_Left_Voltage);
    Set_PWM(3, nOut);
  }                                     
  if(f_Right_Voltage > 0)   //��ǰ
  {
    Set_PWM(1, 0);       //HIGH
    nOut = (int)(f_Right_Voltage);
    Set_PWM(2, nOut);
  } 
  else             //�Һ�
  {
    Set_PWM(2, 0);
    f_Right_Voltage = -f_Right_Voltage;
    nOut = (int)(f_Right_Voltage);
    Set_PWM(1, nOut);
  }
        
}

void Car_Protect(void)
{
  if(L.Left_Voltage_Filter<10 && L.Right_Voltage_Filter<10 && ProtectCount<100 && ProtectEnableSGN && TrueLength>3.0) //�������źű���ʹ��ProtectEnableSGN
    {
      ProtectCount++;
    }
    else
    {
      ProtectCount=0;
    }
    if(ProtectCount>50) ProtectFlag=1;
}

void Motor_Output(void) //�����������
{               
  float f_Left_Motor = 0;
  float f_Right_Motor = 0;
    //�������������
  if(GoFlag==1)
  {
    f_Left_Motor = Angle.Output -  Dir_Control_Output ;
    f_Right_Motor = Angle.Output +  Dir_Control_Output ; 
    //�ҹ�����-��+
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
  //�������޷� PWM��ռ�ձ��Ƿ���Ϊ5000
  if(f_Left_Motor > Motor_Output_Max)   //����
    f_Left_Motor = Motor_Output_Max;
  if(f_Left_Motor < Motor_Output_Min)
    f_Left_Motor = Motor_Output_Min;
  if(f_Right_Motor > Motor_Output_Max)   //�ҵ��
    f_Right_Motor = Motor_Output_Max;
  if(f_Right_Motor < Motor_Output_Min)  
    f_Right_Motor = Motor_Output_Min;
  
  //�����������
  
  if(f_Left_Motor > 0)
    f_Left_Motor += 0;        //      ������ת�������궨
  if(f_Left_Motor < 0)
    f_Left_Motor -= 0;         //      ���ַ�ת�������궨
  if(f_Right_Motor > 0)
    f_Right_Motor += 270;        //      ������ת�������궨
  if(f_Right_Motor < 0)
    f_Right_Motor -= 0;         //      ���ַ�ת�������궨
  Set_Motor_Voltage(f_Left_Motor, f_Right_Motor);
}








/***            done            ***/


/*****************************************************************************************************/


/***           ���Զ���һ��         ***/


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

//��С���������������룬ͬʱ�ɼ�������·��У������ҵ�н��й�һ��
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
//���Pģ������uyt
  int m,n;
  
  for(m=0;m<7;m++)
  {
    if(m==0)//ģ�����1��
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
    
    if(m==1)//ģ�����2��
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
    
    if(m==2)//ģ�����3��
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
    
    if(m==3)//ģ�����4��
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
    
    if(m==4)//ģ�����5��
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
    
    if(m==5)//ģ�����6��
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
    
    if(m==6)//ģ�����7��
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
//���Pģ������uyt
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


//����
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
