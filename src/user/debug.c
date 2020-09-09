#include "include.h"

unsigned int OutData[7],OutData2[4];
char flag_received;

unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT)
{
    unsigned short CRC_Temp;
    unsigned char i,j;
    CRC_Temp = 0xffff;

    for (i=0;i<CRC_CNT; i++)
    {
        CRC_Temp ^= Buf[i];//按位异或
        for (j=0;j<8;j++)
         {
            if (CRC_Temp & 0x01)
                CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
            else
                CRC_Temp = CRC_Temp >> 1;
        } 
     }
    return(CRC_Temp);
}

void OutPut_Data(void)
{
    unsigned short ChxData[4] = {0};
    unsigned char databuf[10] = {0};
    unsigned char i;
    unsigned short CRC16 = 0;
    for(i=0;i<4;i++)
    {
        ChxData[i] = (unsigned short)OutData[i];
    }
    for(i=0;i<4;i++)
    {
        databuf[i*2+0] = (unsigned char)((ChxData[i])&0xff);
        databuf[i*2+1] = (unsigned char)((ChxData[i])>>8);
    }
    CRC16 = CRC_CHECK(databuf,8);
    databuf[8] = CRC16&0xff;
    databuf[9] = CRC16>>8;
    for(i=0;i<10;i++)
      Uart_SendChar(UARTR2,databuf[i]);//根据串口修改
}
void GetData(float Ch1,float Ch2,float Ch3,float Ch4,float Ch5,float Ch6,float Ch7)
{
  int temp[7];
  temp[0]=(int)Ch1;
  temp[1]=(int)Ch2;
  temp[2]=(int)Ch3;
  temp[3]=(int)Ch4;
  temp[4]=(int)Ch5;
  temp[5]=(int)Ch6;
  temp[6]=(int)Ch7;
  
  OutData[0]=(unsigned int)temp[0];
  OutData[1]=(unsigned int)temp[1];
  OutData[2]=(unsigned int)temp[2];
  OutData[3]=(unsigned int)temp[3];
  OutData[4]=(unsigned int)temp[4];
  OutData[5]=(unsigned int)temp[5];
  OutData[6]=(unsigned int)temp[6];
  //OutData[7]=(unsigned int)temp[7];
}

void GetData1(float Ch1,float Ch2,float Ch3,float Ch4)
{
  int temp[4];
  temp[0]=(int)Ch1;
  temp[1]=(int)Ch2;
  temp[2]=(int)Ch3;
  temp[3]=(int)Ch4;
  OutData[0]=(unsigned int)temp[0];
  OutData[1]=(unsigned int)temp[1];
  OutData[2]=(unsigned int)temp[2];
  OutData[3]=(unsigned int)temp[3];
}              
            
 
void uart_putbuff (UARTn uratn, uint8 *buff, uint32 len)
{
    while(len--)
    {
     //   uart_putchar(UART2, *buff);
      Uart_SendChar(uratn,*buff);
        buff++;
    }
}

void vcan_sendware(uint8 *wareaddr, uint32 waresize, UARTn uratn)
{
#define CMD_WARE     3
    uint8 cmdf[2] = {CMD_WARE, ~CMD_WARE};    //串口调试 使用的前命令
    uint8 cmdr[2] = {~CMD_WARE, CMD_WARE};    //串口调试 使用的后命令

    uart_putbuff(uratn, cmdf, sizeof(cmdf));    //先发送前命令
    uart_putbuff(uratn, (uint8_t *)wareaddr, waresize);    //发送数据
    uart_putbuff(uratn, cmdr, sizeof(cmdr));    //发送后命令

}


