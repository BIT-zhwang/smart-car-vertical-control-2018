extern unsigned int OutData[7],OutData2[4];
extern char flag_received;


unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT);
void OutPut_Data(void);
void GetData1(float Ch1,float Ch2,float Ch3,float Ch4);
void GetData(float Ch1,float Ch2,float Ch3,float Ch4,float Ch5,float Ch6,float Ch7);
//void vcan_sendware(uint8 *wareaddr, uint32 waresize);
void uart_putbuff (UARTn uratn, uint8 *buff, uint32 len);
void vcan_sendware(uint8 *wareaddr, uint32 waresize, UARTn uratn);


#define   CHANNAL1     0  
#define   CHANNAL2     100
#define   CHANNAL3      200
#define   CHANNAL4     300
#define   CHANNAL5     line
#define   CHANNAL6     500
#define   CHANNAL7     600
#define   CHANNAL8     700
