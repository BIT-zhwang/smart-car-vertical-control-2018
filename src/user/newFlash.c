#include "include.h"
// flash commands 
#define ERASE_VERITF_ALL_BLOCKS             0x01  // 擦除检验所有区块
#define ERASE_VERITF_BLOCKS                 0x02  // 擦除检验数据块
#define ERASE_VERITF_FLASH_SECTION          0x03  // 擦除检验Flash 段
#define READ_ONCE                           0x04  // 读取一次
#define PROGRAM_FLASH                       0x06  // 编程Flash
#define PROGRAM_ONCE                        0x07  // 编程一次
#define ERASE_ALL_BLOCKS                    0x08  // 擦除所有区块
#define ERASE_FLASH_BLOCKS                  0x09  // 擦除Flash 区块
#define ERASE_FLASH_SECTOR                  0x0A  // 擦除Flash 扇区
#define UNSECURE_FLASH                      0x0B  // 解密的Flash
#define VERITF_BACKDOOR_ACCESS_KEY          0x0C  // 检验后门访问密钥
#define SET_USER_MARGIN_LEVEL               0x0D  // 设置用户裕量水平
#define SET_FACTORY_MARGIN_LEVEL            0x0E  // 设置出厂裕量水平
#define CONFIGURE_NVM                       0x0F  // 配置NVM



volatile uint8 s_flash_command_run[] = {0x00, 0xB5, 0x80, 0x21, 0x01, 0x70, 0x01, 0x78, 0x09, 0x06, 0xFC, 0xD5,0x00, 0xBD};
typedef void (*flash_run_entry_t)(volatile uint8 *reg);
flash_run_entry_t s_flash_run_entry;

//内部使用
uint8 FlashCmdStart(void)
{
    DisableInterrupts;
    
    FTF->FSTAT = FTMRE_FSTAT_FPVIOL_MASK | FTMRE_FSTAT_ACCERR_MASK;
    
    s_flash_run_entry = (flash_run_entry_t)((uint32)s_flash_command_run + 1);
    s_flash_run_entry(&FTF->FSTAT);
    
    EnableInterrupts;
    
    if(FTF->FSTAT & (FTMRE_FSTAT_ACCERR_MASK | FTMRE_FSTAT_FPVIOL_MASK | FTMRE_FSTAT_MGSTAT_MASK)) return 1;	//出现错误
    return 0;												//成功
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      FLASH初始化
//  @return     				返回一个扇区的大小
//  @since      v1.0
//  Sample usage:               uint32 dat = FLASH_GetSectorSize();
//-------------------------------------------------------------------------------------------------------------------
uint32 FLASH_GetSectorSize(void)
{
    return SECTOR_SIZE;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      FLASH初始化
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
uint32 bus_clk_khz;

void FLASH_Init1(void)
{
    uint16 clk;
    
    SIM->SCGC |= SIM_SCGC_FLASH_MASK;                                   //开启flash时钟         
    
    while(!(FTF->FSTAT & FTMRE_FSTAT_CCIF_MASK));                       //等待之前命令完成
    
    clk = bus_clk_khz/1000-1;

    FTF->FCLKDIV = FTMRE_FCLKDIV_FDIV(clk) | FTMRE_FCLKDIV_FDIVLCK_MASK;//设置flash分频系数
    
    FTF->FSTAT = FTMRE_FSTAT_FPVIOL_MASK | FTMRE_FSTAT_ACCERR_MASK;     //清除状态标识
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      FLASH擦除扇区
//  @param      SectorNum 		需要擦除的扇区编号
//  @return     				返回1擦除失败，返回0擦除成功
//  @since      v1.0
//  Sample usage:               uint32 dat = FLASH_GetSectorSize(10);
//-------------------------------------------------------------------------------------------------------------------
uint8 FLASH_EraseSector1(uint32 SectorNum)
{
    uint32 addr;

	addr = (uint32)SectorNum*SECTOR_SIZE;

    FTF->FCCOBIX = 0;
    FTF->FCCOBHI = ERASE_FLASH_SECTOR;
    FTF->FCCOBLO = addr>>16;
    
    FTF->FCCOBIX = 1;
    FTF->FCCOBHI = (addr&0xffff)>>8;
    FTF->FCCOBLO = addr&0xff;
    
    FlashCmdStart();
    
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      FLASH数据写入到扇区
//  @param      SectorNum 		需要写入的扇区编号
//  @param      *buf	 		数据首地址
//  @param      len		 		写入的字节数 特别提醒一个uint16变量是两个字节 一个uint32变量是四个字节 每次写入得长度必须为4得倍数
//  @param      offset		 	必须为4的整数倍
//  @return     				返回1写入失败，返回0写入成功
//  @since      v1.0
//  Sample usage:               FLASH_WriteSector(10,(const uint8 *)buf,4,0);//将buf数组里面的前四位数据写入到10号扇区(每一位是uint8类型)
//  @note                       使用flash时，最好从最后一个扇区开始使用，最后一个扇区号为255
//-------------------------------------------------------------------------------------------------------------------


uint8 FLASH_WriteSector(uint32 SectorNum, const uint8 *buf, uint32 len, uint32 offset)
{
    uint16 step, i;
    uint32 addr;

	addr = (uint32)SectorNum*SECTOR_SIZE + offset;

    step = 4;

	for(i=0; i<len; i+=step)
	{
        FTF->FCCOBIX = 0;
        FTF->FCCOBHI = PROGRAM_FLASH;
        FTF->FCCOBLO = addr>>16;
        
        FTF->FCCOBIX = 1;
        FTF->FCCOBHI = (addr&0xFFFF)>>8;
        FTF->FCCOBLO = addr&0xFC;
        
        FTF->FCCOBIX = 2;
        FTF->FCCOBLO = buf[0];
        FTF->FCCOBHI = buf[1];
        
        FTF->FCCOBIX = 3;
        FTF->FCCOBLO = buf[2];
        FTF->FCCOBHI = buf[3];

        buf += step;
        addr += step;

        FlashCmdStart();
    }
    return 0;
}

void FLASH(void)    //自建Flash调用历程
{
  
uint16 write1[2];
uint16 read1[2];

uint32 write2;
uint32 read2;

uint8 sector = FLASH_SECTOR_NUM - 1;

uint16 run;

    
    //在使用flash的时候，最好以最后一个扇区为起点开始使用
    
    write1[0] = 2017;
    write1[1] = 2018;
    
    write2 = 20172018;
    
    FLASH_Init();
    FLASH_EraseSector(sector);                                      //擦除最后一个扇区
    
    
    FLASH_WriteSector(sector,(const uint8 *)write1,4,4);            //写入四个字节到最后一个扇区 偏移4字节
    read1[0] = flash_read(sector,4,uint16);                         //读取数据
    read1[1] = flash_read(sector,6,uint16);

    FLASH_WriteSector(sector,(const uint8 *)&write2,4,8);            //写入四个字节到最后一个扇区 偏移8字节 这里提示下，如果不是数组记得是加上取地址符号
    read2 = flash_read(sector,8,uint32);                             //读取数据

    
  while(1)
  {
    uint8_t   txt[30]="X:";
    sprintf(txt,"FTM1 PTE7: %05d \n",read1[0]);
    LCD_P6x8Str(0,2,txt);
      time_delay_ms(5);
  }
}





uint16 write1[2];
uint16 read1[2];

uint32 write2;
uint32 read2;

uint8 sector = FLASH_SECTOR_NUM - 1;

uint16 run;



void flash_test()
{

    
    //在使用flash的时候，最好以最后一个扇区为起点开始使用
    
    write1[0] = 2017;
    write1[1] = 2018;
    
    write2 = 20172018;
    
    
    FLASH_EraseSector1(sector); 
    
     FLASH_WriteSector(sector,(const uint8 *)write1,4,4);            //写入四个字节到最后一个扇区 偏移4字节
    read1[0] = flash_read(sector,4,uint16);                         //读取数据
    read1[1] = flash_read(sector,6,uint16);

    FLASH_WriteSector(sector,(const uint8 *)&write2,4,8);            //写入四个字节到最后一个扇区 偏移8字节 这里提示下，如果不是数组记得是加上取地址符号
    read2 = flash_read(sector,8,uint32);                             //读取数据

    
    
  
}

