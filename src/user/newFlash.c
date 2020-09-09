#include "include.h"
// flash commands 
#define ERASE_VERITF_ALL_BLOCKS             0x01  // ����������������
#define ERASE_VERITF_BLOCKS                 0x02  // �����������ݿ�
#define ERASE_VERITF_FLASH_SECTION          0x03  // ��������Flash ��
#define READ_ONCE                           0x04  // ��ȡһ��
#define PROGRAM_FLASH                       0x06  // ���Flash
#define PROGRAM_ONCE                        0x07  // ���һ��
#define ERASE_ALL_BLOCKS                    0x08  // ������������
#define ERASE_FLASH_BLOCKS                  0x09  // ����Flash ����
#define ERASE_FLASH_SECTOR                  0x0A  // ����Flash ����
#define UNSECURE_FLASH                      0x0B  // ���ܵ�Flash
#define VERITF_BACKDOOR_ACCESS_KEY          0x0C  // ������ŷ�����Կ
#define SET_USER_MARGIN_LEVEL               0x0D  // �����û�ԣ��ˮƽ
#define SET_FACTORY_MARGIN_LEVEL            0x0E  // ���ó���ԣ��ˮƽ
#define CONFIGURE_NVM                       0x0F  // ����NVM



volatile uint8 s_flash_command_run[] = {0x00, 0xB5, 0x80, 0x21, 0x01, 0x70, 0x01, 0x78, 0x09, 0x06, 0xFC, 0xD5,0x00, 0xBD};
typedef void (*flash_run_entry_t)(volatile uint8 *reg);
flash_run_entry_t s_flash_run_entry;

//�ڲ�ʹ��
uint8 FlashCmdStart(void)
{
    DisableInterrupts;
    
    FTF->FSTAT = FTMRE_FSTAT_FPVIOL_MASK | FTMRE_FSTAT_ACCERR_MASK;
    
    s_flash_run_entry = (flash_run_entry_t)((uint32)s_flash_command_run + 1);
    s_flash_run_entry(&FTF->FSTAT);
    
    EnableInterrupts;
    
    if(FTF->FSTAT & (FTMRE_FSTAT_ACCERR_MASK | FTMRE_FSTAT_FPVIOL_MASK | FTMRE_FSTAT_MGSTAT_MASK)) return 1;	//���ִ���
    return 0;												//�ɹ�
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      FLASH��ʼ��
//  @return     				����һ�������Ĵ�С
//  @since      v1.0
//  Sample usage:               uint32 dat = FLASH_GetSectorSize();
//-------------------------------------------------------------------------------------------------------------------
uint32 FLASH_GetSectorSize(void)
{
    return SECTOR_SIZE;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      FLASH��ʼ��
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
uint32 bus_clk_khz;

void FLASH_Init1(void)
{
    uint16 clk;
    
    SIM->SCGC |= SIM_SCGC_FLASH_MASK;                                   //����flashʱ��         
    
    while(!(FTF->FSTAT & FTMRE_FSTAT_CCIF_MASK));                       //�ȴ�֮ǰ�������
    
    clk = bus_clk_khz/1000-1;

    FTF->FCLKDIV = FTMRE_FCLKDIV_FDIV(clk) | FTMRE_FCLKDIV_FDIVLCK_MASK;//����flash��Ƶϵ��
    
    FTF->FSTAT = FTMRE_FSTAT_FPVIOL_MASK | FTMRE_FSTAT_ACCERR_MASK;     //���״̬��ʶ
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      FLASH��������
//  @param      SectorNum 		��Ҫ�������������
//  @return     				����1����ʧ�ܣ�����0�����ɹ�
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
//  @brief      FLASH����д�뵽����
//  @param      SectorNum 		��Ҫд����������
//  @param      *buf	 		�����׵�ַ
//  @param      len		 		д����ֽ��� �ر�����һ��uint16�����������ֽ� һ��uint32�������ĸ��ֽ� ÿ��д��ó��ȱ���Ϊ4�ñ���
//  @param      offset		 	����Ϊ4��������
//  @return     				����1д��ʧ�ܣ�����0д��ɹ�
//  @since      v1.0
//  Sample usage:               FLASH_WriteSector(10,(const uint8 *)buf,4,0);//��buf���������ǰ��λ����д�뵽10������(ÿһλ��uint8����)
//  @note                       ʹ��flashʱ����ô����һ��������ʼʹ�ã����һ��������Ϊ255
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

void FLASH(void)    //�Խ�Flash��������
{
  
uint16 write1[2];
uint16 read1[2];

uint32 write2;
uint32 read2;

uint8 sector = FLASH_SECTOR_NUM - 1;

uint16 run;

    
    //��ʹ��flash��ʱ����������һ������Ϊ��㿪ʼʹ��
    
    write1[0] = 2017;
    write1[1] = 2018;
    
    write2 = 20172018;
    
    FLASH_Init();
    FLASH_EraseSector(sector);                                      //�������һ������
    
    
    FLASH_WriteSector(sector,(const uint8 *)write1,4,4);            //д���ĸ��ֽڵ����һ������ ƫ��4�ֽ�
    read1[0] = flash_read(sector,4,uint16);                         //��ȡ����
    read1[1] = flash_read(sector,6,uint16);

    FLASH_WriteSector(sector,(const uint8 *)&write2,4,8);            //д���ĸ��ֽڵ����һ������ ƫ��8�ֽ� ������ʾ�£������������ǵ��Ǽ���ȡ��ַ����
    read2 = flash_read(sector,8,uint32);                             //��ȡ����

    
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

    
    //��ʹ��flash��ʱ����������һ������Ϊ��㿪ʼʹ��
    
    write1[0] = 2017;
    write1[1] = 2018;
    
    write2 = 20172018;
    
    
    FLASH_EraseSector1(sector); 
    
     FLASH_WriteSector(sector,(const uint8 *)write1,4,4);            //д���ĸ��ֽڵ����һ������ ƫ��4�ֽ�
    read1[0] = flash_read(sector,4,uint16);                         //��ȡ����
    read1[1] = flash_read(sector,6,uint16);

    FLASH_WriteSector(sector,(const uint8 *)&write2,4,8);            //д���ĸ��ֽڵ����һ������ ƫ��8�ֽ� ������ʾ�£������������ǵ��Ǽ���ȡ��ַ����
    read2 = flash_read(sector,8,uint32);                             //��ȡ����

    
    
  
}

