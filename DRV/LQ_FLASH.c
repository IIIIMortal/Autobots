/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨�������������ܿƼ�F3277���İ��ĸ��
����    д������Ƽ�
��E-mail  ��chiusir@163.com
������汾��V1.0 ��Ȩ���У���λʹ��������ϵ��Ȩ
�������¡�2020��12��24�գ��������£����ע���°棡
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://longqiu.taobao.com
------------------------------------------------
��IDE��IAR7.8 KEIL5.24�����ϰ汾
��Target �� MM32F3277
��SYS PLL�� 120MHz Ƶ��̫�߿����޷�����system_mm32f327x.c
=================================================================
����������Ƶ��ַ��https://space.bilibili.com/95313236
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include "include.h"
#include "HAL_conf.h"
#include "HAL_device.h"
#include "HAL_flash.h"
#include "LQ_FLASH.h"

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
����������u16 LQFLASH_ReadByte(u32 faddr)
����  �ܡ���ȡָ����ַ�İ���(16λ����)
������ֵ��faddr:����ַ(�˵�ַ����Ϊ2�ı���!!)
������ֵ����Ӧ��ַ������.
����  �ߡ�L Q
�������¡�2021��1��22�� 
������汾��V1.0
������������buf=LQFLASH_ReadByte(faddr)
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
u16 LQFLASH_ReadByte(u32 faddr)
{
  return *(vu16*)faddr; 
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
����������void LQFLASH_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)   
����  �ܡ�������д��(16λ����)
������ֵ��riteAddr:��ʼ��ַ
������ֵ��pBuffer:����ָ��
������ֵ��NumToWrite:����(16λ)��   
������ֵ����
����  �ߡ�L Q
�������¡�2021��1��22�� 
������汾��V1.0
������������buf=LQFLASH_ReadByte(faddr)
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void LQFLASH_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)   
{ 			 		 
  u16 i;
  for(i=0;i<NumToWrite;i++)
  {
    FLASH_ProgramHalfWord(WriteAddr,pBuffer[i]);
    WriteAddr+=2;//��ַ����2.
  }  
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
����������void LQFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)	
����  �ܡ���ָ����ַ��ʼд��ָ�����ȵ�����
������ֵ��WriteAddr:��ʼ��ַ(�˵�ַ����Ϊ2�ı���!!)
������ֵ��pBuffer:����ָ��
������ֵ��NumToWrite:����(16λ)��(����Ҫд���16λ���ݵĸ���.)
������ֵ����
����  �ߡ�L Q
�������¡�2021��1��22�� 
������汾��V1.0
������������buf=LQFLASH_ReadByte(faddr)
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#if LQ_FLASH_SIZE<256
#define LQ_SECTOR_SIZE 1024 //�ֽ�
#else 
#define LQ_SECTOR_SIZE	2048
#endif		 
u16 LQFLASH_BUF[LQ_SECTOR_SIZE/2];//�����2K�ֽ�

void LQFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)	
{
  u32 secpos;	   //������ַ
  u16 secoff;	   //������ƫ�Ƶ�ַ(16λ�ּ���)
  u16 secremain;   //������ʣ���ַ(16λ�ּ���)	   
  u16 i;    
  u32 offaddr;     //ȥ��0X08000000��ĵ�ַ
  if(WriteAddr<FLASH_BASE_ADDR||(WriteAddr>=(FLASH_BASE_ADDR+1024*FLASH_SIZE)))return;//�Ƿ���ַ
  FLASH_Unlock();						//����
  offaddr=WriteAddr-FLASH_BASE_ADDR;		//ʵ��ƫ�Ƶ�ַ.
  secpos=offaddr/LQ_SECTOR_SIZE;			//������ַ  0~127 for LQF103RBT6
  secoff=(offaddr%LQ_SECTOR_SIZE)/2;		//�������ڵ�ƫ��(2���ֽ�Ϊ������λ.)
  secremain=LQ_SECTOR_SIZE/2-secoff;		//����ʣ��ռ��С   
  if(NumToWrite<=secremain)secremain=NumToWrite;//�����ڸ�������Χ
  while(1) 
  {	
    LQFLASH_Read(secpos*LQ_SECTOR_SIZE+FLASH_BASE_ADDR,LQFLASH_BUF,LQ_SECTOR_SIZE/2);//������������������
    for(i=0;i<secremain;i++)//У������
    {
      if(LQFLASH_BUF[secoff+i]!=0XFFFF) break;//��Ҫ����  	  
    }
    if(i<secremain)//��Ҫ����
    {
      FLASH_ErasePage(secpos*LQ_SECTOR_SIZE+FLASH_BASE_ADDR);//�����������
      for(i=0;i<secremain;i++)//����
      {
        LQFLASH_BUF[i+secoff]=pBuffer[i];	  
      }
      LQFLASH_Write_NoCheck(secpos*LQ_SECTOR_SIZE+FLASH_BASE_ADDR,LQFLASH_BUF,LQ_SECTOR_SIZE/2);//д����������  
    }else LQFLASH_Write_NoCheck(WriteAddr,pBuffer,secremain);//д�Ѿ������˵�,ֱ��д������ʣ������. 				   
    if(NumToWrite==secremain)break;//д�������
    else//д��δ����
    {
      secpos++;				//������ַ��1
      secoff=0;				//ƫ��λ��Ϊ0 	 
      pBuffer+=secremain;  	//ָ��ƫ��
      WriteAddr+=secremain;	//д��ַƫ��	   
      NumToWrite-=secremain;	//�ֽ�(16λ)���ݼ�
      if(NumToWrite>(LQ_SECTOR_SIZE/2))secremain=LQ_SECTOR_SIZE/2;//��һ����������д����
      else secremain=NumToWrite;//��һ����������д����
    }	 
  };	
  FLASH_Lock();//����
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
����������void LQFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead)   	
����  �ܡ���ָ����ַ��ʼ����ָ�����ȵ�����
������ֵ��ReadAddr:��ʼ��ַ
������ֵ��pBuffer:����ָ��
������ֵ��NumToWrite:����(16λ)��
������ֵ����
����  �ߡ�L Q
�������¡�2021��1��22�� 
������汾��V1.0
������������buf=LQFLASH_ReadByte(faddr)
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void LQFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead)   	
{
  u16 i;
  for(i=0;i<NumToRead;i++)
  {
    pBuffer[i]=LQFLASH_ReadByte(ReadAddr);//��ȡ2���ֽ�.
    ReadAddr+=2;//ƫ��2���ֽ�.	
  }
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
����������void Test_EEPROM(void)
����  �ܡ�����FLASH��д��ͨ���������в���
������ֵ����
������ֵ����
����  �ߡ�L Q
�������¡�2021��1��22�� 
������汾��V1.0
������������
---------------------------------------------------------------
//����ĸ����KEY0������д�뵱ǰ������ָ����ַ
//����ĸ����KEY2��������ȡָ����ַ���ݵ���Ļ
//��λ��Ƭ���������Ȼ�ܶ�ȡ��ԭ��д������ݣ�˵������ɹ�
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

u8 TEXT_Buffer[]={"000LQ FLASH TEST"};
#define SIZE sizeof(TEXT_Buffer)	 	//���鳤��
#define FLASH_SAVE_ADDR  0X08070000 		//����FLASH �����ַ(����Ϊż��)
u8 datatemp[SIZE];

void Test_EEPROM(void)
{	
  u8 cnt=100;
  char txt[16];  
  
  TFTSPI_Init(0);        //LCD��ʼ��  0:����  1������
  TFTSPI_CLS(u16BLUE);   //��ɫ��Ļ
  TFTSPI_Show_Logo(0,37);//��ʾ����LOGO
  TFTSPI_P16x16Str(0,0,(unsigned char*)"�����������ܿƼ�",u16RED,u16BLUE);//�ַ�����ʾ
  delayms(1000);         //��ʱ�ȴ�
  TFTSPI_Init(1);        //LCD��ʼ��  0:����  1������
  TFTSPI_CLS(u16BLUE);   //��ɫ��Ļ
  GPIO_KEY_Init();
  
  TFTSPI_P8X16Str(1,4,"WriteValue:",u16RED,u16BLUE);       //��Ļ��ʾ����  
  LQFLASH_Read(FLASH_SAVE_ADDR,(u16*)datatemp,SIZE);   //��ȡָ����ַ���ݵ���Ļ
  sprintf(txt,"Read Value:%03d",datatemp[0]);          //ת��Ϊ�ַ������Ա���Ļ��ʾ����
  TFTSPI_P8X16Str(1,3,txt,u16RED,u16BLUE);             //��Ļ��ʾ����ȡ������  
  
  while(1)
  {
    if(KEY_Read(KEY0)==0)                                  //����ĸ����KEY0�����Ƿ��£�
    {	
      TEXT_Buffer[0]=++cnt;                                //�䶯��ֵ���Ա�۲��д�Ƿ�ɹ�
      sprintf(txt,"WriteValue:%03d",cnt);                  //ת��Ϊ�ַ������Ա���Ļ��ʾ����
      TFTSPI_P8X16Str(1,4,txt,u16RED,u16BLUE);             //��Ļ��ʾ����
      
      LQFLASH_Write(FLASH_SAVE_ADDR,(u16*)TEXT_Buffer,SIZE);//д�뵱ǰ������ָ����ַ		
    }
    if(KEY_Read(KEY2)==0)                                  //����ĸ����KEY2�����Ƿ��£�
    {					
      LQFLASH_Read(FLASH_SAVE_ADDR,(u16*)datatemp,SIZE);   //��ȡָ����ַ���ݵ���Ļ
      sprintf(txt,"Read Value:%03d",datatemp[0]);          //ת��Ϊ�ַ������Ա���Ļ��ʾ����
      TFTSPI_P8X16Str(1,3,txt,u16RED,u16BLUE);             //��Ļ��ʾ����ȡ������
    }   
    LED_Ctrl(LED0,RVS);                                    //LED��˸
    delayms(1000);                                         //��ʱ�ȴ�
  }
}


