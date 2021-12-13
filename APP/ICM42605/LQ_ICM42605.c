/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
------------------------------------------------
��dev.env.��IAR/KEIL
��Target �� MM32...G9P
________________________________________________________________

QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include "include.h"
#include "LQ_ICM42605.h"

//#define SPI_SCK      PTE2  //SCK
//#define SPI_MOSI     PTE3  //SDI-MOSI
//#define SPI_MISO     PTE4  //SDO-MISO
//#define SPI_CS       PTE5  //CS
//
//#define SPI_SCK_OUT       PTE2_OUT
//#define SPI_MOSI_OUT      PTE3_OUT
//#define SPI_MISO_IN       PTE4_IN
//#define SPI_CS_OUT        PTE5_OUT

/*************************************************************************
*  �������ƣ�void SPI_SoftInit(void)
*  ����˵����ģ��SPI�ڳ�ʼ��
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��8��10��
*  ��    ע��   
*************************************************************************/
//void SPI_SoftInit(void)
//{
//  GPIO_PinInit(SPI_SCK,  GPO, 1);
//  GPIO_PinInit(SPI_MISO, GPI, 0);
//  GPIO_PinInit(SPI_MOSI, GPO, 0);
//  GPIO_PinInit(SPI_CS,   GPO, 1);  
//}
/*************************************************************************
*  �������ƣ�void SPI_SoftReadWriteNbyte(uint8_t *lqbuff, uint16_t len)
*  ����˵����SPI��д���ݼ�����
*  ����˵����uint8_t *buf����ָ��,uint16_t len����
*  �������أ�
*  �޸�ʱ�䣺2020��8��10��
*  ��    ע��   
*************************************************************************/
//void SPI_SoftReadWriteNbyte(uint8_t *lqbuff, uint16_t len)
//{
//  SPI_CS_OUT = 0;
//  SPI_SCK_OUT= 1;
//  do
//  {
//    for(uint8_t i = 0; i < 8; i++)
//    {
//      SPI_MOSI_OUT = ((*lqbuff) >= 0x80);
//      SPI_SCK_OUT= 0;
//      (*lqbuff) = (*lqbuff)<<1;      
//      
//      SPI_SCK_OUT= 1;
//      
//      (*lqbuff) |= SPI_MISO_IN;          
//    }
//    lqbuff++;
//  }while(--len);
//  SPI_CS_OUT = 1;
//}
/*************************************************************************
*  �������ƣ�void ICM_Read_Len(uint8_t reg,uint8_t len,uint8_t *buf)
*  ����˵����uint8_t reg��ʼ�Ĵ���,uint8_t *buf����ָ��,uint16_t len����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��8��10��
*  ��    ע��   
*************************************************************************/
  void ICM_Read_Len(uint8_t reg,uint8_t len,uint8_t *buf)
  {   
    buf[0] = reg | 0x80;
    
    Soft_SPI_ReadWriteNbyte(buf, len + 1);  
  }
/*************************************************************************
*  �������ƣ�uint8_t icm42605_read_reg(uint8_t reg)
*  ����˵������ȡ����
*  ����˵����uint8_t reg�Ĵ�����ַ
*  �������أ���
*  �޸�ʱ�䣺2020��8��10��
*  ��    ע��   
*************************************************************************/
uint8_t icm42605_read_reg(uint8_t reg)
{
  uint8_t lqbuff[2];
  lqbuff[0] = reg | 0x80;          //�ȷ��ͼĴ���
  
  Soft_SPI_ReadWriteNbyte(lqbuff, 2);
  
  
  return lqbuff[1];
}
/*************************************************************************
*  �������ƣ�uint8_t icm42605_write_reg(uint8_t reg,uint8_t value)
*  ����˵������Ĵ���д����
*  ����˵����uint8_t reg�Ĵ���,uint8_t value����
*  �������أ���
*  �޸�ʱ�䣺2020��8��10��
*  ��    ע��   
*************************************************************************/
uint8_t icm42605_write_reg(uint8_t reg,uint8_t value)
{
  uint8_t lqbuff[2];
  
  lqbuff[0] = reg;          //�ȷ��ͼĴ���
  lqbuff[1] = value;        //�ٷ�������
  
  Soft_SPI_ReadWriteNbyte(lqbuff, 2);//����buff�����ݣ����ɼ��� buff��
  return 0;
}
/*************************************************************************
*  �������ƣ�uint8_t icm42605_read_regs(uint8_t reg,uint8_t *buf,uint16_t len)
*  ����˵����������ȡ����
*  ����˵����uint8_t reg��ʼ�Ĵ���,uint8_t *buf����ָ��,uint16_t len����
*  �������أ���
*  �޸�ʱ�䣺2020��8��10��
*  ��    ע��   
*************************************************************************/
uint8_t icm42605_read_regs(uint8_t reg,uint8_t *buf,uint16_t len)
{   
  buf[0] = reg | 0x80;
  /* д��Ҫ���ļĴ�����ַ */
  Soft_SPI_ReadWriteNbyte(buf, len + 1);
  return 0;
}
/*************************************************************************
*  �������ƣ�void ICM_Get_Gyroscope(short *gx,short *gy,short *gz)
*  ����˵����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��8��10��
*  ��    ע��   
*************************************************************************/
void ICM_Get_Gyroscope(short *gx,short *gy,short *gz)
{
  uint8_t buf[7]; 
  ICM_Read_Len(ICM_GYRO_XOUTH_REG,6,buf);
  
  *gx=((uint16_t)buf[1]<<8)|buf[2];  
  *gy=((uint16_t)buf[3]<<8)|buf[4];  
  *gz=((uint16_t)buf[5]<<8)|buf[6];
  
}
/*************************************************************************
*  �������ƣ�void ICM_Get_Accelerometer(short *ax,short *ay,short *az)
*  ����˵����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��8��10��
*  ��    ע��   
*************************************************************************/
void ICM_Get_Accelerometer(short *ax,short *ay,short *az)
{
  uint8_t buf[7];  
  ICM_Read_Len(ICM_ACCEL_XOUTH_REG,6,buf);
  
  *ax=((uint16_t)buf[1]<<8)|buf[2];  
  *ay=((uint16_t)buf[3]<<8)|buf[4];  
  *az=((uint16_t)buf[5]<<8)|buf[6];  
}

/*************************************************************************
*  �������ƣ�void ICM_Get_Raw_data(short *ax,short *ay,short *az,short *gx,short *gy,short *gz)
*  ����˵������ȡ���ٶ�����������
*  ����˵��������
*  �������أ���
*  �޸�ʱ�䣺2020��8��10��
*  ��    ע��   
*************************************************************************/
void ICM_Get_Raw_data(short *ax,short *ay,short *az,short *gx,short *gy,short *gz)
{
  uint8_t buf[13];  
  ICM_Read_Len(ICM_ACCEL_XOUTH_REG,12,buf);
  
  *ax=((uint16_t)buf[1]<<8)|buf[2];  
  *ay=((uint16_t)buf[3]<<8)|buf[4];  
  *az=((uint16_t)buf[5]<<8)|buf[6];
  *gx=((uint16_t)buf[7]<<8)|buf[8];  
  *gy=((uint16_t)buf[9]<<8)|buf[10];  
  *gz=((uint16_t)buf[11]<<8)|buf[12];  
}

/*************************************************************************
*  �������ƣ�uint8_t icm42605_init(void)
*  ����˵����ICM42605��ʼ��
*  ����˵������
*  �������أ�0 �ɹ���1ʧ��
*  �޸�ʱ�䣺2020��8��10��
*  ��    ע��   
*************************************************************************/
uint8_t icm42605_init(void)
{   
  uint8_t reg_val;
  char  txt[30];
  
  Soft_SPI_Init();
  
  delayms(200);
  
  reg_val = icm42605_read_reg(who_am_i);//who_am_i
  sprintf(txt,"ID:0x%02X",reg_val);
  
  UART_PutStr(UART1, txt); //�������ֲ鿴
  
#ifdef LQ_OLED  
  OLED_P8x16Str(66,2,(uint8_t*)txt);
#else
  TFTSPI_P6X8Str(11,7,(uint8_t*)txt,u16WHITE,u16BLACK);
#endif  
  icm42605_write_reg(reg_bank_sel,0x00);//Set to bank 0
  icm42605_write_reg(reg_bank_sel,0x00);//Set to bank 0
  icm42605_write_reg(device_config_reg,bit_soft_reset_chip_config);//chip soft reset
  delayms(100);  
  
  if(reg_val==0x42)
  {
    icm42605_write_reg(reg_bank_sel,0x01);//Change to bank 1
    icm42605_write_reg(intf_config4,0x02);//4 wire spi mode
    
    icm42605_write_reg(reg_bank_sel,0x00);        
    icm42605_write_reg(fifo_config_reg,0x40);//Stream-to-FIFO Mode
    
    reg_val = icm42605_read_reg(int_source0_reg);      
    icm42605_write_reg(int_source0_reg,0x00);    
    icm42605_write_reg(fifo_config2_reg,0x00);// watermark
    icm42605_write_reg(fifo_config3_reg,0x02);// watermark
    icm42605_write_reg(int_source0_reg, reg_val); 
    icm42605_write_reg(fifo_config1_reg,0x63);// Enable the accel and gyro to the FIFO
    
    icm42605_write_reg(reg_bank_sel,0x00);
    icm42605_write_reg(int_config_reg,0x36);   
    
    icm42605_write_reg(reg_bank_sel, 0x00);
    reg_val = (icm42605_read_reg(int_source0_reg)|bit_int_fifo_ths_int1_en);      
    icm42605_write_reg(int_source0_reg, reg_val);
    
    icm42605_write_reg(reg_bank_sel, 0x00);
    reg_val = ((icm42605_read_reg(accel_config0_reg)&0x1F)|(bit_accel_ui_fs_sel_8g));//8g
    icm42605_write_reg(accel_config0_reg, reg_val);
    
    icm42605_write_reg(reg_bank_sel, 0x00);
    reg_val = ((icm42605_read_reg(accel_config0_reg)&0xF0)|bit_accel_odr_50hz);
    icm42605_write_reg(accel_config0_reg, reg_val); 
    
    icm42605_write_reg(reg_bank_sel, 0x00);
    reg_val = ((icm42605_read_reg(gyro_config0_reg)&0x1F)|(bit_gyro_ui_fs_sel_1000dps));//1000dps
    icm42605_write_reg(gyro_config0_reg,reg_val);
    
    icm42605_write_reg(reg_bank_sel, 0x00);      
    reg_val = ((icm42605_read_reg(gyro_config0_reg)&0xF0)|bit_gyro_odr_50hz);
    icm42605_write_reg(gyro_config0_reg, reg_val); 
    
    icm42605_write_reg(reg_bank_sel, 0x00);
    reg_val = icm42605_read_reg(pwr_mgmt0_reg)|(bit_accel_mode_ln); // Accel on in LNM
    icm42605_write_reg(pwr_mgmt0_reg, reg_val);  
    delayms(1);   
    
    icm42605_write_reg(reg_bank_sel, 0x00);
    reg_val = icm42605_read_reg(pwr_mgmt0_reg)|(bit_gyro_mode_ln); // Gyro on in LNM
    icm42605_write_reg(pwr_mgmt0_reg, reg_val);  
    delayms(1);   
    printf("ok");
    
    UART_PutStr(UART1, "ok");  //����������ʾ
    
#ifdef LQ_OLED
    OLED_P6x8Str(66,4,(uint8_t*)"Init Pass");
#else
    TFTSPI_P6X8Str(11,8,"Init Pass",u16RED,u16BLUE);
    printf("Init Pass!");
#endif
    return 0;
  }
  else 
  {
#ifdef LQ_OLED
    OLED_P6x8Str(66,4,(uint8_t*)"Init Fail");
#else
    TFTSPI_P8X16Str(0,8,"Init Fail",u16RED,u16BLUE);
    printf("Init Fail!");
#endif    
    return 1;
  }   
}
/*************************************************************************
*  �������ƣ�void icm42605_read_fifo(Sample_data_type_t *data,uint16_t len)
*  ����˵������ȡFIFO
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��8��10��
*  ��    ע��   
*************************************************************************/
uint8_t  fifocount_l, fifocount_h;
uint16_t fifocount;

void icm42605_read_fifo(Sample_data_type_t *data,uint16_t len)
{
  uint8_t reg_val;
  uint8_t tempbuff[512]={0};
  reg_val = icm42605_read_reg(int_source0_reg);      
  icm42605_write_reg(int_source0_reg,0x00); 
  
  fifocount_h = icm42605_read_reg(fifo_byte_count_h_res); // Read the FIFO size
  fifocount_l = icm42605_read_reg(fifo_byte_count_l_res);
  fifocount = (fifocount_h<<8)|fifocount_l;
  
  icm42605_read_regs(fifo_data_port,tempbuff,len);
  if(fifocount>=fifo_packet_size) // If we have a complete packet in the FIFO
  {
    for(uint8_t i=0;i<32;i++)
    {
      if((tempbuff[i*16]&fifo_accel_en)&&(tempbuff[i*16]&fifo_gyro_en))
      {
        data->Sample_accdata[0+i*3] = ((int16_t)((tempbuff[1+i*16] << 8) | tempbuff[2+i*16]))*acc_ssl;
        data->Sample_accdata[1+i*3] = ((int16_t)((tempbuff[3+i*16] << 8) | tempbuff[4+i*16]))*acc_ssl;
        data->Sample_accdata[2+i*3] = ((int16_t)((tempbuff[5+i*16] << 8) | tempbuff[6+i*16]))*acc_ssl;
        data->Sample_gyrdata[0+i*3] = ((int16_t)((tempbuff[7+i*16] << 8) | tempbuff[8+i*16]))/gyr_ssl;
        data->Sample_gyrdata[1+i*3] = ((int16_t)((tempbuff[9+i*16] << 8) | tempbuff[10+i*16]))/gyr_ssl;
        data->Sample_gyrdata[2+i*3] = ((int16_t)((tempbuff[11+i*16]<< 8) | tempbuff[12+i*16]))/gyr_ssl;
      }
    }
  }
  icm42605_write_reg(int_source0_reg, reg_val); 
}
/*************************************************************************
*  �������ƣ�void icm42605_stop(void)
*  ����˵����ֹͣ����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��8��10��
*  ��    ע��   
*************************************************************************/
void icm42605_stop(void)
{
  uint8_t reg_val; 
  icm42605_write_reg(reg_bank_sel, 0x00);
  reg_val = icm42605_read_reg(pwr_mgmt0_reg)&(~bit_accel_mode_ln); // Accel off
  icm42605_write_reg(pwr_mgmt0_reg, reg_val);
  delayms(1);   
  
  icm42605_write_reg(reg_bank_sel, 0x00);
  reg_val = icm42605_read_reg(pwr_mgmt0_reg)|(bit_gyro_mode_ln); // Gyro on in LNM
  icm42605_write_reg(pwr_mgmt0_reg, reg_val);   
  delayms(400);  
}

/*************************************************************************
*  �������ƣ�void Test_ICM42605FIFO(void)
*  ����˵�������ԣ���ȡ���ٶ�����������
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��8��10��
*  ��    ע��   
*************************************************************************/
#ifdef LQ_OLED
void Test_ICM42605FIFO(void)
{
  unsigned char  txt[30];
  //Sample_data_type_t *data;
  short aacx,aacy,aacz;	        //���ٶȴ�����ԭʼ����
  short gyrox,gyroy,gyroz;        //������ԭʼ����
  OLED_Init();                  //LCD��ʼ��
  OLED_CLS();                   //LCD����
  OLED_P8x16Str(8,0,"ICM42605 Test"); 
  printf("\r\nLQ ICM42605 Test"); 
    
  UART_PutStr(UART1, "\r\nLQ ICM42605 Test");//����������ʾ

  icm42605_init();
  
  while(1)
  {
    ICM_Get_Raw_data(&aacx,&aacy,&aacz,&gyrox,&gyroy,&gyroz);	//�õ����ٶȴ���������  
    sprintf((char*)txt,"ax:%06d",aacx);
    UART_PutStr(UART1, txt);//����������ʾ
    OLED_P6x8Str(0,2,txt);
    sprintf((char*)txt,"ay:%06d",aacy);
    UART_PutStr(UART1, txt);//����������ʾ
    OLED_P6x8Str(0,3,txt);
    sprintf((char*)txt,"az:%06d",aacz);
    UART_PutStr(UART1, txt);//����������ʾ
    OLED_P6x8Str(0,4,txt);
    sprintf((char*)txt,"gx:%06d",gyrox);
    UART_PutStr(UART1, txt);//����������ʾ
    OLED_P6x8Str(0,5,txt);
    sprintf((char*)txt,"gy:%06d",gyroy);
    UART_PutStr(UART1, txt);//����������ʾ
    OLED_P6x8Str(0,6,txt);
    sprintf((char*)txt,"gz:%06d",gyroz);
    UART_PutStr(UART1, txt);//����������ʾ
    OLED_P6x8Str(0,7,txt);
    
    printf("\r\nAX: %d  ",aacx); 
    printf("\r\nAY: %d  ",aacy);
    printf("\r\nAZ: %d  ",aacz); 
    printf("\r\nGX: %d  ",gyrox);
    printf("\r\nGY: %d  ",gyroy); 
    printf("\r\nGZ: %d  ",gyroz);
    delayms(100);
    printf("\r\n*********************\n");
    UART_PutStr(UART1, "\r\n*********************\n");//����������ʾ
  } 
}
#endif
/*************************************************************************
*  �������ƣ�void Test_ICM42605FIFO(void)
*  ����˵�������ԣ���ȡ���ٶ�����������
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2020��8��10��
*  ��    ע��   
*************************************************************************/
//#ifdef LQ_TFT1_8
#ifdef __LQ_SGP18T_TFTSPI_H__
void Test_ICM42605FIFO(void)
{
  unsigned char  txt[30];
  //Sample_data_type_t *data;
  short aacx,aacy,aacz;	        //���ٶȴ�����ԭʼ����
  short gyrox,gyroy,gyroz;        //������ԭʼ����
  //LED_Init();
  //UART_Init(UART4, 115200);
  TFTSPI_Init(0);                //TFT1.8��ʼ��  
  TFTSPI_CLS(u16BLUE);           //����
  TFTSPI_P6X8Str(11,0,"ICM42605 Test",u16WHITE,u16BLACK); 
  printf("\r\nLQ ICM42605 Test");  
    
  UART_PutStr(UART1, "\r\nLQ ICM42605 Test");//����������ʾ
    
  icm42605_init();
  
  int i = 0;
  while(1)
  {
    ICM_Get_Raw_data(&aacx,&aacy,&aacz,&gyrox,&gyroy,&gyroz);	//�õ����ٶȴ���������  
    sprintf((char*)txt,"ax:%06d",aacx);
    //UART_PutStr(UART1, txt);//����������ʾ
    TFTSPI_P6X8Str(11,1,txt,u16WHITE,u16BLACK);
    sprintf((char*)txt,"ay:%06d",aacy);
    //UART_PutStr(UART1, txt);//����������ʾ
    TFTSPI_P6X8Str(11,2,txt,u16WHITE,u16BLACK);
    sprintf((char*)txt,"az:%06d",aacz);
    //UART_PutStr(UART1, txt);//����������ʾ
    TFTSPI_P6X8Str(11,3,txt,u16WHITE,u16BLACK);
    sprintf((char*)txt,"gx:%06d",gyrox);
    //UART_PutStr(UART1, txt);//����������ʾ
    TFTSPI_P6X8Str(11,4,txt,u16WHITE,u16BLACK);
    sprintf((char*)txt,"gy:%06d",gyroy);
    //UART_PutStr(UART1, txt);//����������ʾ
    TFTSPI_P6X8Str(11,5,txt,u16WHITE,u16BLACK);
    sprintf((char*)txt,"gz:%06d",gyroz);
    //UART_PutStr(UART1, txt);//����������ʾ
    TFTSPI_P6X8Str(11,6,txt,u16WHITE,u16BLACK);
    
    printf("\r\nAX: %d  ",aacx); 
    printf("\r\nAY: %d  ",aacy);
    printf("\r\nAZ: %d  ",aacz); 
    printf("\r\nGX: %d  ",gyrox);
    printf("\r\nGY: %d  ",gyroy); 
    printf("\r\nGZ: %d  ",gyroz);
    delayms(100);
    printf("\r\n*********************\n");
    //UART_PutStr(UART1, "\r\n*********************\n");//����������ʾ
  } 
}
#endif
