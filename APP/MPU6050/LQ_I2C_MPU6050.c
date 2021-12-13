/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 ��ƽ    ̨�������������ܿƼ�SPIN27PS���İ�
 ����    д��chiusir
 ��E-mail  ��chiusir@163.com
 �������汾��V1.1 ��Ȩ���У���λʹ��������ϵ��Ȩ
 �������¡�2020��10��28��
 �������Ϣ�ο����е�ַ��
 ����    վ��http://www.lqist.cn
 ���Ա����̡�http://longqiu.taobao.com
 ------------------------------------------------
 ��IDE��MM32SPIN27PS:IAR7.8/MDK5.2�����ϰ汾  
 ��Target �� SPIN27PS
 ��SYS PLL�� 80MHz
=================================================================
����������Ƶ��ַ��https://space.bilibili.com/95313236
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include <LQ_I2C_MPU6050.h>
#include <LQ_TFT18.h>
#include <LQ_SOFTI2C.h>
#include <stdio.h>

#include <LQ_UART.h>

#include <math.h>
/**
  * @brief    ����ȷ��ʱ
  *
  * @param    ��
  *
  * @return   ��
  *
  * @note     ��
  *
  * @see      delayms_mpu9250(100);
  *
  * @date     2020/12/10 ������
*/
void delayms_mpu6050(uint16_t ms)
{
    int i;
  for (i = 0; i < 300; ++i)
  {
    __asm("NOP"); /* delay */
  }
//  while(ms--)
//  {
//      uint16_t  i = 300;
//      while(i--)
//      {
//          NOP(50);
//      }
//  }
}


/**
  * @brief    ��ʼ��MPU6050
  *
  * @param    ��
  *
  * @return   0����ʼ���ɹ�   1��ʧ��
  *
  * @note     ʹ��ǰ�ȳ�ʼ��IIC�ӿ�
  *
  * @see      MPU9250_Init();
  *
  * @date     2020/12/15 ���ڶ�
  */
unsigned char MPU6050_Init(void)
{
    int  res;
    SOFT_IIC_Init();                                           //MPU9250 ֧��400K IIC

    res = 0;
    res += MPU6050_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT1_REG,0X80);//��λMPU9250
    delayms_mpu6050(100);  //��ʱ100ms
    res += MPU6050_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT1_REG,0X00);//����MPU9250
    res += MPU6050_Set_Gyro_Fsr(3);                                //�����Ǵ�����,��2000dps
    res += MPU6050_Set_Accel_Fsr(1);                               //���ٶȴ�����,��4g
    res += MPU6050_Set_Rate(1000);                                 //���ò�����1000Hz
    res += MPU6050_Write_Byte(MPU6050_ADDR,MPU_INT_EN_REG,0X00);   //�ر������ж�
    res += MPU6050_Write_Byte(MPU6050_ADDR,MPU_CFG_REG,0x02);      //�������ֵ�ͨ�˲���   98hz
    res += MPU6050_Write_Byte(MPU6050_ADDR,MPU_USER_CTRL_REG,0X00);//I2C��ģʽ�ر�
    res += MPU6050_Write_Byte(MPU6050_ADDR,MPU_FIFO_EN_REG,0X00);  //�ر�FIFO
    res += MPU6050_Write_Byte(MPU6050_ADDR,MPU_INTBP_CFG_REG,0X82);//INT���ŵ͵�ƽ��Ч������bypassģʽ
    res += MPU6050_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT1_REG,0X01);//����CLKSEL,PLL X��Ϊ�ο�
    res += MPU6050_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT2_REG,0X00);//���ٶ��������Ƕ�����


    if(res == 0)  //����Ĵ�����д��ɹ�
    {
        UART_PutStr(UART1, "MPU6050 set is OK!\r\n");
    }
    else{
        UART_PutStr(UART1, "MPU6050 set is error!\r\n");
        return 1;
    }

    res = MPU6050_Read_Byte(MPU6050_ADDR,WHO_AM_I);     //��ȡMPU6500��ID
    if( (res == MPU6500_ID1) )  //����ID��ȷ
        UART_PutStr(UART1, "MPU6500_ID1 is OK!\r\n");
    else
    {
        //printf("ID=%#X\r\n",res);
        //printf("MPU9250 is fail!\n");
        UART_PutStr(UART1,"MPU6500_ID1 is error!\r\n");
        UART_PutChar(UART1, res);
        UART_PutStr(UART1,"\r\n");
        UART_PutChar(UART1, MPU6500_ID1);
        UART_PutStr(UART1,"\r\n");
        while(1);
    }


    return 0;
}




/**
  * @brief    ���������ǲ�����Χ
  *
  * @param    fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
  *
  * @return   0 �����óɹ�
  *
  * @note     ��
  *
  * @see      MPU9250_Set_Gyro_Fsr(3);        //�����Ǵ�����,��2000dps
  *
  * @date     2020/12/10 ������
  */
unsigned char MPU6050_Set_Gyro_Fsr(uint8_t fsr)
{
    return MPU6050_Write_Byte(MPU6050_ADDR,MPU_GYRO_CFG_REG,fsr<<3);
}



/**
  * @brief    ���ü��ٶȼƲ�����Χ
  *
  * @param    fsr:0,��2g;1,��4g;2,��8g;3,��16g
  *
  * @return   0�����óɹ�
  *
  * @note     ��
  *
  * @see      MPU9250_Set_Accel_Fsr(1);       //���ٶȴ�����,��4g
  *
  * @date     2020/12/10 ������
  */
unsigned char MPU6050_Set_Accel_Fsr(uint8_t fsr)
{
    return MPU6050_Write_Byte(MPU6050_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);
}



/**
  * @brief    �������ֵ�ͨ�˲�
  *
  * @param    lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
  *
  * @return   0�����óɹ�
  *
  * @note     ��
  *
  * @see      MPU6050_Set_LPF(100);
  *
  * @date     2020/12/10 ������
  */
unsigned char MPU6050_Set_LPF(uint16_t lpf)
{
    unsigned char  data=0;
    if(lpf>=188)data=1;
    else if(lpf>=98)data=2;
    else if(lpf>=42)data=3;
    else if(lpf>=20)data=4;
    else if(lpf>=10)data=5;
    else data=6;
    return MPU6050_Write_Byte(MPU6050_ADDR,MPU_CFG_REG,data);//�������ֵ�ͨ�˲���
}




/**
  * @brief    ���ò�����
  *
  * @param    rate:4~1000(Hz)
  *
  * @return   0�����óɹ�
  *
  * @note     ��
  *
  * @see      MPU9250_Set_Rate(1000);              //���ò�����1000Hz
  *
  * @date     2020/12/10 ������
  */
unsigned char MPU6050_Set_Rate(uint16_t rate)
{
    unsigned char  data;
    if(rate>1000)rate=1000;
    if(rate<4)rate=4;
    data = 1000/rate-1;
    MPU6050_Write_Byte(MPU6050_ADDR,MPU_SAMPLE_RATE_REG,data);      //�������ֵ�ͨ�˲���
    return MPU6050_Set_LPF(rate/2);                                 //�Զ�����LPFΪ�����ʵ�һ��
}



/**
  * @brief    ��ȡ�¶�ֵ
  *
  * @param    ��
  *
  * @return   �¶�ֵ(������100��)
  *
  * @note     ��
  *
  * @see      signed short temp = MPU9250_Get_Temperature();
  *
  * @date     2020/12/10 ������
  */
short MPU6050_Get_Temperature(void)
{
    unsigned char  buf[2];
    short raw;
    float temp;
    MPU6050_Read_Len(MPU6050_ADDR,MPU_TEMP_OUTH_REG,2,buf);
    raw=((uint16_t)buf[0]<<8)|buf[1];
    temp=21+((double)raw)/333.87;
    return (short)temp*100;
}





/**
  * @brief    ��ȡ������ֵ
  *
  * @param    gx,gy,gz:������x,y,z���ԭʼ����(������)
  *
  * @return   0����ȡ�ɹ�
  *
  * @note     ��
  *
  * @see      signed short data[3];
  * @see      MPU9250_Get_Gyroscope(&data[0], &data[1], &data[2]);
  *
  * @date     2020/12/10 ������
  */
unsigned char MPU6050_Get_Gyroscope(signed short *gx,signed short *gy,signed short *gz)
{
    unsigned char  buf[6],res;
    res=MPU6050_Read_Len(MPU6050_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
    if(res==0)
    {
        *gx=((uint16_t)buf[0]<<8)|buf[1];
        *gy=((uint16_t)buf[2]<<8)|buf[3];
        *gz=((uint16_t)buf[4]<<8)|buf[5];
    }
    return res;
}

/**
  * @brief    ��ȡ���ٶ�ֵ
  *
  * @param    ax,ay,az:������x,y,z���ԭʼ����(������)
  *
  * @return   0����ȡ�ɹ�
  *
  * @note     ��
  *
  * @see      signed short data[3];
  * @see      MPU9250_Get_Accelerometer(&data[0], &data[1], &data[2]);
  *
  * @date     2020/12/10 ������
  */
unsigned char MPU6050_Get_Accelerometer(signed short *ax,signed short *ay,signed short *az)
{
    unsigned char  buf[6],res;
    res=MPU6050_Read_Len(MPU6050_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
    if(res==0)
    {
        *ax=((uint16_t)buf[0]<<8)|buf[1];
        *ay=((uint16_t)buf[2]<<8)|buf[3];
        *az=((uint16_t)buf[4]<<8)|buf[5];
    }
    return res;
}

/**
  * @brief    ��ȡ ���ٶ�ֵ ���ٶ�ֵ
  *
  * @param    ax,ay,az:������x,y,z���ԭʼ����(������)
  * @param    gx,gy,gz:������x,y,z���ԭʼ����(������)
  *
  * @return   0����ȡ�ɹ�
  *
  * @note     ��
  *
  * @see      signed short data[6];
  * @see      MPU9250_Get_Raw_data(&data[0], &data[1], &data[2],&data[3], &data[4], &data[5]);
  *
  * @date     2020/12/10 ������
  */
unsigned char MPU6050_Get_Raw_data(signed short *ax,signed short *ay,signed short *az,signed short *gx,signed short *gy,signed short *gz)
{
    unsigned char  buf[14],res;
    res=MPU6050_Read_Len(MPU6050_ADDR,MPU_ACCEL_XOUTH_REG,14,buf);
    if(res==0)
    {
        *ax=((uint16_t)buf[0]<<8)|buf[1];
        *ay=((uint16_t)buf[2]<<8)|buf[3];
        *az=((uint16_t)buf[4]<<8)|buf[5];
        *gx=((uint16_t)buf[8]<<8)|buf[9];
        *gy=((uint16_t)buf[10]<<8)|buf[11];
        *gz=((uint16_t)buf[12]<<8)|buf[13];
    }
    return res;
}

/**
  * @brief    IIC ����д
  *
  * @param    addr:������ַ
  * @param    reg :Ҫд��ļĴ�����ַ
  * @param    len :Ҫд��ĳ���
  * @param    buf :д�뵽�����ݴ洢��
  *
  * @return   0 ��д��ɹ�
  *
  * @note     �ײ����� ��ֲʱ��Ҫ�޸�
  *
  * @see      unsigned char buf[14];
  * @see      MPU9250_Write_Len(MPU9250_ADDR,MPU_ACCEL_XOUTH_REG,14,buf);
  *
  * @date     2020/12/10 ������
  */
unsigned char MPU6050_Write_Len(unsigned char addr,unsigned char reg,unsigned char len,unsigned char *buf)
{
    return IIC_WriteMultByteToSlave(addr<<1, reg, len, buf);
}


/**
  * @brief    IIC ������
  *
  * @param    addr:������ַ
  * @param    reg :Ҫ��ȡ�ļĴ�����ַ
  * @param    len :Ҫ��ȡ�ĳ���
  * @param    buf :��ȡ�������ݴ洢��
  *
  * @return   0 ����ȡ�ɹ�
  *
  * @note     �ײ����� ��ֲʱ��Ҫ�޸�
  *
  * @see      unsigned char buf[14];
  * @see      MPU9250_Read_Len(MPU9250_ADDR,MPU_ACCEL_XOUTH_REG,14,buf);
  *
  * @date     2020/12/10 ������
  */
unsigned char MPU6050_Read_Len(unsigned char addr,unsigned char reg,unsigned char len,unsigned char *buf)
{
    return IIC_ReadMultByteFromSlave(addr<<1, reg, len, buf);
}



/**
  * @brief    IIC дһ���Ĵ���
  *
  * @param    addr  :������ַ
  * @param    reg   :�Ĵ�����ַ
  * @param    value :Ҫд���ֵ
  *
  * @return   0 ����ȡ�ɹ�
  *
  * @note     �ײ����� ��ֲʱ��Ҫ�޸�
  *
  * @see      MPU9250_Write_Byte(MPU9250_ADDR,MPU_SAMPLE_RATE_REG,1);
  *
  * @date     2020/12/10 ������
  */
unsigned char MPU6050_Write_Byte(unsigned char addr,unsigned char reg,unsigned char value)
{
    return IIC_WriteByteToSlave(addr<<1, reg, value);
}


/**
  * @brief    IIC ��һ���Ĵ���
  *
  * @param    addr  :������ַ
  * @param    reg   :�Ĵ�����ַ
  *
  * @return   ��ȡ��ֵ
  *
  * @note     �ײ����� ��ֲʱ��Ҫ�޸�
  *
  * @see      MPU9250_Read_Byte(MPU9250_ADDR,WHO_AM_I);
  *
  * @date     2020/12/10 ������
  */
unsigned char MPU6050_Read_Byte(unsigned char addr,unsigned char reg)
{
    unsigned char value[1];
    IIC_ReadByteFromSlave(addr<<1, reg, &value[0]);
    return value[0];
}

// ͨ�����ٶȼƻ�ȡ��������
float MPU6050_Get_Angle(short accx, short accy, short accz, char choose)
{
    double temp;
    float res = 0;
    switch(choose)
    {
        case 0 ://����Ȼx��ĽǶ�
            temp = (float)accx / sqrt( (accy * accy + accz * accz) );
            res = atan(temp);
            break;
        case 1 ://����Ȼy��ĽǶ�
            temp = (float)accy / sqrt((accx * accx + accz * accz));
            res = atan(temp);
            break;
        case 2 ://����Ȼz��ĽǶ�
            temp = (float)accz / sqrt(accx * accx + accy * accy);
            res = atan(temp);
            break;
    }
    return res * 1800 / 3.1415;
}


//���Գ���
//��λΪ0.1����ÿ��
void Test_MPU6050(void)
{
    TFTSPI_Init(1);     //TFT1.8��ʼ��
    TFTSPI_CLS(u16BLACK);    //����
    char txt[30];
    short aacx, aacy, aacz;
    short gyrox, gyroy, gyroz;
    MPU6050_Set_LPF(10);
    SOFT_IIC_Init();
    TFTSPI_P8X16Str(2,0,"LQ 6050 Test",u16RED,u16BLACK);
    UART_PutStr(UART1, "LQ MPU6050 Test\r\n");
    MPU6050_Init();
    if(MPU6050_Init())
    {
        TFTSPI_P8X16Str(1,0,"6050 Test Fail",u16RED,u16BLACK);
        UART_PutStr(UART1,"MPU6050 init Fail\r\n");
        while(1);
    }
    while(1)
    {
        MPU6050_Get_Raw_data(&aacx,&aacy,&aacz,&gyrox,&gyroy,&gyroz);   //�õ����ٶȴ���������
        sprintf((char*)txt,"ax:%06d",aacx);
        TFTSPI_P8X16Str(0,1,txt,u16RED,u16BLACK);
        sprintf((char*)txt,"ay:%06d",aacy);
        TFTSPI_P8X16Str(0,2,txt,u16RED,u16BLACK);
        sprintf((char*)txt,"az:%06d",aacz);
        TFTSPI_P8X16Str(0,3,txt,u16RED,u16BLACK);
        sprintf((char*)txt,"gx:%06d",gyrox);
        TFTSPI_P8X16Str(0,4,txt,u16RED,u16BLACK);
        sprintf((char*)txt,"gy:%06d",gyroy);
        TFTSPI_P8X16Str(0,5,txt,u16RED,u16BLACK);
        sprintf((char*)txt,"gz:%06d",gyroz);
        TFTSPI_P8X16Str(0,6,txt,u16RED,u16BLACK);



    }
}


