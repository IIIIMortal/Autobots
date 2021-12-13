/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
-----------------------------------------------------------------
Ĭ�ϵ���ӿ�
ʹ��TIM8�飬�ĸ�ͨ���ɲ���4·PWM
˫·�ӿ�1
PC6           TIM8CH1    ����ĸ��MOTOR1_P
PG6           GPIO       ����ĸ��MOTOR1_N
PC7           TIM8CH2    ����ĸ��MOTOR2_P
PG7           GPIO       ����ĸ��MOTOR2_N
˫·�ӿ�2
PC8           TIM8CH3    ����ĸ��MOTOR3_P
PG8           GPIO       ����ĸ��MOTOR3_N
PC9           TIM8CH4    ����ĸ��MOTOR4_P
PG9           GPIO       ����ĸ��MOTOR4_N
-----------------------------------------------------------------
Ĭ����Ļ��ʾ�ӿڣ��û�����ʹ��TFT����OLEDģ��
TFTSPI_CS     PE5      ����ĸ�� CS�ܽ� Ĭ�����ͣ����Բ���
TFTSPI_SCK    PE2      ����ĸ�� SPI SCK�ܽ�
TFTSPI_SDI    PE6      ����ĸ�� SPI MOSI�ܽ�
TFTSPI_DC     PE4      ����ĸ�� D/C�ܽ�
TFTSPI_RST    PE3      ����ĸ�� RESET�ܽ�
----------------------------------------------------------------
Ĭ�ϰ����ӿ�
KEY0p         PD4       ����ĸ���ϰ���0
KEY1p         PD2       ����ĸ���ϰ���1
KEY2p         PD3       ����ĸ���ϰ���2
DSW0p         PD0       ����ĸ���ϲ��뿪��0
DSW1p         PD1       ����ĸ���ϲ��뿪��1
-----------------------------------------------------------------
Ĭ��LED�ӿ�
LED0p         PE0       ����
LED1p         PE1       ����
-----------------------------------------------------------------
Ĭ�Ϸ������ӿ�
BEEPp         PG10      ����ĸ���Ϸ������ӿ�
�ɻɹܽӿ�
REEDp         PA2       ����ĸ���ϸɻɹܽӿ�
-----------------------------------------------------------------
����ͷ�ӿ�    �������ۻ���OV7725ģ��
�� ���ݶ˿ڣ�  PF0-PF7�ڣ���8λ��������ͷ�����ݶ˿ڣ�
�� ʱ�����أ�  �ⲿ�ж�PE7��
�� ���źţ�    �ⲿ�ж�PE8/�����DMA+TIMER1:PE7
�� ���źţ�    �ⲿ�ж�PG1
�� I2C��       PF8 I2C1-SCL
PF9 I2C1-SDA
-----------------------------------------------------------------
Ĭ�϶���ӿ�
ʹ��TIM5�飬�ĸ�ͨ���ɲ���4·PWM
PA0           �����ĸ����
PA1           �����ĸ����
-----------------------------------------------------------------
�������������������źŲɼ�������ѡ����·���ɣ�
TIM1:
PE9 PE11      ����ĸ�������1,  ������Ĭ�϶��ʹ��
TIM2
PA15 PB3      ����ĸ�������2

TIM3:
PB4 PB5       ����ĸ�������3   �Ƽ�����ʹ��
TIM4
PB6 PB7       ����ĸ�������4   �Ƽ�����ʹ��
-----------------------------------------------------------------
��е�ѹ�ɼ�ģ��
�Ƽ�ʹ��ADC4--11����8·ADC�����������ų���е�ѹ�ɼ���
��Դ�ɼ���ADC12:  
-----------------------------------------------------------------

QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include "include.h"

short Accx = 0, Accy = 0, Accz = 0; //���ٶȴ�����ԭʼ����
short Gyrox = 0, Gyroy = 0, Gyroz = 0; //������ԭʼ����
float theta_k = 0, theta_k1 = 0, omega_gyro_k = 0, omega_gyro_k1 = 0;
float P_k = 0, P_k1 = 0, Q = 0, R = 0, dt = 0.01;

int main(void)
{
  int i;
//  ϵͳ����ʱ������ʼ��
  LQ_Init3227();
//   LED��ʼ��
  GPIO_LED_Init();
//   GPIO_KEY_Init();
//   TFT��Ļ��ʼ��
//   TFTSPI_Init(0); //LCD��ʼ��  0:����  1������
//   TFTSPI_CLS(u16BLUE); //��ɫ��Ļ
//   TFTSPI_Show_Logo(0,37); //��ʾ����LOGO
//   TFTSPI_P16x16Str(0,0,(unsigned char*)"�����������ܿƼ�",u16RED,u16BLUE); //�ַ�����ʾ
//   TFTSPI_P8X16Str(0,1,"Long Qiu i.s.t.",u16WHITE,u16BLACK); //�ַ�����ʾ
//   ���ڳ�ʼ��Ϊ
  UART_InitConfig(UART1,115200,UART1_TX_A9,UART1_RX_A10); //��ʼ������1 ������ 115200 ����żУ�� 1ֹͣλ ʹ�ùܽ�
  UART_PutStr(UART1," UART1 LongQiu \r\n"); //�����ַ�������λ��
//  Test_ICM42605FIFO(); //����������
  Boot_gyro_and_acc();
  while(1){
    i = 1;
  }
}




