/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
-----------------------------------------------------------------
默认电机接口
使用TIM8块，四个通道可产生4路PWM
双路接口1
PC6           TIM8CH1    龙邱母板MOTOR1_P
PG6           GPIO       龙邱母板MOTOR1_N
PC7           TIM8CH2    龙邱母板MOTOR2_P
PG7           GPIO       龙邱母板MOTOR2_N
双路接口2
PC8           TIM8CH3    龙邱母板MOTOR3_P
PG8           GPIO       龙邱母板MOTOR3_N
PC9           TIM8CH4    龙邱母板MOTOR4_P
PG9           GPIO       龙邱母板MOTOR4_N
-----------------------------------------------------------------
默认屏幕显示接口，用户可以使用TFT或者OLED模块
TFTSPI_CS     PE5      龙邱母板 CS管脚 默认拉低，可以不用
TFTSPI_SCK    PE2      龙邱母板 SPI SCK管脚
TFTSPI_SDI    PE6      龙邱母板 SPI MOSI管脚
TFTSPI_DC     PE4      龙邱母板 D/C管脚
TFTSPI_RST    PE3      龙邱母板 RESET管脚
----------------------------------------------------------------
默认按键接口
KEY0p         PD4       龙邱母板上按键0
KEY1p         PD2       龙邱母板上按键1
KEY2p         PD3       龙邱母板上按键2
DSW0p         PD0       龙邱母板上拨码开关0
DSW1p         PD1       龙邱母板上拨码开关1
-----------------------------------------------------------------
默认LED接口
LED0p         PE0       翠绿
LED1p         PE1       翠绿
-----------------------------------------------------------------
默认蜂鸣器接口
BEEPp         PG10      龙邱母板上蜂鸣器接口
干簧管接口
REEDp         PA2       龙邱母板上干簧管接口
-----------------------------------------------------------------
摄像头接口    龙邱神眼或者OV7725模块
● 数据端口：  PF0-PF7口，共8位，接摄像头的数据端口；
● 时钟像素：  外部中断PE7；
● 行信号：    外部中断PE8/如果用DMA+TIMER1:PE7
● 场信号：    外部中断PG1
● I2C：       PF8 I2C1-SCL
PF9 I2C1-SDA
-----------------------------------------------------------------
默认舵机接口
使用TIM5块，四个通道可产生4路PWM
PA0           龙邱核母板舵机
PA1           龙邱核母板舵机
-----------------------------------------------------------------
正交解码增量编码器信号采集，任意选择四路即可；
TIM1:
PE9 PE11      龙邱母板编码器1,  四轮组默认舵机使用
TIM2
PA15 PB3      龙邱母板编码器2

TIM3:
PB4 PB5       龙邱母板编码器3   推荐优先使用
TIM4
PB6 PB7       龙邱母板编码器4   推荐优先使用
-----------------------------------------------------------------
电感电压采集模块
推荐使用ADC4--11，共8路ADC，可以满足电磁车电感电压采集；
电源采集用ADC12:  
-----------------------------------------------------------------

QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include "include.h"

short Accx = 0, Accy = 0, Accz = 0; //加速度传感器原始数据
short Gyrox = 0, Gyroy = 0, Gyroz = 0; //陀螺仪原始数据
float theta_k = 0, theta_k1 = 0, omega_gyro_k = 0, omega_gyro_k1 = 0;
float P_k = 0, P_k1 = 0, Q = 0, R = 0, dt = 0.01;

int main(void)
{
  int i;
//  系统及延时函数初始化
  LQ_Init3227();
//   LED初始化
  GPIO_LED_Init();
//   GPIO_KEY_Init();
//   TFT屏幕初始化
//   TFTSPI_Init(0); //LCD初始化  0:横屏  1：竖屏
//   TFTSPI_CLS(u16BLUE); //蓝色屏幕
//   TFTSPI_Show_Logo(0,37); //显示龙邱LOGO
//   TFTSPI_P16x16Str(0,0,(unsigned char*)"北京龙邱智能科技",u16RED,u16BLUE); //字符串显示
//   TFTSPI_P8X16Str(0,1,"Long Qiu i.s.t.",u16WHITE,u16BLACK); //字符串显示
//   串口初始化为
  UART_InitConfig(UART1,115200,UART1_TX_A9,UART1_RX_A10); //初始化串口1 波特率 115200 无奇偶校验 1停止位 使用管脚
  UART_PutStr(UART1," UART1 LongQiu \r\n"); //发送字符串到上位机
//  Test_ICM42605FIFO(); //测试陀螺仪
  Boot_gyro_and_acc();
  while(1){
    i = 1;
  }
}




