#include "include.h"
//��ʱ������TIM_5��ʱ϶10ms
void Boot_gyro_and_acc(void){
  TFTSPI_Init(0);
  TFTSPI_CLS(u16BLUE);
  TFTSPI_P6X8Str(11,0,"Booting!",u16WHITE,u16BLACK);
  icm42605_init(); //8g, 1000dps
  TIMER_InitConfig(TIMER_5,10);
}