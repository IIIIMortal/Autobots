#include "include.h"
#define pi 3.1415926
extern short Accx, Accy, Accz, Gyrox, Gyroy, Gyroz;
extern float theta_k, theta_k1, omega_gyro_k, omega_gyro_k1, P_k, P_k1, Q, R, dt;
//卡尔曼滤波融合，角度估计
void UpdateAngle(void) {
  //还需要对原始数据进行单位的统一
  float Accx_tmp = Accx, Accy_tmp = Accy, Accz_tmp = Accz, Gyrox_tmp = Gyrox, Gyroy_tmp = Gyroy, Gyroz_tmp = Gyroz;
  float theta_acc_k1, theta_k1_k, P_k1_k, Kg_k1;
  unsigned char txt[30];
  //此处对tan函数进行了线性近似
  theta_acc_k1 = -1.0 * Accz_tmp / Accy_tmp * 180 / (2 * pi);
  omega_gyro_k1 = Gyrox_tmp;
  theta_k1_k = theta_k + omega_gyro_k * dt;
  P_k1_k = P_k + Q;
  Kg_k1 = P_k1_k / (P_k1_k + R);
  theta_k1 = theta_k1_k + Kg_k1 * (theta_acc_k1 - theta_k1_k);
  P_k1 = (1 - Kg_k1) * P_k1_k;
//  sprintf((char*)txt,"theta:%03f",theta_k1);
//  TFTSPI_P6X8Str(11,9,txt,u16WHITE,u16BLACK);
  //更新全局变量
  omega_gyro_k = omega_gyro_k1;
  theta_k = theta_k1;
  P_k = P_k1;
}