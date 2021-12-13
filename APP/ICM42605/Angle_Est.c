#include "include.h"
#define pi 3.1415926
extern short Accx, Accy, Accz, Gyrox, Gyroy, Gyroz;
extern float theta_k, theta_k1, omega_gyro_k, omega_gyro_k1, P_k, P_k1, Q, R, dt;
//�������˲��ںϣ��Ƕȹ���
void UpdateAngle(void) {
  //����Ҫ��ԭʼ���ݽ��е�λ��ͳһ
  float Accx_tmp = Accx, Accy_tmp = Accy, Accz_tmp = Accz, Gyrox_tmp = Gyrox, Gyroy_tmp = Gyroy, Gyroz_tmp = Gyroz;
  float theta_acc_k1, theta_k1_k, P_k1_k, Kg_k1;
  unsigned char txt[30];
  //�˴���tan�������������Խ���
  theta_acc_k1 = -1.0 * Accz_tmp / Accy_tmp * 180 / (2 * pi);
  omega_gyro_k1 = Gyrox_tmp;
  theta_k1_k = theta_k + omega_gyro_k * dt;
  P_k1_k = P_k + Q;
  Kg_k1 = P_k1_k / (P_k1_k + R);
  theta_k1 = theta_k1_k + Kg_k1 * (theta_acc_k1 - theta_k1_k);
  P_k1 = (1 - Kg_k1) * P_k1_k;
//  sprintf((char*)txt,"theta:%03f",theta_k1);
//  TFTSPI_P6X8Str(11,9,txt,u16WHITE,u16BLACK);
  //����ȫ�ֱ���
  omega_gyro_k = omega_gyro_k1;
  theta_k = theta_k1;
  P_k = P_k1;
}