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

extern short Accx,Accy,Accz; //���ٶȴ�����ԭʼ����_ȫ�ֱ���
extern short Gyrox,Gyroy,Gyroz; //������ԭʼ����_ȫ�ֱ���
extern float theta_k, theta_k1, omega_gyro_k, omega_gyro_k1;

const u32 TIMERx[] = {TIM1_BASE, TIM2_BASE, TIM3_BASE, TIM4_BASE, TIM5_BASE, TIM6_BASE, TIM7_BASE, TIM8_BASE};
TIM_TypeDef *TIMERxP[8] = {TIM1,TIM2,TIM3,TIM4,TIM5,TIM6,TIM7,TIM8};

int16_t ECPULSE1 = 0;          // �ٶ�ȫ�ֱ���
int16_t ECPULSE2 = 0;          // �ٶ�ȫ�ֱ���
volatile int32_t RAllPulse = 0;// �ٶ�ȫ�ֱ���
extern volatile uint8_t Game_Over;    // С�����ȫ������ͣ��
extern volatile int16_t targetSpeed;

void TIM1_UP_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ
  {
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ  
  }  
}
void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ
    //�û�����    
    LED_Ctrl(LED1,RVS);//LED��ת��˸
    
  }  
}
void TIM3_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ
  {
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ
    //�û�����    
    LED_Ctrl(LED1,RVS);//LED��ת��˸
    
  }  
}
void TIM4_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ
  {
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ
    //�û�����    
    LED_Ctrl(LED1,RVS);//LED��ת��˸
    
  }  
}


void TIM5_IRQHandler(void){ //IMU���ݲɼ�����ǽǶȽ���
  unsigned char  txt[30];
//  short aacx,aacy,aacz; //���ٶȴ�����ԭʼ����
//  short gyrox,gyroy,gyroz; //������ԭʼ����
  if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ
  {
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update  ); //���TIMx���жϴ�����λ:TIM �ж�Դ
    //��ȡ���ٶȴ�����ԭʼ����  
    ICM_Get_Raw_data(&Accx,&Accy,&Accz,&Gyrox,&Gyroy,&Gyroz);
    sprintf((char*)txt,"ax:%06d",Accx);
    TFTSPI_P6X8Str(11,1,txt,u16WHITE,u16BLACK);
    sprintf((char*)txt,"ay:%06d",Accy);
    TFTSPI_P6X8Str(11,2,txt,u16WHITE,u16BLACK);
    sprintf((char*)txt,"az:%06d",Accz);
    TFTSPI_P6X8Str(11,3,txt,u16WHITE,u16BLACK);
    sprintf((char*)txt,"gx:%06d",Gyrox);
    TFTSPI_P6X8Str(11,4,txt,u16WHITE,u16BLACK);
    sprintf((char*)txt,"gy:%06d",Gyroy);
    TFTSPI_P6X8Str(11,5,txt,u16WHITE,u16BLACK);
    sprintf((char*)txt,"gz:%06d",Gyroz);
    TFTSPI_P6X8Str(11,6,txt,u16WHITE,u16BLACK);
    //�������˲�
    UpdateAngle();
    LED_Ctrl(LED0,RVS);//LED��ת��˸ 
  }  
}

void TIM6_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ
  {
    TIM_ClearITPendingBit(TIM6, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ
    //�û�����    
    LED_Ctrl(LED1,RVS);//LED��ת��˸
    ECPULSE1 = Read_Encoder(3); //���� ĸ���ϱ�����1��С��ǰ��Ϊ��ֵ
    ECPULSE2 = Read_Encoder(4); //�ҵ�� ĸ���ϱ�����2��С��ǰ��Ϊ��ֵ 
  }  
}
void TIM7_IRQHandler(void)
{
  long sta = TIM1->SR;			// ��ȡ�ж�״̬
  TIM1->SR &= ~sta;					// ����ж�״̬
  //�û�����    
  LED_Ctrl(LED1,RVS);//LED��ת��˸
 
  
}
void TIM8_UP_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ
  {
    TIM_ClearITPendingBit(TIM8, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ
    //�û�����    
    LED_Ctrl(LED1,RVS);//LED��ת��˸
    
  }  
}

/*************************************************************************
*  �������ƣ�void TIMER_InitConfig(TIMER_Name_t timern, u16 nms)
*  ����˵�������PWM��ʼ��
*  ����˵����
//  @param      timern      ��ʱ��ͨ��
//  @param      nms          ��ʱ����pch,PWMͨ������Ӧ�ĵĶ�ʱ�����ܽ�

*  �������أ�void
*  �޸�ʱ�䣺2020��3��10��
*  ��    ע��TIMER_InitConfig(TIMER_1, 5); ʹ�ö�ʱ��1��Ϊ5msһ�ε������ж�
*************************************************************************/
void TIMER_InitConfig(TIMER_Name_t timern, u16 nms)
{  
  u32 freq_div = 0;                                          //��Ƶֵ
  u32 tmperiod;                                             //����ֵ
  
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  
  if(TIMER_1 == timern)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);                //ʱ��ʹ��
  else if(TIMER_2 == timern)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  else if(TIMER_3 == timern)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);    
  else if(TIMER_4 == timern)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);    
  else if(TIMER_5 == timern)
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM5, ENABLE);
  else if(TIMER_6 == timern)
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM6, ENABLE);    
  else if(TIMER_7 == timern)
    RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM7, ENABLE);
  else if(TIMER_8 == timern)
    RCC_APB2PeriphClockCmd(RCC_APB2ENR_TIM8, ENABLE);
  
  freq_div = SystemCoreClock/10000;                                       //���ٷ�Ƶ ,����Ϊ10K ,100us����һ��                        
  if(freq_div<1) freq_div=1;
  tmperiod = nms*10;                                                      //����ֵ nms*10*100us=nms
  if(tmperiod<1) tmperiod=1;
  
  //��ʱ����ʼ��
  TIM_TimeBaseStructure.TIM_Period = tmperiod-1;                          //�����Զ���װ�ؼĴ�������nms*10*100us=nms
  TIM_TimeBaseStructure.TIM_Prescaler = freq_div-1;                       //����Ϊ100us����һ��
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;                 //����ʱ��Tck_tim
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;             //���ϼ���ģʽ
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;                        //�ظ�����������Ϊ0
  TIM_TimeBaseInit((TIM_TypeDef *)TIMERx[timern], &TIM_TimeBaseStructure);//��ʼ��TIMxʱ��
  
  TIM_ITConfig((TIM_TypeDef *)TIMERx[timern],TIM_IT_Update,ENABLE );      //ʹ��TIM�ж�
  
  //�ж����ȼ�����
  if(TIMER_1 == timern)   nvic_init(TIM1_UP_IRQn, 0, 2, ENABLE);
  else if(TIMER_2 == timern)   nvic_init(TIM2_IRQn, 0, 2, ENABLE);
  else if(TIMER_3 == timern)   nvic_init(TIM3_IRQn, 0, 2, ENABLE);
  else if(TIMER_4 == timern)   nvic_init(TIM4_IRQn, 0, 2, ENABLE);
  else if(TIMER_5 == timern)   nvic_init(TIM5_IRQn, 0, 2, ENABLE);
  else if(TIMER_6 == timern)   nvic_init(TIM6_IRQn, 0, 2, ENABLE);
  else if(TIMER_7 == timern)   nvic_init(TIM7_IRQn, 0, 2, ENABLE);
  else if(TIMER_8 == timern)   nvic_init(TIM8_UP_IRQn, 0, 2, ENABLE);
  
  TIM_Cmd((TIM_TypeDef *)TIMERx[timern], ENABLE);  //ʹ��TIMx  
}



/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
���������� void Test_Timer56(void)
����  �ܡ� ���Զ�ʱ�жϺ�LED����˸
������ֵ�� ��
������ֵ�� ��
������ֵ�� �� 
����  �ߡ� L Q
�������¡� 2021��1��22�� 
������汾�� V1.0
������������ Test_GPIO_LED()
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Test_TimerPIT56(void)
{
  GPIO_LED_Init();
  TIMER_InitConfig(TIMER_5, 50);
  TIMER_InitConfig(TIMER_6, 25);
  while(1)
  {
    LED_Ctrl(LED2,RVS);     //LED��ת��˸
    delayms(50);
  }
}
