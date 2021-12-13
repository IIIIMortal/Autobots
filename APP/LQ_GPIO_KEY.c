/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【平    台】北京龙邱智能科技F3277核心板和母板
【编    写】龙邱科技
【E-mail  】chiusir@163.com
【软件版本】V1.0 版权所有，单位使用请先联系授权
【最后更新】2020年12月24日，持续更新，请关注最新版！
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://longqiu.taobao.com
------------------------------------------------
【IDE】IAR7.8 KEIL5.24及以上版本
【Target 】 MM32F3277
【SYS PLL】 120MHz 频率太高可能无法启动system_mm32f327x.c
=================================================================
程序配套视频地址：https://space.bilibili.com/95313236
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include "include.h"

/*************************************************************************
*  函数名称：void GPIO_KEY_Init(void)
*  功能说明：GPIO初始化函数
*  参数说明：无
*  函数返回：无
*  修改时间：2020年3月10日
*  备    注：
*************************************************************************/
void GPIO_KEY_Init(void)
{
  PIN_InitConfig(PD0, GPI, 1,GPIO_Mode_IPU);
  PIN_InitConfig(PD1, GPI, 1,GPIO_Mode_IPU);
  PIN_InitConfig(PD2, GPI, 1,GPIO_Mode_IPU);
  PIN_InitConfig(PD3, GPI, 1,GPIO_Mode_IPU);    
  PIN_InitConfig(PD4, GPI, 1,GPIO_Mode_IPU);
  PIN_InitConfig(PA2, GPI, 1,GPIO_Mode_IPU);    
}

/*************************************************************************
*  函数名称：unsigned char KEY_Read(KEYn_e KEYno)
*  功能说明：读取按键状态
*  参数说明：KEYn_e KEYno按键编号
*  函数返回：按键状态，0/1
*  修改时间：2020年3月10日
*  备    注：
*************************************************************************/
unsigned char KEY_Read(KEYn_e KEYno)
{
  switch(KEYno)
  {
  case KEY0:
    return GPIO_ReadInputDataBit(KEY0p);//母板上按键0
    //break;
    
  case KEY1:
    return GPIO_ReadInputDataBit(KEY1p);//母板上按键1
    //break;
    
  case KEY2:
    return GPIO_ReadInputDataBit(KEY2p);//母板上按键2
    //break;
    
  case DSW0:
    return GPIO_ReadInputDataBit(DSW0p);//母板上拨码开关0
    //break;
    
  case DSW1:
    return GPIO_ReadInputDataBit(DSW1p);//母板上拨码开关1
    //break;
  default:
    return 0XFF;
  }
  //return 0;
}


/*************************************************************************
*  函数名称：unsigned char KEY_Read_All(void)
*  功能说明：读取全部按键状态
*  参数说明：无
*  函数返回：按键组合状态，0--7八种状态
*  修改时间：2020年3月10日
*  备    注：读取三个按键状态，方便组合键使用
*************************************************************************/
unsigned char KEY_Read_All(void)
{
  unsigned char tm=0;
  
  tm = (GPIO_ReadInputDataBit(KEY0p)|(GPIO_ReadInputDataBit(KEY1p)<<1)|(GPIO_ReadInputDataBit(KEY2p)<<2));//读取各个按键状态并编码
  if(tm==0x07)
  {
    return 0;
  }
  //   while(tm == (GPIO_ReadInputDataBit(KEY0p)|(GPIO_ReadInputDataBit(KEY1p)<<1)|(GPIO_ReadInputDataBit(KEY2p)<<2)));//等待按键释放
  
  return  (~tm)&0X07;
}


/*************************************************************************
*  函数名称：void Test_GPIO_KEY(void)
*  功能说明：测试程序
*  参数说明：无
*  函数返回：无
*  修改时间：2020年3月10日
*  备    注：按下KEY0/1/2任意键 LED全亮
*************************************************************************/
void Test_GPIO_KEY(void)
{
  unsigned char k=0xFF;
  
  GPIO_KEY_Init();
  while(1)
  {
    k= KEY_Read(KEY0)& KEY_Read(KEY1)& KEY_Read(KEY2);
    
    if(k==0) LED_Ctrl(LEDALL, ON); //按下KEY0/1/2任意键 LED全亮
    else     LED_Ctrl(LEDALL, OFF);//松开KEY0/1/2任意键 LED全灭
    
    delayms(100);                //延时等待
  }
}


/*************************************************************************
*  函数名称：void Test_ComKEY_Tft(void)
*  功能说明：测试组合按键并在彩屏显示
*  参数说明：无
*  函数返回：无
*  修改时间：2020年3月10日
*  备    注：按键状态彩屏显示
*************************************************************************/
void Test_ComKEY_Tft(void)
{
  unsigned char k=0xFF;
  
  TFTSPI_Init(0);        //LCD初始化  0:横屏  1：竖屏
  TFTSPI_CLS(u16BLUE);   //蓝色屏幕
  TFTSPI_Show_Logo(0,37);//显示龙邱LOGO
  TFTSPI_P16x16Str(0,0,(unsigned char*)"北京龙邱智能科技",u16RED,u16BLUE);		//字符串显示
  TFTSPI_P8X16Str(0,1,"Long Qiu i.s.t.",u16WHITE,u16BLACK);		//字符串显示
  delayms(1000);              //延时等待
  TFTSPI_Init(1);        //LCD初始化  0:横屏  1：竖屏
  TFTSPI_CLS(u16BLACK);   //黑色屏幕
  TFTSPI_P8X16Str(0,1,"KEY Pressed:  ",u16WHITE,u16BLACK);		//字符串显示
  
  GPIO_KEY_Init();
  while(1)
  {
    k= KEY_Read_All();
    switch(k)
    {
    case NOKEYDOWN:
      //TFTSPI_P8X16Str(35,3,"NO key!  ",u16WHITE,u16BLACK);		//字符串显示
      break;
    case KEY0DOWN:
      TFTSPI_P8X16Str(35,3,"KEY0!    ",u16WHITE,u16BLACK);		//字符串显示
      break;
    case KEY1DOWN:
      TFTSPI_P8X16Str(35,3,"KEY1!    ",u16WHITE,u16BLACK);		//字符串显示
      break;
    case KEY2DOWN:
      TFTSPI_P8X16Str(35,3,"KEY2!    ",u16WHITE,u16BLACK);		//字符串显示
      break;
    case KEY01DOWN:
      TFTSPI_P8X16Str(35,3,"KEY0&1!  ",u16WHITE,u16BLACK);		//字符串显示
      break;
    case KEY02DOWN:
      TFTSPI_P8X16Str(35,3,"KEY0&2!  ",u16WHITE,u16BLACK);		//字符串显示
      break;
    case KEY12DOWN:
      TFTSPI_P8X16Str(35,3,"KEY1&2!  ",u16WHITE,u16BLACK);		//字符串显示
      break;
    case KEY012DOWN:
      TFTSPI_P8X16Str(35,3,"KEY0&1&2!",u16WHITE,u16BLACK);		//字符串显示
      break;
    default:
      break;
    }
    LED_Ctrl(LED0,RVS);        //LED闪烁
    delayms(100);              //延时等待
  }
}



