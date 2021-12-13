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
-----------------------------------------------------------------
����ͷ�ӿ�    �������ۻ���OV7725ģ��
�� ���ݶ˿ڣ�  PF0-PF7�ڣ���8λ��������ͷ�����ݶ˿ڣ�
�� ʱ�����أ�  �ⲿ�ж�PE7��
�� ���źţ�    �ⲿ�ж�PE8/�����DMA+TIMER1:PE7
�� ���źţ�    �ⲿ�ж�PG1
�� I2C��       PF8 I2C1-SCL
�� I2C��       PF9 I2C1-SDA
-----------------------------------------------------------------
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#ifndef __LQ_CAMERA_H
#define __LQ_CAMERA_H

#include "LQ_MT9V034.h"
#include "LQ_AAConfig.h"


#define IMAGEH  MT9V034_IMAGEH   /*!< ����ͷ�ɼ��߶� */
#define IMAGEW  MT9V034_IMAGEW   /*!< ����ͷ�ɼ���� */

/* ʹ�������� �޸����Ｔ�� */
#define LCDH    120  /*!< TFT��ʾ�߶ȣ��û�ʹ�ã��߶� */
#define LCDW    160  /*!< TFT��ʾ��ȣ��û�ʹ�ã���� */


//#define SMALLIMAGE  // ʹ��С�ߴ���ʾ60*94
#define MAX_ROW   LCDH
#define MAX_COL   LCDW

extern int16_t OFFSET0;      //��Զ������������ֵ�ۺ�ƫ����
extern int16_t OFFSET1;      //�ڶ���
extern int16_t OFFSET2;      //�����������

/** ͼ��ԭʼ���ݴ�� */
extern unsigned char Image_Data[IMAGEH][IMAGEW];

/** ѹ����֮�����ڴ����Ļ��ʾ����  */
extern unsigned char Image_Use[LCDH][LCDW];

/** ��ֵ��������OLED��ʾ������ */
extern unsigned char Bin_Image[LCDH][LCDW];



void MT9V034_DMA_START(void);
void MT9V034_DMA_CLS(void);
void CAMERA_Reprot(void);
void CAMERA_Init(unsigned char fps);
void Get_Use_Image(void);
void Get_Bin_Image(unsigned char mode);

/*!
* @brief    �������ֵ��С 
*
* @param    tmImage �� ͼ������
*
* @return   ��ֵ
*
* @note     �ο���https://blog.csdn.net/zyzhangyue/article/details/45841255
* @note     https://www.cnblogs.com/moon1992/p/5092726.html
* @note     https://www.cnblogs.com/zhonghuasong/p/7250540.html     
* @note     Ostu������������������ͨ��ͳ������ͼ���ֱ��ͼ������ʵ��ȫ����ֵT���Զ�ѡȡ�����㷨����Ϊ��
* @note     1) �ȼ���ͼ���ֱ��ͼ������ͼ�����е����ص㰴��0~255��256��bin��ͳ������ÿ��bin�����ص�����
* @note     2) ��һ��ֱ��ͼ��Ҳ����ÿ��bin�����ص����������ܵ����ص�
* @note     3) i��ʾ�������ֵ��Ҳ��һ���Ҷȼ�����0��ʼ����	1
* @note     4) ͨ����һ����ֱ��ͼ��ͳ��0~i �Ҷȼ�������(��������ֵ�ڴ˷�Χ�����ؽ���ǰ������) ��ռ����ͼ��ı���w0����ͳ��ǰ�����ص�ƽ���Ҷ�u0��ͳ��i~255�Ҷȼ�������(��������ֵ�ڴ˷�Χ�����ؽ�����������) ��ռ����ͼ��ı���w1����ͳ�Ʊ������ص�ƽ���Ҷ�u1��
* @note     5) ����ǰ�����غͱ������صķ��� g = w0*w1*(u0-u1) (u0-u1)
* @note     6) i++��ת��4)��ֱ��iΪ256ʱ��������
* @note     7) �����g��Ӧ��iֵ��Ϊͼ���ȫ����ֵ
* @note     ȱ��:OSTU�㷨�ڴ�����ղ����ȵ�ͼ���ʱ��Ч�������Բ��ã���Ϊ���õ���ȫ��������Ϣ��
* @note     ������ղ�����  https://blog.csdn.net/kk55guang2/article/details/78475414
* @note     https://blog.csdn.net/kk55guang2/article/details/78490069
* @note     https://wenku.baidu.com/view/84e5eb271a37f111f0855b2d.html
*
* @see      GetOSTU(Image_Use);//�����ֵ
*
* @date     2019/6/25 ���ڶ�
*/ 
short GetOSTU(unsigned char tmImage[LCDH][LCDW]);


/*!
* @brief    ����ͷ��������
*
* @param
*
* @return
*
* @note     ����MT9V034  ע����Ҫʹ��  ������Ű棨��ɫ��ת����
*
* @example
*
* @date     2019/10/22 ���ڶ�
*/
void Test_CAMERA(void);


/*!
* @brief    ����soble���ؼ�����ӵ�һ�ֱ��ؼ��
*
* @param    imageIn    ��������
*           imageOut   �������      ����Ķ�ֵ����ı�����Ϣ
*           Threshold  ��ֵ
*
* @return
*
* @note
*
* @example
*
* @date     2020/5/15
*/
void lq_sobel(unsigned char imageIn[LCDH][LCDW], unsigned char imageOut[LCDH][LCDW], unsigned char Threshold);


/*!
* @brief    ����soble���ؼ�����ӵ�һ���Զ���ֵ���ؼ��
*
* @param    imageIn    ��������
*           imageOut   �������      ����Ķ�ֵ����ı�����Ϣ
*
* @return
*
* @note
*
* @example
*
* @date     2020/5/15
*/
void lq_sobelAutoThreshold(unsigned char imageIn[LCDH][LCDW], unsigned char imageOut[LCDH][LCDW]);
void Seek_Road(void);
void Bin_Image_Filter(void);
#endif



