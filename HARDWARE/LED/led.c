#include "main.h"

/*
 * ��������LED_GPIO_Config
 * ����  ������LED�õ���I/O��
 * ����  ����
 * ���  ����
 */
void LED_GPIO_Config(void)
{	

	/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
  GPIO_InitTypeDef  GPIO_InitStructure;
  	/*����GPIOF������ʱ��*/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
/*ѡ��Ҫ���Ƶ�GPIOF����*/		
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
  /*��������ģʽΪͨ���������*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  /*������������Ϊ100MHz */   
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	/*���ÿ⺯������ʼ��GPIOF*/
  GPIO_Init(GPIOF, &GPIO_InitStructure);
  
}