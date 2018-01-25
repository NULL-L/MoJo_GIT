#ifndef __LED_H
#define	__LED_H

#include "main.h"

#define LED1_OFF GPIO_SetBits(GPIOF,GPIO_Pin_9);    //PF9输出高电平
#define LED1_ON GPIO_ResetBits(GPIOF,GPIO_Pin_9);   //PF9输出低电平
#define LED1_NEG GPIO_WriteBit(GPIOF, GPIO_Pin_9, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOF, GPIO_Pin_9)));

#define LED2_OFF GPIO_SetBits(GPIOF,GPIO_Pin_10);    //PF10输出高电平
#define LED2_ON GPIO_ResetBits(GPIOF,GPIO_Pin_10);   //PF10输出低电平
#define LED2_NEG GPIO_WriteBit(GPIOF, GPIO_Pin_10, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOF, GPIO_Pin_10)));

void LED_GPIO_Config(void);   /* LED 端口初始化 */

#endif /* __LED_H */
