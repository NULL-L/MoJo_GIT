#include "main.h"
/***********************************************************************
函数名称：NVIC_Configuration(void)
功    能：完成中断优先级的配置
输入参数：
输出参数：
编写时间：
编 写 人：
注    意：
***********************************************************************/
void NVIC_Configuration(void)
{
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	/* Enable CANx RX0 interrupt IRQ channel */
	
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;	
	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//编码器
		NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	
}
