#include "main.h"

/*
 * 
 * 描述  ：配置编码器
 * 输入  ：无
 * 输出  ：无
 */
 
extern int ENC_CNT_MAX;
void ENC_Configuration(void)
{	

	GPIO_InitTypeDef GPIO_InitStructure;
 TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
 TIM_ICInitTypeDef  TIM_ICInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	
	//PB6,7--Encoder
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  
	GPIO_Init(GPIOA, &GPIO_InitStructure);   
	
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1, GPIO_AF_TIM5);

	
	TIM_DeInit(TIM5);
	TIM_TimeBaseStructure.TIM_Period = ENC_CNT_MAX;  //设定计数器重装值   TIMx_ARR = 359*2
	TIM_TimeBaseStructure.TIM_Prescaler = 0; //TIM3时钟预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;//设置时钟分割 T_dts = T_ck_int	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数 
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);   
	
	TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);//使用编码器模式3，上升下降都计数
	TIM_ICStructInit(&TIM_ICInitStructure);//将结构体中的内容缺省输入	
	TIM_ICInitStructure.TIM_ICFilter = 0x0;  //选择输入比较滤波器 	
	TIM_ICInit(TIM5, &TIM_ICInitStructure);//将TIM_ICInitStructure中的指定参数初始化TIM4
	
	
	//TIM_ARRPreloadConfig(TIM2, ENABLE);//使能预装载
	TIM_ClearFlag(TIM5, TIM_FLAG_Update);//清除TIM3的更新标志位
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);//运行更新中断
	//Reset counter
	TIM5->CNT = 0;//
	TIM_Cmd(TIM5, DISABLE);
	TIM_Cmd(TIM5, ENABLE);   //启动TIM4定时器
	TIM_Cmd(TIM5, DISABLE);   //启动TIM4定时器
}