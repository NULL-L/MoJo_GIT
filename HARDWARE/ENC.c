#include "main.h"

/*
 * 
 * ����  �����ñ�����
 * ����  ����
 * ���  ����
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
	TIM_TimeBaseStructure.TIM_Period = ENC_CNT_MAX;  //�趨��������װֵ   TIMx_ARR = 359*2
	TIM_TimeBaseStructure.TIM_Prescaler = 0; //TIM3ʱ��Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;//����ʱ�ӷָ� T_dts = T_ck_int	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ��� 
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);   
	
	TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);//ʹ�ñ�����ģʽ3�������½�������
	TIM_ICStructInit(&TIM_ICInitStructure);//���ṹ���е�����ȱʡ����	
	TIM_ICInitStructure.TIM_ICFilter = 0x0;  //ѡ������Ƚ��˲��� 	
	TIM_ICInit(TIM5, &TIM_ICInitStructure);//��TIM_ICInitStructure�е�ָ��������ʼ��TIM4
	
	
	//TIM_ARRPreloadConfig(TIM2, ENABLE);//ʹ��Ԥװ��
	TIM_ClearFlag(TIM5, TIM_FLAG_Update);//���TIM3�ĸ��±�־λ
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);//���и����ж�
	//Reset counter
	TIM5->CNT = 0;//
	TIM_Cmd(TIM5, DISABLE);
	TIM_Cmd(TIM5, ENABLE);   //����TIM4��ʱ��
	TIM_Cmd(TIM5, DISABLE);   //����TIM4��ʱ��
}