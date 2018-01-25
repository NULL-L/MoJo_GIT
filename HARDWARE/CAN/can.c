#include "can.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
//////////////////////////////////////////////////////////////////////////////////	 

u16 CanStdId = 0x01;
#define CanExtId 					0x0001
u16 CanFilterIdHigh= 			0x0000;// ��������ʶ���ĸ�16λֵ
u16 CanFilterIdLow=				0x0000;//	 ��������ʶ���ĵ�16λֵ
u16 CanFilterMask=				0x007f;//	 ��������ʶ���ĵ�16λֵ
u16 CanFilterMaskIdHigh =		0x0000;//���������α�ʶ���ĸ�16λֵ
u16 CanFilterMaskIdLow =			0x0000;//	���������α�ʶ���ĵ�16λֵ

//ͨ�Ŵ����ź�����
	//����ֵ
extern	u8 speed_send[2];//�ٶ�2byte
extern	u8 speed_send_sym;
extern	u8 position_send[3];//λ��3byte

extern	u8 temperature[2];//�¶�2byte
extern	u8 current[2];//����2byte
extern	u8 current_send_sym;


extern	u8 acceleration_1[2];//���ٶ��ܹ�6byte
extern	u8 acceleration_2[2];
extern	u8 acceleration_3[2];

extern	u8 angular_speed_1[2];//���ٶ��ܹ�6byte
extern	u8 angular_speed_2[2];
extern	u8 angular_speed_3[2];

extern	u8 rpy_angle_r[2];//��̬��RPY�ܹ�6byte
extern	u8 rpy_angle_p[2];
extern	u8 rpy_angle_y[2];

extern	u8 hardware_id[8];//�ؽ�Ӳ��ID

extern float speed_actual;
extern float position_actual;
extern float current_actual;	//��������ʵֵ

extern	float _position;
extern	float _speed;
extern	float _current;//���յ��ĸ���Ŀ��

extern  float speed_control[2];//�ٶ�ָ��(��ǰ���ϴε�)
extern	float current_control[2];
extern	float position_control[2];

extern  int  num_spd_PID_count;//�ٶȿ������ڼ���
extern  int  num_cur_PID_count;
extern  int  num_pos_PID_count;

extern float pos_coeff[4];
extern float speed_coeff[3];//���߲�ֵϵ��

extern float time_PID_delta_ms;//PID����
extern float time_UpToDown_delta_ms;//������������



extern struct object_dictionary OD;
extern	u8  mode;

unsigned char CAN1_data[8];
unsigned char can1_rec_flag = 0;
unsigned char CAN2_data[8];
unsigned char can2_rec_flag = 0;

const unsigned int CAN_baud_table[CAN_BAUD_NUM][5] = 
{
//�����ʣ� CAN_SJW��   CAN_BS1��    CAN_BS2��CAN_Prescaler 
	{5,   CAN_SJW_1tq,CAN_BS1_13tq,CAN_BS2_2tq,450},		//δͨ			
	{10,  CAN_SJW_1tq,CAN_BS1_6tq,CAN_BS2_2tq, 400},		//δͨ			
	{15,  CAN_SJW_1tq,CAN_BS1_13tq,CAN_BS2_2tq,150},		//15K  δͨ
	{20,  CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_2tq,200},		//20k //δͨ
	{25,  CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_8tq,112},		//25k  δͨ
	{40,  CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_2tq,100},		//40k  δͨ
	{50,  CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_8tq,56},			//50k	ok
	{62,  CAN_SJW_1tq,CAN_BS1_13tq,CAN_BS2_2tq,36},			//62.5k
	{80,  CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_2tq,50},			//80k   δͨ
	{100, CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_8tq,28},			//100K	ok
	{125, CAN_SJW_1tq,CAN_BS1_13tq, CAN_BS2_2tq,18},		//125K δͨ
	{200, CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_8tq,14},			//200K  ok
	{250, CAN_SJW_1tq,CAN_BS1_15tq,CAN_BS2_5tq,8},		    //250k  ok
	{400, CAN_SJW_1tq,CAN_BS1_15tq, CAN_BS2_5tq,5},			//400K  ok
	//{500, CAN_SJW_1tq,CAN_BS1_15tq,CAN_BS2_5tq,4},			//500K	ok
	//{500, CAN_SJW_1tq,CAN_BS1_9tq,CAN_BS2_8tq,4},			//500K	ok
	{500, CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6},			//500K	ok
	{666, CAN_SJW_1tq,CAN_BS1_5tq, CAN_BS2_2tq,8},			//δͨ
	{800, CAN_SJW_1tq,CAN_BS1_8tq, CAN_BS2_3tq,14},			//800K δͨ
	{1000,CAN_SJW_1tq,CAN_BS1_15tq,CAN_BS2_5tq,2},			//1000K	ok
};
//CAN1����
void CAN1_Configuration(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	/* CAN GPIOs configuration **************************************************/

//	STMFLASH_Read(0X08070000,&(OD.CanStdId),17);//��ȡFlash
//	//CanStdId=OD.CanStdId;
//	//CanStdId=0x7f;//
////	CAN_BS1= OD.CAN_BS1;
////	CAN_BS2= OD.CAN_BS2;		
	
	
	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* Configure CAN RX and TX pins */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* Connect CAN pins to AF9 */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1); 

	/* CAN configuration ********************************************************/  
	/* Enable CAN clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	/* CAN register init */
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = DISABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = ENABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;



	CAN_Baud_Process(500,&CAN_InitStructure);
	
	//CAN_InitStructure.CAN_Prescaler = 4;  //ʱ�䵥λ����Ϊ
	
	CAN_Init(CAN1, &CAN_InitStructure);

	CAN_FilterInitStructure.CAN_FilterNumber = 0;	   //CAN1�˲����Ŵ�0��13

	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;	   //�˲�����ģʽ
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = CanStdId<<5;
	CAN_FilterInitStructure.CAN_FilterIdLow = CanFilterIdLow;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = CanFilterMask<<5;//���������α�ʶ���ĸ�16λֵ
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = CanFilterMaskIdLow;//	���������α�ʶ���ĵ�16λֵ
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;// �趨��ָ���������FIFOΪ0
													  //This parameter can be a value of @ref CAN_filter_FIFO */
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure); 

	/* Enable FIFO 0 message pending Interrupt */
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}
//CAN2����
void CAN2_Configuration(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	/* CAN GPIOs configuration **************************************************/

	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* Configure CAN RX and TX pins */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Connect CAN pins to AF9 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_CAN2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_CAN2); 

	/* CAN configuration ********************************************************/  
	/* Enable CAN clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);//��can2ʱ��can1ʱ��ҲҪ����

	/* CAN register init */
	CAN_DeInit(CAN2);
	CAN_StructInit(&CAN_InitStructure);

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = DISABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;


	CAN_Baud_Process(500,&CAN_InitStructure);
	CAN_Init(CAN2, &CAN_InitStructure);

	CAN_FilterInitStructure.CAN_FilterNumber = 14;	   //CAN2�˲����Ŵ�14��27

	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;	   //�˲�����ģʽ
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;	//�������κ�ID
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;		//�������κ�ID
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;	  // /*!< Specifies the FIFO (0 or 1) which will be assigned to the filter.
													  //This parameter can be a value of @ref CAN_filter_FIFO */
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure); 

	/* Enable FIFO 0 message pending Interrupt */
	CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);
}
/***********************************************************************
�������ƣ�CCAN_Baud_Process(unsigned int Baud,CAN_InitTypeDef *CAN_InitStructure)
��    �ܣ����㲨���ʣ�����
��дʱ�䣺2013.4.25
�� д �ˣ�
ע    �⣺CANʱ��Ϊ42M
CAN_SJW : CAN_SJW_1tq - CAN_SJW_4tq	  ���ܱ��κ�һ��λ����γ�
CAN_BS1 : CAN_BS1_1tq - CAN_BS1_16tq
CAN_BS2 : CAN_BS2_1tq - CAN_BS2_8tq
CAN_Prescaler : 1 - 1024
	����˵����
CAN_SJW + CAN_BS1 / (CAN_SJW + CAN_BS1 + CAN_BS2)
	0.75     baud > 800k
	0.80     baud > 500k
	0.875    baud <= 500k
	baud = 42M / (CAN_SJW + CAN_BS1 + CAN_BS2) / CAN_Prescaler
***********************************************************************/
void CAN_Baud_Process(unsigned int Baud,CAN_InitTypeDef *CAN_InitStructure)
{
	unsigned int i = 0;
	for(i = 0;i < CAN_BAUD_NUM;i ++)
	{
		if(Baud == CAN_baud_table[i][0])
		{
			CAN_InitStructure->CAN_SJW = CAN_baud_table[i][1];
			CAN_InitStructure->CAN_BS1 = CAN_baud_table[i][2];
			CAN_InitStructure->CAN_BS2 = CAN_baud_table[i][3];
			CAN_InitStructure->CAN_Prescaler = CAN_baud_table[i][4];
			return;	
		}
	}	
}
//CAN1�����жϺ���
void CAN1_RX0_IRQHandler(void)
{
	u8 i = 0;
	CanRxMsg RxMessage;

	RxMessage.StdId = 0x00;//״̬����
	RxMessage.ExtId = 0x00;
	RxMessage.IDE = 0;
	RxMessage.DLC = 0;
	RxMessage.FMI = 0;
	//  RxMessage.Data[0]=0x00;
	//  RxMessage.Data[1]=0x00;

	for (i = 0; i < 8; i++)
	{
		RxMessage.Data[i] = 0x00;//����������
	}

	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage); //����FIFO0�е�����  

	
	
if ((u8)(RxMessage.StdId >> 7) == 11)//SDO
{
	u8  ccs;

	ccs = ccs & 0x00;
	ccs = (u8)(RxMessage.Data[0] >> 5);
	if (ccs == 1) {
		
		u16 idx=((u16)RxMessage.Data[1]<<8)|((u16)RxMessage.Data[2]<<0);
		u16 data=((u16)RxMessage.Data[4]<<8)|((u16)RxMessage.Data[5]<<0);
//		STMFLASH_Read(0X08070000,&(OD.CanStdId),17);
//		*(&(OD.CanStdId)+idx)=data;
//		STMFLASH_Write(0X08070000,&(OD.CanStdId),17);
		
		if(idx<2)
		{
			//CAN_INIT();
			CAN1_Configuration();
		}
		
	}
	
	
}else if ((u8)(RxMessage.StdId >> 7) == 3)//PDO
{
	u8  _mode;

	_mode = _mode & 0x00;
	_mode = (u8)(RxMessage.Data[0] >> 5);

	if (_mode == 0) {
			mode = 0;//stop
		}
		else if (_mode == 1) {
			//λ�ÿ���


			u32 int_position;
			u16 int_speed;
			u16 int_current;
			u8  sign;

			
			
			float  flag;

			float temp_position;
			float temp_speed;
			float temp_current;


			mode = 1;

			int_position = int_position & 0x00000000;
			int_position = int_position | (0x00ff0000 & ((unsigned int)RxMessage.Data[1] << 16));
			int_position = int_position | (0x0000ff00 & ((unsigned int)RxMessage.Data[2] << 8));
			int_position = int_position | (0x000000ff & ((unsigned int)RxMessage.Data[3] << 0));

			sign = sign & 0x00;
			if (!(0x01 & ((unsigned int)RxMessage.Data[0] >> 0))) {
				flag = 1;
			}
			else {
				flag = -1;
			}


			int_speed = int_speed & 0x0000;
			int_speed = int_speed | (0xff00 & ((unsigned int)RxMessage.Data[4] << 8));
			int_speed = int_speed | (0x00ff & ((unsigned int)RxMessage.Data[5] << 0));

//			int_current = int_current & 0x0000;
//			int_current = int_current | (0xff00 & ((unsigned int)RxMessage.Data[6] << 8));
//			int_current = int_current | (0x00ff & ((unsigned int)RxMessage.Data[7] << 0));

			_position = (float)int_position *PI/180000.0;
			_speed = (float)int_speed*(float)flag *PI/36000.0;
			//_current = (float)int_current *5/65535;

//			speed_control[0]=speed_control[1];
//			speed_control[1]=_speed;//0.06f;
		//	speed_remain=0;
			
			position_control[0]=position_control[1];
			position_control[1]=_position;//0.06f;
			
			TIM3->CNT=0;
			
			pos_coeff[0]=0;
			pos_coeff[1]=0;
			pos_coeff[2]=0;
			pos_coeff[3]=0;
			
//			pos_coeff[0]=position_actual;
//			pos_coeff[1]=speed_actual;
//			pos_coeff[2]=(3*_position-3*position_actual-tu*_speed-2*tu*speed_actual)/(tu*tu);
//			pos_coeff[3]=(tu*_speed+tu*speed_actual+2*position_actual-2*_position)/(tu*tu*tu);
			
			pos_coeff[0]=position_actual;
			pos_coeff[1]=(3*_position-time_UpToDown_delta_ms*_speed-3*position_actual)/(2*time_UpToDown_delta_ms);
			pos_coeff[2]=0;
			pos_coeff[3]=(3*time_UpToDown_delta_ms*_speed-3*_position+3*position_actual)/(6*time_UpToDown_delta_ms*time_UpToDown_delta_ms*time_UpToDown_delta_ms);
			
			
		//	my_mode_1_CNT++;
//			coeff[0]=2*(speed_control[0]-speed_control[1])/time_change/time_change/time_change;
//			coeff[1]=3*(speed_control[1]-speed_control[0])/time_change/time_change;
//			coeff[2]=speed_control[0];
			num_pos_PID_count=0;

			//time_change_count=0;
			
			
			
//		position_send[0] = (u8)((0x00ff0000 & (u32)(position_receive*3.6f*10000)) >>16);
//		position_send[1] = (u8)((0x0000ff00 & (u32)(position_receive*3.6f*10000)) >>8);
//		position_send[2] = (u8)((0x000000ff & (u32)(position_receive*3.6f*10000)));
//		
//		speed_send[0] = (u8)((0xff00 & (u16)(abs((int)(speed_receive*3.6*200)))) >>8);
//		speed_send[1] = (u8)((0x00ff & (u16)(abs((int)(speed_receive*3.6*200)))));
//		
////		speed_send[0] = (u8)((0xff00 & (u16)(abs((int)(200*_speed)))) >>8);
////		speed_send[1] = (u8)((0x00ff & (u16)(abs((int)(200*_speed)))));
//		
////		position_send[0] = RxMessage.Data[1];
////		position_send[1] = RxMessage.Data[2];
////		position_send[2] = RxMessage.Data[3];
////		speed_send[0] = RxMessage.Data[4];
////		speed_send[1] = RxMessage.Data[5];
//		position_speed_send();

//		position_send[0] = 0xfa;
//		position_send[1] = 0xfa;
//		position_send[2] = 0xfa;
//		speed_send[0] = 0xfa;
//		speed_send[1] = 0xfa;
//		position_speed_send();


			//buffer_enqueue(&command_buffer_front,&command_buffer_rear, max_size, position_recv, speed_recv, time_recv, temp_position, temp_speed, temp_time);
			//can_control_send();//���Ϳ���֡	
		}
		else if (_mode == 2||_mode == 4) {
			//�ٶȿ���



			u16 int_speed;
			//u8  sign;
			float  flag;
			float temp_speed;

			mode = 2;
			

			//sign = sign & 0x00;
			if ( !(0x01 & ((unsigned int)RxMessage.Data[0] >> 0))) {
				flag = 1;
			}
			else {
				flag = -1;
			}


			int_speed = int_speed & 0x0000;
			int_speed = int_speed | (0xff00 & ((unsigned int)RxMessage.Data[4] << 8));
			int_speed = int_speed | (0x00ff & ((unsigned int)RxMessage.Data[5] << 0));

			
			
			_speed = (float)int_speed*(float)flag *PI/36000.0;
			
		//flag_speed = 1;
		//	_speed_control_speed = temp_speed/0.06f;
			speed_control[0]=speed_control[1];
			speed_control[1]=_speed;//0.06f;
		//	speed_remain=0;
			TIM3->CNT=0;
			
			speed_coeff[0]=2*(speed_control[0]-speed_control[1])/time_UpToDown_delta_ms/time_UpToDown_delta_ms/time_UpToDown_delta_ms;
			speed_coeff[1]=3*(speed_control[1]-speed_control[0])/time_UpToDown_delta_ms/time_UpToDown_delta_ms;
			speed_coeff[2]=speed_control[0];
			num_spd_PID_count=0;

		}		
		else if (_mode == 3) {
			mode = 3;//У׼��λ
		}
//		else if (_mode == 4) {
//			//������ֱ�ӷ���
//			
//			u16 int_speed;
//			//u8  sign;
//			float  flag;
//			float temp_speed;
//			
//			mode = 4;
//			
//			if ( !(0x01 & ((unsigned int)RxMessage.Data[0] >> 0))) {
//				flag = 1;
//			}
//			else {
//				flag = -1;
//			}


//			int_speed = int_speed & 0x0000;
//			int_speed = int_speed | (0xff00 & ((unsigned int)RxMessage.Data[4] << 8));
//			int_speed = int_speed | (0x00ff & ((unsigned int)RxMessage.Data[5] << 0));

//			//		command_buffer_front=0;
//			//		command_buffer_rear=1;

//			temp_speed = (float)int_speed*(float)flag *0.005f;
//			//flag_speed = 1;
//			_speed_direct = temp_speed;
//			speed_control[0]=speed_control[1];
//			speed_control[1]=temp_speed/0.06f;
//			TIM3->CNT=0;
//			//time_change_count=0;
//			
//			
//			
//		}	
//		else  if (_mode==5) {
//			
//			mode=5;
//	
//			
//		}
		else  if (_mode==6)////////��������
		{
			u16 int_current;
			//u8  sign;
			float  flag;
			float temp_current;
			
			mode=6;
			
			if ( !(0x01 & ((unsigned int)RxMessage.Data[0] >> 1))) {
				flag = 1;
			}
			else {
				flag = -1;
			}
			int_current=RxMessage.Data[6]*256+RxMessage.Data[7];
//			int_current = int_current & 0x0000;
//			int_current = int_current | (0xff00 & ((unsigned int)RxMessage.Data[6] << 8));
//			int_current = int_current | (0x00ff & ((unsigned int)RxMessage.Data[7] << 0));

			temp_current = (float)int_current*(float)flag/65535.0f*5.0f;
			
			////////////////////////////////
			//_current_debug=temp_current;
			/////////////////////////////////
			
		//flag_speed = 1;
			_current=temp_current;
		//	_speed_control_speed = temp_speed/0.06f;
			
			
			//-18.1.13--current_receive_test=flag;
			
			current_control[1]=temp_current;
			//current_control[0]=current_actual;
			current_control[0]=current_control[1];
		//	speed_remain=0;
			TIM3->CNT=0;
//			current_coeff[0]=2*(current_control[0]-current_control[1])/time_change/time_change/time_change;
//			current_coeff[1]=3*(current_control[1]-current_control[0])/time_change/time_change;
//			current_coeff[2]=current_control[0];
			
			/////////////////////////////////////
			num_cur_PID_count=0;
			/////////////////////////////////////
			
			
//		
//_current_control	

//				
//	current_measure_1=(u16)abs((int)(current_measure/5*65535));
//	
//	current[0]=(u8)((0xff00 & current_measure_1) >>8);
//	current[1]=(u8)((0xff00 & current_measure_1));
//	
//	if (current_measure>=0.000f)
//	{current_send_sym=0;}
//	else
//	{	current_send_sym=1;	}
			
		}
		
	}
}
//CAN1���ͺ���
void can1_tx(unsigned int ID,unsigned char  Data)
{
	
	unsigned char i;
	uint8_t transmit_mailbox = 0;
	CanTxMsg TxMessage;
	
	TxMessage.StdId=ID;	//��׼��ʶ��Ϊ0x00
  TxMessage.ExtId=0x0000; //��չ��ʶ��0x0000
	TxMessage.IDE = CAN_ID_STD;//ʹ�ñ�׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA; /* ����Ϊ����֡ */
	TxMessage.DLC = 8;            /* ���ݳ���, can���Ĺ涨�������ݳ���Ϊ8�ֽ� */
	CAN1_data[0]=Data;
	for(i = 0;i < 8; i ++)
	{
		TxMessage.Data[i] = CAN1_data[i];
	}
	transmit_mailbox = CAN_Transmit(CAN1,&TxMessage);  /* ���������Ϣ�����͵������0,1,2��û���������뷢��no_box */	
//	while((CAN_TransmitStatus(CAN1, transmit_mailbox)  !=  CANTXOK) && (i  !=  0xFFFF))
//	{
//		i ++;
//	}
}

void can_send(u8 type, u8* Data, u8 num)
{
	u8 i = 0;
	CanTxMsg TxMessage;
	//u8 mbox;int_speed | (0xff00 & ((unsigned int)RxMessage.Data[4] << 8));
	TxMessage.StdId = (CanStdId&0x07F) | (0x780 & (type << 7));	//��׼��ʶ��
	TxMessage.ExtId = CanExtId; //��չ��ʶ��
	TxMessage.IDE = CAN_ID_STD;//ʹ�ñ�׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA;//Ϊ����֡
	TxMessage.DLC = num;	//	��Ϣ�����ݳ���
  //  TxMessage.Data[0]=Data1; //��һ���ֽ�����
  //  TxMessage.Data[1]=Data2; //�ڶ����ֽ����� 

	for (i = 0; i < num; i++)
	{
		TxMessage.Data[i] = Data[i];//д�뻺����
	}

	CAN_Transmit(CAN1, &TxMessage); //��������

}

//CAN2�����жϺ���
void CAN2_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage; 
	RxMessage.StdId=0x00;
	RxMessage.ExtId=0x00;
	RxMessage.IDE=0;
	RxMessage.DLC=0;
	RxMessage.FMI=0;
	RxMessage.Data[0]=0x00;     
	
	CAN_Receive(CAN2,CAN_FIFO0, &RxMessage);  /* �˺��������ͷ���������˵�,�ڷǱ�Ҫʱ,����Ҫ�Լ��ͷ� */
	
	if((RxMessage.StdId==0x11)&&(RxMessage.Data[0]==0xaa)) { ;}	

	can2_rec_flag = 1;
	CAN_ClearITPendingBit(CAN2,CAN_IT_FMP0);  /* ��������ж� */
}
//CAN2���ͺ���
void can2_tx(unsigned int ID,unsigned char  Data)
{
	
	unsigned char i;
	uint8_t transmit_mailbox = 0;
	CanTxMsg TxMessage;
	
	TxMessage.StdId=ID;	//��׼��ʶ��Ϊ0x00
  TxMessage.ExtId=0x0000; //��չ��ʶ��0x0000
	TxMessage.IDE = CAN_ID_STD;//ʹ�ñ�׼��ʶ��
	TxMessage.RTR = CAN_RTR_DATA; /* ����Ϊ����֡ */
	TxMessage.DLC = 8;            /* ���ݳ���, can���Ĺ涨�������ݳ���Ϊ8�ֽ� */
	CAN2_data[0]=Data;
	for(i = 0;i < 8; i ++)
	{
		TxMessage.Data[i] = CAN2_data[i];
	}
	transmit_mailbox = CAN_Transmit(CAN2,&TxMessage);  /* ���������Ϣ�����͵������0,1,2��û���������뷢��no_box */	
//	while((CAN_TransmitStatus(CAN2, transmit_mailbox)  !=  CANTXOK) && (i  !=  0xFFFF))
//	{
//		i ++;
//	}
}


void position_speed_send(void)//�����˶�״̬֡
{
	u8 CanSendData[8] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00 };//����������

	CanSendData[1] = position_send[0];
	CanSendData[2] = position_send[1];
	CanSendData[3] = position_send[2];
	CanSendData[4] = speed_send[0];
	CanSendData[5] = speed_send[1];
	CanSendData[6] = current[0];//����
	CanSendData[7] = current[1];
	
	if(speed_send_sym>0)
	{
		CanSendData[0]=CanSendData[0]|0x01;
	}
	
	if(current_send_sym>0)
	{
		CanSendData[0]=CanSendData[0]|0x02;
	}
	
//	if (position_send[0]<7)
//{
//	position_send[0]=position_send[0];
//}

	
	
	
	can_send(0, CanSendData, 8);//�����˶�״̬֡
}

void temperature_send(void)//�����¶�֡
{
	u8 CanSendData[8] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00 };//����������

	CanSendData[0] = temperature[0];
	CanSendData[1] = temperature[1];

	can_send(1, CanSendData, 8);//�����¶�֡
}

void acceleration_send(void)//���ͼ��ٶ�֡
{
	u8 CanSendData[8] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00 };//����������

	CanSendData[0] = acceleration_1[0];
	CanSendData[1] = acceleration_1[1];
	CanSendData[2] = acceleration_2[0];
	CanSendData[3] = acceleration_2[1];
	CanSendData[4] = acceleration_3[0];
	CanSendData[5] = acceleration_3[1];

	can_send(2, CanSendData, 8);//���ͼ��ٶ�֡
}

void angular_speed_send(void)//���ͽ��ٶ�֡
{
	u8 CanSendData[8] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00 };//����������

	CanSendData[0] = angular_speed_1[0];
	CanSendData[1] = angular_speed_1[1];
	CanSendData[2] = angular_speed_2[0];
	CanSendData[3] = angular_speed_2[1];
	CanSendData[4] = angular_speed_3[0];
	CanSendData[5] = angular_speed_3[1];

	can_send(3, CanSendData, 8);//���ͽ��ٶ�֡
}

void rpy_angle_send(void)//������̬��֡
{
	u8 CanSendData[8] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00 };//����������

	CanSendData[0] = rpy_angle_r[0];
	CanSendData[1] = rpy_angle_r[1];
	CanSendData[2] = rpy_angle_p[0];
	CanSendData[3] = rpy_angle_p[1];
	CanSendData[4] = rpy_angle_y[0];
	CanSendData[5] = rpy_angle_y[1];

	can_send(4, CanSendData, 8);//������̬��֡
}

void hardware_id_send(void)//���͹ؽ�Ӳ������֡
{
	u8 CanSendData[8] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00 };//����������

	CanSendData[0] = hardware_id[0];
	CanSendData[1] = hardware_id[1];
	CanSendData[2] = hardware_id[2];
	CanSendData[3] = hardware_id[3];
	CanSendData[4] = hardware_id[4];
	CanSendData[5] = hardware_id[5];
	CanSendData[6] = hardware_id[6];
	CanSendData[7] = hardware_id[7];

	can_send(5, CanSendData, 8);//���͹ؽ�Ӳ��ID֡
}

//void speed_pid_send(void)//�����ٶ�PID֡
//{
//	u8 CanSendData[8] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00 };//����������

//	STMFLASH_Read(0X08070000,&(OD.CanStdId),17);
//	CanSendData[0] = (u8)(OD.SpeedKp>>8);
//	CanSendData[1] = (u8)(OD.SpeedKp>>0);
//	CanSendData[2] = (u8)(OD.SpeedKi>>8);
//	CanSendData[3] = (u8)(OD.SpeedKi>>0);
//	CanSendData[4] = (u8)(OD.SpeedKd>>8);
//	CanSendData[5] = (u8)(OD.SpeedKd>>0);

//	can_send(6, CanSendData, 8);//�����ٶ�PID֡
//}

//void position_pid_send(void)//����λ��PID֡
//{
//	u8 CanSendData[8] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00 };//����������

//	STMFLASH_Read(0X08070000,&(OD.CanStdId),17);
//	CanSendData[0] = (u8)(OD.PositionKp>>8);
//	CanSendData[1] = (u8)(OD.PositionKp>>0);
//	CanSendData[2] = (u8)(OD.PositionKi>>8);
//	CanSendData[3] = (u8)(OD.PositionKi>>0);
//	CanSendData[4] = (u8)(OD.PositionKd>>8);
//	CanSendData[5] = (u8)(OD.PositionKd>>0);

//	can_send(7, CanSendData, 8);//����λ��PID֡
//}

void can_monitor_send()
{
	position_speed_send();
	delay_us(200);
	temperature_send();
	delay_us(200);
	acceleration_send();
	delay_us(200);
	angular_speed_send();
	delay_us(200);
	rpy_angle_send();
	delay_us(200);
	hardware_id_send();
	delay_us(200);
//	speed_pid_send();
//	delay_us(200);
//	position_pid_send();
}

void Data_send()
{	
	u16 current_measure_1=0;
	//Get_Adc();
//	position_send[0] = (u8)((0x00ff0000 & (u32)((float)position_actual*(float)180000.0/(float)PI)) >>16);
//	position_send[1] = (u8)((0x0000ff00 & (u32)((float)position_actual*(float)180000.0/(float)PI)) >>8);
//	position_send[2] = (u8)((0x000000ff & (u32)((float)position_actual*(float)180000.0/(float)PI)));


	position_send[0] = (u8)((0x00ff0000 & (u32)((float)position_actual*(float)180000.0/(float)PI)) >>16);
	position_send[1] = (u8)((0x0000ff00 & (u32)((float)position_actual*(float)180000.0/(float)PI)) >>8);
	position_send[2] = (u8)((0x000000ff & (u32)((float)position_actual*(float)180000.0/(float)PI)));
	
	
	
	
	if (speed_actual>=0)
		{speed_send_sym=0;}
	else
		{speed_send_sym=1;}
	
	speed_send[0] = (u8)((0xff00 & (u16)(abs((int)((float)speed_actual*36000.0/PI)))) >>8);
	speed_send[1] = (u8)((0x00ff & (u16)(abs((int)((float)speed_actual*36000.0/PI)))));
	
	if (position_send[0]<7)
	{
		position_send[0]=position_send[0];
	}
	
		current_measure_1=(u16)abs((int)(current_actual/5*65535));	
	if (current_actual>=0.000f)
	{current_send_sym=0;}
	else
	{	current_send_sym=1;	}
	current[0]=(u8)((0xff00 & current_measure_1) >>8);
	current[1]=(u8)((0xff00 & current_measure_1));
	
	position_speed_send();
}










