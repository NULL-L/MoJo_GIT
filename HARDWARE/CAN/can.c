#include "can.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
//////////////////////////////////////////////////////////////////////////////////	 

u16 CanStdId = 0x01;
#define CanExtId 					0x0001
u16 CanFilterIdHigh= 			0x0000;// 过滤器标识符的高16位值
u16 CanFilterIdLow=				0x0000;//	 过滤器标识符的低16位值
u16 CanFilterMask=				0x007f;//	 过滤器标识符的低16位值
u16 CanFilterMaskIdHigh =		0x0000;//过滤器屏蔽标识符的高16位值
u16 CanFilterMaskIdLow =			0x0000;//	过滤器屏蔽标识符的低16位值

//通信传输信号声明
	//发送值
extern	u8 speed_send[2];//速度2byte
extern	u8 speed_send_sym;
extern	u8 position_send[3];//位置3byte
extern	u8 position_send_sym;
extern	u8 temperature[2];//温度2byte
extern	u8 current[2];//电流2byte
extern	u8 current_send_sym;


extern	u8 acceleration_1[2];//加速度总共6byte
extern	u8 acceleration_2[2];
extern	u8 acceleration_3[2];

extern	u8 angular_speed_1[2];//角速度总共6byte
extern	u8 angular_speed_2[2];
extern	u8 angular_speed_3[2];

extern	u8 rpy_angle_r[2];//姿态角RPY总共6byte
extern	u8 rpy_angle_p[2];
extern	u8 rpy_angle_y[2];

extern	u8 hardware_id[8];//关节硬件ID

extern float speed_actual;
extern float position_actual;
extern float current_actual;	//测量的真实值

extern	float _position;
extern	float _speed;
extern	float _current;//接收到的跟随目标

extern  float speed_control[2];//速度指令(当前和上次的)
extern	float current_control[2];
extern	float position_control[2];

extern  int  num_spd_PID_count;//速度控制周期计数
extern  int  num_cur_PID_count;
extern  int  num_pos_PID_count;

extern float pos_coeff[4];
extern float speed_coeff[3];//曲线插值系数

extern float time_PID_delta_ms;//PID周期
extern float time_UpToDown_delta_ms;//控制命令周期

extern float position_raw;

extern struct object_dictionary OD;
extern	u8  mode;

unsigned char CAN1_data[8];
unsigned char can1_rec_flag = 0;
unsigned char CAN2_data[8];
unsigned char can2_rec_flag = 0;

const unsigned int CAN_baud_table[CAN_BAUD_NUM][5] = 
{
//波特率， CAN_SJW，   CAN_BS1，    CAN_BS2，CAN_Prescaler 
	{5,   CAN_SJW_1tq,CAN_BS1_13tq,CAN_BS2_2tq,450},		//未通			
	{10,  CAN_SJW_1tq,CAN_BS1_6tq,CAN_BS2_2tq, 400},		//未通			
	{15,  CAN_SJW_1tq,CAN_BS1_13tq,CAN_BS2_2tq,150},		//15K  未通
	{20,  CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_2tq,200},		//20k //未通
	{25,  CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_8tq,112},		//25k  未通
	{40,  CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_2tq,100},		//40k  未通
	{50,  CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_8tq,56},			//50k	ok
	{62,  CAN_SJW_1tq,CAN_BS1_13tq,CAN_BS2_2tq,36},			//62.5k
	{80,  CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_2tq,50},			//80k   未通
	{100, CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_8tq,28},			//100K	ok
	{125, CAN_SJW_1tq,CAN_BS1_13tq, CAN_BS2_2tq,18},		//125K 未通
	{200, CAN_SJW_1tq,CAN_BS1_6tq, CAN_BS2_8tq,14},			//200K  ok
	{250, CAN_SJW_1tq,CAN_BS1_15tq,CAN_BS2_5tq,8},		    //250k  ok
	{400, CAN_SJW_1tq,CAN_BS1_15tq, CAN_BS2_5tq,5},			//400K  ok
	//{500, CAN_SJW_1tq,CAN_BS1_15tq,CAN_BS2_5tq,4},			//500K	ok
	//{500, CAN_SJW_1tq,CAN_BS1_9tq,CAN_BS2_8tq,4},			//500K	ok
	{500, CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6},			//500K	ok
	{666, CAN_SJW_1tq,CAN_BS1_5tq, CAN_BS2_2tq,8},			//未通
	{800, CAN_SJW_1tq,CAN_BS1_8tq, CAN_BS2_3tq,14},			//800K 未通
	{1000,CAN_SJW_1tq,CAN_BS1_15tq,CAN_BS2_5tq,2},			//1000K	ok
};
//CAN1配置
void CAN1_Configuration(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	/* CAN GPIOs configuration **************************************************/

//	STMFLASH_Read(0X08070000,&(OD.CanStdId),17);//读取Flash
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
	
	//CAN_InitStructure.CAN_Prescaler = 4;  //时间单位长度为
	
	CAN_Init(CAN1, &CAN_InitStructure);

	CAN_FilterInitStructure.CAN_FilterNumber = 0;	   //CAN1滤波器号从0到13

	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;	   //滤波屏蔽模式
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = CanStdId<<5;
	CAN_FilterInitStructure.CAN_FilterIdLow = CanFilterIdLow;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = CanFilterMask<<5;//过滤器屏蔽标识符的高16位值
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = CanFilterMaskIdLow;//	过滤器屏蔽标识符的低16位值
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;// 设定了指向过滤器的FIFO为0
													  //This parameter can be a value of @ref CAN_filter_FIFO */
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure); 

	/* Enable FIFO 0 message pending Interrupt */
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}
//CAN2配置
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
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);//用can2时，can1时钟也要开启

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

	CAN_FilterInitStructure.CAN_FilterNumber = 14;	   //CAN2滤波器号从14到27

	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;	   //滤波屏蔽模式
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;	//不屏蔽任何ID
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;		//不屏蔽任何ID
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;	  // /*!< Specifies the FIFO (0 or 1) which will be assigned to the filter.
													  //This parameter can be a value of @ref CAN_filter_FIFO */
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure); 

	/* Enable FIFO 0 message pending Interrupt */
	CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);
}
/***********************************************************************
函数名称：CCAN_Baud_Process(unsigned int Baud,CAN_InitTypeDef *CAN_InitStructure)
功    能：计算波特率，返回
编写时间：2013.4.25
编 写 人：
注    意：CAN时钟为42M
CAN_SJW : CAN_SJW_1tq - CAN_SJW_4tq	  不能比任何一相位缓冲段长
CAN_BS1 : CAN_BS1_1tq - CAN_BS1_16tq
CAN_BS2 : CAN_BS2_1tq - CAN_BS2_8tq
CAN_Prescaler : 1 - 1024
	配置说明：
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
//CAN1接收中断函数
void CAN1_RX0_IRQHandler(void)
{
	u8 i = 0;
	CanRxMsg RxMessage;

	RxMessage.StdId = 0x00;//状态清零
	RxMessage.ExtId = 0x00;
	RxMessage.IDE = 0;
	RxMessage.DLC = 0;
	RxMessage.FMI = 0;
	//  RxMessage.Data[0]=0x00;
	//  RxMessage.Data[1]=0x00;

	for (i = 0; i < 8; i++)
	{
		RxMessage.Data[i] = 0x00;//缓冲区清零
	}

	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage); //接收FIFO0中的数据  

	
	
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
			//位置控制


			u32 int_position;
			u16 int_speed;
			u16 int_current;
			u8  sign;

			
			
			float  flag;

			float temp_position;
			float temp_speed;
			float temp_current;


			

				sign = sign & 0x00;
			if (!(0x01 & ((unsigned int)RxMessage.Data[0] >> 2))) {
				flag = 1;
			}
			else {
				flag = -1;
			}
			
			
			int_position = int_position & 0x00000000;
			int_position = int_position | (0x00ff0000 & ((unsigned int)RxMessage.Data[1] << 16));
			int_position = int_position | (0x0000ff00 & ((unsigned int)RxMessage.Data[2] << 8));
			int_position = int_position | (0x000000ff & ((unsigned int)RxMessage.Data[3] << 0));

_position = (float)int_position*(float)flag*PI/180000.0;


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
mode = 1;
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
			//can_control_send();//发送控制帧	
		}
		else if (_mode == 2||_mode == 4) {
			//速度控制



			u16 int_speed;
			//u8  sign;
			float  flag;
			float temp_speed;

			
			

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
mode = 2;

		}		
		else if (_mode == 3) {
			mode = 3;//校准零位
		}
//		else if (_mode == 4) {
//			//驱动器直接发送
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
		else  if (_mode==6)////////电流控制
		{
			u16 int_current;
			//u8  sign;
			float  flag;
			float temp_current;
			
			
			
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
			mode=6;
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
//CAN1发送函数
void can1_tx(unsigned int ID,unsigned char  Data)
{
	
	unsigned char i;
	uint8_t transmit_mailbox = 0;
	CanTxMsg TxMessage;
	
	TxMessage.StdId=ID;	//标准标识符为0x00
  TxMessage.ExtId=0x0000; //扩展标识符0x0000
	TxMessage.IDE = CAN_ID_STD;//使用标准标识符
	TxMessage.RTR = CAN_RTR_DATA; /* 设置为数据帧 */
	TxMessage.DLC = 8;            /* 数据长度, can报文规定最大的数据长度为8字节 */
	CAN1_data[0]=Data;
	for(i = 0;i < 8; i ++)
	{
		TxMessage.Data[i] = CAN1_data[i];
	}
	transmit_mailbox = CAN_Transmit(CAN1,&TxMessage);  /* 返回这个信息请求发送的邮箱号0,1,2或没有邮箱申请发送no_box */	
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
	TxMessage.StdId = (CanStdId&0x07F) | (0x780 & (type << 7));	//标准标识符
	TxMessage.ExtId = CanExtId; //扩展标识符
	TxMessage.IDE = CAN_ID_STD;//使用标准标识符
	TxMessage.RTR = CAN_RTR_DATA;//为数据帧
	TxMessage.DLC = num;	//	消息的数据长度
  //  TxMessage.Data[0]=Data1; //第一个字节数据
  //  TxMessage.Data[1]=Data2; //第二个字节数据 

	for (i = 0; i < num; i++)
	{
		TxMessage.Data[i] = Data[i];//写入缓冲区
	}

	CAN_Transmit(CAN1, &TxMessage); //发送数据

}

//CAN2接收中断函数
void CAN2_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage; 
	RxMessage.StdId=0x00;
	RxMessage.ExtId=0x00;
	RxMessage.IDE=0;
	RxMessage.DLC=0;
	RxMessage.FMI=0;
	RxMessage.Data[0]=0x00;     
	
	CAN_Receive(CAN2,CAN_FIFO0, &RxMessage);  /* 此函数包含释放提出报文了的,在非必要时,不需要自己释放 */
	
	if((RxMessage.StdId==0x11)&&(RxMessage.Data[0]==0xaa)) { ;}	

	can2_rec_flag = 1;
	CAN_ClearITPendingBit(CAN2,CAN_IT_FMP0);  /* 清除挂起中断 */
}
//CAN2发送函数
void can2_tx(unsigned int ID,unsigned char  Data)
{
	
	unsigned char i;
	uint8_t transmit_mailbox = 0;
	CanTxMsg TxMessage;
	
	TxMessage.StdId=ID;	//标准标识符为0x00
  TxMessage.ExtId=0x0000; //扩展标识符0x0000
	TxMessage.IDE = CAN_ID_STD;//使用标准标识符
	TxMessage.RTR = CAN_RTR_DATA; /* 设置为数据帧 */
	TxMessage.DLC = 8;            /* 数据长度, can报文规定最大的数据长度为8字节 */
	CAN2_data[0]=Data;
	for(i = 0;i < 8; i ++)
	{
		TxMessage.Data[i] = CAN2_data[i];
	}
	transmit_mailbox = CAN_Transmit(CAN2,&TxMessage);  /* 返回这个信息请求发送的邮箱号0,1,2或没有邮箱申请发送no_box */	
//	while((CAN_TransmitStatus(CAN2, transmit_mailbox)  !=  CANTXOK) && (i  !=  0xFFFF))
//	{
//		i ++;
//	}
}


void position_speed_send(void)//发送运动状态帧
{
	u8 CanSendData[8] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00 };//缓冲区声明

	CanSendData[1] = position_send[0];
	CanSendData[2] = position_send[1];
	CanSendData[3] = position_send[2];
	CanSendData[4] = speed_send[0];
	CanSendData[5] = speed_send[1];
	CanSendData[6] = current[0];//电流
	CanSendData[7] = current[1];
		if(position_send_sym>0)
	{
		CanSendData[0]=CanSendData[0]|0x04;
	}
	
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

	
	
	
	can_send(0, CanSendData, 8);//发送运动状态帧
}

void temperature_send(void)//发送温度帧
{
	u8 CanSendData[8] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00 };//缓冲区声明

	CanSendData[0] = temperature[0];
	CanSendData[1] = temperature[1];

	can_send(1, CanSendData, 8);//发送温度帧
}

void acceleration_send(void)//发送加速度帧
{
	u8 CanSendData[8] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00 };//缓冲区声明

	CanSendData[0] = acceleration_1[0];
	CanSendData[1] = acceleration_1[1];
	CanSendData[2] = acceleration_2[0];
	CanSendData[3] = acceleration_2[1];
	CanSendData[4] = acceleration_3[0];
	CanSendData[5] = acceleration_3[1];

	can_send(2, CanSendData, 8);//发送加速度帧
}

void angular_speed_send(void)//发送角速度帧
{
	u8 CanSendData[8] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00 };//缓冲区声明

	CanSendData[0] = angular_speed_1[0];
	CanSendData[1] = angular_speed_1[1];
	CanSendData[2] = angular_speed_2[0];
	CanSendData[3] = angular_speed_2[1];
	CanSendData[4] = angular_speed_3[0];
	CanSendData[5] = angular_speed_3[1];

	can_send(3, CanSendData, 8);//发送角速度帧
}

void rpy_angle_send(void)//发送姿态角帧
{
	u8 CanSendData[8] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00 };//缓冲区声明

	CanSendData[0] = rpy_angle_r[0];
	CanSendData[1] = rpy_angle_r[1];
	CanSendData[2] = rpy_angle_p[0];
	CanSendData[3] = rpy_angle_p[1];
	CanSendData[4] = rpy_angle_y[0];
	CanSendData[5] = rpy_angle_y[1];

	can_send(4, CanSendData, 8);//发送姿态角帧
}

void hardware_id_send(void)//发送关节硬件类型帧
{
	u8 CanSendData[8] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00 };//缓冲区声明

	CanSendData[0] = hardware_id[0];
	CanSendData[1] = hardware_id[1];
	CanSendData[2] = hardware_id[2];
	CanSendData[3] = hardware_id[3];
	CanSendData[4] = hardware_id[4];
	CanSendData[5] = hardware_id[5];
	CanSendData[6] = hardware_id[6];
	CanSendData[7] = hardware_id[7];

	can_send(5, CanSendData, 8);//发送关节硬件ID帧
}

//void speed_pid_send(void)//发送速度PID帧
//{
//	u8 CanSendData[8] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00 };//缓冲区声明

//	STMFLASH_Read(0X08070000,&(OD.CanStdId),17);
//	CanSendData[0] = (u8)(OD.SpeedKp>>8);
//	CanSendData[1] = (u8)(OD.SpeedKp>>0);
//	CanSendData[2] = (u8)(OD.SpeedKi>>8);
//	CanSendData[3] = (u8)(OD.SpeedKi>>0);
//	CanSendData[4] = (u8)(OD.SpeedKd>>8);
//	CanSendData[5] = (u8)(OD.SpeedKd>>0);

//	can_send(6, CanSendData, 8);//发送速度PID帧
//}

//void position_pid_send(void)//发送位置PID帧
//{
//	u8 CanSendData[8] = { 0x00,0x00,0x00,0x00,0x00,0x00,0x00 };//缓冲区声明

//	STMFLASH_Read(0X08070000,&(OD.CanStdId),17);
//	CanSendData[0] = (u8)(OD.PositionKp>>8);
//	CanSendData[1] = (u8)(OD.PositionKp>>0);
//	CanSendData[2] = (u8)(OD.PositionKi>>8);
//	CanSendData[3] = (u8)(OD.PositionKi>>0);
//	CanSendData[4] = (u8)(OD.PositionKd>>8);
//	CanSendData[5] = (u8)(OD.PositionKd>>0);

//	can_send(7, CanSendData, 8);//发送位置PID帧
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



//		position_send[0] = (u8)((0x00ff0000 & (u32)(fabs((float)position_raw)*(float)180000.0/(float)PI)) >>16);
//	position_send[1] = (u8)((0x0000ff00 & (u32)(fabs((float)position_raw)*(float)180000.0/(float)PI)) >>8);
//	position_send[2] = (u8)((0x000000ff & (u32)(fabs((float)position_raw)*(float)180000.0/(float)PI)));
	
	position_send[0] = (u8)((0x00ff0000 & (u32)(fabs((float)position_actual)*(float)180000.0/(float)PI)) >>16);
	position_send[1] = (u8)((0x0000ff00 & (u32)(fabs((float)position_actual)*(float)180000.0/(float)PI)) >>8);
	position_send[2] = (u8)((0x000000ff & (u32)(fabs((float)position_actual)*(float)180000.0/(float)PI)));
	
	
	
	
		if (position_actual>=0.000f)
	{position_send_sym=0;}
	else
	{	position_send_sym=1;	}
	
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










