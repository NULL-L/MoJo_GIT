#include "main.h"


//CAN通信实验-库函数版本
//STM32F4工程模板-库函数版本
//淘宝店铺：http://mcudev.taobao.com		

float test_encoder_overflow_count=0;

u8 hardware_id[8]={0x23,0x83,0x03,0x53,0x73,0x63,0x83,0x77};//关节类型标识
//18.0f


float current_Kp=0.05;
float current_Ki=0.001;//0.0001
float current_Kd=0.0;//.3/time_current_PID_delta_ms;//0.0005

float speed_Kp=0.006f;
float speed_Ki=0.0001f;//0.00001;
float speed_Kd=0.8f/time_speed_PID_delta_ms;//0.5;


float position_Kp=0.0005;//1000;
float position_Ki=0.00001;
float position_Kd=3.0f/time_speed_PID_delta_ms;//1000;


float time_PID_delta_ms=	0.5;//0.0394;
//const float sample_delta_ms=	0.1;//0.0394;//修改kalman文件里的值
float time_UpToDown_delta_ms=	1;



int num_pos_PID_count=0;
int num_spd_PID_count=0;//速度控制周期计数
int num_cur_PID_count=0;

//通信传输信号声明
	//发送值
u8 speed_send[2]={0x00,0x00};//速度2byte
u8 speed_send_sym=0;
u8 position_send[3]={0x00,0x00,0x00};//位置3byte
u8 position_send_sym=0;
u8 temperature[2]={0x91,0x91};//温度2byte
u8 current[2]={0x19,0x91};//电流2byte
u8 current_send_sym=0;


u8 acceleration_1[2]={0x91,0x12};//加速度总共6byte
u8 acceleration_2[2]={0x91,0x41};
u8 acceleration_3[2]={0x91,0x83};

u8 angular_speed_1[2]={0x91,0x41};//角速度总共6byte
u8 angular_speed_2[2]={0x91,0x41};
u8 angular_speed_3[2]={0x91,0x41};

u8 rpy_angle_r[2]={0x91,0x41};//姿态角RPY总共6byte
u8 rpy_angle_p[2]={0x91,0x41};
u8 rpy_angle_y[2]={0x91,0x41};


ADC_InitTypeDef ADC_InitStructure;      //ADC初始化结构体声明

TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
TIM_ICInitTypeDef  TIM_ICInitStructure;


float speed_actual=0;
float position_actual=0;
float current_actual=0;	


float _position=0;
float _speed=0;
float _current=0;

float speed_control[2]={0,0};//速度指令(当前和上次的)
float current_control[2]={0,0};
float position_control[2]={0,0};

float pos_coeff[4]={0,0,0,0};
float speed_coeff[3]={0,0,0};

double encoder_overflow_count=0;

struct object_dictionary	OD;
u8  mode;
//mode
//0	stop
//1	position control
//2	speed control
//3	null position
//6	current control


int iii_t=0;

float pos_read_before[24]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


int i=0,t=0,j=0;
int main(void)
{ 

	
//	u8 cnt=0;
//	u8 canbuf[8];
//	u8 res;	
//	u8 ctr_cnt=0;
	encoder_overflow_count=0;
//	int i_enc_cnt_1;
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	NVIC_Configuration();
	delay_init(168);    //初始化延时函数
	uart_init(115200);	//初始化串口波特率为115200
	LED_GPIO_Config();
	//CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);//CAN初始化环回模式,波特率500Kbps
	CAN1_Configuration();
	ENC_Configuration();
	pos_spd_kalman_init();
	TIM_Cmd(TIM5, ENABLE);
	hardware_id_send();
	SERVO_Configuration();
	
	mode=4;
	while(1)
	{
		hardware_id_send();
		delay_us(sample_delta_ms*1000.0f);			
//			//can1_tx(0x01,0x23);		
//		delay_us(sample_delta_ms*1000.0);
//			//LED2_NEG;
//		iii_t+=1;
//		speed_position_measure();
//		
//		if(1)//ctr_cnt>=1/time_PID_delta_ms)
//		{
//			
//			for(i_enc_cnt_1=23;i_enc_cnt_1>0;i_enc_cnt_1--)
//			{
////		encoder_count_last[i_enc_cnt]=encoder_count_last[i_enc_cnt-1];
////		tim_4_last[i_enc_cnt]=tim_4_last[i_enc_cnt-1];
//				pos_read_before[i_enc_cnt_1]=pos_read_before[i_enc_cnt_1-1];				
//			}
//			pos_read_before[0]=position_actual;
//			
//			Data_send();
//			LED1_NEG;
//			ctr_cnt=0;
//		}
//		ctr_cnt+=1;



while (mode==0)//停止状态，
{
	speed_position_measure();
	Data_send();
	
	GPIO_ResetBits(GPIOA,GPIO_Pin_6);
	GPIO_ResetBits(GPIOA,GPIO_Pin_7);
	
	delay_us(sample_delta_ms*1000.0f);			
	
	hardware_id_send();
		delay_us(sample_delta_ms*1000.0f);	
		
}

if (mode==1)//位置控制
{
	while(num_pos_PID_count<500)
	{
	//Current_DAC_Out(_current);	
	Current_DAC_Out(PID_control_current(PID_control_speed(PID_control_position(_position))));
	speed_position_measure();
	Data_send();
delay_us(1000.0f*time_position_PID_delta_ms);
		//delay_ms(100.0);
	num_pos_PID_count++;

//		Current_DAC_Out(-0.3);
//	speed_position_measure();
//	Data_send();
//	delay_ms(1000.0);		
	}
	clear_PID_buf();
	num_pos_PID_count=0;
	mode=0;
}
if (mode==2)//speed control
{
	
while(num_spd_PID_count<100)
	{
	//Current_DAC_Out(_current);	
	Current_DAC_Out(PID_control_current(PID_control_speed(_speed)));
	speed_position_measure();
		if(num_spd_PID_count%10==0)
		{
	Data_send();
		}
delay_us(1000.0f*time_speed_PID_delta_ms);
		//delay_ms(100.0);
	num_spd_PID_count++;

//		Current_DAC_Out(-0.3);
//	speed_position_measure();
//	Data_send();
//	delay_ms(1000.0);		
	}
	clear_PID_buf();
	num_spd_PID_count=0;
	mode=0;
}
	

if (mode==3)
{
	
//	for(i=0;i<200;i++)
//	{
//		hardware_id_send();
//		delay_ms(5);
//		Data_send();
//	delay_ms(5);
//	}
//	
//	for(i=0;i<(200000000.0f/time_speed_PID_delta_ms);i++)
//	{
//	Current_DAC_Out((PID_control_speed(PID_control_position(4.0*sin(2*PI*i*(1000.0f*time_speed_PID_delta_ms))))));
//	speed_position_measure();
//	Data_send();
//delay_us(1000.0f*time_speed_PID_delta_ms);
//	}
		
}
if (mode==4)
{
	for(i=0;i<200;i++)//等待2s
	{
		hardware_id_send();
		delay_ms(5);
		Data_send();
	delay_ms(5);
	}
	delay_ms(5);
		for(j=0;j<5;j++)//10s方波
	{
	for(i=0;i<(1000.0f/time_speed_PID_delta_ms);i++)
	{
	Current_DAC_Out((PID_control_speed((1.0f+0.5f*j+0*sin(2*PI*i*(1000.0f*time_speed_PID_delta_ms))))));
	speed_position_measure();
	Data_send();
delay_us(1000.0f*time_speed_PID_delta_ms);
	}
	for(i=0;i<(1000.0f/time_speed_PID_delta_ms);i++)
	{
	Current_DAC_Out((PID_control_speed((-1.0f-0.5f*j+0*sin(2*PI*i*(1000.0f*time_speed_PID_delta_ms))))));
	speed_position_measure();
	Data_send();
delay_us(1000.0f*time_speed_PID_delta_ms);
	}
}
	
	for(i=0;i<(5000.0f/time_speed_PID_delta_ms);i++)//5s正弦波
	{
	Current_DAC_Out((PID_control_speed((5.0*sin(2*PI*i*(time_speed_PID_delta_ms/1000.0f))))));
	speed_position_measure();
	Data_send();
delay_us(1000.0f*time_speed_PID_delta_ms);
	}
	
		for(i=0;i<(3000.0f/time_speed_PID_delta_ms);i++)//5s正弦波
	{
	Current_DAC_Out((PID_control_speed(fabs(5.0*sin(2*PI*i*(time_speed_PID_delta_ms/1000.0f))))));
	speed_position_measure();
	Data_send();
delay_us(1000.0f*time_speed_PID_delta_ms);
	}
		for(i=0;i<(3000.0f/time_speed_PID_delta_ms);i++)//5s正弦波
	{
	Current_DAC_Out((PID_control_speed(-fabs(5.0*sin(2*PI*i*(time_speed_PID_delta_ms/1000.0f))))));
	speed_position_measure();
	Data_send();
delay_us(1000.0f*time_speed_PID_delta_ms);
	}
	
	
		for(j=0;j<(5);j++)
	{
	for(i=0;i<(1000.0f/time_speed_PID_delta_ms);i++)//1s锯齿波
	{
	Current_DAC_Out((PID_control_speed(-5.0f+(i*(10.0f/(1000.0f/time_speed_PID_delta_ms))))));
	speed_position_measure();
	Data_send();
delay_us(1000.0f*time_speed_PID_delta_ms);
	}
	}
	
		for(j=0;j<(5);j++)
	{
	for(i=0;i<(1000.0f/time_speed_PID_delta_ms);i++)//1s锯齿波
	{
	Current_DAC_Out((PID_control_speed(5.0f-(i*(10.0f/(1000.0f/time_speed_PID_delta_ms))))));
	speed_position_measure();
	Data_send();
delay_us(1000.0f*time_speed_PID_delta_ms);
	}
	}
	



	

	mode=0;
}
if (mode==5)
{
	speed_position_measure();
	Data_send();
	Current_DAC_Out(0.1);	
	delay_us(sample_delta_ms*1000.0f);
}
if (mode==6)//current control
{
	while(num_cur_PID_count<100)
	{
	//Current_DAC_Out(_current);	
	Current_DAC_Out(PID_control_current(_current));
	speed_position_measure();
	Data_send();
delay_us(1000.0f*time_current_PID_delta_ms);
		//delay_ms(100.0);
	num_cur_PID_count++;


//		Current_DAC_Out(-0.3);
//	speed_position_measure();
//	Data_send();
//	delay_ms(1000.0);		
	}
	clear_PID_buf();
	num_cur_PID_count=0;
	mode=0;
}

} 

}

