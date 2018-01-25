#include "servo.h"

/*
 * 
 * 描述  ：伺服控制
 * 输入  ：无
 * 输出  ：无
 */

extern float speed_actual;
extern float position_actual;
extern float current_actual;	

extern float current_control[2];
extern	float speed_control[2];//速度指令(当前和上次的)
extern	float position_control[2];

extern	float _position;
extern	float _speed;
extern	float _current;//接收到的跟随目标

extern	float current_Kp;
extern	float current_Ki;//0.0001
extern	float current_Kd;//0.0005

extern	float speed_Kp;
extern	float speed_Ki;//0.00001;
extern	float speed_Kd;//0.5;


extern	float position_Kp;//1000;
extern	float position_Ki;
extern	float position_Kd;//1000;



void SERVO_DAC_Configuration(void)
{
	
	  GPIO_InitTypeDef  GPIO_InitStructure;
	DAC_InitTypeDef DAC_InitType;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);//使能DAC时钟
	   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化

	DAC_InitType.DAC_Trigger=DAC_Trigger_None;	//不使用触发功能 TEN1=0
	DAC_InitType.DAC_WaveGeneration=DAC_WaveGeneration_None;//不使用波形发生
	DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;//屏蔽、幅值设置
	DAC_InitType.DAC_OutputBuffer=DAC_OutputBuffer_Disable ;	//DAC1输出缓存关闭 BOFF1=1
  DAC_Init(DAC_Channel_1,&DAC_InitType);	 //初始化DAC通道1

	DAC_Cmd(DAC_Channel_1, ENABLE);  //使能DAC通道1
  
  DAC_SetChannel1Data(DAC_Align_12b_R, 0);  //12位右对齐数据格式设置DAC值
	
	/*定义一个GPIO_InitTypeDef类型的结构体*/
//  GPIO_InitTypeDef  GPIO_InitStructure;
  	/*开启GPIOF的外设时钟*/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
/*选择要控制的GPIOF引脚*/		
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  /*设置引脚模式为通用推挽输出*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  /*设置引脚速率为100MHz */   
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  	/*调用库函数，初始化GPIOF*/
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOA,GPIO_Pin_6);
	GPIO_ResetBits(GPIOA,GPIO_Pin_7);
	
	
}

void SERVO_ADC_Configuration(void)
{

	GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //使能ADC1时钟

  //先初始化ADC1通道5 IO口
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;//PA5 通道5
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化  
  
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1复位
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//复位结束	 
	
	
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样阶段之间的延迟5个时钟
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMA失能
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz 
  ADC_CommonInit(&ADC_CommonInitStructure);//初始化
  
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非扫描模式	
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//关闭连续转换
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐	
  ADC_InitStructure.ADC_NbrOfConversion = 1;//1个转换在规则序列中 也就是只转换规则序列1 
  ADC_Init(ADC1, &ADC_InitStructure);//ADC初始化
  
  
	ADC_Cmd(ADC1, ENABLE);//开启AD转换器
	
}

u16 SERVO_Get_Adc(u8 ch)   
{
	  	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_28Cycles );	//ADC1,ADC通道,480个周期,提高采样时间可以提高精确度			    
  
	ADC_SoftwareStartConv(ADC1);		//使能指定的ADC1的软件转换启动功能	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}

float current_output_DAC=0;
int my_if_clockwise_test=0;
void Current_DAC_Out(float num)//电流输出
{
	if (num>=0)
	{		
		GPIO_SetBits(GPIOA,GPIO_Pin_7);
		GPIO_ResetBits(GPIOA,GPIO_Pin_6);
		GPIO_ResetBits(GPIOA,GPIO_Pin_8);
		my_if_clockwise_test=1;
		current_output_DAC=num*4096/max_current;
	}
	else
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_6);
		GPIO_ResetBits(GPIOA,GPIO_Pin_7);
		GPIO_ResetBits(GPIOA,GPIO_Pin_8);
		my_if_clockwise_test=0;
		current_output_DAC=-num*4096/max_current;
	}
	
  DAC_SetChannel1Data(DAC_Align_12b_R,current_output_DAC);//12??????????DAC?
}
void SERVO_Configuration(void)
{
	SERVO_DAC_Configuration();
	SERVO_ADC_Configuration();
}

float position_bias[12]={0,0,0,0,0,0,0,0,0,0,0,0};
 float speed_bias[12]={0,0,0,0,0,0,0,0,0,0,0,0};//PID控制用偏差，每一组间隔2ms
 float current_bias[12]={0,0,0,0,0,0,0,0,0,0,0,0};//PID控制用偏差，每一组间隔2ms
 float speed_output[2]={0,0};
 float current_output[2]={0,0};
 int clear_PID_buf_cnt=0;
 float speed_change;
 float current_change;
 
 
 float position_output[2]={0,0};
 float position_demand=0;
 
 float current_PID_x[3]={0,0,0};//PID控制状态量
 float speed_PID_x[3]={0,0,0};
 float position_PID_x[3]={0,0,0};
 
 void clear_PID_buf()
 {
 	for(clear_PID_buf_cnt=0;clear_PID_buf_cnt<12;clear_PID_buf_cnt++)
 	{
 		speed_bias[clear_PID_buf_cnt]=0;
 		current_bias[clear_PID_buf_cnt]=0;
 		position_bias[clear_PID_buf_cnt]=0;
 	}
 	speed_output[0]=0;
 	speed_output[1]=0;
 	current_output[0]=0;
 	current_output[1]=0;
 	
 	current_control[0]=0;
 	current_control[1]=0;
 	speed_change=0;
 	
 	position_demand=0;
 	position_output[0]=0;
 	position_output[1]=0;
 }

 int i_enc_cnt;
 float pos_count_last[24]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
 float pos_before[24]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
 float encoder_overflow_count=0;
 int ENC_CNT_MAX=0xFFFF;
 float delta_pos=0;
 
 float original_absolute_position=PI;
 float speed_raw=0;
 float position_raw=0;
 float delta_count_tmp=0;
 float current_measure=0;				
 float ADCConvertedValue=0;
 float current_raw=0;
	 
 void speed_position_measure()
 {
 	
 	for(i_enc_cnt=23;i_enc_cnt>0;i_enc_cnt--)
 	{
//		encoder_count_last[i_enc_cnt]=encoder_count_last[i_enc_cnt-1];
//		tim_4_last[i_enc_cnt]=tim_4_last[i_enc_cnt-1];
 		pos_count_last[i_enc_cnt]=pos_count_last[i_enc_cnt-1];
		//pos_count_smooth_last[i_enc_cnt]=pos_count_smooth_last[i_enc_cnt-1];
 		pos_before[i_enc_cnt]=pos_before[i_enc_cnt-1];
 		
		//delta_measure_time_before[i_enc_cnt]=delta_measure_time_before[i_enc_cnt-1];
 	}
 	
 	

	//delta_measure_time_before[0]=(cnt6_max*cnt6_overflow+TIM6->CNT+1.0f)*2.0f/1000000.0f;
 	pos_count_last[0]=(encoder_overflow_count*ENC_CNT_MAX+(TIM5->CNT));
 	
	//pos_count_smooth_last[0]=(pos_count_last[0]*16.0+pos_count_last[1]*8.0+pos_count_last[2]*4.0+pos_count_last[3]*2.0+pos_count_last[4]*1.0)/31.0;
 	
	//same_pos_cnt=0;
	//va[2]=(pos_count_last[0]-pos_count_last[23])/24.0f;
 	
 	
 	delta_pos=(pos_count_last[0]-pos_count_last[23])/24.0f;
 	
 	position_raw=original_absolute_position+PI*(float)(pos_count_last[0])/frequency_ratio/line_num/reduc_ratio;
 	

 	delta_count_tmp=(1*pos_before[0]-1*pos_before[1]+1*pos_before[2]-pos_before[3]);
 	
	speed_raw=(4*pos_before[0]-4*pos_before[1]+3*pos_before[2]-3*pos_before[3]+1*pos_before[4]-pos_before[5]+1*pos_before[6]-pos_before[7]/**/)/9.0/4.0/(sample_delta_ms/1000.0);
 	
	//cnt6_overflow=0;
	//TIM6->CNT = 0;

//current measure

	ADCConvertedValue = SERVO_Get_Adc(Current_ADC_Channel) ;
	current_measure=(ADCConvertedValue-2048)*2.0f*max_current/4096.0f;	
	current_raw=current_measure;

pos_spd_kalman_cycle();
 	pos_before[0]=position_actual;

	//Data_send(); 	
 }

 int i_pid_cur;
 float current_bias_max=10.0;
 float current_demand=0;
 float PID_control_current(float current_to_follow)
 {
	//current_change=current_control[1];
	//current_demand=current_change;
 	current_demand=current_to_follow;
 	if (current_demand>=max_current/current_protect_ratio)
 		{current_demand=max_current/current_protect_ratio;}
 	if (current_demand<=-max_current/current_protect_ratio)
 		{current_demand=-max_current/current_protect_ratio;}

 	current_bias[0]=current_demand-current_actual;
//	
//	if(current_bias[0]>current_bias_max)
//	{current_bias[0]=current_bias_max;}
//	if(current_bias[0]<-current_bias_max)
//	{current_bias[0]=-current_bias_max;}

	/////////////////////////
//	speed_PID_x[0]=speed_bias[2]-speed_bias[1];
//	speed_PID_x[1]=speed_bias[2];
//	speed_PID_x[2]=speed_bias[2]-2*speed_bias[1]+speed_bias[0];
	//////////////////////////

	current_PID_x[0]=current_bias[0];/////////P
	current_PID_x[1]=0.0;///////////////////I
	for(i_pid_cur=0;i_pid_cur<12;i_pid_cur++)
	{
		current_PID_x[1]=current_PID_x[1]+current_bias[i_pid_cur];
	}
	
	current_PID_x[2]=3*current_bias[0]-4*current_bias[1]+current_bias[2];///D
	
	current_output[1]=current_output[0]+current_Kp*current_PID_x[0]+current_Ki*current_PID_x[1]+current_Kd*current_PID_x[2];
	//speed_output[1]=speed_demand;
	//////////////////////////////////////////////////////////
//	if (speed_output[1]>=max_speed)
//	{speed_output[1]=max_speed;}
//	if (speed_output[1]<=-max_speed)
//	{speed_output[1]=-max_speed;}
	//////////////////////////////////////////////////////////
	//my_current_output_test=current_output[1];
	if (current_output[1]>=max_current/current_protect_ratio)//电流保护
		{current_output[1]=max_current/current_protect_ratio;}
	if (current_output[1]<=-max_current/current_protect_ratio)
		{current_output[1]=-max_current/current_protect_ratio;}
	
//	current_DAC_Out(current_output[1]);

	current_output[0]=current_output[1];
	for(i_pid_cur=11;i_pid_cur>0;i_pid_cur--)
	{
		current_bias[i_pid_cur]=current_bias[i_pid_cur-1];
	}
	
	return current_output[1];
	
}


int i_pid_spd=0;
float speed_bias_max=2.0;
float speed_demand=0;
float PID_control_speed(float speed_to_follow)
{
	speed_demand=speed_to_follow;
	if (speed_demand>=max_speed)
		{speed_demand=max_speed;}
	if (speed_demand<=-max_speed)
		{speed_demand=-max_speed;}

	speed_bias[0]=speed_demand-speed_actual;
	
	if(speed_bias[0]>speed_bias_max)
		{speed_bias[0]=speed_bias_max;}
	if(speed_bias[0]<-speed_bias_max)
		{speed_bias[0]=-speed_bias_max;}
	
	/////////////////////////
//	speed_PID_x[0]=speed_bias[2]-speed_bias[1];
//	speed_PID_x[1]=speed_bias[2];
//	speed_PID_x[2]=speed_bias[2]-2*speed_bias[1]+speed_bias[0];
	//////////////////////////
	
	speed_PID_x[0]=speed_bias[0];/////////P
	speed_PID_x[1]=0.0;///////////////////I
	for(i_pid_spd=0;i_pid_spd<12;i_pid_spd++)
	{
		speed_PID_x[1]=speed_PID_x[1]+speed_bias[i_pid_spd];
	}
	
	speed_PID_x[2]=3*speed_bias[0]-4*speed_bias[1]+speed_bias[2];///D
	

	speed_output[1]=speed_output[0]+speed_Kp*speed_PID_x[0]+speed_Ki*speed_PID_x[1]+speed_Kd*speed_PID_x[2];
	//speed_output[1]=speed_demand;
	//////////////////////////////////////////////////////////
//	if (speed_output[1]>=max_speed)
//	{speed_output[1]=max_speed;}
//	if (speed_output[1]<=-max_speed)
//	{speed_output[1]=-max_speed;}
	//////////////////////////////////////////////////////////
//	if (speed_output[1]>=max_speed/speed_protect_ratio)
//	{speed_output[1]=max_speed/speed_protect_ratio;}
//	if (speed_output[1]<=-max_speed/speed_protect_ratio)
//	{speed_output[1]=-max_speed/speed_protect_ratio;}
	//////////////////////////////////////////////////////////
	
	
	if (speed_output[1]>=max_current/current_protect_ratio)
		{speed_output[1]=max_current/current_protect_ratio;}
	if (speed_output[1]<=-max_current/current_protect_ratio)
		{speed_output[1]=-max_current/current_protect_ratio;}
	
	current_control[1]=speed_output[1];
	//PID_control_current();
	//////////////////////////////DAC_Out(speed_output[1]);

	speed_output[0]=speed_output[1];
	for(i_pid_spd=11;i_pid_spd>0;i_pid_spd--)
	{
		speed_bias[i_pid_spd]=speed_bias[i_pid_spd-1];
	}
	return speed_output[1];
}


int i_pid_pos=0;
float position_ignore_delta=0.000;
float position_bias_max=0.20;
float PID_control_position(float position_to_follow)
{
	//position_demand=t_control*t_control*t_control*pos_coeff[3]+t_control*t_control*pos_coeff[2]+t_control*pos_coeff[1]+pos_coeff[0];
	
	position_demand=position_to_follow;
//	if (position_demand>=2*PI)
//	{position_demand=2*PI;}
//	if (position_demand<=0)
//	{position_demand=0;}
	
	
	
	position_bias[0]=position_demand-position_actual;
	
//	if(abs(position_bias[0])<position_ignore_delta)
//	{position_bias[0]=0;}
	
	if(position_bias[0]>position_bias_max)
		{position_bias[0]=position_bias_max;}
	if(position_bias[0]<-position_bias_max)
		{position_bias[0]=-position_bias_max;}
	/////////////////////////
//	speed_PID_x[0]=speed_bias[2]-speed_bias[1];
//	speed_PID_x[1]=speed_bias[2];
//	speed_PID_x[2]=speed_bias[2]-2*speed_bias[1]+speed_bias[0];
	//////////////////////////
	
	position_PID_x[0]=position_bias[0];/////////P
	position_PID_x[1]=0.0;///////////////////I
	for(i_pid_pos=0;i_pid_pos<12;i_pid_pos++)
	{
		position_PID_x[1]=position_PID_x[1]+position_bias[i_pid_pos];
	}
	
	position_PID_x[2]=3*position_bias[0]-4*position_bias[1]+position_bias[2];///D
	
	
	
	
	position_output[1]=position_output[0]+position_Kp*position_PID_x[0]+position_Ki*position_PID_x[1]+position_Kd*position_PID_x[2];
	//speed_output[1]=speed_demand;
	//////////////////////////////////////////////////////////
//	if (speed_output[1]>=max_speed)
//	{speed_output[1]=max_speed;}
//	if (speed_output[1]<=-max_speed)
//	{speed_output[1]=-max_speed;}
	//////////////////////////////////////////////////////////
//	if (speed_output[1]>=max_speed/speed_protect_ratio)
//	{speed_output[1]=max_speed/speed_protect_ratio;}
//	if (speed_output[1]<=-max_speed/speed_protect_ratio)
//	{speed_output[1]=-max_speed/speed_protect_ratio;}
	//////////////////////////////////////////////////////////
	
	
	if (position_output[1]>=max_speed/speed_protect_ratio)
		{position_output[1]=max_speed/speed_protect_ratio;}
	if (position_output[1]<=-max_speed/speed_protect_ratio)
		{position_output[1]=-max_speed/speed_protect_ratio;}
	
	//speed_control[1]=position_output[1];
	//speed_change=position_output[1];
	//PID_control_speed();
	//////////////////////////////DAC_Out(speed_output[1]);

	position_output[0]=position_output[1];
	for(i_pid_pos=11;i_pid_pos>0;i_pid_pos--)
	{
		position_bias[i_pid_pos]=position_bias[i_pid_pos-1];
	}
	return position_output[1];
}




