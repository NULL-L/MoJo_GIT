#ifndef __CAN_H
#define __CAN_H	 
#include "main.h"	    
//////////////////////////////////////////////////////////////////////////////////	 

//CAN驱动 代码	   
//STM32F4工程模板-库函数版本
//淘宝店铺：http://mcudev.taobao.com										  
////////////////////////////////////////////////////////////////////////////////// 	 

	
//CAN1接收RX0中断使能
#define CAN1_RX0_INT_ENABLE	0		//0,不使能;1,使能.								    
										 							 				    
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN初始化
 
u8 CAN1_Send_Msg(u8* msg,u8 len);						//发送数据

u8 CAN1_Receive_Msg(u8 *buf);							//接收数据


#define CAN_BAUD_NUM    18		//可用配置波特率个数

extern unsigned char CAN1_data[8];
extern unsigned char can1_rec_flag;
extern unsigned char CAN2_data[8];
extern unsigned char can2_rec_flag;
void CAN1_Configuration(void);
void CAN2_Configuration(void);
void can1_tx(unsigned int ID,unsigned char  Data);
void can2_tx(unsigned int ID,unsigned char  Data);
void CAN1_WriteData(unsigned int ID);
void CAN_Baud_Process(unsigned int Baud,CAN_InitTypeDef *CAN_InitStructure);
void position_speed_send(void);
void Data_send(void);

void hardware_id_send(void);

struct object_dictionary{
		u16 CanStdId;				//00		
		//u16 CanFilterIdHigh;		//01		过滤器标识符的高16位值
		//u16 CanFilterIdLow;		//02		过滤器标识符的低16位值
		//u16 CanFilterMaskIdHigh;	//01		过滤器屏蔽标识符的高16位值
		//u16 CanFilterMaskIdLow;	//02		过滤器屏蔽标识符的低16位值
		u8  CAN_BS1;				//01		时间段1
		u8  CAN_BS2;				//01		时间段2
		u16 SpeedKp;				//02
		u16 SpeedKi;				//03
		u16 SpeedKd;				//04
		u16 TempSpeedKp;			//05
		u16 TempSpeedKi;			//06
		u16 TempSpeedKd;			//07
		u16 PositionKp;				//08
		u16 PositionKi;				//09
		u16 PositionKd;				//10
		u16 TempPositionKp;			//11
		u16 TempPositionKi;			//12
		u16 TempPositionKd;			//13
		
	};
#endif







