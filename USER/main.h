/***********************************************************************
文件名称：main.h
功    能：
编写时间：
编 写 人：
注    意：
AD1 PA4
DA1 PA2
***********************************************************************/
#ifndef _MAIN_H_
#define _MAIN_H_

#include "stm32f4xx_it.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "can.h"
#include "NVIC.H"
#include "ENC.h"
#include "SERVO.h"
#include "Kalman.h"
#include "arm_math.h"
#include "stm32f4xx_adc.h"

#define Current_ADC_Channel ((uint8_t)0x05)

#define sample_delta_ms 1  //;//
//#define PI 3.14159265359
#define line_num 500.0
#define reduc_ratio 100.0
#define max_speed			31.4159265359
#define max_current		5.0
#define frequency_ratio 2.0f
#define speed_protect_ratio 5.0f
#define current_protect_ratio 2.0f


#define  time_position_PID_delta_ms	0.5
#define  time_speed_PID_delta_ms	0.5
#define  time_current_PID_delta_ms	0.5

//0.0394;


#endif

