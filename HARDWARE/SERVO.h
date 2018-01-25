#ifndef __SERVO_H
#define	__SERVO_H

#include "main.h"

void SERVO_Configuration(void);   /* 电机输出初始化 */
void SERVO_DAC_Configuration(void);/* 电机DAC输出端口初始化 */
void SERVO_ADC_Configuration(void);
void speed_position_measure(void);
u16 SERVO_Get_Adc(u8 ch);
void Current_DAC_Out(float num);
void clear_PID_buf();
void speed_position_measure();

float PID_control_current(float current_to_follow);
float PID_control_speed(float speed_to_follow);
float PID_control_position(float position_to_follow);


#endif /* __SERVO_H */
