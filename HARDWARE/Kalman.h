#ifndef __KALMAN_H
#define	__KALMAN_H

#include "main.h"

void pos_spd_kalman_init(void); /* 卡尔曼滤波器初始化 */
void pos_spd_kalman_cycle(void);/* 卡尔曼滤波器循环*/
	 
#endif /* __KALMAN_H */
