#include "main.h"

/*
 * 
 * 描述  ：卡尔曼滤波测速
 * 输入  ：无
 * 输出  ：无
 */

extern float position_raw;
extern float position_actual;
extern float speed_actual;

//extern float sample_delta_ms;//=0.01;//0.0394;修改main文件里的值

//const float Q=3.229820487531232e-17;
//const float R=3.229820487531232e-16;

//const float32_t Q_init[1] ={  Q};
//const float32_t R_init[1] ={  R};				
//const float32_t zk_init[1] ={ 0.0};
//const float32_t H_init[2] ={  1.0, 0.0};
//const float32_t HT_init[2] ={  1.0, 0.0};
//float32_t A_init[4] ={  1.0, sample_delta_ms ,0.0, 1.0 };
//const float32_t AT_init[4] ={  1.0, sample_delta_ms ,0.0, 1.0 };
//const float32_t eye_2_init[4] ={  1.0, 0.0 ,0.0, 1.0 };
//const float32_t Pk__init[4] ={  1.0, 0.0 ,0.0, 1.0 };
//const float32_t Pk_init[4] ={  1.0, 0.0 ,0.0, 1.0 };
//const float32_t Pkk_init[4] ={  1.0, 0.0 ,0.0, 1.0 };
//const float32_t zeros_2_init[2] ={  0.0, 0.0};
//const float32_t xk__init[2] ={  0.0, 0.0};
//const float32_t xk_init[2] ={  0.0, 0.0};
//const float32_t K_init[2] ={  0.0, 0.0};
//const float32_t xkk_init[2] ={  0.0, 0.0};

//const float32_t temp_1x1_0_init[1] ={0.0};
//const float32_t temp_1x1_1_init[1] ={0.0};
//const float32_t temp_1x1_2_init[1] ={0.0};
//const float32_t temp_1x1_3_init[1] ={0.0};
//const float32_t temp_1x1_4_init[1] ={0.0};
//const float32_t temp_1x1_5_init[1] ={0.0};

//const float32_t temp_2x1_0_init[2] ={0.0,0.0};
//const float32_t temp_2x1_1_init[2] ={0.0,0.0};
//const float32_t temp_2x1_2_init[2] ={0.0,0.0};
//const float32_t temp_2x1_3_init[2] ={0.0,0.0};
//const float32_t temp_2x1_4_init[2] ={0.0,0.0};
//const float32_t temp_2x1_5_init[2] ={0.0,0.0};

//const float32_t temp_1x2_0_init[2] ={0.0,0.0};
//const float32_t temp_1x2_1_init[2] ={0.0,0.0};
//const float32_t temp_1x2_2_init[2] ={0.0,0.0};
//const float32_t temp_1x2_3_init[2] ={0.0,0.0};
//const float32_t temp_1x2_4_init[2] ={0.0,0.0};
//const float32_t temp_1x2_5_init[2] ={0.0,0.0};

//const float32_t temp_2x2_0_init[4] ={0.0,0.0,0.0,0.0};
//const float32_t temp_2x2_1_init[4] ={0.0,0.0,0.0,0.0};
//const float32_t temp_2x2_2_init[4] ={0.0,0.0,0.0,0.0};
//const float32_t temp_2x2_3_init[4] ={0.0,0.0,0.0,0.0};
//const float32_t temp_2x2_4_init[4] ={0.0,0.0,0.0,0.0};
//const float32_t temp_2x2_5_init[4] ={0.0,0.0,0.0,0.0}; 

// const float Q=3.229820487531232e-17;
//const float R=3.229820487531232e-16;

const float Q_value=3.229820487531232e-17;
const float R_value=3.229820487531232e-16;

float32_t Q_init[1] ={  Q_value};
float32_t R_init[1] ={  R_value};				
float32_t zk_init[1] ={ 0.0};
float32_t H_init[2] ={  1.0, 0.0};
float32_t HT_init[2] ={  1.0, 0.0};
float32_t A_init[4] ={  1.0, sample_delta_ms ,0.0, 1.0 };
float32_t AT_init[4] ={  1.0, sample_delta_ms ,0.0, 1.0 };
float32_t eye_2_init[4] ={  1.0, 0.0 ,0.0, 1.0 };
float32_t Pk__init[4] ={  1.0, 0.0 ,0.0, 1.0 };
float32_t Pk_init[4] ={  1.0, 0.0 ,0.0, 1.0 };
float32_t Pkk_init[4] ={  1.0, 0.0 ,0.0, 1.0 };
float32_t zeros_2_init[2] ={  0.0, 0.0};
float32_t xk__init[2] ={  0.0, 0.0};
float32_t xk_init[2] ={  0.0, 0.0};
float32_t K_init[2] ={0.156588980622307,0.091837411732784};
float32_t xkk_init[2] ={  0.0, 0.0};

float32_t temp_1x1_0_init[1] ={0.0};
float32_t temp_1x1_1_init[1] ={0.0};
float32_t temp_1x1_2_init[1] ={0.0};
float32_t temp_1x1_3_init[1] ={0.0};
float32_t temp_1x1_4_init[1] ={0.0};
float32_t temp_1x1_5_init[1] ={0.0};

float32_t temp_2x1_0_init[2] ={0.0,0.0};
float32_t temp_2x1_1_init[2] ={0.0,0.0};
float32_t temp_2x1_2_init[2] ={0.0,0.0};
float32_t temp_2x1_3_init[2] ={0.0,0.0};
float32_t temp_2x1_4_init[2] ={0.0,0.0};
float32_t temp_2x1_5_init[2] ={0.0,0.0};

float32_t temp_1x2_0_init[2] ={0.0,0.0};
float32_t temp_1x2_1_init[2] ={0.0,0.0};
float32_t temp_1x2_2_init[2] ={0.0,0.0};
float32_t temp_1x2_3_init[2] ={0.0,0.0};
float32_t temp_1x2_4_init[2] ={0.0,0.0};
float32_t temp_1x2_5_init[2] ={0.0,0.0};

float32_t temp_2x2_0_init[4] ={0.0,0.0,0.0,0.0};
float32_t temp_2x2_1_init[4] ={0.0,0.0,0.0,0.0};
float32_t temp_2x2_2_init[4] ={0.0,0.0,0.0,0.0};
float32_t temp_2x2_3_init[4] ={0.0,0.0,0.0,0.0};
float32_t temp_2x2_4_init[4] ={0.0,0.0,0.0,0.0};
float32_t temp_2x2_5_init[4] ={0.0,0.0,0.0,0.0};

arm_matrix_instance_f32 Pk_;
arm_matrix_instance_f32 xk_;
arm_matrix_instance_f32 xk;
arm_matrix_instance_f32 xkk;
arm_matrix_instance_f32 Pkk;
arm_matrix_instance_f32 A;
arm_matrix_instance_f32 AT;
arm_matrix_instance_f32 H;
arm_matrix_instance_f32 HT;
arm_matrix_instance_f32 K;
arm_matrix_instance_f32 zk;
arm_matrix_instance_f32 Pk;
arm_matrix_instance_f32 Q;
arm_matrix_instance_f32 R;

arm_matrix_instance_f32 eye_2;

arm_matrix_instance_f32 temp_1x1_0;
arm_matrix_instance_f32 temp_1x1_1;
arm_matrix_instance_f32 temp_1x1_2;
arm_matrix_instance_f32 temp_1x1_3;
arm_matrix_instance_f32 temp_1x1_4;
arm_matrix_instance_f32 temp_1x1_5;

arm_matrix_instance_f32 temp_2x1_0;
arm_matrix_instance_f32 temp_2x1_1;
arm_matrix_instance_f32 temp_2x1_2;
arm_matrix_instance_f32 temp_2x1_3;
arm_matrix_instance_f32 temp_2x1_4;
arm_matrix_instance_f32 temp_2x1_5;

arm_matrix_instance_f32 temp_1x2_0;
arm_matrix_instance_f32 temp_1x2_1;
arm_matrix_instance_f32 temp_1x2_2;
arm_matrix_instance_f32 temp_1x2_3;
arm_matrix_instance_f32 temp_1x2_4;
arm_matrix_instance_f32 temp_1x2_5;

arm_matrix_instance_f32 temp_2x2_0;
arm_matrix_instance_f32 temp_2x2_1;
arm_matrix_instance_f32 temp_2x2_2;
arm_matrix_instance_f32 temp_2x2_3;
arm_matrix_instance_f32 temp_2x2_4;
arm_matrix_instance_f32 temp_2x2_5;

void pos_spd_kalman_init()
{
	arm_mat_init_f32(&H  , 1, 2, (float32_t *)H_init);
	arm_mat_init_f32(&HT , 2, 1, (float32_t *)HT_init);
	arm_mat_init_f32(&Pk_, 2, 2, (float32_t *)Pk__init);
	arm_mat_init_f32(&Pk, 2, 2, (float32_t *)Pk_init);
	arm_mat_init_f32(&xk_, 2, 1, (float32_t *)xk__init);
	arm_mat_init_f32(&xk, 2, 1, (float32_t *)xk_init);
	arm_mat_init_f32(&xkk, 2, 1, (float32_t *)xkk_init);
	arm_mat_init_f32(&Pkk, 2, 2, (float32_t *)Pkk_init);
	arm_mat_init_f32(&A  , 2, 2, (float32_t *)A_init);
	arm_mat_init_f32(&AT , 2, 2, (float32_t *)AT_init);
	arm_mat_init_f32(&K  , 2, 1, (float32_t *)K_init);
	arm_mat_init_f32(&zk , 1, 1, (float32_t *)zk_init);
	arm_mat_init_f32(&Q, 2, 2, (float32_t *)Q_init);
	arm_mat_init_f32(&R, 2, 2, (float32_t *)R_init);
	arm_mat_init_f32(&eye_2, 2, 2, (float32_t *)eye_2_init);
	
	arm_mat_init_f32(&temp_1x1_0 , 1, 1, (float32_t *)temp_1x1_0_init);
	arm_mat_init_f32(&temp_1x1_1 , 1, 1, (float32_t *)temp_1x1_1_init);
	arm_mat_init_f32(&temp_1x1_2 , 1, 1, (float32_t *)temp_1x1_2_init);
	arm_mat_init_f32(&temp_1x1_3 , 1, 1, (float32_t *)temp_1x1_3_init);
	arm_mat_init_f32(&temp_1x1_4 , 1, 1, (float32_t *)temp_1x1_4_init);
	arm_mat_init_f32(&temp_1x1_5 , 1, 1, (float32_t *)temp_1x1_5_init);
	
	arm_mat_init_f32(&temp_2x1_0 , 2, 1, (float32_t *)temp_2x1_0_init);
	arm_mat_init_f32(&temp_2x1_1 , 2, 1, (float32_t *)temp_2x1_1_init);
	arm_mat_init_f32(&temp_2x1_2 , 2, 1, (float32_t *)temp_2x1_2_init);
	arm_mat_init_f32(&temp_2x1_3 , 2, 1, (float32_t *)temp_2x1_3_init);
	arm_mat_init_f32(&temp_2x1_4 , 2, 1, (float32_t *)temp_2x1_4_init);
	arm_mat_init_f32(&temp_2x1_5 , 2, 1, (float32_t *)temp_2x1_5_init);
	
	
	arm_mat_init_f32(&temp_1x2_0 , 2, 1, (float32_t *)temp_1x2_0_init);
	arm_mat_init_f32(&temp_1x2_1 , 2, 1, (float32_t *)temp_1x2_1_init);
	arm_mat_init_f32(&temp_1x2_2 , 2, 1, (float32_t *)temp_1x2_2_init);
	arm_mat_init_f32(&temp_1x2_3 , 2, 1, (float32_t *)temp_1x2_3_init);
	arm_mat_init_f32(&temp_1x2_4 , 2, 1, (float32_t *)temp_1x2_4_init);
	arm_mat_init_f32(&temp_1x2_5 , 2, 1, (float32_t *)temp_1x2_5_init);
	
	arm_mat_init_f32(&temp_2x2_0 , 2, 2, (float32_t *)temp_2x2_0_init);
	arm_mat_init_f32(&temp_2x2_1 , 2, 2, (float32_t *)temp_2x2_1_init);
	arm_mat_init_f32(&temp_2x2_2 , 2, 2, (float32_t *)temp_2x2_2_init);
	arm_mat_init_f32(&temp_2x2_3 , 2, 2, (float32_t *)temp_2x2_3_init);
	arm_mat_init_f32(&temp_2x2_4 , 2, 2, (float32_t *)temp_2x2_4_init);
	arm_mat_init_f32(&temp_2x2_5 , 2, 2, (float32_t *)temp_2x2_5_init);
	
	
}


void pos_spd_kalman_cycle()
{
	arm_status status;//arm_mat_add_f32
	status = arm_mat_mult_f32(&A, &xkk, &xk_);
//		xk_=A*xkk;
	
//status = arm_mat_trans_f32(&A, &AT);
//status = arm_mat_mult_f32(&A, &Pkk, &temp_2x2_0);
//		if(status==ARM_MATH_SIZE_MISMATCH){	while(1);}
//status = arm_mat_mult_f32(&temp_2x2_0, &AT, &temp_2x2_1);
//status = arm_mat_add_f32(&temp_2x2_1, &Q, &Pk_);	
////    Pk_=A*Pkk*A'+Q;
//		
//	if(status==ARM_MATH_SIZE_MISMATCH){	while(1);}
//status = arm_mat_trans_f32(&H, &HT);
//status = arm_mat_mult_f32(&Pk_, &HT, &temp_2x1_0);
//status = arm_mat_mult_f32(&H, &temp_2x1_0, &temp_1x1_1);
//status = arm_mat_add_f32(&temp_1x1_1, &R, &temp_1x1_2);	
//status = arm_mat_inverse_f32(&temp_1x1_2, &temp_1x1_3);	
//status = arm_mat_mult_f32(&temp_2x1_0, &temp_1x1_3, &K);
////    K=(Pk_*H')/(H*Pk_*H'+R);
	
	zk.pData[0]=position_raw;
//    zk=[pos(j)];
	
	if(status==ARM_MATH_SIZE_MISMATCH){	while(1);}
	status = arm_mat_mult_f32(&H, &xk_, &temp_1x1_0);
	status = arm_mat_sub_f32(&zk, &temp_1x1_0, &temp_1x1_1);
	status = arm_mat_mult_f32(&K, &temp_1x1_1, &temp_2x1_2);
	status = arm_mat_add_f32(&xk_, &temp_2x1_2, &xk);
//    xk=xk_+K*(zk-H*xk_);
	
//if(status==ARM_MATH_SIZE_MISMATCH){	while(1);}
//status = arm_mat_mult_f32(&K, &H, &temp_2x2_0);
//status = arm_mat_sub_f32(&eye_2, &temp_2x2_0, &temp_2x2_1);
//status = arm_mat_mult_f32(&temp_2x2_1, &Pk_, &Pk);
////    Pk=(eye(2)-K*H)*Pk_;
	
	status = arm_mat_mult_f32(&eye_2, &xk, &xkk);  
//    xkk=xk;
	
//if(status==ARM_MATH_SIZE_MISMATCH){	while(1);}
//status = arm_mat_mult_f32(&eye_2, &Pkk, &Pk);  
////    Pkk=Pk;
	
position_actual=xk.pData[0];//K.pData[0];//
//    pos1(j)=xk(1);

speed_actual=xk.pData[1]*1000.0;
//    speed(j)=xk(2);

if(status==ARM_MATH_SIZE_MISMATCH){	while(1);}
}
