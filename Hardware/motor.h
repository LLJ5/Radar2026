#ifndef MOTHR__H
#define MOTHR__H
#include "stm32f4xx.h"                  // Device header

typedef struct{
	int32_t raw_value;   					 //����������������ԭʼֵ
	int32_t last_raw_value;					 //��һ�εı�����ԭʼֵ
	int32_t ecd_value;                       //���������������ı�����ֵ
	int32_t diff;							 //���α�����֮��Ĳ�ֵ
	int32_t temp_count;                      //������
	uint8_t buf_count;						 //�˲�����buf��
	int32_t ecd_bias;						 //��ʼ������ֵ	
	int32_t ecd_raw_rate;					 //ͨ������������õ����ٶ�ԭʼֵ
	int32_t rate_buf[6];	     //buf��for filter
	int32_t round_cnt;						 //Ȧ��
	int32_t filter_rate;					 //�ٶ�
	int32_t filter_rate_max;				 //����ٶ�
	float ecd_angle;						 //�Ƕ�
	float init_angle; 						 //��ʼ��һ������
	int16_t current;
	int16_t velo;
	int16_t electricity;
	int16_t re_speed;
	int8_t tempature;	
//	float real_angle;
//	float zero_angle;
//	float relative_zero_angle;
//	float ALL_angle;
//	int tool_flag;
}dji_motor;


//typedef struct
//{
//	double kp;
//	float ki;
//	float kd;
//	
//	float pout;
//	float iout;
//	double dout;
//	
//	float poutmax;
//	float ioutmax;
//	double doutmax;
//	float outmax;
//	
//	float set;
//	float real;
//	float out;
//	
//	float err;							
//	double err_last;					
//	double err_llast;				
//	float integral;					
//	
//}PID_T;

typedef struct
{
	double kp;
	float ki;
	float kd;
	float kf;
	
	float pout;
	float iout;
	double dout;
	float fout;
	
	float poutmax;
	float ioutmax;
	double doutmax;
	float foutmax;
	float outmax;
	float I_Separate_Threshold;
	
	float set;
	float preset;
	float real;
	float pre_real;
	float out;
	float out_last;
	float dead;
	
	float err;
	double err_last;
	float integral;
	
	float dt;
	float i_var_speed_a;
	float i_var_speed_b;
	uint8_t d_first_enable;
	float lpf_alpha;
	
	struct {
		float out;
	} d_filter;
} PID_T;

extern volatile dji_motor FEED,SHOOT_L,SHOOT_R;
float first_order_filter(float *filter_state, float input, float alpha);
void GMmotor_Stop (float Pit_Groy,float Yaw_Groy);
void GMmotor_run (float setPit,float setYaw,float Pit_Groy,float Yaw_Groy, float Pit_ang, float Yaw_ang);
void PID_Init_All(void);
void GMPID_Init_Hand(PID_T*GMPitchPid_Ang,PID_T*GMYawPid_Ang,PID_T*GMPitchPid_Spe,PID_T*GMYawPid_Spe);

#endif
