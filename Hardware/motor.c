#include "main.h"
#include "usart.h"
uint8_t usedata[8];
#define RATE_BUF_SIZE 6
#define MYABS(a)    ((a >= 0) ? (a) : -(a)) 
#define VALUE_LIMIT(v, l, h)   ((v) < (l) ? (l) : ((v) > (h) ? (h) : (v)))
#define VALUE_ABS_LIMIT(v, a)  VALUE_LIMIT(v, -a, a)
volatile dji_motor Pitch={0,0,0,0,0,0,0,0,0,0,0,0};
volatile dji_motor Yaw={0,0,0,0,0,0,0,0,0,0,0,0};
volatile dji_motor FEED,SHOOT_L,SHOOT_R;
PID_T GMPitchPid_Ang={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PID_T GMPitchPid_Spe={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PID_T GMYawPid_Ang={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PID_T GMYawPid_Spe={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
typedef union {
    float vf;          // 4�ֽڸ�����
    uint8_t vu8[4];    // 6�ֽ����飨4�ֽ����� + 2�ֽ�֡β��
} typedefvofa;

extern VisionUnion vision_recieve_union;
extern int ZM_yaw_counts;

typedefvofa sendarray[5];


void getEncoderData(volatile dji_motor *v)
{
	int i=0;
	int32_t temp_sum = 0;    
	v->last_raw_value = v->raw_value;
	v->raw_value = (usedata[0]<<8)|usedata[1];
	v->diff = v->raw_value - v->last_raw_value;
	if(v->diff < -4096)    //���α������ķ���ֵ���̫�󣬱�ʾȦ�������˸ı�
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff>4096)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff - 8192;
	}		
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	//����õ������ı��������ֵ
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	//����õ��Ƕ�ֵ����Χ���������
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	
	
	//�����ٶ�ƽ��ֵ
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);
	
	if(fabs((float)v->filter_rate_max) < fabs((float)v->filter_rate))
	{
		v->filter_rate_max = v->filter_rate;
	}
	v->current = (uint16_t)(usedata[4] << 8 | usedata[5]);
	v->tempature = usedata[6];
	v->re_speed= (uint16_t)(usedata[2] << 8 | usedata[3]);//�ٶ������

}


extern uint32_t dead_flag;
extern uint32_t first_wait_time;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//�����жϻص�
{   
  CAN_RxHeaderTypeDef	RxHeader;
  if(hcan->Instance==CAN1)
	{
		dead_flag = 0;
		first_wait_time = 0;
		HAL_CAN_GetRxMessage(&hcan1,CAN_FILTER_FIFO0,&RxHeader,usedata);
		switch(RxHeader.StdId)
		{
			case 0x205 : getEncoderData(&Yaw);   	break;		
      case 0x206 : getEncoderData(&Pitch);   break; 
		}	
	}
  else
	{
	 
	}
}



//float PID_Cal(PID_T *pid, float get, double set)  // getΪʵ��ֵ  setΪĿ��ֵ
//{	
//	pid->set = set ;
//	pid->real = get;
//	
//	pid->err_last = pid->err;
//	pid->err = pid->set - pid->real;
//	pid->integral += pid->err;
//	
//	pid->pout = pid->kp * pid->err;
//	pid->pout = pid->pout < pid->poutmax ? pid->pout : pid->poutmax;
//	pid->pout = pid->pout > -pid->poutmax ? pid->pout : -pid->poutmax;
//	
//	pid->iout = pid->ki * pid->integral;
//	pid->iout = pid->iout < pid->ioutmax  ? pid->iout : pid->ioutmax;
//	pid->iout = pid->iout > -pid->ioutmax ? pid->iout : -pid->ioutmax;
//	
//	pid->dout = pid->kd * (pid->err - pid->err_last);
//	pid->dout = pid->dout < pid->doutmax ? pid->dout : pid->doutmax;
//	pid->dout = pid->dout > -pid->doutmax ? pid->dout : -pid->doutmax;
//	
//	pid->out = pid->pout + pid->iout + pid->dout;
//	pid->out = pid->out < pid->outmax ? pid->out : pid->outmax;
//	pid->out = pid->out > -pid->outmax ? pid->out : -pid->outmax;
//	
//	return pid->out;
//}

float first_order_filter(float *filter_state, float input, float alpha)
{
	*filter_state = alpha * input + (1.0f - alpha) * (*filter_state);
	return *filter_state;
}

float PID_Cal(PID_T *pid, float get, double set)
{
	float abs_error;
	float speed_ratio = 1.0f;

	pid->set = set;
	pid->real = get;

	pid->err_last = pid->err;
	pid->err = pid->set - pid->real;

	abs_error = fabs(pid->err);

	if (abs_error < pid->dead)
	{
		pid->err = 0.0f;
		abs_error = 0.0f;
	}

	pid->pout = pid->kp * pid->err;
	pid->pout = VALUE_ABS_LIMIT(pid->pout, pid->poutmax);

	if (abs_error < pid->I_Separate_Threshold)
	{
		pid->integral += pid->err;
		pid->iout = pid->ki * pid->integral;
	}
	else
	{
		pid->integral = 0.0f;
		pid->iout = 0.0f;
	}
	pid->iout = VALUE_ABS_LIMIT(pid->iout, pid->ioutmax);

	pid->dout = pid->kd * (pid->err - pid->err_last);
	pid->dout = VALUE_ABS_LIMIT(pid->dout, pid->doutmax);

	pid->fout = (pid->set - pid->preset) * pid->kf;
	pid->fout = VALUE_ABS_LIMIT(pid->fout, pid->foutmax);

	pid->out = pid->pout + pid->iout + pid->dout + pid->fout;
	pid->out = VALUE_ABS_LIMIT(pid->out, pid->outmax);

	pid->preset = pid->set;
	pid->pre_real = pid->real;
	pid->out_last = pid->out;
	return pid->out;
}

void GMmotor_Stop (float Pit_Groy,float Yaw_Groy) 
{
	float M0,M1=0;
	M0 = PID_Cal(&GMPitchPid_Spe, Pit_Groy, 0);
  M1 = PID_Cal(&GMYawPid_Spe, Yaw_Groy, 0);        
}



//��̨
void GMmotor_run (float setPit,float setYaw,float Pit_Groy,float Yaw_Groy, float Pit_ang, float Yaw_ang) 
{
	if(setPit>25.0f)	setPit = 25.0f;
	if(setPit<-35.0f)	setPit = -35.0f;
	
	if(setYaw>55.0f)	setYaw = 55.0f;
	if(setYaw<-55.0f)	setYaw = -55.0f;
	
	float m0,m1=0;
	float M0,M1=0;
//	    sendarray[2].vu8[0] = 0x00;
//      sendarray[2].vu8[1] = 0x00;
//      sendarray[2].vu8[2] = 0x80;
//      sendarray[2].vu8[3] = 0x7f;

	m0 = PID_Cal(&GMPitchPid_Ang, Pit_ang, setPit);//Pit �ǶȻ�
	M0 = -PID_Cal(&GMPitchPid_Spe, Pit_Groy, m0);//Pit �ٶȻ�
	
	m1 = PID_Cal(&GMYawPid_Ang, Yaw_ang, setYaw);//Yaw Angle
  M1 = PID_Cal(&GMYawPid_Spe, Yaw_Groy, m1);//Yaw Speed
	
//	sendarray[0].vf = imu_angle.yaw;//0;
//  sendarray[1].vf = imu_angle.yaw;//vision_recieve_union.receive_vision.mode;
//	sendarray[2].vf = imu_angle.yaw;//ZM_yaw_counts;
//	sendarray[3].vf = imu_angle.yaw;//imu_angle.yaw_counts;
//  HAL_UART_Transmit(&huart6,&sendarray[0].vu8[0],12,1000);
	
	PHC_GIMBAL(M1,M0);
//	PHC_GIMBAL_PITCH(M0);
//		PHC_GIMBAL(0);
//	PHC_GIMBAL_PITCH(0);
}

/*��̨pid*/
void GMPID_Init_Hand(PID_T*GMPitchPid_Ang,PID_T*GMYawPid_Ang,PID_T*GMPitchPid_Spe,PID_T*GMYawPid_Spe)
{
	GMPitchPid_Ang->kp = 60.0f;
	GMPitchPid_Ang->ki = 2.0f;
	GMPitchPid_Ang->kd = 0.01f;
	GMPitchPid_Ang->kf = 0.0f;
	GMPitchPid_Ang->dead = 0.0f;
	GMPitchPid_Ang->I_Separate_Threshold = 0.4f;
	GMPitchPid_Ang->poutmax = 30000.0f;
	GMPitchPid_Ang->ioutmax = 2000.0f;
	GMPitchPid_Ang->doutmax = 12000.0f;
	GMPitchPid_Ang->foutmax = 10000.0f;
	GMPitchPid_Ang->outmax = 16000.0f;
	GMPitchPid_Ang->dt = 0.001f;
	GMPitchPid_Ang->i_var_speed_a = 0.0f;
	GMPitchPid_Ang->i_var_speed_b = 0.0f;
	GMPitchPid_Ang->d_first_enable = 0;
	GMPitchPid_Ang->lpf_alpha = 0.1f;
	GMPitchPid_Ang->integral = 0.0f;
	GMPitchPid_Ang->d_filter.out = 0.0f;
	GMPitchPid_Ang->err = 0.0f;
	GMPitchPid_Ang->err_last = 0.0f;
	
	GMPitchPid_Spe->kp = 30.0f;
	GMPitchPid_Spe->ki = 0.15f;
	GMPitchPid_Spe->kd = 0.0f;
	GMPitchPid_Spe->kf = 0.0f;
	GMPitchPid_Spe->dead = 0.0f;
	GMPitchPid_Spe->I_Separate_Threshold = 2.0f;
	GMPitchPid_Spe->poutmax = 40000.0f;
	GMPitchPid_Spe->ioutmax = 2000.0f;
	GMPitchPid_Spe->doutmax = 10000.0f;
	GMPitchPid_Spe->foutmax = 10000.0f;
	GMPitchPid_Spe->outmax = 16000.0f;
	GMPitchPid_Spe->dt = 0.001f;
	GMPitchPid_Spe->i_var_speed_a = 0.0f;
	GMPitchPid_Spe->i_var_speed_b = 0.0f;
	GMPitchPid_Spe->d_first_enable = 0;
	GMPitchPid_Spe->lpf_alpha = 0.1f;
	GMPitchPid_Spe->integral = 0.0f;
	GMPitchPid_Spe->d_filter.out = 0.0f;
	GMPitchPid_Spe->err = 0.0f;
	GMPitchPid_Spe->err_last = 0.0f;

	GMYawPid_Spe->kp = 25.0f;
	GMYawPid_Spe->ki = 0.0f;
	GMYawPid_Spe->kd = 0.0f;
	GMYawPid_Spe->kf = 0.0f;
	GMYawPid_Spe->dead = 0.0f;
	GMYawPid_Spe->I_Separate_Threshold = 3.0f;
	GMYawPid_Spe->poutmax = 30000.0f;
	GMYawPid_Spe->ioutmax = 2000.0f;
	GMYawPid_Spe->doutmax = 10000.0f;
	GMYawPid_Spe->foutmax = 10000.0f;
	GMYawPid_Spe->outmax = 16000.0f;
	GMYawPid_Spe->dt = 0.001f;
	GMYawPid_Spe->i_var_speed_a = 0.0f;
	GMYawPid_Spe->i_var_speed_b = 0.0f;
	GMYawPid_Spe->d_first_enable = 0;
	GMYawPid_Spe->lpf_alpha = 0.1f;
	GMYawPid_Spe->integral = 0.0f;
	GMYawPid_Spe->d_filter.out = 0.0f;
	GMYawPid_Spe->err = 0.0f;
	GMYawPid_Spe->err_last = 0.0f;

	GMYawPid_Ang->kp = 85.0f;
	GMYawPid_Ang->ki = 2.0f;
	GMYawPid_Ang->kd = 0.01f;
	GMYawPid_Ang->kf = 0.0f;
	GMYawPid_Ang->dead = 0.0f;
	GMYawPid_Ang->I_Separate_Threshold = 0.2f;
	GMYawPid_Ang->poutmax = 10000.0f;
	GMYawPid_Ang->ioutmax = 5000.0f;
	GMYawPid_Ang->doutmax = 8000.0f;
	GMYawPid_Ang->foutmax = 10000.0f;
	GMYawPid_Ang->outmax = 13000.0f;
	GMYawPid_Ang->dt = 0.001f;
	GMYawPid_Ang->i_var_speed_a = 0.0f;
	GMYawPid_Ang->i_var_speed_b = 0.0f;
	GMYawPid_Ang->d_first_enable = 0;
	GMYawPid_Ang->lpf_alpha = 0.1f;
	GMYawPid_Ang->integral = 0.0f;
	GMYawPid_Ang->d_filter.out = 0.0f;
	GMYawPid_Ang->err = 0.0f;
	GMYawPid_Ang->err_last = 0.0f;
}

void PID_Init_All(void)
{
	GMPID_Init_Hand(&GMPitchPid_Ang,&GMYawPid_Ang,&GMPitchPid_Spe,&GMYawPid_Spe);
}
