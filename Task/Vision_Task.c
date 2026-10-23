#include "main.h"
#include "usbd_cdc_if.h"

extKalman_t yaw_kalman;
extKalman_t pitch_kalman;
int fire_advise;
int aim_ok_flag=0;
extern float INS_quat[4];
extern GIMBAL_control gimbal_control;
float ZM_last_yaw,ZM_yaw_diff,ZM_yaw_counts, ZM_j,ZM_yaw_diff_abs;
extern IMU_angle imu_angle;
extern uint8_t Usb_flag;

Status_zimiao status_zimiao;
Status_c status_c;
int ZM_head_flag;
send_struct send_vision;
//JudgePacket packet;

VisionUnion vision_recieve_union;

void Vision_Control(void const * argument)
{
  /* USER CODE BEGIN state_task */
		
	KalmanCreate( &yaw_kalman, 1, 40 );
	KalmanCreate( &pitch_kalman, 1, 40 );
	static uint8_t ZM_last_status = NO;
  /* Infinite loop */
  for(;;)	
  {
		send_vision_task();
		gimbal_control.pitch_zimiao=KalmanFilter(&pitch_kalman,vision_recieve_union.receive_vision.pitch);
		gimbal_control.yaw_zimiao=KalmanFilter(&yaw_kalman,vision_recieve_union.receive_vision.yaw);
		status_zimiao= vision_recieve_union.receive_vision.mode;
		if(status_zimiao != NO && ZM_last_status != NO) 
		{
	  ZM_yaw_diff=gimbal_control.yaw_zimiao-ZM_last_yaw;

	    if(ZM_yaw_diff > 150.0f)
			  {ZM_yaw_counts --;
				ZM_head_flag = 1;}
			else
        {ZM_head_flag = 0;}
		if(ZM_yaw_diff < -150.0f)
			   {ZM_yaw_counts ++;
			    ZM_head_flag = 1;}
			else
            {ZM_head_flag = 0;}
			
		ZM_last_yaw = gimbal_control.yaw_zimiao;//再更新
		}		
		else if(status_zimiao != NO) 
			{
            ZM_last_yaw = gimbal_control.yaw_zimiao;
            ZM_head_flag = 0;
        }
        
        ZM_last_status = status_zimiao;  // 更新
		
    osDelay(5);
  }
}

u8 vsi;
void recieve_vision_cdc(u8 *buf)//调用该函数接收
{vsi++;
	if(buf[0]==0xA5&&Verify_CRC16_Check_Sum(buf,16))
	{
			memcpy(&vision_recieve_union.vision_data,buf,16);
	}
}
  
extern u8 shoot_17mmspe;
extern VT13_ctrl_t VT13_ctrl;
void send_vision_task()
{
    send_vision.header = 0x5A;
    send_vision.yaw    = imu_angle.yaw_real;
    send_vision.pitch  = imu_angle.pitch;
    send_vision.roll   = imu_angle.roll;
    send_vision.zhanwei_speed = 0;
//  	send_vision.timestamp_v = imu_angle.timestamp;
//	HAL_GetTick();   
    send_vision.ender  = 0x0d;

    uint8_t buf[sizeof(send_vision)];
    memcpy(buf, &send_vision, sizeof(send_vision));
    Append_CRC16_Check_Sum(buf, sizeof(send_vision));
	  
  	CDC_Transmit_FS(buf, sizeof(send_vision));
}

//void send_vision_task()
//{
//	
//	send_vision.header=0x5A;
//  send_vision.yaw=imu_angle.yaw;
//	send_vision.pitch=imu_angle.pitch;
//	send_vision.roll=imu_angle.roll;
//  send_vision.zhanwei_speed=0;
//	send_vision.ender=0x0d;
//	
//	u8 buf[sizeof(send_vision)];//取字节数
//	Verify_CRC16_Check_Sum(buf,sizeof(send_vision)-2);
//	
//	memcpy(buf,&send_vision,sizeof(send_vision));//
//	CDC_Transmit_FS(buf,sizeof(send_vision));
//}

void deal_diff()
{
			if(imu_angle.yaw_diff > 180.0f)
				ZM_yaw_counts--;
			else if(imu_angle.yaw_diff < -180.0f)
				ZM_yaw_counts++;
}

