#include "main.h"
GIMBAL_control gimbal_control={0};
CHASSIS_Task chassis_task;
extern VT13_ctrl_t VT13_ctrl;
chassis_mode_e chassis_mode;
receive_struct receive_struct_a;
extern dji_motor Yaw;
extern dji_motor Pitch;
int head_moad;
extern int fire_advise;
extern float ZM_yaw_counts;
extern uint8_t spin ;
extern VisionUnion vision_recieve_union;
extern int ZM_head_flag;
extern Status_zimiao status_zimiao;
extern Status_c status_c;
extern u8 live_3;
float now_yaw,now_pitch;
extern int mode;
extern union{
VT13_data use_data;
uint8_t  temp_data[21];
}VT13_USE;

uint32_t dead_flag = 0;
#define ZHONGZHI_YAW 0
#define ZHONGZHI_PITCH 0
//26   -45
extern uint32_t wait_time;

extern uint32_t first_wait_time;
extern uint8_t begain_flag;

void Gimbal_Control(void const * argument)
{
    PID_Init_All();

    float scan_yaw = 0.0f;        // yaw
    float scan_pitch = 0.0f;    // 
    int8_t yaw_dir = 1;           // yaw 
    int8_t pitch_dir = 1;         // 

    const float yaw_speed = 0.007f;   // 0.009
    const float pitch_speed = 0.015f; //0.028
		vision_recieve_union.receive_vision.mode = -1;
    for (;;)
    {
					dead_flag ++;
					first_wait_time ++;
			
					if(first_wait_time > 1000)	begain_flag = 1;
			
					if(dead_flag>500)	PID_Init_All();
					
					if(vision_recieve_union.receive_vision.mode == -1 || dead_flag>500)	PHC_GIMBAL(0,0);
					else
					{
						if(vision_recieve_union.receive_vision.mode == 1||vision_recieve_union.receive_vision.mode == 2)
						{
							now_pitch = gimbal_control.pitch_zimiao;
							now_yaw   = gimbal_control.yaw_zimiao;

							if (now_yaw > 30.0f) now_yaw = 30.0f;
							if (now_yaw < -40.0f) now_yaw = -40.0f;
							if (now_pitch > 10.0f) now_pitch = 10.0f;
							if (now_pitch < -5.0f) now_pitch = -5.0f;
						}
						else    
						{
							scan_yaw += yaw_speed * yaw_dir;
							scan_pitch += pitch_speed * pitch_dir;

							if (scan_yaw > 24.0f) { scan_yaw = 24.0f; yaw_dir = -1; }
							if (scan_yaw < -34.0f) { scan_yaw = -34.0f; yaw_dir = 1; }
							if (scan_pitch > 9.0f) { scan_pitch = 9.0f; pitch_dir = -1; }
							if (scan_pitch < -4.0f) { scan_pitch = -4.0f; pitch_dir = 1; }

							now_yaw = scan_yaw;
							now_pitch = scan_pitch;
						}

						gimbal_control.pitch_set = now_pitch;
						gimbal_control.yaw_set   = now_yaw;
        
						GMmotor_run(
            gimbal_control.pitch_set,
            gimbal_control.yaw_set,
            imu_angle.gyro_y,
            imu_angle.gyro_z,
            imu_angle.pitch,
            imu_angle.yaw_real);
					}

        osDelay(2);
    }
}

WindowFilter_Parameter pit_window = {50};
WindowFilter_Parameter yaw_window = {50};
float WindowFilter(WindowFilter_Parameter *Window_Struct,float inPut)
{
	float outPut = 0;
  
  Window_Struct -> buf[Window_Struct -> index ++] = inPut;
  
  if(Window_Struct -> index >= Window_Struct -> len)	{Window_Struct -> index = 0;}
        
  for(uint8_t i = 0;i < Window_Struct -> len;i ++)
  {
		outPut += Window_Struct -> buf[i];
  }
  
  outPut /= Window_Struct -> len;
  
  return outPut;
}

