#ifndef __VISION_TASK_H
#define __VISION_TASK_H
#include "main.h"
#include "CRC.h"

typedef enum{
	NO=0,
  YES
}Status_zimiao;
typedef enum{
	SHOUMIAO=0,
    ZIMIAO,
	//XIAOFU
	//DAFU
}Status_c;

#pragma pack(push, 1)
typedef struct 
{
  uint8_t header;
  int8_t mode;
	float pitch;
	float yaw;
  float distance;
  uint16_t ender;//接收的视觉crc的计算结果
} receive_struct;//16
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
	uint8_t header;
	float yaw;
	float pitch;
	float roll;
	char zhanwei_speed;//0
//	uint32_t timestamp_v;
	uint16_t ender;
} send_struct;//16+4
#pragma pack(pop)

typedef union {
    receive_struct receive_vision;
    u8 vision_data[16];
} VisionUnion;
//#pragma pack()

void receive_vision_task(uint8_t* Buf, uint32_t *Len);
void vision_task(void const * argument);
void send_vision_task(void);
void send_with_crc16(send_struct *data);
void send_with_crc16(send_struct *data);
void recieve_vision_cdc(uint8_t *buf);
void deal_diff(void);

#endif
