#ifndef _NEW_RC_H
#define _NEW_RC_H
#include  "main.h"
#include "struct_typedef.h"

typedef __packed struct
{
    uint8_t sof_1;
    uint8_t sof_2;
    uint64_t ch_0:11;//rx
    uint64_t ch_1:11;//ry
    uint64_t ch_2:11;//ly
    uint64_t ch_3:11;//lx
    uint64_t mode_sw:2;
    uint64_t pause:1;
    uint64_t fn_1:1;//自定义按键左
    uint64_t fn_2:1;//自定义按键右
    uint64_t wheel:11;
    uint64_t trigger:1;//扳机

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left:2;
    uint8_t mouse_right:2;
    uint8_t mouse_middle:2;
    uint16_t key;
    uint16_t crc16;
}VT13_data;


typedef struct
{
  int16_t R_x;  //右手x		右1684 左364 中1024  1684-1024 = 1024 - 364 = 660
	int16_t R_y;	 //右手y    上1684 下364 中1024
	int16_t L_x;
	int16_t L_y;
	int16_t mouse_x;	//鼠标
	int16_t mouse_y;
	int16_t mouse_z;
	uint8_t mouse_l;	//鼠标按键
	uint8_t mouse_r;
	uint8_t mouse_mid;
	uint16_t key;		//键盘
	uint8_t W;
	uint8_t S;
	uint8_t A;
	uint8_t D;
	uint8_t Q;
	uint8_t E;
	uint8_t F;
	uint8_t G;
	uint8_t Z;
	uint8_t X;
	uint8_t C;
	uint8_t V;
	uint8_t B;
	uint8_t Shift;
	uint8_t Ctrl;
	uint8_t R;
	uint8_t trigger;
	uint8_t mode;
	int16_t lun;
	uint64_t cnts;
	uint64_t off_line_times;
	uint64_t last_cnts;
	uint64_t now_cnts;
	uint8_t off_line_flag;     
	uint8_t s_l;
	uint8_t s_r;
}VT13_ctrl_t;


void tele_link_init(void);
void work_out_tele_data(VT13_data *data);
void work_out_init(void);
#endif
