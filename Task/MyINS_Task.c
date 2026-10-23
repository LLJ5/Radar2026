#include "main.h"

void Start_INS_Task(void const * argument)
{
	for(;;)
	{
	 INS_task();
		if(offset_OK_flag==0) offset_OK_flag = cali_gyro_hook();//͓҇У׼´ú«£¬offset_OK_flag==0ʱ²ŉúЧ£¬У׼½ኸº󯦦set_OK_flag=1£¬͓҇¶ÁȡÁㆯʽ¾ݺ󣬯ffset_OK_flag=2
		                       //ȴҪУ׼Áㆯ£¬ψ½«offset_OK_flagԚINS_task.cµĵڋĐУ¬ʖ¶¯¸Ďª0,У׼ͪ±Ϻ󱘐븄Ϊ1
	
	
	osDelay(1);
	}
}

