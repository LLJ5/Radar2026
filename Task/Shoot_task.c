#include "main.h"
extern int fire_ok;
float L_setST, R_setST, Feed_set=3500;//3500
int shoot=6100;
extern int fire_advise;
extern VT13_ctrl_t VT13_ctrl;
extern int DaFu_Flag;
void Shoot_Control(void const * argument)
{
	for(;;)
	{
	 
	 if(rc.F==1)   //F-
	 {shoot -= 1;}
	  if(rc.G==1)   //G+
	 {shoot += 1;}
	 if(shoot<5500)
	 {shoot=5500;}
	 if(shoot>6600)
	 {shoot=6600;}
	 if(rc.R==1)    //R»Ö¸´
	 {shoot=6200;}
		 L_setST = -shoot;
		 R_setST = -shoot;

//	if(rc.sl==1)    	 
//	{

//		if(fire_advise==1)
//		 {Feed_set =900;}//      3000
//		 else
//		 {Feed_set=0;}
//	}
	switch (rc.sl)
	  {
//		  case 1:
//				{Feed_set =900;}
//				break;
			case 1:
			  {Feed_set =0;}
			  break;
		}	

	 	  Firemotor_run (L_setST,R_setST,SHOOT_L.re_speed,SHOOT_R.re_speed,FEED.re_speed,Feed_set) ;

	osDelay(1);
	}
}

