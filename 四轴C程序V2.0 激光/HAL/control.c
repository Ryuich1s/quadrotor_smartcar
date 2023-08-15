//========================================================================
//	°®ºÃÕßµç×Ó¹¤×÷ÊÒ-ÌÔ±¦ https://devotee.taobao.com/
//	STM32ËÄÖá°®ºÃÕßQQÈº: 799870988
//	×÷Õß£ºĞ¡Áõ
//	µç»°:13728698082
//	ÓÊÏä:1042763631@qq.com
//	ÈÕÆÚ£º2020.05.17
//	°æ±¾£ºV1.0
//========================================================================
//Ì×¼ş¹ºÂòµØÖ·£ºhttps://devotee.taobao.com/
//Ì×¼ş¹ºÂòµØÖ·£ºhttps://devotee.taobao.com/
//                 °®ºÃÕßµç×Ó¹¤×÷ÊÒ
//ÌØ´ËÉùÃ÷£º
//
//         ´Ë³ÌĞòÖ»ÄÜÓÃ×÷Ñ§Ï°£¬ÈçÓÃÉÌÒµÓÃÍ¾¡£±Ø×·¾¿ÔğÈÎ£¡
//          
//
//
#include "ALL_DATA.h" 
#include "ALL_DEFINE.h" 
#include "control.h"
#include "pid.h"
#include "attitude_process.h"
#include "flow.h"
#include "spl06.h"
//------------------------------------------------------------------------------
#undef NULL
#define NULL 0
#undef DISABLE 
#define DISABLE 0
#undef ENABLE 
#define ENABLE 1
#undef REST
#define REST 0
#undef SET 
#define SET 1 
#undef EMERGENT
#define EMERGENT 0

#define REMOTE_THR Remote.thr
#define REMOTE_PITCH Remote.pitch
#define REMOTE_ROLL Remote.roll
#define REMOTE_YAW Remote.yaw
//#define measured FeedBack
#define Expect desired 	

 float Throttle_out; //·É¿ØÓÍÃÅÊä³öÖµ //Ò£¿Ø + ¶¨¸ß Êä³öÖµ
//------------------------------------------------------------------------------
PidObject *(pPidObject[])={&pidRateX,&pidRateY,&pidRateZ,&pidRoll,&pidPitch,&pidYaw   //½á¹¹ÌåÊı×é£¬½«Ã¿Ò»¸öÊı×é·ÅÒ»¸öpid½á¹¹Ìå£¬ÕâÑù¾Í¿ÉÒÔÅúÁ¿²Ù×÷¸÷¸öPIDµÄÊı¾İÁË  ±ÈÈç½âËøÊ±ÅúÁ¿¸´Î»pid¿ØÖÆÊı¾İ£¬ĞÂÊÖÃ÷°×Õâ¾ä»°µÄ×÷ÓÃ¾Í¿ÉÒÔÁË
		,&pidHeightRate,&pidHeightHigh,&Flow_SpeedPid_x,&Flow_PosPid_x,&Flow_SpeedPid_y,&Flow_PosPid_y
};



//×ËÌ¬Òì³£¼ì²â
void Att_Err_Check(float dT)
{
//	static float time;
//  int16_t acc;       //µ±Ç°Äã·ÉĞĞÆ÷µÄ¼ÓËÙ¶ÈÖµ
	
	
	if(!ALL_flag.unlock) return;  
	
//  pixel_flow.Att_Err_ACC=acc = (int16_t)GetAccz();
	
	if(-Angle.roll>30||-Angle.roll<-30||Angle.pitch>30||Angle.pitch<-30||FlightData.High.bara_height>800)
	{
			ALL_flag.unlock = EMERGENT;
		
		LED.status = AllFlashLight;	                        
		LED.FlashTime = 300; 					
	}
	

//	
//	if( acc >60 ) time += dT;//»ñÈ¡ZÖáACC
//	else 	time = 0;

//	if(time>0.2f)
//	{
//		ALL_flag.unlock = 0;
//	}
}



/**************************************************************
 *  Height control
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/

//static uint8_t set_high_desired = 0; //¶¨¸ß¸ß¶ÈÒÑÉè¶¨

int16_t  HIGH_START =150;   //Ò»¼üÆğ·ÉÄ¿±ê¸ß¶È


uint32_t Control_high = 0; //µ±Ç°¸ß¶È
/**************************************************************
 *  //¸ß¶È¿ØÖÆÆ÷     ÆøÑ¹
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void HeightPidControl(float dt)  //¸ß¶È¿ØÖÆÆ÷     ÆøÑ¹
{
	volatile static uint8_t status=WAITING_1;
		int16_t acc;       //µ±Ç°Ä·ÉĞĞÆ÷µÄ¼ÓËÙ¶ÈÖµ
   	int16_t acc_error; //µ±Ç°¼ÓËÙ¶È¼õÈ¥ÖØÁ¦¼ÓËÙ¶ÈÔòÎªÉÏÏÂÒÆ¶¯µÄ¼ÓËÙ¶È
   static int16_t acc_offset;//ÖØÁ¦¼ÓËÙ¶ÈÖµ	 
		static float thr_hold = 0; 
		static uint8_t set_high = 0,High_breaktime;
	
	 if(mini.flow_High<400)
	 {
			Control_high=mini.flow_High;      //¸üĞÂ¼¤¹â¸ß¶È
	 }
	 else
	 {
	 	  Control_high=FlightData.High.bara_height ;	 //¸üĞÂÆøÑ¹¸ß¶È
	 }
		 
	
	 
	 
	 
	 
	//----------------------------------------------	
	{ //»ñÈ¡´¹Ö±ËÙ¶ÈÊı¾İ

		  acc = (int16_t)GetAccz(); //»ñÈ¡ZÖáACC  
		
			if(!ALL_flag.unlock)      //È¡µÃ¾²Ì¬ACCÖµ 
			{
				acc_offset = acc;
			}
			acc_error = acc - acc_offset;	
						
			{//´Ë´¦×öÒ»¸öËÙ¶ÈÓë¸ß¶ÈµÄ»¥²¹ÂË²¨ 
				static float last_high;
				pidHeightRate.measured = (pidHeightRate.measured + acc_error * dt)*0.98f + 0.02f*(Control_high - last_high)/dt; //ËÙ¶È»·ÓÀÔ¶ÊÇÖ÷µ÷£¬ËùÒÔ»¥²¹ÂË²¨¹Ø¼ü¾Í×¥ÔÚËÙ¶È»·ÕâÀï
				last_high =  pidHeightHigh.measured = Control_high;  //ÊµÊ±¸ß¶È·´À¡¸øÍâ»·
			}	
	}
	//----------------------------------------------½ô¼±ÖÕÖ¹·ÉĞĞ
	if(ALL_flag.unlock == EMERGENT) //ÒâÍâÇé¿ö£¬ÇëÊ¹ÓÃÒ£¿Ø½ô¼±ÉÏËø£¬·É¿Ø¾Í¿ÉÒÔÔÚÈÎºÎÇé¿öÏÂ½ô¼±ÖĞÖ¹·ÉĞĞ£¬Ëø¶¨·ÉĞĞÆ÷£¬ÍË³öPID¿ØÖÆ
		status = EXIT_255;
	//----------------------------------------------¿ØÖÆ
	switch(status)
	{
		case WAITING_1: //¼ì²â¶¨¸ß
		  if(ALL_flag.height_lock && ALL_flag.unlock) 
			{
				LED.status = DANGEROURS;
				status = WAITING_2;
				thr_hold=0; 				        //Çå³ı¼ÇÂ¼¶¨¸ßÊ±µÄÓÍÃÅ
			}
			break;
		case WAITING_2: //¶¨¸ßÇ°×¼±¸
			thr_hold = Remote.thr -1000;  //¼ÇÂ¼¶¨¸ßÊ±µÄÓÍÃÅ
		  set_high = 0;
		  pidHeightHigh.desired = HIGH_START;  //ÆÚÍû¸ß¶È Éè¶¨Öµ
		
		  Flow_PosPid_y.desired = pixel_flow.loc_y;//¼ÇÂ¼Î»ÖÃ //Ë¢ĞÂÎ»ÖÃÆÚÍû
			Flow_PosPid_x.desired = pixel_flow.loc_x;//¼ÇÂ¼Î»ÖÃ	//Ë¢ĞÂÎ»ÖÃÆÚÍû
		
 			status = PROCESS_31;
			break; 
		
		case PROCESS_31://½øÈë¶¨¸ß	

//				if(((Remote.thr -1000) > (thr_hold+10)) ||((Remote.thr -1000) < (thr_hold-10))) //²¦¶¯Ò£¸ËÖ»½øĞĞË®Æ½ËÙ¶È¿ØÖÆ
//				{			 																			  
//						if(High_breaktime++ >20)
//						{
//							thr_hold = Remote.thr -1000;  //¼ÇÂ¼¶¨¸ßÊ±µÄÓÍÃÅ	
//							High_breaktime = 0;
//							set_high = 0;
//						}														
//				}
//				else
//				{}		 
//				if(set_high == 0) //Èç¹û¸ÕÍË³öµ÷¸ß
//				{
//					set_high = 1;
//					pidHeightHigh.desired = Control_high;//¼ÇÂ¼¸ß¶Èµ±×öµ±Ç°¶¨¸ß¸ß¶È		  
//				}
		
				pidUpdate(&pidHeightHigh,dt);    //µ÷ÓÃPID´¦Àíº¯ÊıÀ´´¦ÀíÍâ»·	¸©Ñö½ÇPID	
				pidHeightRate.desired = pidHeightHigh.out;  //¸ß¶È»·Êä³ö¸ß¶ÈËÙ¶ÈÉè¶¨Öµ
						 					 										 
				pidUpdate(&pidHeightRate,dt); //ÔÙµ÷ÓÃ¸ß¶ÈËÙ¶ÈÄÚ»·
					 
	//		  pidHeightRate.out += Remote.thr -1000;//¼ÓÈëĞüÍ£Ê±µÄÓÍÃÅ
					 
				if(!ALL_flag.height_lock)     //ÍË³ö¶¨¸ß
				{
					LED.status = AlwaysOn ;
					status = EXIT_255;
				}
			break;
		case EXIT_255: //ÍË³ö¶¨¸ß
			pidRest(&pPidObject[6],1);	//Çå³ıµ±Ç°µÄ¶¨¸ßÊä³öÖµ
			status = WAITING_1;//»Øµ½µÈ´ı½øÈë¶¨¸ß
			break;
		default:
			status = WAITING_1;
			break;	
	}	
				
}

void Mode_Controler(float dt)
{
		const float roll_pitch_ratio = 0.04f;
	
//		if(ALL_flag.unlock == 1)  //ÅĞ¶Ï½âËø
		{
//			if(Remote.AUX2 < 1700)   //Èç¹û´óÓÚ1700 Ôò½øÈë¶¨µãÄ£Ê½ÏÔÊ¾
//			{
//				Command.FlightMode = HEIGHT;
//				ALL_flag.height_lock = 1;
				Flow_mode_two();       // Ò£¿Ø-¹âÁ÷¿ØÖÆ×Ë
//				set_flag=0x21;         // OLED¶¨¸ß¶¨µãÄ£Ê½ÏÔÊ¾
//			}
//			else                     //×ËÌ¬Ä£Ê½
//			{  
//				Command.FlightMode = NORMOL;	
//				ALL_flag.height_lock = 0;
//				set_flag=0x00;        //OLED×ËÌ¬Ä£Ê½ÏÔÊ¾
//				
//				pidPitch.desired = -(Remote.pitch-1500)*roll_pitch_ratio;  //Ò¡¸Ë¿ØÖÆ
//				pidRoll.desired  = -(Remote.roll-1500) *roll_pitch_ratio;  //Ò¡¸Ë¿ØÖÆ
//			}
		}					
}

u32 altHold_Pos_Save = 0,Pos_breaktime = 0,Pos_break_save = 0;

///////////////////////// Ò£¿Ø-¹âÁ÷¿ØÖÆ×ËÌ¬////////////////////////////////////////////////
void Flow_mode_two(void)
{

		const float roll_pitch_ratio = 8.00f;	

		if((mini.ok == 1) && (Control_high>5) && (Control_high<400))//ÅĞ¶ÏÊÇ·ñ´æÔÚ¹âÁ÷  ¸ß¶È¸ßÓÚ5CM ¹âÁ÷²Å¿ÉÒÔ¶¨µã
		{
			Flow_SpeedPid_x.desired = (-(Remote.pitch-1500)*0.06f)*roll_pitch_ratio;   //Ö±½Ó¿ØÖÆËÙ¶È ²¢ÇÒ¹ØµôÍâ»·¼ÆËã
			Flow_SpeedPid_y.desired = (-(Remote.roll-1500)*0.06f)*roll_pitch_ratio;
			
			pidPitch.desired =LIMIT(Flow_SpeedPid_x.out*0.1,-15,15) ; //×ËÌ¬Íâ»·ÆÚÍûÖµ
			pidRoll.desired = LIMIT(Flow_SpeedPid_y.out*0.1,-15,15) ; //×ËÌ¬Íâ»·ÆÚÍûÖµ
			
		}
//		else 
//		{
//			pidPitch.desired = -(Remote.pitch-1500)*0.04f; 
//			pidRoll.desired  = -(Remote.roll-1500)*0.04f;  
//		}
}



uint16_t SStime,move_X,move_Y;

void move_Controler(void)   
{	
	 SStime++;
	 pixel_flow.Att_Err_ACC=SStime;
	
	 if(SStime>=15)	
	 {  		
		  SStime=15;
		  Remote.roll = 1500; 
			Remote.pitch = 1500; 
			 move_X=0;	
			 move_X=0;			 
	 }
//	 else
//	 if(SStime>8)	
//	 {
//	    Remote.roll = 1700;        //ÓÒ
//			Remote.pitch = 1600;	
//      move_X=0;	
//		  move_Y=1;			 
//	 }
//	 else
//	 if(SStime>6)	
//	 {
//	    Remote.roll = 1300;        //×ó
//			Remote.pitch = 1400;
//      move_X=0;	
//			move_Y=1;		 
//	 }
	 else
	 if(SStime>8)	
	 {
//	    Remote.roll = 1500; 		
//		  Remote.pitch = 1400;			 //ºó	
//		  move_X=1;
//			move_Y=0;	
		 
		        
		    Flow_PosPid_x.desired = -70;//¼ÇÂ¼Î»ÖÃ //Ë¢ĞÂÎ»ÖÃÆÚÍû
		    Flow_PosPid_y.desired = 40;//¼ÇÂ¼Î»ÖÃ //Ë¢ĞÂÎ»ÖÃÆÚÍû
	 }
	 else
	 if(SStime>5)	
	 {
//	    Remote.roll = 1500; 
//			Remote.pitch = 1600;			 //Ç°
//      move_X=1;	
//			move_Y=0;
        Flow_PosPid_x.desired = -70;//¼ÇÂ¼Î»ÖÃ //Ë¢ĞÂÎ»ÖÃÆÚÍû
		    Flow_PosPid_y.desired = 0;//¼ÇÂ¼Î»ÖÃ //Ë¢ĞÂÎ»ÖÃÆÚÍû
	 }
	 else
	 if(SStime>0)
	 { 	 
//	    Remote.roll = 1500; 
//			Remote.pitch = 1500;	 
		  move_X=0;	
			move_Y=0;
	 }
	 
	 
}


/**************************************************************
 * //Î»ÖÃ¶¨µã¿ØÖÆÆ÷  ¹âÁ÷
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void Flow_Pos_Controler(float dt)  //Î»ÖÃ¶¨µã¿ØÖÆÆ÷  ¹âÁ÷
{
		const uint16_t DEADBAND=150;	
  if((mini.ok == 1) && (Control_high>5) && (Control_high<400))//ÅĞ¶ÏÊÇ·ñ´æÔÚ¹âÁ÷  ¸ß¶È¸ßÓÚ20CM ¹âÁ÷²Å¿ÉÒÔ¶¨µã
	{	

//		   //·½ÏòÒ¡¸Ë»ØÖĞ
//			if((Remote.pitch>(1500+DEADBAND))||(Remote.pitch<(1500-DEADBAND))||(Remote.roll>(1500+DEADBAND))||(Remote.roll<(1500-DEADBAND))) //²¦¶¯Ò£¸ËÖ»½øĞĞË®Æ½ËÙ¶È¿ØÖÆ
//			{				
//					mini.flow_x_i = 0;    			//Çå³ı¹âÁ÷Êı¾İ
//					mini.flow_y_i = 0;					//Çå³ı¹âÁ÷Êı¾İ
//					altHold_Pos_Save = 1;  			//¼ÇÂ¼Î»ÖÃ±êÖ¾ÖØĞÂ´ò¿ª
//				  Pos_break_save = 0;
//			}
//			else	//²¦¶¯Ò¡¸ËµÄÊ±ºò
			{				
//				  if(altHold_Pos_Save == 1)   //¼ÇÂ¼Î»ÖÃ Ò»´Î
//					{
//						altHold_Pos_Save = 0; 		//¹Ø±Õ¼ÇÂ¼Î»ÖÃ±êÖ¾
//						Pos_break_save = 1;       //´ò¿ª²¹³¥É²³µ»º³å±êÖ¾
//						
//								
//					}
					
					if(move_Y==0)//Íâ»·Î»ÖÃ¿ØÖÆ
					{	
						Flow_PosPid_y.measured = pixel_flow.loc_y;//ÊµÊ±Î»ÖÃ·´À¡
						pidUpdate(&Flow_PosPid_y,dt);//Î»ÖÃÔËËãPID
						Flow_SpeedPid_y.desired = LIMIT(Flow_PosPid_y.out,-1000,1000);//Î»ÖÃPIDÊä³ö¸øËÙ¶ÈÆÚÍû
					}
					else
					{
//					  Flow_PosPid_y.desired = pixel_flow.loc_y;//¼ÇÂ¼Î»ÖÃ //Ë¢ĞÂÎ»ÖÃÆÚÍû
					}
					
					if(move_X==0)//Íâ»·Î»ÖÃ¿ØÖÆ
					{
						Flow_PosPid_x.measured = pixel_flow.loc_x;//ÊµÊ±Î»ÖÃ·´À¡
						pidUpdate(&Flow_PosPid_x,dt);//Î»ÖÃÔËËãPID
						Flow_SpeedPid_x.desired = LIMIT(Flow_PosPid_x.out,-1000,1000);//Î»ÖÃPIDÊä³ö¸øËÙ¶ÈÆÚÍû
					}
					else
					{
//					   Flow_PosPid_x.desired = pixel_flow.loc_x;//¼ÇÂ¼Î»ÖÃ	//Ë¢ĞÂÎ»ÖÃÆÚÍû
					}
					
					//ÄÚ»·ÆÚÍû  
					
					
			}
						
				//ÄÚ»·                                                                                                
				Flow_SpeedPid_y.measured = pixel_flow.loc_y;//ËÙ¶È·´À¡
				pidUpdate(&Flow_SpeedPid_y,dt);//ËÙ¶ÈÔËËã
			
			
				Flow_SpeedPid_x.measured = pixel_flow.loc_x;//ËÙ¶È·´À¡
				pidUpdate(&Flow_SpeedPid_x,dt);//ËÙ¶ÈÔËËã
					
	}
	else
	{
					mini.flow_x_i = 0;				//¹âÁ÷¸´Î»
					mini.flow_y_i = 0;				//¹âÁ÷¸´Î»
					Flow_SpeedPid_x.out = 0;	//µÖÏû¹ßĞÔ
					Flow_SpeedPid_y.out = 0;	//µÖÏû¹ßĞÔ
	}

	
}

/**************************************************************
 * ×ËÌ¬¿ØÖÆ
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void FlightPidControl(float dt)
{
	volatile static uint8_t status=WAITING_1;

	switch(status)
	{		
		case WAITING_1: //µÈ´ı½âËø
			if(ALL_flag.unlock)
			{
				status = READY_11;	
			}			
			break;
		case READY_11:  //×¼±¸½øÈë¿ØÖÆ
			pidRest(pPidObject,8); //ÅúÁ¿¸´Î»PIDÊı¾İ£¬·ÀÖ¹ÉÏ´ÎÒÅÁôµÄÊı¾İÓ°Ïì±¾´Î¿ØÖÆ

			Angle.yaw = pidYaw.desired =  pidYaw.measured = 0;   //Ëø¶¨Æ«º½½Ç
		
			status = PROCESS_31;
		
			break;			
		case PROCESS_31: //ÕıÊ½½øÈë¿ØÖÆ
			
      pidRateX.measured = MPU6050.gyroX * Gyro_G; //ÄÚ»·²âÁ¿Öµ ½Ç¶È/Ãë
			pidRateY.measured = MPU6050.gyroY * Gyro_G; //ÄÚ»·²âÁ¿Öµ ½Ç¶È/Ãë
			pidRateZ.measured = MPU6050.gyroZ * Gyro_G; //ÄÚ»·²âÁ¿Öµ ½Ç¶È/Ãë
		
			pidPitch.measured = Angle.pitch; 		//Íâ»·²âÁ¿Öµ µ¥Î»£º½Ç¶È
		  pidRoll.measured = Angle.roll;			//Íâ»·²âÁ¿Öµ µ¥Î»£º½Ç¶È
			pidYaw.measured = Angle.yaw;				//Íâ»·²âÁ¿Öµ µ¥Î»£º½Ç¶È
		
		 	pidUpdate(&pidRoll,dt);    //µ÷ÓÃPID´¦Àíº¯ÊıÀ´´¦ÀíÍâ»·	ºá¹ö½ÇPID		
			pidRateX.desired = pidRoll.out; //½«Íâ»·µÄPIDÊä³ö×÷ÎªÄÚ»·PIDµÄÆÚÍûÖµ¼´Îª´®¼¶PID
			pidUpdate(&pidRateX,dt);  //ÔÙµ÷ÓÃÄÚ»·

		 	pidUpdate(&pidPitch,dt);    //µ÷ÓÃPID´¦Àíº¯ÊıÀ´´¦ÀíÍâ»·	¸©Ñö½ÇPID	
			pidRateY.desired = pidPitch.out;  
			pidUpdate(&pidRateY,dt); //ÔÙµ÷ÓÃÄÚ»·

			CascadePID(&pidRateZ,&pidYaw,dt);	//Ò²¿ÉÒÔÖ±½Óµ÷ÓÃ´®¼¶PIDº¯ÊıÀ´´¦Àí
			break;
		case EXIT_255:  						//ÍË³ö¿ØÖÆ
			pidRest(pPidObject,8);		//¸´Î»PID²ÎÊı
			status = WAITING_1;				//·µ»ØµÈ´ı½âËø
		  break;
		default:
			status = EXIT_255;
			break;
	}
	if(ALL_flag.unlock == EMERGENT) //ÒâÍâÇé¿ö£¬ÇëÊ¹ÓÃÒ£¿Ø½ô¼±ÉÏËø£¬·É¿Ø¾Í¿ÉÒÔÔÚÈÎºÎÇé¿öÏÂ½ô¼±ÖĞÖ¹·ÉĞĞ£¬Ëø¶¨·ÉĞĞÆ÷£¬ÍË³öPID¿ØÖÆ
		status = EXIT_255;
}


int16_t motor[4];
#define MOTOR1 motor[0] 
#define MOTOR2 motor[1] 
#define MOTOR3 motor[2] 
#define MOTOR4 motor[3] 

void MotorControl(void) //µç»ú¿ØÖÆ
{	
	volatile static uint8_t status=WAITING_1;
	
	
	if(ALL_flag.unlock == EMERGENT) //ÒâÍâÇé¿ö£¬ÇëÊ¹ÓÃÒ£¿Ø½ô¼±ÉÏËø£¬·É¿Ø¾Í¿ÉÒÔÔÚÈÎºÎÇé¿öÏÂ½ô¼±ÖĞÖ¹·ÉĞĞ£¬Ëø¶¨·ÉĞĞÆ÷£¬ÍË³öPID¿ØÖÆ
		status = EXIT_255;	
	switch(status)
	{		
		case WAITING_1: 	     //µÈ´ı½âËø	
			MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;  //Èç¹ûËø¶¨£¬Ôòµç»úÊä³ö¶¼Îª0
			if(ALL_flag.unlock)
			{
				status = WAITING_2;
			}
		case WAITING_2: //½âËøÍê³ÉºóÅĞ¶ÏÊ¹ÓÃÕßÊÇ·ñ¿ªÊ¼²¦¶¯Ò£¸Ë½øĞĞ·ÉĞĞ¿ØÖÆ
			if(Remote.thr>1100)
			{
				status = PROCESS_31;
			}
			break;
		case PROCESS_31:
			{
				int16_t thr_temp;
								
				if(ALL_flag.height_lock) //¶¨¸ßÄ£Ê½ÏÂ ÓÍÃÅÒ£¸Ë×÷Îªµ÷Õû¸ß¶ÈÊ¹ÓÃ   
				{		
					thr_temp = pidHeightRate.out + (Remote.thr -1000); //Êä³ö¸øµç»úµÄÊÇ¶¨¸ßÊä³öÖµ
				}
				else 										 //Õı³£·ÉĞĞ×´Ì¬£¬ÓÍÃÅÕı³£Ê¹ÓÃ
				{
					thr_temp = Remote.thr -1000; //Êä³ö¸øµç»úµÄÊÇÓÍÃÅÊä³öÖµ
				}
				
				if(Remote.thr<1020)		//ÓÍÃÅÌ«µÍÁË£¬ÔòÏŞÖÆÊä³ö  ²»È»·É»úÂÒ×ª												
				{
					MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4=0;
					break;
				}
				MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = LIMIT(thr_temp,0,900); //Áô100¸ø×ËÌ¬¿ØÖÆ

				MOTOR1 +=    + pidRateX.out - pidRateY.out - pidRateZ.out;//; ×ËÌ¬Êä³ö·ÖÅä¸ø¸÷¸öµç»úµÄ¿ØÖÆÁ¿
				MOTOR2 +=    + pidRateX.out + pidRateY.out + pidRateZ.out ;//;
				MOTOR3 +=    - pidRateX.out + pidRateY.out - pidRateZ.out;
				MOTOR4 +=    - pidRateX.out - pidRateY.out + pidRateZ.out;//;
			}	
			break;
		case EXIT_255:
			MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;  //Èç¹ûËø¶¨£¬Ôòµç»úÊä³ö¶¼Îª0
			status = WAITING_1;	
			break;
		default:
			break;
	}
	
	
//	TIM2->CCR1 = LIMIT(MOTOR1,0,1000);  //¸üĞÂPWM1
//	TIM2->CCR2 = LIMIT(MOTOR2,0,1000);  //¸üĞÂPWM2
//	TIM3->CCR1 = LIMIT(MOTOR3,0,1000);  //¸üĞÂPWM3
//	TIM3->CCR2 = LIMIT(MOTOR4,0,1000);  //¸üĞÂPWM4
////	TIM2->CCR3 = LIMIT(MOTOR3,0,1000);  //¸üĞÂPWM3
////	TIM2->CCR4 = LIMIT(MOTOR4,0,1000);  //¸üĞÂPWM4
	
	#if (FLY_TYPE == 1 || FLY_TYPE == 2)
	
	PWM0 = LIMIT(MOTOR1,0,1000);  //¸üĞÂPWM1
	PWM1 = LIMIT(MOTOR2,0,1000);  //¸üĞÂPWM2
	PWM2 = LIMIT(MOTOR3,0,1000);  //¸üĞÂPWM3
	PWM3 = LIMIT(MOTOR4,0,1000);  //¸üĞÂPWM4
	
#elif (FLY_TYPE >= 3)
	
	PWM0 = 1000 + LIMIT(MOTOR1,0,1000);  //¸üĞÂPWM1
	PWM1 = 1000 + LIMIT(MOTOR2,0,1000);  //¸üĞÂPWM2
	PWM2 = 1000 + LIMIT(MOTOR3,0,1000);  //¸üĞÂPWM3] ;     
	PWM3 = 1000 + LIMIT(MOTOR4,0,1000);  //¸üĞÂPWM4
	
#else
	#error Please define FLY_TYPE!
		
#endif

} 


/************************************END OF FILE********************************************/ 



