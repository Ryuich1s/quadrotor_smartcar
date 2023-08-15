//========================================================================
//	�����ߵ��ӹ�����-�Ա� https://devotee.taobao.com/
//	STM32���ᰮ����QQȺ: 799870988
//	���ߣ�С��
//	�绰:13728698082
//	����:1042763631@qq.com
//	���ڣ�2020.05.17
//	�汾��V1.0
//========================================================================
//�׼������ַ��https://devotee.taobao.com/
//�׼������ַ��https://devotee.taobao.com/
//                 �����ߵ��ӹ�����
//�ش�������
//
//         �˳���ֻ������ѧϰ��������ҵ��;����׷�����Σ�
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

 float Throttle_out; //�ɿ��������ֵ //ң�� + ���� ���ֵ
//------------------------------------------------------------------------------
PidObject *(pPidObject[])={&pidRateX,&pidRateY,&pidRateZ,&pidRoll,&pidPitch,&pidYaw   //�ṹ�����飬��ÿһ�������һ��pid�ṹ�壬�����Ϳ���������������PID��������  �������ʱ������λpid�������ݣ�����������仰�����þͿ�����
		,&pidHeightRate,&pidHeightHigh,&Flow_SpeedPid_x,&Flow_PosPid_x,&Flow_SpeedPid_y,&Flow_PosPid_y
};



//��̬�쳣���
void Att_Err_Check(float dT)
{
//	static float time;
//  int16_t acc;       //��ǰ��������ļ��ٶ�ֵ
	
	
	if(!ALL_flag.unlock) return;  
	
//  pixel_flow.Att_Err_ACC=acc = (int16_t)GetAccz();
	
	if(-Angle.roll>30||-Angle.roll<-30||Angle.pitch>30||Angle.pitch<-30||FlightData.High.bara_height>800)
	{
			ALL_flag.unlock = EMERGENT;
		
		LED.status = AllFlashLight;	                        
		LED.FlashTime = 300; 					
	}
	

//	
//	if( acc >60 ) time += dT;//��ȡZ��ACC
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

//static uint8_t set_high_desired = 0; //���߸߶����趨

int16_t  HIGH_START =150;   //һ�����Ŀ��߶�


uint32_t Control_high = 0; //��ǰ�߶�
/**************************************************************
 *  //�߶ȿ�����     ��ѹ
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void HeightPidControl(float dt)  //�߶ȿ�����     ��ѹ
{
	volatile static uint8_t status=WAITING_1;
		int16_t acc;       //��ǰķ������ļ��ٶ�ֵ
   	int16_t acc_error; //��ǰ���ٶȼ�ȥ�������ٶ���Ϊ�����ƶ��ļ��ٶ�
   static int16_t acc_offset;//�������ٶ�ֵ	 
		static float thr_hold = 0; 
		static uint8_t set_high = 0,High_breaktime;
	
	 if(mini.flow_High<400)
	 {
			Control_high=mini.flow_High;      //���¼���߶�
	 }
	 else
	 {
	 	  Control_high=FlightData.High.bara_height ;	 //������ѹ�߶�
	 }
		 
	
	 
	 
	 
	 
	//----------------------------------------------	
	{ //��ȡ��ֱ�ٶ�����

		  acc = (int16_t)GetAccz(); //��ȡZ��ACC  
		
			if(!ALL_flag.unlock)      //ȡ�þ�̬ACCֵ 
			{
				acc_offset = acc;
			}
			acc_error = acc - acc_offset;	
						
			{//�˴���һ���ٶ���߶ȵĻ����˲� 
				static float last_high;
				pidHeightRate.measured = (pidHeightRate.measured + acc_error * dt)*0.98f + 0.02f*(Control_high - last_high)/dt; //�ٶȻ���Զ�����������Ի����˲��ؼ���ץ���ٶȻ�����
				last_high =  pidHeightHigh.measured = Control_high;  //ʵʱ�߶ȷ������⻷
			}	
	}
	//----------------------------------------------������ֹ����
	if(ALL_flag.unlock == EMERGENT) //�����������ʹ��ң�ؽ����������ɿؾͿ������κ�����½�����ֹ���У��������������˳�PID����
		status = EXIT_255;
	//----------------------------------------------����
	switch(status)
	{
		case WAITING_1: //��ⶨ��
		  if(ALL_flag.height_lock && ALL_flag.unlock) 
			{
				LED.status = DANGEROURS;
				status = WAITING_2;
				thr_hold=0; 				        //�����¼����ʱ������
			}
			break;
		case WAITING_2: //����ǰ׼��
			thr_hold = Remote.thr -1000;  //��¼����ʱ������
		  set_high = 0;
		  pidHeightHigh.desired = HIGH_START;  //�����߶� �趨ֵ
		
		  Flow_PosPid_y.desired = pixel_flow.loc_y;//��¼λ�� //ˢ��λ������
			Flow_PosPid_x.desired = pixel_flow.loc_x;//��¼λ��	//ˢ��λ������
		
 			status = PROCESS_31;
			break; 
		
		case PROCESS_31://���붨��	

//				if(((Remote.thr -1000) > (thr_hold+10)) ||((Remote.thr -1000) < (thr_hold-10))) //����ң��ֻ����ˮƽ�ٶȿ���
//				{			 																			  
//						if(High_breaktime++ >20)
//						{
//							thr_hold = Remote.thr -1000;  //��¼����ʱ������	
//							High_breaktime = 0;
//							set_high = 0;
//						}														
//				}
//				else
//				{}		 
//				if(set_high == 0) //������˳�����
//				{
//					set_high = 1;
//					pidHeightHigh.desired = Control_high;//��¼�߶ȵ�����ǰ���߸߶�		  
//				}
		
				pidUpdate(&pidHeightHigh,dt);    //����PID�������������⻷	������PID	
				pidHeightRate.desired = pidHeightHigh.out;  //�߶Ȼ�����߶��ٶ��趨ֵ
						 					 										 
				pidUpdate(&pidHeightRate,dt); //�ٵ��ø߶��ٶ��ڻ�
					 
	//		  pidHeightRate.out += Remote.thr -1000;//������ͣʱ������
					 
				if(!ALL_flag.height_lock)     //�˳�����
				{
					LED.status = AlwaysOn ;
					status = EXIT_255;
				}
			break;
		case EXIT_255: //�˳�����
			pidRest(&pPidObject[6],1);	//�����ǰ�Ķ������ֵ
			status = WAITING_1;//�ص��ȴ����붨��
			break;
		default:
			status = WAITING_1;
			break;	
	}	
				
}

void Mode_Controler(float dt)
{
		const float roll_pitch_ratio = 0.04f;
	
//		if(ALL_flag.unlock == 1)  //�жϽ���
		{
//			if(Remote.AUX2 < 1700)   //�������1700 ����붨��ģʽ��ʾ
//			{
//				Command.FlightMode = HEIGHT;
//				ALL_flag.height_lock = 1;
				Flow_mode_two();       // ң��-����������
//				set_flag=0x21;         // OLED���߶���ģʽ��ʾ
//			}
//			else                     //��̬ģʽ
//			{  
//				Command.FlightMode = NORMOL;	
//				ALL_flag.height_lock = 0;
//				set_flag=0x00;        //OLED��̬ģʽ��ʾ
//				
//				pidPitch.desired = -(Remote.pitch-1500)*roll_pitch_ratio;  //ҡ�˿���
//				pidRoll.desired  = -(Remote.roll-1500) *roll_pitch_ratio;  //ҡ�˿���
//			}
		}					
}

u32 altHold_Pos_Save = 0,Pos_breaktime = 0,Pos_break_save = 0;

///////////////////////// ң��-����������̬////////////////////////////////////////////////
void Flow_mode_two(void)
{

		const float roll_pitch_ratio = 8.00f;	

		if((mini.ok == 1) && (Control_high>5) && (Control_high<400))//�ж��Ƿ���ڹ���  �߶ȸ���5CM �����ſ��Զ���
		{
			Flow_SpeedPid_x.desired = (-(Remote.pitch-1500)*0.06f)*roll_pitch_ratio;   //ֱ�ӿ����ٶ� ���ҹص��⻷����
			Flow_SpeedPid_y.desired = (-(Remote.roll-1500)*0.06f)*roll_pitch_ratio;
			
			pidPitch.desired =LIMIT(Flow_SpeedPid_x.out*0.1,-15,15) ; //��̬�⻷����ֵ
			pidRoll.desired = LIMIT(Flow_SpeedPid_y.out*0.1,-15,15) ; //��̬�⻷����ֵ
			
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
//	    Remote.roll = 1700;        //��
//			Remote.pitch = 1600;	
//      move_X=0;	
//		  move_Y=1;			 
//	 }
//	 else
//	 if(SStime>6)	
//	 {
//	    Remote.roll = 1300;        //��
//			Remote.pitch = 1400;
//      move_X=0;	
//			move_Y=1;		 
//	 }
	 else
	 if(SStime>8)	
	 {
//	    Remote.roll = 1500; 		
//		  Remote.pitch = 1400;			 //��	
//		  move_X=1;
//			move_Y=0;	
		 
		        
		    Flow_PosPid_x.desired = -70;//��¼λ�� //ˢ��λ������
		    Flow_PosPid_y.desired = 40;//��¼λ�� //ˢ��λ������
	 }
	 else
	 if(SStime>5)	
	 {
//	    Remote.roll = 1500; 
//			Remote.pitch = 1600;			 //ǰ
//      move_X=1;	
//			move_Y=0;
        Flow_PosPid_x.desired = -70;//��¼λ�� //ˢ��λ������
		    Flow_PosPid_y.desired = 0;//��¼λ�� //ˢ��λ������
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
 * //λ�ö��������  ����
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void Flow_Pos_Controler(float dt)  //λ�ö��������  ����
{
		const uint16_t DEADBAND=150;	
  if((mini.ok == 1) && (Control_high>5) && (Control_high<400))//�ж��Ƿ���ڹ���  �߶ȸ���20CM �����ſ��Զ���
	{	

//		   //����ҡ�˻���
//			if((Remote.pitch>(1500+DEADBAND))||(Remote.pitch<(1500-DEADBAND))||(Remote.roll>(1500+DEADBAND))||(Remote.roll<(1500-DEADBAND))) //����ң��ֻ����ˮƽ�ٶȿ���
//			{				
//					mini.flow_x_i = 0;    			//�����������
//					mini.flow_y_i = 0;					//�����������
//					altHold_Pos_Save = 1;  			//��¼λ�ñ�־���´�
//				  Pos_break_save = 0;
//			}
//			else	//����ҡ�˵�ʱ��
			{				
//				  if(altHold_Pos_Save == 1)   //��¼λ�� һ��
//					{
//						altHold_Pos_Save = 0; 		//�رռ�¼λ�ñ�־
//						Pos_break_save = 1;       //�򿪲���ɲ�������־
//						
//								
//					}
					
					if(move_Y==0)//�⻷λ�ÿ���
					{	
						Flow_PosPid_y.measured = pixel_flow.loc_y;//ʵʱλ�÷���
						pidUpdate(&Flow_PosPid_y,dt);//λ������PID
						Flow_SpeedPid_y.desired = LIMIT(Flow_PosPid_y.out,-1000,1000);//λ��PID������ٶ�����
					}
					else
					{
//					  Flow_PosPid_y.desired = pixel_flow.loc_y;//��¼λ�� //ˢ��λ������
					}
					
					if(move_X==0)//�⻷λ�ÿ���
					{
						Flow_PosPid_x.measured = pixel_flow.loc_x;//ʵʱλ�÷���
						pidUpdate(&Flow_PosPid_x,dt);//λ������PID
						Flow_SpeedPid_x.desired = LIMIT(Flow_PosPid_x.out,-1000,1000);//λ��PID������ٶ�����
					}
					else
					{
//					   Flow_PosPid_x.desired = pixel_flow.loc_x;//��¼λ��	//ˢ��λ������
					}
					
					//�ڻ�����  
					
					
			}
						
				//�ڻ�                                                                                                
				Flow_SpeedPid_y.measured = pixel_flow.loc_y;//�ٶȷ���
				pidUpdate(&Flow_SpeedPid_y,dt);//�ٶ�����
			
			
				Flow_SpeedPid_x.measured = pixel_flow.loc_x;//�ٶȷ���
				pidUpdate(&Flow_SpeedPid_x,dt);//�ٶ�����
					
	}
	else
	{
					mini.flow_x_i = 0;				//������λ
					mini.flow_y_i = 0;				//������λ
					Flow_SpeedPid_x.out = 0;	//��������
					Flow_SpeedPid_y.out = 0;	//��������
	}

	
}

/**************************************************************
 * ��̬����
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void FlightPidControl(float dt)
{
	volatile static uint8_t status=WAITING_1;

	switch(status)
	{		
		case WAITING_1: //�ȴ�����
			if(ALL_flag.unlock)
			{
				status = READY_11;	
			}			
			break;
		case READY_11:  //׼���������
			pidRest(pPidObject,8); //������λPID���ݣ���ֹ�ϴ�����������Ӱ�챾�ο���

			Angle.yaw = pidYaw.desired =  pidYaw.measured = 0;   //����ƫ����
		
			status = PROCESS_31;
		
			break;			
		case PROCESS_31: //��ʽ�������
			
      pidRateX.measured = MPU6050.gyroX * Gyro_G; //�ڻ�����ֵ �Ƕ�/��
			pidRateY.measured = MPU6050.gyroY * Gyro_G; //�ڻ�����ֵ �Ƕ�/��
			pidRateZ.measured = MPU6050.gyroZ * Gyro_G; //�ڻ�����ֵ �Ƕ�/��
		
			pidPitch.measured = Angle.pitch; 		//�⻷����ֵ ��λ���Ƕ�
		  pidRoll.measured = Angle.roll;			//�⻷����ֵ ��λ���Ƕ�
			pidYaw.measured = Angle.yaw;				//�⻷����ֵ ��λ���Ƕ�
		
		 	pidUpdate(&pidRoll,dt);    //����PID�������������⻷	�����PID		
			pidRateX.desired = pidRoll.out; //���⻷��PID�����Ϊ�ڻ�PID������ֵ��Ϊ����PID
			pidUpdate(&pidRateX,dt);  //�ٵ����ڻ�

		 	pidUpdate(&pidPitch,dt);    //����PID�������������⻷	������PID	
			pidRateY.desired = pidPitch.out;  
			pidUpdate(&pidRateY,dt); //�ٵ����ڻ�

			CascadePID(&pidRateZ,&pidYaw,dt);	//Ҳ����ֱ�ӵ��ô���PID����������
			break;
		case EXIT_255:  						//�˳�����
			pidRest(pPidObject,8);		//��λPID����
			status = WAITING_1;				//���صȴ�����
		  break;
		default:
			status = EXIT_255;
			break;
	}
	if(ALL_flag.unlock == EMERGENT) //�����������ʹ��ң�ؽ����������ɿؾͿ������κ�����½�����ֹ���У��������������˳�PID����
		status = EXIT_255;
}


int16_t motor[4];
#define MOTOR1 motor[0] 
#define MOTOR2 motor[1] 
#define MOTOR3 motor[2] 
#define MOTOR4 motor[3] 

void MotorControl(void) //�������
{	
	volatile static uint8_t status=WAITING_1;
	
	
	if(ALL_flag.unlock == EMERGENT) //�����������ʹ��ң�ؽ����������ɿؾͿ������κ�����½�����ֹ���У��������������˳�PID����
		status = EXIT_255;	
	switch(status)
	{		
		case WAITING_1: 	     //�ȴ�����	
			MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;  //������������������Ϊ0
			if(ALL_flag.unlock)
			{
				status = WAITING_2;
			}
		case WAITING_2: //������ɺ��ж�ʹ�����Ƿ�ʼ����ң�˽��з��п���
			if(Remote.thr>1100)
			{
				status = PROCESS_31;
			}
			break;
		case PROCESS_31:
			{
				int16_t thr_temp;
								
				if(ALL_flag.height_lock) //����ģʽ�� ����ң����Ϊ�����߶�ʹ��   
				{		
					thr_temp = pidHeightRate.out + (Remote.thr -1000); //�����������Ƕ������ֵ
				}
				else 										 //��������״̬����������ʹ��
				{
					thr_temp = Remote.thr -1000; //�������������������ֵ
				}
				
				if(Remote.thr<1020)		//����̫���ˣ����������  ��Ȼ�ɻ���ת												
				{
					MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4=0;
					break;
				}
				MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = LIMIT(thr_temp,0,900); //��100����̬����

				MOTOR1 +=    + pidRateX.out - pidRateY.out - pidRateZ.out;//; ��̬����������������Ŀ�����
				MOTOR2 +=    + pidRateX.out + pidRateY.out + pidRateZ.out ;//;
				MOTOR3 +=    - pidRateX.out + pidRateY.out - pidRateZ.out;
				MOTOR4 +=    - pidRateX.out - pidRateY.out + pidRateZ.out;//;
			}	
			break;
		case EXIT_255:
			MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;  //������������������Ϊ0
			status = WAITING_1;	
			break;
		default:
			break;
	}
	
	
//	TIM2->CCR1 = LIMIT(MOTOR1,0,1000);  //����PWM1
//	TIM2->CCR2 = LIMIT(MOTOR2,0,1000);  //����PWM2
//	TIM3->CCR1 = LIMIT(MOTOR3,0,1000);  //����PWM3
//	TIM3->CCR2 = LIMIT(MOTOR4,0,1000);  //����PWM4
////	TIM2->CCR3 = LIMIT(MOTOR3,0,1000);  //����PWM3
////	TIM2->CCR4 = LIMIT(MOTOR4,0,1000);  //����PWM4
	
	#if (FLY_TYPE == 1 || FLY_TYPE == 2)
	
	PWM0 = LIMIT(MOTOR1,0,1000);  //����PWM1
	PWM1 = LIMIT(MOTOR2,0,1000);  //����PWM2
	PWM2 = LIMIT(MOTOR3,0,1000);  //����PWM3
	PWM3 = LIMIT(MOTOR4,0,1000);  //����PWM4
	
#elif (FLY_TYPE >= 3)
	
	PWM0 = 1000 + LIMIT(MOTOR1,0,1000);  //����PWM1
	PWM1 = 1000 + LIMIT(MOTOR2,0,1000);  //����PWM2
	PWM2 = 1000 + LIMIT(MOTOR3,0,1000);  //����PWM3] ;     
	PWM3 = 1000 + LIMIT(MOTOR4,0,1000);  //����PWM4
	
#else
	#error Please define FLY_TYPE!
		
#endif

} 


/************************************END OF FILE********************************************/ 



