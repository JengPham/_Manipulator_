#include "Step_motor.h"
#include <math.h>
#include "Analog.h"
#include "main.h"
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim6;
Motion mProfile[6];
Stepper StepVar[6];

void delay_us(uint16_t cnt){
	// max timer = 1ms
	if(cnt > 1000)
		cnt = 1000;
	__HAL_TIM_SetCounter (&htim6,0);
	while(__HAL_TIM_GetCounter(&htim6) < cnt);
	if(__HAL_TIM_GET_FLAG(&htim6,TIM_FLAG_UPDATE) ==1)
		   __HAL_TIM_CLEAR_FLAG(&htim6,TIM_FLAG_UPDATE);
}
float CurrentSet(float Iset)
{
	//Sensitive = 2.5V/1A, Imax = 1.2A, Imin = 0A
	static float Sensitive = 1.375;	//V/1A
	if(Iset < 0)	
		return 0.0;
	else if(Iset > 2.4)
		return 3.3; 
	else
		return Iset*Sensitive; 
}

void TB67S_Mode(uint8_t Mode)
{
	switch(Mode){
		case Standby_mode:
		{
			    HAL_GPIO_WritePin(StpMode_PORT,  StpMode2_PIN,GPIO_PIN_RESET) ;
					HAL_GPIO_WritePin(StpMode_PORT,  StpMode1_PIN, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(StpMode_PORT, StpMode0_PIN, GPIO_PIN_RESET);
					break;
		}			
		case Full:
		{
			    HAL_GPIO_WritePin(StpMode_PORT, StpMode2_PIN,GPIO_PIN_SET) ;
					HAL_GPIO_WritePin(StpMode_PORT, StpMode1_PIN, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(StpMode_PORT, StpMode0_PIN, GPIO_PIN_RESET);
					break;
		}					
		case Half_A:
				{
					HAL_GPIO_WritePin(StpMode_PORT, StpMode2_PIN, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(StpMode_PORT,  StpMode1_PIN, GPIO_PIN_SET);
					HAL_GPIO_WritePin(StpMode_PORT, StpMode0_PIN, GPIO_PIN_RESET);
					break;
				}
	  case Half_B:
				{
					HAL_GPIO_WritePin(StpMode_PORT, StpMode2_PIN, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(StpMode_PORT,  StpMode1_PIN, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(StpMode_PORT, StpMode0_PIN, GPIO_PIN_SET);
					break;
				}
			case Micro_1_4:
				{
					HAL_GPIO_WritePin(StpMode_PORT, StpMode2_PIN, GPIO_PIN_SET);
					HAL_GPIO_WritePin(StpMode_PORT, StpMode1_PIN, GPIO_PIN_SET);
					HAL_GPIO_WritePin(StpMode_PORT,  StpMode0_PIN, GPIO_PIN_RESET);
					break;
				}	
			case Micro_1_8:
				{
					HAL_GPIO_WritePin(StpMode_PORT, StpMode2_PIN, GPIO_PIN_SET);
					HAL_GPIO_WritePin(StpMode_PORT,  StpMode1_PIN, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(StpMode_PORT, StpMode0_PIN, GPIO_PIN_SET);
					break;
				}	
			case Micro_1_16:
				{
					HAL_GPIO_WritePin(StpMode_PORT, StpMode2_PIN, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(StpMode_PORT,  StpMode1_PIN, GPIO_PIN_SET);
					HAL_GPIO_WritePin(StpMode_PORT,  StpMode0_PIN, GPIO_PIN_SET);
					break;
				}	
			case Micro_1_32:
				{
					HAL_GPIO_WritePin(StpMode_PORT, StpMode2_PIN, GPIO_PIN_SET);
					HAL_GPIO_WritePin(StpMode_PORT,  StpMode1_PIN, GPIO_PIN_SET);
					HAL_GPIO_WritePin(StpMode_PORT, StpMode0_PIN, GPIO_PIN_SET);
					break;
				}	
	}
}

void TB67S_Enable(uint8_t Motor_id)
{  
	if(Motor_id == M1)
	HAL_GPIO_WritePin(En1_PORT  , En1_PIN, GPIO_PIN_SET);
	if(Motor_id == M2)
	HAL_GPIO_WritePin(En2_PORT  , En2_PIN, GPIO_PIN_SET);
	if(Motor_id == M3)
	HAL_GPIO_WritePin(En3_PORT  , En3_PIN, GPIO_PIN_SET);
	if(Motor_id == M4)
	HAL_GPIO_WritePin(En4_PORT  , En4_PIN, GPIO_PIN_SET);
	if(Motor_id == M5)
	HAL_GPIO_WritePin(En5_PORT  , En5_PIN, GPIO_PIN_SET);
	if(Motor_id == M6)
	HAL_GPIO_WritePin(En6_PORT  , En6_PIN, GPIO_PIN_SET);
}

void TB67S_Disable(uint8_t Motor_id)
{ 
	if(Motor_id == M1)
	HAL_GPIO_WritePin(En1_PORT  , En1_PIN, GPIO_PIN_RESET);
	if(Motor_id == M2)
	HAL_GPIO_WritePin(En2_PORT  , En2_PIN, GPIO_PIN_RESET);
	if(Motor_id == M3)
	HAL_GPIO_WritePin(En3_PORT  , En3_PIN, GPIO_PIN_RESET);
	if(Motor_id == M4)
	HAL_GPIO_WritePin(En4_PORT  , En4_PIN, GPIO_PIN_RESET);
	if(Motor_id == M5)
	HAL_GPIO_WritePin(En5_PORT  , En5_PIN, GPIO_PIN_RESET);
	if(Motor_id == M6)
	HAL_GPIO_WritePin(En6_PORT  , En6_PIN, GPIO_PIN_RESET);
}

void TB67S_Reset(uint8_t Motor_id)	//ID = 1, 2, 3, 4, 5, 6
{     
	if(Motor_id ==M1)
	{
			HAL_GPIO_WritePin(Rst1_PORT, Rst1_PIN, GPIO_PIN_SET);
	    delay_us(10);
			HAL_GPIO_WritePin(Rst1_PORT, Rst1_PIN, GPIO_PIN_RESET);
	}
	if(Motor_id ==M2)
	{
			HAL_GPIO_WritePin(Rst2_PORT, Rst2_PIN, GPIO_PIN_SET);
	    delay_us(10);
			HAL_GPIO_WritePin(Rst2_PORT, Rst2_PIN, GPIO_PIN_RESET);
	}
	if(Motor_id ==M3)
	{
			HAL_GPIO_WritePin(Rst3_PORT, Rst3_PIN, GPIO_PIN_SET);
	    delay_us(10);
			HAL_GPIO_WritePin(Rst3_PORT, Rst3_PIN, GPIO_PIN_RESET);
	}
	if(Motor_id ==M4)
	{
			HAL_GPIO_WritePin(Rst4_PORT, Rst4_PIN, GPIO_PIN_SET);
	    delay_us(10);
			HAL_GPIO_WritePin(Rst4_PORT, Rst4_PIN, GPIO_PIN_RESET);
	}
	if(Motor_id ==M5)
	{
			HAL_GPIO_WritePin(Rst5_PORT, Rst5_PIN, GPIO_PIN_SET);
	    delay_us(10);
			HAL_GPIO_WritePin(Rst5_PORT, Rst5_PIN, GPIO_PIN_RESET);
	}
	if(Motor_id ==M6)
	{
			HAL_GPIO_WritePin(Rst6_PORT, Rst6_PIN, GPIO_PIN_SET);
	    delay_us(10);
			HAL_GPIO_WritePin(Rst6_PORT, Rst6_PIN, GPIO_PIN_RESET);
	}
}
void TB67S_Dir_fwd(uint8_t Motor_id)
{ 
	if(Motor_id == M1){
	HAL_GPIO_WritePin(Dir1_PORT , Dir1_PIN , GPIO_PIN_SET);}
	if(Motor_id == M2){
	HAL_GPIO_WritePin(Dir2_PORT , Dir2_PIN , GPIO_PIN_SET);}
	if(Motor_id == M3){
	HAL_GPIO_WritePin(Dir3_PORT , Dir3_PIN , GPIO_PIN_SET);}
	if(Motor_id == M4){
	HAL_GPIO_WritePin(Dir4_PORT , Dir4_PIN , GPIO_PIN_SET);}
	if(Motor_id == M5){
	HAL_GPIO_WritePin(Dir5_PORT , Dir5_PIN , GPIO_PIN_SET);}
	if(Motor_id == M6){
	HAL_GPIO_WritePin(Dir6_PORT , Dir6_PIN , GPIO_PIN_SET);}
}

void TB67S_Dir_rev(uint8_t Motor_id){
	 if(Motor_id == M1){
	HAL_GPIO_WritePin(Dir1_PORT , Dir1_PIN , GPIO_PIN_RESET);}
	if(Motor_id == M2){
	HAL_GPIO_WritePin(Dir2_PORT , Dir2_PIN , GPIO_PIN_RESET);}
	if(Motor_id == M3){
	HAL_GPIO_WritePin(Dir3_PORT , Dir3_PIN , GPIO_PIN_RESET);}
	if(Motor_id == M4){
	HAL_GPIO_WritePin(Dir4_PORT , Dir4_PIN , GPIO_PIN_RESET);}
	if(Motor_id == M5){
	HAL_GPIO_WritePin(Dir5_PORT , Dir5_PIN , GPIO_PIN_RESET);}
	if(Motor_id == M6){
	HAL_GPIO_WritePin(Dir6_PORT , Dir6_PIN , GPIO_PIN_RESET);}
}

void step(uint8_t Motor_id)
{  
	if(Motor_id == M1){
	HAL_GPIO_WritePin(Pul1_PORT,Pul1_PIN,GPIO_PIN_SET);
	delay_us(1);
	HAL_GPIO_WritePin(Pul1_PORT,Pul1_PIN,GPIO_PIN_RESET);
	}
	if(Motor_id == M2){
	HAL_GPIO_WritePin(Pul2_PORT,Pul2_PIN,GPIO_PIN_SET);
	delay_us(1);
	HAL_GPIO_WritePin(Pul2_PORT,Pul2_PIN,GPIO_PIN_RESET);
	}
	if(Motor_id == M3){
	HAL_GPIO_WritePin(Pul3_PORT,Pul3_PIN,GPIO_PIN_SET);
	delay_us(1);
	HAL_GPIO_WritePin(Pul3_PORT,Pul3_PIN,GPIO_PIN_RESET);
	}
	if(Motor_id == M4){
	HAL_GPIO_WritePin(Pul4_PORT,Pul4_PIN,GPIO_PIN_SET);
	delay_us(1);
	HAL_GPIO_WritePin(Pul4_PORT,Pul4_PIN,GPIO_PIN_RESET);
	}
	if(Motor_id == M5){
	HAL_GPIO_WritePin(Pul5_PORT,Pul5_PIN,GPIO_PIN_SET);
	delay_us(1);
	HAL_GPIO_WritePin(Pul5_PORT,Pul5_PIN,GPIO_PIN_RESET);
	}
	if(Motor_id == M6){
	HAL_GPIO_WritePin(Pul6_PORT,Pul6_PIN,GPIO_PIN_SET);
	delay_us(1);
	HAL_GPIO_WritePin(Pul6_PORT,Pul6_PIN,GPIO_PIN_RESET);
	}
}
void Stepper_Init(float StepAngle_Deg, uint8_t stpMode, uint8_t CtrlMode, float GearRatio,int Motor_id)
{
	TB67S_Disable(Motor_id);
	StepVar[Motor_id].GearRatio = GearRatio;     								// Ti so truyen 1/GearRatio
	StepVar[Motor_id].StepMode = stpMode;
	StepVar[Motor_id].BaseStepAngle_Degree = StepAngle_Deg;  //Step angle of the motor, which can be seen in the name plate
	TB67S_Reset(Motor_id);
			switch (StepVar[Motor_id].StepMode)
	{
			case Full:
					StepVar[Motor_id].RealStepAngle_Rad = (StepVar[Motor_id].BaseStepAngle_Degree)*(PI/180);  			
					TB67S_Mode(Full);
					break;
			case Half_A:
					StepVar[Motor_id].RealStepAngle_Rad  = (StepVar[Motor_id].BaseStepAngle_Degree*0.5)*(PI/180); 
					TB67S_Mode(Half_A);
					break;
			case Micro_1_4:
					StepVar[Motor_id].RealStepAngle_Rad  = (StepVar[Motor_id].BaseStepAngle_Degree*0.25)*(PI/180); 
					TB67S_Mode(Micro_1_4);
					break;
			case Micro_1_8:
					StepVar[Motor_id].RealStepAngle_Rad  = (StepVar[Motor_id].BaseStepAngle_Degree*0.125)*(PI/180); 
					TB67S_Mode(Micro_1_8);
					break;
			case Micro_1_16:
					StepVar[Motor_id].RealStepAngle_Rad  = (StepVar[Motor_id].BaseStepAngle_Degree*0.0625)*(PI/180); 
					TB67S_Mode(Micro_1_16);
					break;
			case Micro_1_32:
					StepVar[Motor_id].RealStepAngle_Rad  = (StepVar[Motor_id].BaseStepAngle_Degree*0.03125)*(PI/180); 
					TB67S_Mode(Micro_1_32);
					break;
	}
	/*Compute real pulse per round*/
	StepVar[Motor_id].ppr = (2*PI*StepVar[Motor_id].GearRatio)/StepVar[Motor_id].RealStepAngle_Rad;  
	//Control mode: mPOSITION or mSPEED
	StepVar[Motor_id].CtrlMode = CtrlMode;
  StepVar[Motor_id].count = 0;	
	// Disable stepper motor driver		
	vDAC_Write(Motor_id+1,CurrentSet(0),0.0,3.3);
		}

void Stepper_LockRotor(uint8_t mID)
{  
vDAC_Write(mID+1,CurrentSet(mProfile[mID].holdI),0.0,3.3);
	TB67S_Enable(mID);
}
void Stepper_FreeRotor(uint8_t mID)
{
	vDAC_Write((mID+1),CurrentSet(0),0.0,3.3);
	TB67S_Disable(mID);
}

void MotionProfile_Set(uint8_t mID, float Accel_Radps2, float Decel_Radps2, float maxRPM, float SetAngle_Deg)
{  
switch(mID){
		case M1: {
	mProfile[mID].setI = 1.5;
	mProfile[mID].holdI = 2;
			break;
		}
		case M2: {
	mProfile[mID].setI = 2.4;
	mProfile[mID].holdI = 2.4;
			break;
		}
		case M3: {
	mProfile[mID].setI = 1.5;
	mProfile[mID].holdI = 1.5;
			break;
		}
		case M4: {
	mProfile[mID].setI = 0.6;
	mProfile[mID].holdI = 1;
			break;
		}
		case M5: {
	mProfile[mID].setI = 1.5;
	mProfile[mID].holdI = 1;
			break;
		}
		case M6: {
	mProfile[mID].setI = 1.5;
	mProfile[mID].holdI = 1.5;
			break;
		}
	}
	
	if(SetAngle_Deg < 0)
	{
		SetAngle_Deg = - SetAngle_Deg;
		TB67S_Dir_rev(mID);
	}
	else 
	{
		TB67S_Dir_fwd(mID);
	}
	/*Compute total pulse per set angle*/
	mProfile[mID].Total_cnt=(SetAngle_Deg/360)*StepVar[mID].ppr; 
	/*Built the speed profile*/
	mProfile[mID].Accel_Radps2 = Accel_Radps2;
	mProfile[mID].Decel_Radps2 = Decel_Radps2;
	mProfile[mID].SetAngle_Deg = SetAngle_Deg;
	mProfile[mID].maxSpeed_RadpSec = maxRPM/9.492966; 	//rpm to rad/s		
	mProfile[mID].max_s_lim=(mProfile[mID].maxSpeed_RadpSec*mProfile[mID].maxSpeed_RadpSec)/(2*StepVar[mID].RealStepAngle_Rad*Accel_Radps2);
	mProfile[mID].accel_lim=(Decel_Radps2*mProfile[mID].Total_cnt)/(Accel_Radps2+Decel_Radps2);
	if(mProfile[mID].max_s_lim<=mProfile[mID].accel_lim) 																									//accel_run_deccel
	{
				mProfile[mID].decel_set_cnt = -(float)(mProfile[mID].max_s_lim*Accel_Radps2)/Decel_Radps2;
				mProfile[mID].run_set_cnt =(mProfile[mID].Total_cnt-(fabs)((double)(mProfile[mID].decel_set_cnt))-mProfile[mID].max_s_lim);
				mProfile[mID].accel_set_cnt = mProfile[mID].max_s_lim;
	}
	else
	{
				mProfile[mID].decel_set_cnt=-(mProfile[mID].Total_cnt-mProfile[mID].accel_lim);
				mProfile[mID].run_set_cnt=0;
				mProfile[mID].accel_set_cnt=mProfile[mID].accel_lim;
	}
	mProfile[mID].decel_cnt=mProfile[mID].decel_set_cnt;
}

void MotionVar_Prepare(uint8_t mID)
{
	/*Compute the first value which is loaded to the Auto Reload Register*/
	mProfile[mID].Cn=(f_Timer*sqrt(2*StepVar[mID].RealStepAngle_Rad/mProfile[mID].Accel_Radps2)*676); /*f_timer=500kHz*/
	mProfile[mID].run_cnt=0;
	mProfile[mID].accel_cnt=0;
	mProfile[mID].Cn_1=mProfile[mID].Cn;
	mProfile[mID].rest=0;
}

void Stepper_StateRst(uint8_t mID)
{
	StepVar[mID].move=0; 
	StepVar[mID].cmd =0;
	StepVar[mID].flag=0;
}

void Stepper_Start(uint8_t mID,TIM_HandleTypeDef *htim ) 
{ 
	StepVar[mID].mProfileID = mID;
	/*Set time delay for linear stepper motor control algorithm*/
	if(mProfile[mID].Cn > AutReloadReg_Max)
	{
		__HAL_TIM_SET_COUNTER(htim, 0);
		__HAL_TIM_SET_AUTORELOAD(htim, AutReloadReg_Max);
	}
	else
	{
		__HAL_TIM_SET_COUNTER(htim, 0);
		__HAL_TIM_SET_AUTORELOAD(htim, mProfile[mID].Cn);
	}
	//Set current and Enable the Driver
	vDAC_Write(mID+1,CurrentSet(mProfile[mID].setI),0.0,3.3);
	TB67S_Enable(mID);
	/*Generate pulse and switch stepper motor state to STEPPER_Run*/
	step(mID);
	StepVar[mID].state = stRUN; /*Trang thai*/
	StepVar[mID].cmd = 1;
	StepVar[mID].PosFlag = 0;
}

void Stepper_Stop(uint8_t mID)
{	
	if(StepVar[mID].move !=0)       //Stepper motor is running
	{
			StepVar[mID].move = -1;     //Decelerate and stop
			StepVar[mID].cmd = 0;
	}
}
void reset(uint8_t mID){
	mProfile[mID].Accel_Radps2 = 0;
	mProfile[mID].Decel_Radps2 = 0;
	mProfile[mID].SetAngle_Deg = 0;
	mProfile[mID].maxSpeed_RadpSec = 0; 	//rpm to rad/s		
	mProfile[mID].max_s_lim=0;
	mProfile[mID].accel_lim=0;
	mProfile[mID].accel_cnt = 0;
	mProfile[mID].accel_set_cnt = 0;
	mProfile[mID].Cn = 0;
	mProfile[mID].Cn_1 = 0;
	mProfile[mID].decel_cnt = 0;
	mProfile[mID].decel_set_cnt = 0;
	mProfile[mID].max_s_lim = 0;
	mProfile[mID].run_cnt = 0;
	mProfile[mID].run_set_cnt = 0;
	mProfile[mID].Total_cnt = 0;
	mProfile[mID].Crun = 0;
}

void Stepper_Control(uint8_t mID,TIM_HandleTypeDef *htim)	//Called in Timer ISR - Only if State = STEPPER_Run
{  	
	step(mID);
	switch(StepVar[mID].CtrlMode)
	{
			case mPOSITION:
			{
					if(StepVar[mID].cmd == 1)
					{
							if(++mProfile[mID].accel_cnt<mProfile[mID].accel_set_cnt)         //Keep accelerating
							{
								StepVar[mID].move=2;
							}
							else
							{
									mProfile[mID].accel_cnt=mProfile[mID].accel_set_cnt;          //Keep accel_count constant
									if(mProfile[mID].run_set_cnt>0)                               //2-1-2 with V = const 
									{
											 if(++mProfile[mID].run_cnt<mProfile[mID].run_set_cnt)
											 {
														StepVar[mID].move = 1;                                	//Run with v = constant
														mProfile[mID].Crun = mProfile[mID].Cn;										
											 }
											 else                                                     //Decelerate
											 {
														mProfile[mID].run_cnt = mProfile[mID].run_set_cnt;
														StepVar[mID].move = -1;
											 }
									}
									else    //Decelerate
											 StepVar[mID].move = -1;
							}
					}
					else
							StepVar[mID].move = -1;              
					break;
			}
			case mSPEED:
			{
					if(StepVar[mID].cmd == 1)
					{
							if(++mProfile[mID].accel_cnt<mProfile[mID].accel_set_cnt)     //Keep accelerating
							{
								StepVar[mID].move=2;
							
							}
							else
							{
									mProfile[mID].accel_cnt=mProfile[mID].accel_set_cnt;      //Keep accel_count constant
									StepVar[mID].move = 1;                                      	//Run with constant speed
									mProfile[mID].Crun = mProfile[mID].Cn;
							}
					}
					else
							StepVar[mID].move = -1;                                         //Decelerate and stop
					break;
			}
	}
	switch(StepVar[mID].move)
	{
				case -1: 	//Decel
				{
						 if(++mProfile[mID].decel_cnt < 0)
						 {
							    mProfile[mID].Cn_1 = mProfile[mID].Cn; 				
									mProfile[mID].Cn=mProfile[mID].Cn_1-(2*mProfile[mID].Cn_1+mProfile[mID].rest)/(4*mProfile[mID].decel_cnt+1);
							    mProfile[mID].rest=(2*mProfile[mID].Cn_1+mProfile[mID].rest)%(4*mProfile[mID].decel_cnt+1); 							 		
	                 
									if(mProfile[mID].Cn > AutReloadReg_Max)
									{  	
										__HAL_TIM_SET_AUTORELOAD(htim, AutReloadReg_Max);
									}
									
									else
									{
	                 __HAL_TIM_SET_AUTORELOAD(htim, mProfile[mID].Cn);//AutReloadReg_Max  
									
									}
						 }
						 else
						 {    					   
									StepVar[mID].move = 0;
									StepVar[mID].state = stLOCKROTOR;		
									vDAC_Write((mID+1),mProfile[mID].setI,0.0,3.3);
									StepVar[mID].cmd = 0;
							    StepVar[mID].count = StepVar[mID].count+1;
									if(StepVar[mID].CtrlMode == mPOSITION)
									{ 
									 StepVar[mID].PosFlag = 1; 									//Reach the set position
									}
               	__HAL_TIM_SET_AUTORELOAD(htim,49999);    //49999
						 break;
				}
				case 1: 	//Run
				{
					if(mProfile[mID].Cn > AutReloadReg_Max)
					{
            	__HAL_TIM_SET_AUTORELOAD(htim, AutReloadReg_Max); 
					}
					else
					{
								__HAL_TIM_SET_AUTORELOAD(htim,mProfile[mID].Cn);
					}
					break;
				}
				case 2: 	//Accel
				{  
					 mProfile[mID].Cn_1=mProfile[mID].Cn;
					 mProfile[mID].Cn=mProfile[mID].Cn_1-(2*mProfile[mID].Cn_1+mProfile[mID].rest)/(4*mProfile[mID].accel_cnt+1);
					 mProfile[mID].rest=(2*mProfile[mID].Cn_1+mProfile[mID].rest)%(4*mProfile[mID].accel_cnt+1);
					 if(mProfile[mID].Cn > AutReloadReg_Max)
						{
            __HAL_TIM_SET_AUTORELOAD(htim, AutReloadReg_Max);
							
						}
						else
						{
								__HAL_TIM_SET_AUTORELOAD(htim,mProfile[mID].Cn);
						}
						
						break;
				}
	 }
	 
}
	}
void Motor_run(uint8_t mID)
{  
	switch(mID){
		case M1:
			Stepper_StateRst(mID);
			MotionVar_Prepare(mID);
			Stepper_Start(mID,&htim2);
		break;
		case M2:
			Stepper_StateRst(mID);
			MotionVar_Prepare(mID);
			Stepper_Start(mID,&htim7);
		break;
		case M3:
			Stepper_StateRst(mID);
			MotionVar_Prepare(mID);
			Stepper_Start(mID,&htim10);
		break;
		case M4:
			Stepper_StateRst(mID);
			MotionVar_Prepare(mID);
			Stepper_Start(mID,&htim11);
		break;
		case M5:
			Stepper_StateRst(mID);
			MotionVar_Prepare(mID);
			Stepper_Start(mID,&htim13);
		break;
		case M6:
			Stepper_StateRst(mID);
			MotionVar_Prepare(mID);
			Stepper_Start(mID,&htim14);
		break;
	
	}
	
}