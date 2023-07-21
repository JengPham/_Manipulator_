#include "Stepper.h"
#include <math.h>
#include "Analog.h"
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim5;
extern Motion mProfile[10];
extern Stepper StepVar;

/********************************/
void delay_us(uint16_t cnt){
	// max timer = 1ms
	if(cnt > 1000)
		cnt = 1000;
	__HAL_TIM_SetCounter (&htim7,0);
	while(__HAL_TIM_GetCounter(&htim7) < cnt);
	if(__HAL_TIM_GET_FLAG(&htim7,TIM_FLAG_UPDATE) ==1)
		   __HAL_TIM_CLEAR_FLAG(&htim7,TIM_FLAG_UPDATE);
}
/************************************************/
/*Input is the set current(A), output is the corresponding voltage sent to DAC*/
float CurrentSet(float Iset)
{
	//Sensitive = 2.5V/1A, Imax = 1.2A, Imin = 0A
	static float Sensitive = 2.5;	//V/1A
	if(Iset < 0)	
		return 0.0;
	else if(Iset > 1.3)
		return 1.3*Sensitive; 
	else
		return Iset*Sensitive; 
}

/********************************/
// set mode for step motor
void TB67S_Mode(uint8_t Mode)
{
	switch(Mode){
		case Standby_mode:
		{
			    HAL_GPIO_WritePin(M2_GPIO_Port, M2_Pin,GPIO_PIN_RESET) ;
					HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(M0_GPIO_Port, M0_Pin, GPIO_PIN_RESET);
					break;
		}			
		case Full:
		{
			    HAL_GPIO_WritePin(M2_GPIO_Port, M2_Pin,GPIO_PIN_SET) ;
					HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(M0_GPIO_Port, M0_Pin, GPIO_PIN_RESET);
					break;
		}					
		case Half_A:
				{
					HAL_GPIO_WritePin(M2_GPIO_Port,M2_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(M0_GPIO_Port, M0_Pin, GPIO_PIN_RESET);
					break;
				}
	  case Half_B:
				{
					HAL_GPIO_WritePin(M2_GPIO_Port,M2_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(M0_GPIO_Port, M0_Pin, GPIO_PIN_SET);
					break;
				}
			case Micro_1_4:
				{
					HAL_GPIO_WritePin(M2_GPIO_Port,M2_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(M0_GPIO_Port, M0_Pin, GPIO_PIN_RESET);
					break;
				}	
			case Micro_1_8:
				{
					HAL_GPIO_WritePin(M2_GPIO_Port,M2_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(M0_GPIO_Port, M0_Pin, GPIO_PIN_SET);
					break;
				}	
			case Micro_1_16:
				{
					HAL_GPIO_WritePin(M2_GPIO_Port,M2_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(M0_GPIO_Port, M0_Pin, GPIO_PIN_SET);
					break;
				}	
			case Micro_1_32:
				{
					HAL_GPIO_WritePin(M2_GPIO_Port,M2_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(M1_GPIO_Port, M1_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(M0_GPIO_Port, M0_Pin, GPIO_PIN_SET);
					break;
				}	
	}
}

/*********************************************************/
void TB67S_Enable(void)
{
	HAL_GPIO_WritePin(En_GPIO_Port, En_Pin, GPIO_PIN_SET);
}
/*********************************************************/
void TB67S_Disable(void)
{
	HAL_GPIO_WritePin(En_GPIO_Port, En_Pin, GPIO_PIN_RESET);
}
/********************************************************/
void TB67S_Reset(void)	//ID = 1, 2, 3
{     
			HAL_GPIO_WritePin(Reset_GPIO_Port, Reset_Pin, GPIO_PIN_SET);
	    delay_us(10);
			HAL_GPIO_WritePin(Reset_GPIO_Port, Reset_Pin, GPIO_PIN_RESET);
}
/********************************************************/
void TB67S_Dir_fwd(void)
{
	HAL_GPIO_WritePin(Dir_GPIO_Port, Dir_Pin, GPIO_PIN_SET);
}
/*******************************************************/
void TB67S_Dir_rev(void){
	 HAL_GPIO_WritePin(Dir_GPIO_Port, Dir_Pin, GPIO_PIN_RESET);
}
/******************************************************/
void step(void)
{
	HAL_GPIO_WritePin(Pulse_GPIO_Port,Pulse_Pin,GPIO_PIN_SET);
	delay_us(2);
	HAL_GPIO_WritePin(Pulse_GPIO_Port,Pulse_Pin,GPIO_PIN_RESET);
}
/*******************************************************/
/*Set initial parameters for each stepper motors*/
void Stepper_Init(float StepAngle_Deg, uint8_t stpMode, uint8_t CtrlMode, float GearRatio)
{
	TB67S_Disable();
	StepVar.GearRatio = GearRatio;     								// Ti so truyen 1/GearRatio
	StepVar.StepMode = stpMode;
	StepVar.BaseStepAngle_Degree = StepAngle_Deg;  //Step angle of the motor, which can be seen in the name plate
	TB67S_Reset();
	switch (StepVar.StepMode)
	{
			case Full:
					StepVar.RealStepAngle_Rad = (StepVar.BaseStepAngle_Degree/StepVar.GearRatio)*(PI/180);  			
					TB67S_Mode(Full);
					break;
			case Half_A:
					StepVar.RealStepAngle_Rad  = (StepVar.BaseStepAngle_Degree*0.5/StepVar.GearRatio)*(PI/180); 
					TB67S_Mode(Half_A);
					break;
			case Micro_1_4:
					StepVar.RealStepAngle_Rad  = (StepVar.BaseStepAngle_Degree*0.25/StepVar.GearRatio)*(PI/180); 
					TB67S_Mode(Micro_1_4);
					break;
			case Micro_1_8:
					StepVar.RealStepAngle_Rad  = (StepVar.BaseStepAngle_Degree*0.125/StepVar.GearRatio)*(PI/180); 
					TB67S_Mode(Micro_1_8);
					break;
			case Micro_1_16:
					StepVar.RealStepAngle_Rad  = (StepVar.BaseStepAngle_Degree*0.0625/StepVar.GearRatio)*(PI/180); 
					TB67S_Mode(Micro_1_16);
					break;
			case Micro_1_32:
					StepVar.RealStepAngle_Rad  = (StepVar.BaseStepAngle_Degree*0.03125/StepVar.GearRatio)*(PI/180); 
					TB67S_Mode(Micro_1_32);
					break;
	}
	/*Compute real pulse per round*/
	StepVar.ppr = (2*PI)/StepVar.RealStepAngle_Rad;  
	//Control mode: mPOSITION or mSPEED
	StepVar.CtrlMode = CtrlMode;	
	// Disable stepper motor driver		
	vDAC_Write(1,CurrentSet(0),0.0,3.3);
	vDAC_Write(2,CurrentSet(0),0.0,3.3);
}
/*******************************************************/
void Stepper_LockRotor(uint8_t mID)
{
	vDAC_Write(1,CurrentSet(mProfile[mID].holdI),0.0,3.3);
	vDAC_Write(2,CurrentSet(mProfile[mID].holdI),0.0,3.3);
	TB67S_Enable();
}
/*******************************************************/
void Stepper_FreeRotor(void)
{
	vDAC_Write(1,CurrentSet(0),0.0,3.3);
	vDAC_Write(2,CurrentSet(0),0.0,3.3);
	TB67S_Disable();
}
/*******************************************************/
/*Set motion profile*/
void MotionProfile_Set(uint8_t mID, float Accel_Radps2, float Decel_Radps2, float maxRPM, float SetAngle_Deg, float setI, float holdI)
{
	mProfile[mID].setI = setI;
	mProfile[mID].holdI = holdI;
	/*Compute total pulse per set angle*/
	mProfile[mID].Total_cnt=(SetAngle_Deg/360)*StepVar.ppr; 
	/*Built the speed profile*/
	mProfile[mID].Accel_Radps2 = Accel_Radps2;
	mProfile[mID].Decel_Radps2 = Decel_Radps2;
	mProfile[mID].SetAngle_Deg = SetAngle_Deg;
	mProfile[mID].maxSpeed_RadpSec = maxRPM/9.492966; 	//rpm to rad/s		
	mProfile[mID].max_s_lim=(mProfile[mID].maxSpeed_RadpSec*mProfile[mID].maxSpeed_RadpSec)/(2*StepVar.RealStepAngle_Rad*Accel_Radps2);
	mProfile[mID].accel_lim=(Decel_Radps2*mProfile[mID].Total_cnt)/(Accel_Radps2+Decel_Radps2);
	if(mProfile[mID].max_s_lim<=mProfile[mID].accel_lim) 																									//accel_run_deccel
	{
				mProfile[mID].decel_set_cnt = -(float)(mProfile[mID].max_s_lim*Accel_Radps2)/Decel_Radps2;
				mProfile[mID].run_set_cnt =(mProfile[mID].Total_cnt-(fabs)((double)(mProfile[mID].decel_set_cnt))-mProfile[mID].max_s_lim);
				mProfile[mID].accel_set_cnt = mProfile[mID].max_s_lim;
	}
	else //accel_deccel
	{
				mProfile[mID].decel_set_cnt=-(mProfile[mID].Total_cnt-mProfile[mID].accel_lim);
				mProfile[mID].run_set_cnt=0;
				mProfile[mID].accel_set_cnt=mProfile[mID].accel_lim;
	}
	mProfile[mID].decel_cnt=mProfile[mID].decel_set_cnt;
}
/************************************************************************/
void MotionVar_Prepare(uint8_t mID)
{
	/*Compute the first value which is loaded to the Auto Reload Register*/
	mProfile[mID].Cn=(f_Timer567*sqrt(2*StepVar.RealStepAngle_Rad/mProfile[mID].Accel_Radps2)*676);
	mProfile[mID].run_cnt=0;
	mProfile[mID].accel_cnt=0;
	mProfile[mID].Cn_1=mProfile[mID].Cn;
	mProfile[mID].rest=0;
}
/************************************************************************/
void Stepper_StateRst(void)
{
	StepVar.move=0; 
	StepVar.cmd = 0;
	StepVar.flag = 0;
}
/********************************************************************/
/*Run stepper motor with corresponding motion profile - Note: MotionProfile_ReSet() must be call first*/
void Stepper_Start(uint8_t mID) 
{ 
	StepVar.mProfileID = mID;
	/*Set time delay for linear stepper motor control algorithm*/
	if(mProfile[mID].Cn > AutReloadReg_Max)
	{
		__HAL_TIM_SET_COUNTER(&htim5, 0);
		__HAL_TIM_SET_AUTORELOAD(&htim5, AutReloadReg_Max);
	}
	else
	{
		__HAL_TIM_SET_COUNTER(&htim5, 0);
		__HAL_TIM_SET_AUTORELOAD(&htim5, mProfile[mID].Cn);
		
	}
	//Set current and Enable the Driver
	vDAC_Write(1,CurrentSet(mProfile[mID].setI),0.0,3.3);
	vDAC_Write(2,CurrentSet(mProfile[mID].setI),0.0,3.3);
	TB67S_Enable();
	/*Generate pulse and switch stepper motor state to STEPPER_Run*/
	step();
	StepVar.state = stRUN;
	StepVar.cmd = 1;
}
/***********************************************************************/
void Stepper_Stop(void)
{	
	if(StepVar.move !=0)       //Stepper motor is running
	{
			StepVar.move = -1;     //Decelerate and stop
			StepVar.cmd = 0;
	}
}
/***********************************************************************/
void Stepper_Control(uint8_t mID)	//Called in Timer ISR - Only if State = STEPPER_Run
{  	
	step();
	switch(StepVar.CtrlMode)
	{
			case mPOSITION:
			{
					if(StepVar.cmd == 1)
					{
							if(++mProfile[mID].accel_cnt<mProfile[mID].accel_set_cnt)         //Keep accelerating
							{
								StepVar.move=2;
							}
							else
							{
									mProfile[mID].accel_cnt=mProfile[mID].accel_set_cnt;          //Keep accel_count constant
									if(mProfile[mID].run_set_cnt>0)                               //2-1-2 with V = const 
									{
											 if(++mProfile[mID].run_cnt<mProfile[mID].run_set_cnt)
											 {
														StepVar.move = 1;                                	//Run with v = constant
														mProfile[mID].Crun = mProfile[mID].Cn;										
											 }
											 else                                                     //Decelerate
											 {
														mProfile[mID].run_cnt = mProfile[mID].run_set_cnt;
														StepVar.move = -1;
											 }
									}
									else    //Decelerate
											 StepVar.move = -1;
							}
					}
					else
							StepVar.move = -1;              
					break;
			}
			case mSPEED:
			{
					if(StepVar.cmd == 1)
					{
							if(++mProfile[mID].accel_cnt<mProfile[mID].accel_set_cnt)     //Keep accelerating
							{
								StepVar.move=2;
							
							}
							else
							{
									mProfile[mID].accel_cnt=mProfile[mID].accel_set_cnt;      //Keep accel_count constant
									StepVar.move = 1;                                      	//Run with constant speed
									mProfile[mID].Crun = mProfile[mID].Cn;
							}
					}
					else
							StepVar.move = -1;                                         //Decelerate and stop
					break;
			}
	}
	switch(StepVar.move)
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
										__HAL_TIM_SET_AUTORELOAD(&htim5, AutReloadReg_Max);
							    }
									else
									{
											__HAL_TIM_SET_AUTORELOAD(&htim5, mProfile[mID].Cn);
									
									}
						 }
						 else
						 {    					   
									StepVar.move = 0;
									StepVar.state = stLOCKROTOR;		
									Stepper_LockRotor(mID);
									StepVar.cmd = 0;
									if(StepVar.CtrlMode == mPOSITION)
											StepVar.PosFlag = 1; 									//Reach the set position
									__HAL_TIM_SET_AUTORELOAD(&htim5, 49999);	//period = 100ms
						 }
						 break;
				}
				case 1: 	//Run
				{
					if(mProfile[mID].Cn > AutReloadReg_Max)
					{
							__HAL_TIM_SET_AUTORELOAD(&htim5, AutReloadReg_Max);	
					}
					else
					{
							__HAL_TIM_SET_AUTORELOAD(&htim5, mProfile[mID].Cn);
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
								__HAL_TIM_SET_AUTORELOAD(&htim5, AutReloadReg_Max);
							
						}
						else
								__HAL_TIM_SET_AUTORELOAD(&htim5, mProfile[mID].Cn);
						
						break;
				}
	 }
	 
}

