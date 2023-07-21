# include "stm32f1xx_hal.h"
#include "main.h"
/**************************************************/
typedef struct{
	uint16_t ppr,rpm,Jog_rpm,accel,decel;
	uint8_t nEn;     	// Low : Enable , High : Disable
	int StepMode ;   	// Full , Half, 1/4,1/8,1/16,1/32
	uint8_t CtrlMode; // Position : 0 ; Speed :1
  float GearRatio, BaseStepAngle_Degree, RealStepAngle_Rad;
	int8_t move, Direction, state, cmd, PosFlag;
  int32_t Position_count;
	uint8_t flag ;
	uint8_t mProfileID;
}Stepper;

typedef struct{
	float maxSpeed_RadpSec, Accel_Radps2, Decel_Radps2, SetAngle_Deg;
	float setI, holdI;
	long Total_cnt;
	long run_cnt , run_set_cnt;
	long accel_cnt, accel_set_cnt;
	long decel_set_cnt, decel_cnt, half_decel_cnt;
	long max_s_lim, accel_lim;
	long Cn, Cn_1, Crun ,rest;
}Motion;

enum Step_Mode{
	Standby_mode,
	Full,
	Half_A,
	Half_B,
	Micro_1_4,
	Micro_1_8,
	Micro_1_16,
	Micro_1_32,
};

enum Motor_id{
	M1 = 1,
	M2,
	M3,
	All,
};

/***************************************************/

void delay_us(uint16_t us);
float CurrentSet(float Iset);
void TB67S_Mode(uint8_t Mode);
void TB67S_Enable(void);
void TB67S_Reset(void)	;
void TB67S_Dir_fwd(void);
void TB67S_Dir_rev(void);

//void MotionProfile_Set(Motion *mProfile, uint8_t mID, float Accel_Radps2, float Decel_Radps2, float maxRPM, float SetAngle_Deg, Stepper *StepVar);
//void MotionVar_ReSet(Motion *mProfile, uint8_t mID, Stepper *StepVar);
void Stepper_LockRotor(uint8_t mID);
void Stepper_FreeRotor(void);
void step(void);
//void Stepper_Init(Stepper *StepVar, float GRatio, float StepAngle_Deg, uint8_t stpMode, uint8_t CtrlMode);
//void Stepper_Start(Stepper *StepVar, Motion *mProfile, uint8_t mID, float setI);
//void Stepper_StateRst(Stepper *StepVar);
//void Stepper_Stop(Stepper *StepVar);
//void Stepper_Control(Stepper *StepVar, Motion *mProfile, uint8_t mID);

void Stepper_Init(float StepAngle_Deg, uint8_t stpMode, uint8_t CtrlMode, float N);
void MotionProfile_Set(uint8_t mID, float Accel_Radps2, float Decel_Radps2, float maxRPM, float SetAngle_Deg, float setI, float holdI);
void MotionVar_Prepare(uint8_t mID);
void Stepper_StateRst(void);
void Stepper_Start(uint8_t mID);
void Stepper_Stop(void);
void Stepper_Control(uint8_t mID);
/**************************************************/
#define NumofStepMotor 3

// Stepper motor state
#define stRUN    2			//Acc, Run or Dec
#define stLOCKROTOR 1		//Lock rotor by holding torque
#define stOFF    0			//Free wheeling - Iset = 0

// Stepper motor command
#define Runcmd  1
#define Stopcmd 0

//Direction
#define FWD 0
#define REV 1

//Control mode
#define mPOSITION 0
#define mSPEED    1

//Timer 5,6,7 is use for Stepper Motor control
#define f_Timer567 500 //kHz
#define AutReloadReg_Max 65535

#define PI  3.141592
#define Accel_min 5 // rad/s^2
#define Accel_max 200
#define Decel_min 5
#define Decel_max 200
#define rpm_min  10  // round per minute
#define rpm_max  300
#define Jog_rpm_max 100
#define Jog_rpm_min 10


