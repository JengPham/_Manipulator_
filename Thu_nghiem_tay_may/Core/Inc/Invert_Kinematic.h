#include <math.h>
#include "stm32f7xx_hal.h"

#define row  4
#define col  4
#define pi   3.141592

typedef struct{
	float a1,d1;
	float a2,d2;
	float a3,d3;
	float a4,d4;
	float a5,d5;
	float a6,d6;
	float t1,t2,t3,t4,t5,t6;
	int Q[6];
	float T1_max,T1_min;
  float T2_max,T2_min;
	float T3_max,T3_min;
	float T4_max,T4_min;
	float T5_max,T5_min;
	float T6_max,T6_min;
}Robot;


void Robot_Init(Robot *vRobot);  
void MatrixMultiply(float matrix1[row][col], float matrix2[row][col], float res[row][col]);  
void Invert_Kinematic(Robot *vRobot); 
void Convert_Angle(Robot *vRobot);