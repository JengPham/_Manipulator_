#include "Invert_Kinematic.h"
#include <math.h>
#include <stdlib.h>
Robot pRobot;
float nx,ny,nz,ox,oy,oz,ax,ay,az,px,py,pz;
float P,Q,X,k1;
// ma tran dong nhat cua khop 6 so voi goc toa dô khi o vi tri 0
float T06[4][4] ={0.000,0.000,1.000,0.393,
	                      0.000,-1.000,0.000,0.000,
                        1.000,0.000,0.000,-0.1345,
                        0.000,0.000,0.000,1.000};  
// ma tran dông nhat c?a camera so voi khop 6
float T6C[4][4] ={0.000,-1.000,0.000,-0.09,
	                1.000,0.000,0.000,0.000,
	                0.000,0.000,1.000,0.000,
	                0.000,0.000,0.000,1.000};
float TCV[4][4] ;      // ma tran vi trí cua vat so voi camera   
float T6V[4][4] ;    // ma tran dong nhat cua vat so voi khop 6
float T0V[4][4];     // ma tran dông nhat cua vat so voi goc toa dô
void Robot_Init(Robot *vRobot){
	/** Don vi : m **/
	vRobot->a1 = 0.07;   
	vRobot->a2 = 0.305;
	vRobot->a3 = 0;
	vRobot->a4 = 0;
	vRobot->a5 = 0;
	vRobot->a6 = 0;
	vRobot->d1 = 0.1705;
	vRobot->d2 = 0;
	vRobot->d3 = 0;
	vRobot->d4 = 0.263;
	vRobot->d5 = 0 ;
	vRobot->d6 = 0.060 ;
	// Gioi han goc quay
	vRobot->T1_max = 170;
	vRobot->T1_min = -170;
	vRobot->T2_max = 90;
	vRobot->T2_min = -42;
	vRobot->T3_max = 52;
	vRobot->T3_min = -89;
	vRobot->T4_max = 165;
	vRobot->T4_min = -165;
	vRobot->T5_max = 105;
	vRobot->T5_min = -105;
	vRobot->T6_max = 155;
	vRobot->T6_min = -155;
}

void MatrixMultiply(float matrix1[row][col], float matrix2[row][col], float res[row][col]){
	for(int i =0 ; i <row;i++){
		for(int j = 0; j < col; j++){
			float sum =0 ;
			for(int k =0 ; k <row;k++){
				sum += matrix1[i][k] * matrix2[k][j];
			}
			res[i][j] = sum;
		}
	}
}

float Select ( float A, float B, float T1, float T2){
	float T;
		if (T1*180/pi<=B && T1*180/pi>=A){
			if (T2*180/pi<=B && T2*180/pi>=A){
					if (abs((int)T1)-abs((int)T2)<=0){
						T=T1;
					}
					else {
						T=T2;
					}
				}
			else {
				T=T1;
			}
			}
		else if (T2*180/pi<=B && T2*180/pi>=A){
			T=T2;
		}
		else 
		{
			// error = 1;
			T=9999;
		}
		return T;
}
void Invert_Kinematic(Robot *vRobot){
	  nx = T0V[0][0];    ox = T0V[0][1];    ax = T0V[0][2];    px = T0V[0][3];
    ny = T0V[1][0];    oy = T0V[1][1];    ay = T0V[1][2];    py = T0V[1][3];
    nz = T0V[2][0];    oz = T0V[2][1];    az = T0V[2][2];    pz = T0V[2][3];
	  
	// Bien khop so 1
float  A = atan2(ay*vRobot->d6-py,ax*vRobot->d6-px);    
float  B = atan2(-ay*vRobot->d6+py,-ax*vRobot->d6+px);
//	if(A*180/pi > vRobot->T1_max || A*180/pi < vRobot->T1_min)
//		vRobot->t1 = B;
//	if(B*180/pi > vRobot->T1_max || B*180/pi < vRobot->T1_min) 
//		vRobot->t1 = A;  

//	if(A*180/pi < vRobot->T1_max && A*180/pi >vRobot->T1_min && B*180/pi < vRobot->T1_max && B*180/pi > vRobot->T1_min){
//		vRobot->t1 = B;
//	//	vRobot->t1 = atan2(0.1,-0.433);
//	}
	vRobot->t1 = Select(vRobot->T1_min, vRobot->T1_max, A, B);
  P = cos(vRobot->t1)*(-ax*vRobot->d6+px)+sin(vRobot->t1)*(-ay*vRobot->d6+py);
  Q = -az*vRobot->d6+pz;
  X = (pow(P-vRobot->a1,2)+pow(Q-vRobot->d1,2)-pow(vRobot->d4,2)-pow(vRobot->a2,2))/(2*vRobot->a2*vRobot->d4);
 // Bien khop so 3
float C = atan2(X,sqrt(1-pow(X,2)));
float D = atan2(X,-sqrt(1-pow(X,2)));
// 	if((C*180/pi) > vRobot->T3_max || (C*180/pi) < vRobot->T3_min)
//		vRobot->t3 = D;
//	if((D*180/pi) > vRobot->T3_max || (D*180/pi) < vRobot->T3_min)
//		vRobot->t3 = C;
//	if(C*180/pi < vRobot->T1_max && C*180/pi >vRobot->T1_min && D*180/pi < vRobot->T1_max && D*180/pi > vRobot->T1_min){
//		vRobot->t3 = D;
//	}
	vRobot->t3 = Select(vRobot->T3_min, vRobot->T3_max, C, D);
 float a = vRobot->d4*cos(vRobot->t3);
 float b = vRobot->d4*sin(vRobot->t3)+vRobot->a2;
//Bien khop so 2
 vRobot->t2 = atan2((a*(P-vRobot->a1)-b*(Q-vRobot->d1)),(a*(Q-vRobot->d1)+b*(P-vRobot->a1)));
  k1 = -(ax*sin(vRobot->t2+vRobot->t3)*cos(vRobot->t1)+ay*sin(vRobot->t1)*sin(vRobot->t2+vRobot->t3)+az*cos(vRobot->t2+vRobot->t3));
// Bien khop so 5
float E = atan2(sqrt(1-pow(k1,2)),k1);
float F = atan2(-sqrt(1-pow(k1,2)),k1);
//  if(E*180/pi > vRobot->T5_max || E*180/pi < vRobot->T5_min)
//		vRobot->t5 = F;
//	if(F*180/pi > vRobot->T5_max || F*180/pi < vRobot->T5_min)
//		vRobot->t5 = E;
//	if(E*180/pi < vRobot->T1_max && E*180/pi >vRobot->T1_min && F*180/pi < vRobot->T1_max && F*180/pi > vRobot->T1_min){
//		vRobot->t5 = F;
//	}
	vRobot->t5 = Select(vRobot->T5_min, vRobot->T5_max, E, F);
 float x1 = (-ax*sin(vRobot->t1)+ay*cos(vRobot->t1))/sin(vRobot->t5);
 float y1 = (ax*cos(vRobot->t1)*cos(vRobot->t2+vRobot->t3)+ay*sin(vRobot->t1)*cos(vRobot->t2+vRobot->t3)-az*sin(vRobot->t2+vRobot->t3))/sin(vRobot->t5);
// Bien khop so 4
 vRobot->t4 = atan2(x1,y1);
 float x2 = -(ox*sin(vRobot->t2+vRobot->t3)*cos(vRobot->t1) + oy*sin(vRobot->t1)*sin(vRobot->t2+vRobot->t3)+oz*cos(vRobot->t2+vRobot->t3))/sin(vRobot->t5);
 float y2 = (nx*cos(vRobot->t1)*sin(vRobot->t2+vRobot->t3)+ny*sin(vRobot->t1)*sin(vRobot->t2+vRobot->t3)+nz*cos(vRobot->t2+vRobot->t3))/sin(vRobot->t5);
// Bien khop so 6
 vRobot->t6 = atan2(x2,y2);
}
void Convert_Angle(Robot *vRobot){
	 vRobot->Q[0] = (int)(vRobot->t1*180)/(pi);
	 vRobot->Q[1] = (int)(vRobot->t2*180)/(pi);
	 vRobot->Q[2] = (int)(vRobot->t3*180)/(pi);
	 vRobot->Q[3] = (int)(vRobot->t4*180)/(pi);
	 vRobot->Q[4] = (int)(vRobot->t5*180)/(pi);
	 vRobot->Q[5] = (int)(vRobot->t6*180)/(pi);
}	


