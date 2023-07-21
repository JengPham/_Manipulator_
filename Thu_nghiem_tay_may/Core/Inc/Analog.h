#include "stm32f7xx_hal.h"

typedef struct 
{
	float Range1, Range2;				//Max range of the output voltage
	float Offset1, Offset2;			
	float VO1, VO2;							//Output voltage
	uint16_t AO1, AO2;					//Value to be sent to the DAC
}DAC_OUT;

//DAC initialization
void vDAC_Init();					
//DAC write, Vout = 0V->3V
void vDAC_Write(uint8_t DAC_Channel, float Vout, float Offset, float Range);	