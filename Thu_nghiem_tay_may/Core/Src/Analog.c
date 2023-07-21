#include "Analog.h"
#include "main.h"
#include "MCP4822.h"
extern void MCP4822_SP(uint8_t iref_chanel,float Vout);
extern DAC_HandleTypeDef hdac;
void vDAC_Init()
{
HAL_DAC_Start(&hdac,DAC_CHANNEL_1);
HAL_DAC_Start(&hdac,DAC_CHANNEL_2);
}
void vDAC_Write(uint8_t DAC_Channel, float Vout, float Offset, float Range)
{
	int32_t DAC_Val;
	
	DAC_Val = (int32_t )((Vout + Offset)*(4095.0/Range));
	if(DAC_Val > 4095)
		DAC_Val = 4095;
	if(DAC_Val < 0)
		DAC_Val = 0;
	switch(DAC_Channel)
	{
		case 1:
		{
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, DAC_Val);		
			break;
		}
		case 2:
		{
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_Val);
			break;
		}
		case 3:
		{
		 MCP4822_SP(3,Vout + Offset);
			break;
		}
			case 4:
		{
		 MCP4822_SP(4,Vout + Offset);
			break;
		}
			case 5:
		{
		 MCP4822_SP(5,Vout + Offset);
			break;
		}
			case 6:
		{
		 MCP4822_SP(6,Vout + Offset);
			break;
		}
	}
} 
