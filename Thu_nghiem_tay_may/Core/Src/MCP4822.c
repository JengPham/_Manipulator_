#include "MCP4822.h"


extern SPI_HandleTypeDef hspi4,hspi5;

void MCP4822_SP(uint8_t iref_chanel,float Vout)
{   

	uint16_t Cnt_DAC = (uint16_t)(1000*Vout);
	uint16_t TxBuff;			// Gi? tri de truyen v?o IC c? 16 bit: 4 bit dau la bit lenh, 12 bit sau l? bit data

	switch (iref_chanel)			// iref_chanel l? k?nh dien ?p dau ra
	{
		case 5:
		{
			TxBuff=0x1000+Cnt_DAC;																				// Cnt_DAC l? gia tri dien ap ra m? m?nh mong muon
			HAL_GPIO_WritePin(LDAC4_PORT,LDAC4_PIN,GPIO_PIN_SET);		// pin LDAC c? muc logic tu 1 -> 0 d?ng de chot du lieu truyen vao IC
			HAL_GPIO_WritePin(CS4_PORT,CS4_PIN,GPIO_PIN_RESET);			// pin CS o muc logic thap th? moi truyen duoc data
	
			HAL_SPI_Transmit(&hspi4,(uint8_t *)&TxBuff,2,10);
	
			HAL_GPIO_WritePin(CS4_PORT,CS4_PIN,GPIO_PIN_SET);
			HAL_GPIO_WritePin(LDAC4_PORT,LDAC4_PIN,GPIO_PIN_RESET);
			break;
		}
		case 6:
		{
			TxBuff=0x9000+Cnt_DAC;
			HAL_GPIO_WritePin(LDAC4_PORT,LDAC4_PIN,GPIO_PIN_SET);
			HAL_GPIO_WritePin(CS4_PORT,CS4_PIN,GPIO_PIN_RESET);
	
			HAL_SPI_Transmit(&hspi4,(uint8_t *)&TxBuff,2,10);
	
			HAL_GPIO_WritePin(CS4_PORT,CS4_PIN,GPIO_PIN_SET);
			HAL_GPIO_WritePin(LDAC4_PORT,LDAC4_PIN,GPIO_PIN_RESET);
			break;
		}
		case 3:
		{
			TxBuff=0x1000+Cnt_DAC;
			HAL_GPIO_WritePin(LDAC5_PORT,LDAC5_PIN,GPIO_PIN_SET);
			HAL_GPIO_WritePin(CS5_PORT,CS5_PIN,GPIO_PIN_RESET);
	
			HAL_SPI_Transmit(&hspi5,(uint8_t *)&TxBuff,2,10);
	
			HAL_GPIO_WritePin(CS5_PORT,CS5_PIN,GPIO_PIN_SET);
			HAL_GPIO_WritePin(LDAC5_PORT,LDAC5_PIN,GPIO_PIN_RESET);
			break;
		}
		case 4:
		{
			TxBuff=0x9000+Cnt_DAC;
			HAL_GPIO_WritePin(LDAC5_PORT,LDAC5_PIN,GPIO_PIN_SET);
			HAL_GPIO_WritePin(CS5_PORT,CS5_PIN,GPIO_PIN_RESET);
	
			HAL_SPI_Transmit(&hspi5,(uint8_t *)&TxBuff,2,10);
	
			HAL_GPIO_WritePin(CS5_PORT,CS5_PIN,GPIO_PIN_SET);
			HAL_GPIO_WritePin(LDAC5_PORT,LDAC5_PIN,GPIO_PIN_RESET);
			break;
		}
	}
}