#include "main.h"

typedef struct{
	uint8_t FrameStart;
	uint8_t GroupID;
	uint8_t FrameLength;
	uint8_t Send_Add;
	uint8_t Receive_Add;
	uint8_t DataLength;
	uint8_t FrameEnd;
	volatile uint8_t state;
	volatile uint8_t Pointer;
	int8_t Rx_buff[16];
	int8_t Rx_data;
	 volatile uint8_t Flag ;
}Uart1;

enum State{
	Ready = 1,
	Run,
	Process,
};

#define Start        0x7A
#define End          0x7F


void Uart_Init( Uart1 *vUart);
void Uart_Read( Uart1 *vUart,UART_HandleTypeDef *huart);