#include "UART.h"
extern UART_HandleTypeDef huart1;
extern float TCV[4][4] ;      // ma tran vi trí cua vat so voi camera 

Uart1 pUart;
void Uart_Init( Uart1 *vUart){
	vUart->FrameStart = Start;
	vUart->GroupID    = 0x00;
	vUart->FrameLength= 0x10;
	vUart->Send_Add   = 0x02;
	vUart->Receive_Add= 0x04;
	vUart->DataLength = 0x03;
	vUart->FrameEnd   = End;
	vUart->state      = Ready;
	vUart->Pointer    = 0;
	vUart->Rx_data    = 0;
	vUart->Flag       = 0;
}

void Uart_Read( Uart1 *vUart,UART_HandleTypeDef *huart){
	switch(vUart->state){
		case Ready: {
		if( vUart ->Rx_data == 0x7A)
			{ 
				vUart->Rx_buff[vUart->Pointer] = vUart->Rx_data;
				vUart ->Pointer = 1;
				vUart->state =Run;
			}
			else {
					vUart ->Pointer = 0;
				  for(int i =0 ; i < 16 ; i++)
				  {
						vUart->Rx_buff[i] = 0;
					}
					vUart->state = Ready;
			}
		break;
}
		case Run: {
	if( vUart ->Rx_data == 0x7F)
			{ 
					if(vUart->Pointer == 15)
				{ 
					vUart->Rx_buff[vUart->Pointer] = vUart->Rx_data;
					vUart->state = Process;
				
				}
				 else
				{
					vUart ->Pointer = 0;
				  for(int i =0 ; i < 16 ; i++)
				  {
						vUart->Rx_buff[i] = 0;
					}
					vUart->state = Ready;
					
				}
			}
			else if( vUart->Pointer == 15)
			{
				if( vUart ->Rx_data !=  End)
				{
					vUart ->Pointer = 0;
				  for(int i =0 ; i < 16 ; i++)
				  {
						vUart->Rx_buff[i] = 0;
					}
					vUart->state = Ready;
					
				}
				
			}
			else {
				if( vUart->Pointer < 16){
				vUart->Rx_buff[vUart->Pointer] = vUart->Rx_data;
				vUart ->Pointer +=1;
				}
				else{
					vUart ->Pointer = 0;
				  for(int i =0 ; i < 16 ; i++)
				  {
						vUart->Rx_buff[i] = 0;
					}
					vUart->state = Ready;
					
					
				}
				
			}
			break;
}
		}
	} 
	
	
/*	void Uart_Read(Uart1 *vUart)
{
	if(vUart->Pointer == 0)
		{
			if(vUart->Rx_data == 0x7A)
		{
			pUart.Rx_buff[vUart->Pointer++] = vUart->Rx_data;
		}	
		}
		else if(vUart->Pointer == 9){
				vUart->Rx_buff[vUart->Pointer] = vUart->Rx_data;
		if(pUart.Rx_data == 0x7F)
		{ 
			vUart->state = Process;
			vUart->Pointer = 0;
		}
	} 
		else
		vUart->Rx_buff[vUart->Pointer++] = vUart->Rx_data;
} */