/*
 * UART.c
 *
 *  Created on: Oct 10, 2017
 *      Author: Andres Hernandez
 */
#include "UART.h"


void UART0_Status_IRQHandler(void){

}

void UART_init(UART_ChannelType uartChannel, uint32 systemClk, UART_BaudRateType baudRate){
	float uartBaudrate;
	float bRFD;
	uint8 bRFA;

	switch(uartChannel)
	{
	case UART_0:
		uartBaudrate = ((float)(systemClk))/((16*((float)baudRate))+ bRFD);
		bRFD = 0;
		//bRFD = (float)bRFA/32;
		SIM->SCGC4 |= 0x0400;

		break;
	case UART_1:
		SIM->SCGC4 |= 0x0800;

		break;
	case UART_2:
		SIM->SCGC4 |= 0x1000;

		break;
	case UART_3:
		SIM->SCGC4 |= 0x2000;

		break;
	case UART_4:
		SIM->SCGC1 |= 0x0400;

		break;
	case UART_5:
		SIM->SCGC1 |= 0x0800;

		break;
	default:
		break;
	}

}

void UART0_interruptEnable(UART_ChannelType uartChannel){

}

void UART_putChar (UART_ChannelType uartChannel, uint8 character){

}

void UART_putString(UART_ChannelType uartChannel, sint8* string){

}





