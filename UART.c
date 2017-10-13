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
	float bRFD = 0;
	float sBR;
	uint32 sbrHigh;
	uint32 sbrLow;
	uint32 sbrTemp;
	//float diff;
	uint8 bRFA;

	sBR = (float)(systemClk/(baudrate*16));
	//diff = sBr - (uint32)sBR;
	//BRFD = 30/32 = 0.9375
	//BRFA = 11110
	bRFA = 0x1E;
	sbrTemp = (uint32)sBR;
	sbrHigh = sbrTemp & 0x1F00;
	sbrLow = sbrTemp & 0xFF;

	switch(uartChannel)
	{
	case UART_0:
	SIM->SCGC4 |= 0x0400;
	UART0->C2 &= ~(0x04);
	UART0->C2 &= ~(0x08);
	UART0->BDH |= sbrHigh<<BIT8;
	UART0->BDL |= sbrLow;
	UART0->C4 |= bRFA;
	UART0->C2 |= (0x04);
	UART0->C2 |= (0x08);

}

void UART0_interruptEnable(UART_ChannelType uartChannel){
	while (!(UART0->S1 & UART_S1_RDRF_MASK)){
		UART0->C2 |= 0x02;
	}
}

void UART_putChar (UART_ChannelType uartChannel, uint8 character){
	while(!(UART0_S1 & UART_S1_TC_MASK)){
		UART0->D |= character;
	}
}

void UART_putString(UART_ChannelType uartChannel, sint8* string){


}





