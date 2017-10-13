/*
 * UART.c
 *
 *  Created on: Oct 10, 2017
 *      Author: Andres Hernandez
 */
#include "UART.h"
#include "GPIO.h"

/*Global variable that saves the info*/
UART_MailBoxType UART0_MailBox;


void UART0_Status_IRQHandler(void){
	/*First is verified if the serial port finished to transmit*/
	while(!(UART0->S1 & UART_S1_RDRF_MASK)){
		/*The info is saved in Data Register*/
		UART0->D |= UART0_MailBox.mailBox;
		/*There are new data*/
		UART0_MailBox.flag = 1;
	}
}

void UART_init(UART_ChannelType uartChannel, uint32 systemClk, UART_BaudRateType baudRate){
	/*Value of adjust desired based on n/32*/
	float bRFD = 0;
	/*Variable saves the original value of Baud Rate*/
	float sBR;
	/*Variables saves the Baud Rate by parts*/
	uint32 sbr_HIGH;
	uint32 sbr_LOW;
	/*Variable saves the complete Baud Rate*/
	uint32 sbr_TEMP;
	/*Variables saves the Baud Rate Fine Adjust*/
	uint8 bRFA;
	/*Variables saves the difference of BaudRate expected and the original*/
	//float diff;

	sBR = (float)(systemClk/(baudRate*16));
	//diff = sBr - (uint32)sBR;
	//BRFD = 30/32 = 0.9375
	//BRFA = 11110
	/*Assigns the indicated BRFA*/
	bRFA = 0x1E;

	/*Considers only the integer part*/
	sbr_TEMP = (uint32)sBR;
	/*Saves the High part of Baud Rate*/
	sbr_HIGH = sbr_TEMP & UART_SBR_MASK_HIGH;
	/*Saves the Low part of Baud Rate*/
	sbr_LOW = sbr_TEMP & UART_SBR_MASK_LOW;
	/*Enable the clock of UART 0 */
	SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;
	/*Disable the Rx of UART*/
	UART0->C2 &= ~(UART_C2_RE_MASK);
	/*Disable the Tx of UART*/
	UART0->C2 &= ~(UART_C2_TE_MASK);
	/*Send the High part of SBR*/
	UART0->BDH |= sbr_HIGH<<BIT8;
	/*Send the Low part of SBR*/
	UART0->BDL |= sbr_LOW;
	/*Send the Baud Rate Fine Adjust to register*/
	UART0->C4 |= bRFA;
	/*Enable Rx of UART*/
	UART0->C2 |= (UART_C2_RE_MASK);
	/*Enable Tx of UART*/
	UART0->C2 |= (UART_C2_TE_MASK);

}

void UART0_interruptEnable(UART_ChannelType uartChannel){
	/*Verifies if the data is complete*/
	while (!(UART0->S1 & UART_S1_RDRF_MASK)){
		/*Enable the interrupter of reception*/
		UART0->C2 |= UART_C2_RWU_MASK;
	}
}

void UART_putChar (UART_ChannelType uartChannel, uint8 character){
	/*Check if there isn't data transmission*/
	while(!(UART0->S1 & UART_S1_TC_MASK)){
		/*Send character to Data Register*/
		UART0->D |= character;
	}
}

void UART_putString(UART_ChannelType uartChannel, sint8* string){
	/*Counter that verifies each position of the array*/
	uint8 counter = 0;
	/*Check if there isn't data transmission*/
	while(!(UART0->S1 & UART_S1_TC_MASK)){
		/*Transmit the data until find the NULL value*/
		while(string[counter] != '\0'){
			/*Each character of string is send to Data Register*/
			UART0->D |= string[counter];
			/*Move to next position in the array*/
			counter++;
		}
	}
}

