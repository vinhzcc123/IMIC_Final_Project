#include "custom.h"

uint8_t data_receive [128] = {0};
char buffer[4096] = {0};

void custom_clock_init()
{
	uint32_t* RCC_CR = (uint32_t*)0x40021000;
	*RCC_CR = (*RCC_CR & ~(uint32_t) 1) | 1;                   //enable internal 8MHz RC oscillator ON

	uint32_t* RCC_CFGR = (uint32_t*)(0x40021000 + 0x04);
	*RCC_CFGR &= (uint32_t)3;                                  //system clock switch not allowed

	uint32_t* RCC_APB2ENR = (uint32_t*)(0x40021000 + 0x18);
	uint32_t* RCC_AHBENR = (uint32_t*)(0x40021000 + 0x14);

	*RCC_APB2ENR = (*RCC_APB2ENR & ~(uint32_t) 0x08) | 0x08;     // enable port B
	*RCC_APB2ENR = (*RCC_APB2ENR & ~(uint32_t) 0x04) | 0x04;     //
	*RCC_APB2ENR |= 1 << 2;        // enable clock port A
	*RCC_APB2ENR |= 1 << 14;       // enable clock USART1
	*RCC_AHBENR |= 1;              // enable clock DMA1
}

void custom_systick_init()
{
	uint32_t* SYSTICK_CSR = (uint32_t *)0xE000E010;
	uint32_t* SYSTICK_RVR = (uint32_t *)0xE000E014;
	*SYSTICK_CSR = (*SYSTICK_CSR & ~(0x07)) | 7;
	*SYSTICK_RVR = 8000;
}

void custom_delay(uint16_t miliseconds)
{
	uint32_t* SYST_CSR = (uint32_t*)(0xE000E010);
	*SYST_CSR = (*SYST_CSR & ~(uint32_t)0x0001) | 1;
	uint16_t count = 0;
	while (count < miliseconds)
	{
		if(((*SYST_CSR & (uint32_t)0x10000) >> 16) == 1)
		count++;
	}
	*SYST_CSR = (*SYST_CSR & ~(uint32_t)0x0001);
}

void custom_UART_init()
{
	uint32_t *GPIOA_CRH = (uint32_t*)(0x40010804);
	uint32_t *USART_BRR = (uint32_t*)(0x40013808);
	uint32_t *USART_CR1 = (uint32_t*)(0x4001380C);
	*GPIOA_CRH &= 0xFFFFF00F;
	*GPIOA_CRH |= (1 << 10) | (3 << 6) | (3 << 4); // set Tx as Alternate function output open-dair and set Rx as Floating input

	*USART_BRR = 52<<4 | 1;        //Set BaudRate in 9600 bits/sec
	*USART_CR1 |= 1<<3;		       //Enable transmitter
	*USART_CR1 |= 1<<2; 	       //Enable receiver
	*USART_CR1 |= 1<<13;	       //Enable UART
}

void UART_write(uint8_t data)
{
	uint32_t *USART_DR = (uint32_t*)(0x40013804);
	uint32_t *USART_SR = (uint32_t*)(0x40013800);

//	while(((*USART_SR>>7)&1) !=1);			//wait TXE (Transmit data register empty)
	*USART_DR = data & 0x000000ff;			//write data to USART_DR
	while(((*USART_SR>>6)&1) !=1);			//wait TC (Transmission complete)
	*USART_SR &= ~(uint32_t)(1<<6);
}

uint8_t UART_read()
{
	uint32_t *USART_DR = (uint32_t*)(0x40013804);
	uint32_t *USART_SR = (uint32_t*)(0x40013800);

	uint8_t result;
	while(((*USART_SR>>5)&1) !=1);			//wait RXNE flag is set
	result = (uint8_t)(*USART_DR);			//Read data to USART_DR
	return result;
}

void custom_DMA()
{
	uint32_t *DMA_CCR5 = (uint32_t*)(0x40020058);
	uint32_t *DMA_CNDTR5 = (uint32_t*)(0x4002005C);
	uint32_t *DMA_CPAR5 = (uint32_t*)(0x40020060);
	uint32_t *DMA_CMAR5 = (uint32_t*)(0x40020064);
	uint32_t *USART_CR3 = (uint32_t*)(0x40013814);

	*USART_CR3 |= 1 << 6; // enable DMA receiver

	//set these before enable DMA channel 5
	*DMA_CNDTR5 = sizeof(buffer);   //length of data
	*DMA_CMAR5 = (uint32_t) buffer; //memory address register
	*DMA_CPAR5 = (0x40013804);            //peripheral address register

	//enable DMA channel 5
	*DMA_CCR5 = (1<<0)|(1<<5)|(1<<7);
}

char find_OK()
{
	int size = sizeof(buffer) - 1;
	for(int i = 0; i < size; i++)
	{
		if((buffer[i] == 'O') && (buffer[i+1] == 'K'))
		{
			return 1;
		}
	}
	return 0;
}
