#include "UART.h"

void UART1_Init(void) {
	// [TODO] 
	
}

void UART2_Init(void) {
	// [TODO]
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN; //enable USART2 in peripheral clock register
	RCC->CCIPR &= ~RCC_CCIPR_USART2SEL; // clear bits for USART2 clock source
	RCC->CCIPR |= RCC_CCIPR_USART2SEL_0; // set bits for USART2 clock source to 01 (system clock)
}

void UART1_GPIO_Init(void) {
	// [TODO] enable and set PB6 as AF output and PB7 as AF input
}

void UART2_GPIO_Init(void) {
	// [TODO] enable PA2 and set PA2 as AF output and PA3 AF input
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; //enable port A clock
	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED2 | GPIO_OSPEEDR_OSPEED3; //set output speed to very high for pin 2 and 3
	GPIOA->MODER &= ~(GPIO_MODER_MODE2_0 | GPIO_MODER_MODE3_0); //set to AF mode; each pin has [0:1] bits, we are clearing the lsb => 10 (alternating function)
	GPIOA->AFR[0] |= (GPIO_AFRL_AFSEL2_0 | GPIO_AFRL_AFSEL2_1 | GPIO_AFRL_AFSEL2_2); //set PA2 to 0111 (AF7, USART2_TX)
	GPIOA->AFR[0] |= (GPIO_AFRL_AFSEL3_0 | GPIO_AFRL_AFSEL3_1 | GPIO_AFRL_AFSEL3_2); //set PA3 to 0111 (AF7, USART2_RX)
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT2 | GPIO_OTYPER_OT3); //set port A to push-pull output
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD2 | GPIO_PUPDR_PUPD3); //clear both bits for pin 2 and pin 3 PUPDR
	GPIOA->PUPDR |= (GPIO_PUPDR_PUPD2_0 | GPIO_PUPDR_PUPD3_0); //set PUPDR to PU; lsb for configuration bits for pins 2 & 3 (01 => pull up)
}

void USART_Init(USART_TypeDef* USARTx) {
	// [TODO]
	USARTx->CR1 &= ~USART_CR1_UE; //disable USART
	USARTx->CR1 &= ~(USART_CR1_OVER8 | USART_CR1_M); //set oversampling mode to oversample by 16 (0) and set word length to 8 (M[1:0] = M1M0 = 00)
	USARTx->CR2 &= ~(USART_CR2_STOP); //set stop bit to 1
	USARTx->BRR |= (8333 << 4); //(416 << 4) | 0b1010); // set USARTDIV to 8333. Ty Arthur <3
	USARTx->CR1 |= USART_CR1_TE | USART_CR1_RE; //enable trasmitter and reciever 
	USARTx->CR1 |= USART_CR1_UE; //enable USART
}

uint8_t USART_Read (USART_TypeDef * USARTx) {
	// SR_RXNE (Read data register not empty) bit is set by hardware
	while (!(USARTx->ISR & USART_ISR_RXNE));  // Wait until RXNE (RX not empty) bit is set
	// USART resets the RXNE flag automatically after reading DR
	return ((uint8_t)(USARTx->RDR & 0xFF));
	// Reading USART_DR automatically clears the RXNE flag 
}

void USART_Write(USART_TypeDef * USARTx, uint8_t *buffer, uint32_t nBytes) {
	int i;
	// TXE is cleared by a write to the USART_DR register.
	// TXE is set by hardware when the content of the TDR 
	// register has been transferred into the shift register.
	for (i = 0; i < nBytes; i++) {
		while (!(USARTx->ISR & USART_ISR_TXE));   	// wait until TXE (TX empty) bit is set
		// Writing USART_DR automatically clears the TXE flag 	
		USARTx->TDR = buffer[i] & 0xFF;
		USART_Delay(300);
	}
	while (!(USARTx->ISR & USART_ISR_TC));   		  // wait until TC bit is set
	USARTx->ISR &= ~USART_ISR_TC;
}   

void USART_Delay(uint32_t us) {
	uint32_t time = 100*us/7;    
	while(--time);   
}
