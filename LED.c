#include "LED.h"

void LED_Init(void) {
	// Enable GPIO Clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; // Enable Port A Clock

    // Initialize Green LED
	GPIOA->MODER ^= 0x800U; //bitwise XOR 0x00000800U => 0b00000000000000000000100000000000 since MODER for GPIO A is 0xFFFFFFFF by default (must toggle 11th bit)
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT5; // set GPIO A output type to push-pull (0x0)
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD5; // set GPIO A to have no pull up or pull down
}

void Green_LED_Off(void) {
	GPIOA->ODR &= ~GPIO_ODR_OD5;
}

void Green_LED_On(void) {
	GPIOA->ODR |= GPIO_ODR_OD5;
}

void Green_LED_Toggle(void){
	GPIOA->ODR ^= GPIO_ODR_OD5;
}