#include "stm32f10x.h"
#include "LED.h"
#include "ALL_DATA.h"
#include "ALL_DEFINE.h" 

void KY008Init(void)	
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable GPIOA clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // Configure PA12 as output push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Set PA12 to low initially (optional, if you want to start with the laser off)
    GPIO_ResetBits(GPIOA, GPIO_Pin_12);
}


void GPIO_Config(void);
{
    // Initialize GPIO configuration
    GPIO_Config();

    while (1) 
		{
        // Set PA12 to high (5V) to keep the laser transmitter emitting laser
        GPIO_SetBits(GPIOA, GPIO_Pin_12);
    }
}