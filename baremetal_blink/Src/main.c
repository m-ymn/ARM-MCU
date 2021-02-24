/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/*
#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif
*/

#include <stdio.h>
#include "stm32f4xx.h"

volatile int32_t delay ;

void Delay(uint32_t tme) ;
void systick_init() ;
void Systick_SetPriority(IRQn_Type IRQn, uint32_t priority);



int main(void)
{

	RCC->AHB1ENR = 1<<3 ; //GPIOD CLK enable
	GPIOD->MODER = 1<<24 ;   // 12*2
	GPIOD->OSPEEDR = 00 ;
	GPIOD->PUPDR = 00 ;
	systick_init() ;
	//int i ;
	while(1)
	{

		GPIOD->BSRR = 1<<12 ;   // led On
		//while (i<1000000) {i++ ; }
		Delay(1) ;    // onetime systick max timeout delay
		//i =0 ;
		GPIOD->BSRR = 1<<28 ;   //led off
		Delay(1) ;
		//while (i<1000000) {i++ ; }
		//i=0 ;
		//printf("hello im here\n") ;
	}

}


void systick_init()
{
	SYSTICK->CTRL = 0 ; //reset

	SYSTICK->LOAD = 0x00FFFFFE ;  // 24bits only, max val - 1
	Systick_SetPriority(SysTick_IRQn, (1<<__NVIC_PRIO_BITS)  - 1) ;  //lowest priority
	SYSTICK->VAL = 0 ;
	SYSTICK->CTRL |= 1<<1 ; // interrupt enable
	SYSTICK->CTRL |= 1<<2 ; // 0 = AHB/8  , 1 = AHB proc clk
	SYSTICK->CTRL |= 1<<0 ;  //enable systick
}


void Systick_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
	SCB->SHP[(((uint32_t)IRQn) & 0xFUL)-4UL] = (uint8_t)((priority << (8U - __NVIC_PRIO_BITS)) & (uint32_t)0xFFUL);

}


void SysTick_Handler(void)
{
	//SYSTICK->CTRL = 0<<16 ;
	if(delay > 0)
		delay-- ;
}

void Delay(uint32_t tme){

	delay = tme ;
	while(delay !=0) ;

}
