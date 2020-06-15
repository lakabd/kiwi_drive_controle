#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f4xx_hal.h"              // Keil::Device:STM32Cube HAL:Common
#include "pwm.h"
#include<math.h>

/*
DESCRIPTION : handeling encoder signals A and B to calculate the speed of the motor and it's direction

//Configuring timer 2, timer 3 and timer 4 in capture mode in channel 1  for the 3 motor's encoder

Moteur 1 :
	- Cod_A sur PA15 ---> AF : TIM2_CH1
    - Cod_B sur PB3  ---> Input
Moteur 2 :
    - Cod_A sur PB4  ---> AF : TIM3_CH1
    - Cod_B sur PB5  ---> input
Moteur 3 :
    - Cod_A sur PB7  ---> input
    - Cod_B sur PB6  ---> AF : TIM4_CH1
*/



void encoder_handler1(void);
void encoder_handler2(void);
void encoder_handler3(void);




#endif

