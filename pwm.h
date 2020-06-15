#ifndef PWM_H
#define PWM_H            // Keil::Device:STM32Cube HAL:Common

#include "stm32f4xx_hal.h"              // Keil::Device:STM32Cube HAL:Common
#include<stdlib.h>
#include<stdint.h>

void PWM_gen1(int32_t duty);
void PWM_gen2(int32_t duty);
void PWM_gen3(int32_t duty);

void dir_conf1(_Bool dir);
void dir_conf2(_Bool dir);
void dir_conf3(_Bool dir);


#endif 



