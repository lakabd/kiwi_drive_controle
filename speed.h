#ifndef SPEED_H
#define SPEED_H

#include "stm32f4xx_hal.h"              // Keil::Device:STM32Cube HAL:Common
#include "encoder.h"
#include "pwm.h"

void Init(void);
void speed_def(float speed_m1,float speed_m2,float speed_m3);
uint16_t speed_to_count(float);
float count_to_speed(uint32_t);



#endif

