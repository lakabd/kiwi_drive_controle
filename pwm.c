//==================================================================================
//	FILE : PWM.c
//	AUTHOR : A.LAKBIR
//==================================================================================
//	DESCRIPTION : 	Generate command signals - PWM and direction for each motor. 
//	DETAIL : using TIM1 channel 3,1 and 4 for motor 1, 2 and 3 respectively																				 
//						Motor 1 :
//							- PWM on PA10
//							- DIR on PC1
//						Motor 2 :
//							- PWM on PA8 
//							- DIR on PC0
//						Motor 3 :
//							- PWM on PA11
//							- DIR on PC2
//==================================================================================

#include "pwm.h"


/** 
*	FUNCTION : dir_confX -> set motor X direction by toggling the corresponding GPIO pin state.
*	PARAM : _Bool dir -> 1 for clockwise rotation, 0 for the inverse.
*/

/*Direction config for Motor 1 
------------------------------------------------------------------------*/
void dir_conf1(_Bool dir){
	/*configure pin PC1 to be the direction pin*/
	__HAL_RCC_GPIOC_CLK_ENABLE(); /*enable GPIOC clock*/
	GPIO_TypeDef *pgpio;
	pgpio=GPIOC;
	pgpio->MODER |= GPIO_MODER_MODE1_0; /*configure pin1 in output mode*/
	if(dir)pgpio->BSRR|= (1u<<1); /*set pin to VCC*/
	else pgpio->BSRR |= (1u<<17);	/*reset pin = GND*/
}

/*Direction config for Motor 2 
------------------------------------------------------------------------*/
void dir_conf2(_Bool dir){
	/*Configure pin PC0 to be the direction pin*/
	__HAL_RCC_GPIOC_CLK_ENABLE(); /*enable GPIOC clock*/
	GPIO_TypeDef *pgpio;
	pgpio=GPIOC;
	pgpio->MODER |= GPIO_MODER_MODE0_0; /*configure pin1 in output mode*/
	if(dir)pgpio->BSRR|= (1u<<0); /*set pin = VCC*/
	else pgpio->BSRR |= (1u<<16);	/*reset pin = GND*/
}

/*Direction config for Motor 3
------------------------------------------------------------------------*/
void dir_conf3(_Bool dir){
	/*Configure pin PC2 to be the direction pin*/
	__HAL_RCC_GPIOC_CLK_ENABLE(); /*enable GPIOC clock*/
	GPIO_TypeDef *pgpio;
	pgpio=GPIOC;
	pgpio->MODER |= GPIO_MODER_MODE2_0; /*configure pin1 in output mode*/
	if(dir)pgpio->BSRR|= (1u<<2); /*set pin to VCC*/
	else pgpio->BSRR |= (1u<<18);	/*reset pin = GND*/
}


/** 
*	FUNCTION : PWM_genX -> configure PWM for motor X by configuring the corresponding TIM1 channel in output compare PWM1 mode.
*	PARAM : int32_t duty -> PWM duty cycle : between -100% and +100% -> duty sign is used as direction indicator.
*/


void PWM_gen1(int32_t duty){
	
	/*Set direction
	-----------------------------------------------------------------------*/
	if(duty<0) dir_conf1(1);
	else dir_conf1(0);
	
	/*GPIO config
	-----------------------------------------------------------------------*/
	/*default settings: low speed, output in push pull, no pull-up or pull-down for the output*/
	
	__HAL_RCC_GPIOA_CLK_ENABLE(); /*Enable the clock for gpioA*/
	
	GPIO_TypeDef *pgpio; 
	pgpio = GPIOA;              /*init the structure pointer with the base_adresse of GPIOA*/
	
	pgpio->MODER |= GPIO_MODER_MODE10_1; /*set gpioA output mode in alternate function for pin 10*/
	pgpio->AFR[1] |= (1u<<8);          /*set the alternate function AF1 for the 10th pin corresponding to TIM1 */
	
	/*Timer 1 config
	------------------------------------------------------------------------*/
	__HAL_RCC_TIM1_CLK_ENABLE(); /*Enable the clock for TIM1*/
	
	TIM_TypeDef *ptimer; 
	ptimer=TIM1;   /*init the structure pointer with the base_adresse of TIM1*/
	
	ptimer->SMCR = 0x0u;  /*Disable the slave mode control to use the internal clock of the APB2 bus CK_INT*/
	ptimer->PSC=9u;  /*define the prescaller value to 10*/
	ptimer->ARR=99u; /* and the ARR (auto-reload reg) value to obtain 16 KHz in timer output*/
	
	
	ptimer->CCMR2 |= 0x60u; /*select output mode + PWM mode 1 + OC preload disable + OC fast disable => all for channel 3*/
	ptimer->CCER |= (1u<<8); /*enable output on OC3 (output compare channel 3)*/
	
	TIM1->EGR |=0x1u; /*generate an UEV (update event) to update registers value*/
	TIM1->BDTR |= (1u<<15); /*enable the main output MOE bit */
	
	ptimer->CCR3=abs(duty);	/* set the capture compare register: if the counter value is <= CCR3 the output is high if > the output is low*/
	ptimer->CR1 |= 0x1u; /* Enable the counter*/
}


void PWM_gen2(int32_t duty){
		
	/*define direction
	------------------------------------------------------------------------*/
	if(duty<0) dir_conf2(1);
	else dir_conf2(0);
	
	/*GPIO config
	------------------------------------------------------------------------*/
	/*default settings: low speed, output in push pull, no pull-up or pull-down for the output*/
	
	__HAL_RCC_GPIOA_CLK_ENABLE(); /*Enable the clock for gpioA*/
	
	GPIO_TypeDef *pgpio; 
	pgpio = GPIOA;              /*init the structure pointer with the base_adresse of GPIOA*/
	
	pgpio->MODER |= GPIO_MODER_MODE8_1; /*set gpioA output mode in alternate function for pin 8*/
	pgpio->AFR[1] |= 0x1u;          /*set the alternate function AF1 for the 8th pin corresponding to TIM1 */
	
	/*Timer 1 config
	------------------------------------------------------------------------*/
	__HAL_RCC_TIM1_CLK_ENABLE(); /*Enable the clock for TIM1*/
	
	TIM_TypeDef *ptimer; 
	ptimer=TIM1;   /*init the structure pointer with the base_adress of TIM1*/
	
	ptimer->SMCR = 0x0u;  /*Disable the slave mode control to use the internal clock of the APB2 bus CK_INT*/
	ptimer->PSC=9u;  /*define the prescaller value to 10*/
	ptimer->ARR=99u; /* and the ARR (auto-reload reg) value to obtain 16 KHz in timer output*/
	
	
	ptimer->CCMR1 = 0x60u; /*select output mode + PWM mode 1 + OC preload disable + OC fast disable => all for channel 1*/
	ptimer->CCER |= 0x1u; /*enable output on OC1 (output compare channel 1)*/
	
	TIM1->EGR |=0x1u; /*generate an UEV (update event) to update registers value*/
	TIM1->BDTR |= (1u<<15); /*enable the main output MOE bit */
	
	ptimer->CCR1=abs(duty);	/* set the capture compare register: if the counter value is <= CCR1 the output is high if > the output is low*/
	ptimer->CR1 |= 0x1u; /* Enable the counter */
}

void PWM_gen3(int32_t duty){
		
	/*define direction
	------------------------------------------------------------------------*/
	if(duty<0) dir_conf3(1);
	else dir_conf3(0);
	
	/*GPIO config
	------------------------------------------------------------------------*/
	/*default settings : low speed, output in push pull, no pull-up or pull-down for the output*/
	
	__HAL_RCC_GPIOA_CLK_ENABLE(); /*Enable the clock for gpioA*/
	
	GPIO_TypeDef *pgpio; 
	pgpio = GPIOA;              /*init the structure pointer with the base_adresse of the GPIOA*/
	
	pgpio->MODER |= GPIO_MODER_MODE11_1; /*set gpioA output mode in alternate function for pin 11*/
	pgpio->AFR[1] |= (1u<<12);          /*set the alternate function AF1 for the 11th pin corresponding to TIM1 */
	
	/*Timer 1 config
	------------------------------------------------------------------------*/
	__HAL_RCC_TIM1_CLK_ENABLE(); /*Enable the clock for TIM1*/
	
	TIM_TypeDef *ptimer; 
	ptimer=TIM1;   /*init the structure pointer with the base_adresse of the TIM1*/
	
	ptimer->SMCR = 0x0u;  /*Disable the slave mode control to use the internal clock of the APB2 bus CK_INT*/
	ptimer->PSC=9u;  /*define the prescaller value to 10*/
	ptimer->ARR=99u; /* and the ARR (auto-reload reg) value to obtain 16 KHz in timer output*/
	
	
	ptimer->CCMR2 |= 0x6000u; /*select output mode + PWM mode 1 + OC preload disable + OC fast disable => all for channel 4*/
	ptimer->CCER |= (1u<<12); /*enable output on OC4 (output compare channel 4)*/
	
	TIM1->EGR |=0x1u; /*generate an UEV (update event) to update registers value*/
	TIM1->BDTR |= (1u<<15); /*enable the main output MOE bit */
	
	ptimer->CCR4=abs(duty);	/* set the capture compare register: if the counter value is <= CCR4 the output is high if > the output is low*/
	ptimer->CR1 |= 0x1u; /* Enable the counter*/
}
