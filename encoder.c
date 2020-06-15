//==========================================================================================================================
//	FILE : encoder.c
//	AUTHOR : A.LAKBIR
//==========================================================================================================================
//	DESCRIPTION : - API for handling encoder signals A and B => calculate speed of the motor and it's direction.
//								- Also apply odometry to calculate delta_x, delt_y and delta_r for each wheel by incrementing/decrementing
//									the contribution of each motor on every encoder pulse
//
//	DETAIL : Configuring timer 2, timer 3 and timer 4 in input capture mode in channel 1  for the 3 motors.
//
//						Motor 1 :
//								- Cod_A sur PA15 ---> AF : TIM2_CH1   ---> speed
//								- Cod_B sur PB3  ---> AF : GPIO input ---> direction
//						Motor 2 :
//								- Cod_A sur PB4  ---> AF : TIM3_CH1   ---> speed
//								- Cod_B sur PB5  ---> AF : GPIO input ---> direction
//						Motor 3 :
//								- Cod_A sur PB7  ---> AF : GPIO input ---> direction
//								- Cod_B sur PB6  ---> AF : TIM4_CH1   ---> speed
//==========================================================================================================================


#include "encoder.h"


#define cw1 0u   // clockwise direction value for motor 1
#define ccw1 8u  // counterclockwise direction value for motor 1
#define cw2 0u   // clockwise direction value for motor 2
#define ccw2 32u // counterclockwise direction value for motor 2
#define cw3 128u // clockwise direction value for motor 3
#define ccw3 0u  // counterclockwise direction value for motor 3

#define encoder_resolution 700.0f
#define circonference 188.5f  //wheel circumference
#define R_robot 110.0f //length of each wheel axis
#define 2pi 6.28318f


GPIO_TypeDef *pgpioa,*pgpiob,*pgpioc;
TIM_TypeDef *ptimer2,*ptimer3,*ptimer4;
uint32_t volatile count1=1000000u;
uint32_t dir1=0,i1=0;
uint32_t volatile count2=1000000u;
uint32_t dir2=0,i2=0;
uint32_t volatile count3=1000000u;
uint32_t dir3=0,i3=0;
int32_t pulse1=0u,pulse2=0u, pulse3=0;
float delta_x=0.0f, delta_y=0.0f, delta_r=0.0f;



void encoder_handler2(void){
		//----GPIO configuration PB4 and PB5------------------------------------------------------------------------------
				/*default settings: pin config : push-pull , no pull-up pull-down; speed :low*/
			__HAL_RCC_GPIOB_CLK_ENABLE(); /*Enable the clock for GPIOB*/
			
			//Pin PB4 settings
			pgpiob = GPIOB;
			pgpiob->MODER |= GPIO_MODER_MODE4_1;  /*set GPIOB pin 4 in alternate mode*/
			pgpiob->AFR[0] |= (2u<<16); /*Set the alternate function AF2 corresponding to TIM3*/
		 
			//Pin PB5 settings
			pgpiob->MODER &= ~(3u<<10);  /*set GPIOB pin 5 in input mode*/
			
		//----Timer 3 configuration------------------------------------------------------------------------------------------
			if((ptimer3->CR1 & 0x1u) == 0u){   /*if timer not already enabled*/
				__HAL_RCC_TIM3_CLK_ENABLE(); /*Enable the clock for TIMER 3*/
				ptimer3 = TIM3;
				
				ptimer3->PSC=3u; /*  CK_CNT = CK_PSC/4  = 4 MHz*/
				ptimer3->ARR=65535u; /* setting the max value of the counter to it's max 2^16*/
				
				//channel 1 settings
				ptimer3->CCMR1 |= 0x1u; /* setting input capture mode in TI1 channel 1 (connect TI1 to IC1)*/
				ptimer3->CCER |= 0x1u; /* enable input on TI1 */
				ptimer3->CCER &= ~(1u<<1); /* set TI1 input polarity to active high CC1NP/CC1P = 00  non-inverted/rising edge detection*/
				
				ptimer3->CR1 |= (1u<<2); /*set the URS bit : generate an interrupt only when the counter overflow the ARR reg*/
				ptimer3->DIER |= (1u<<1); /* enable interrupt from TIM3 on channel1*/
				
				ptimer3->SMCR |= (1u<<2); /* enable Slave Reset mode : to reset the counter */
				ptimer3->SMCR |= (5u<<4); /* TS = 101 => select TI1FP1 filtered signal => configure the reset event on every rising edge of TI1 */
				
				NVIC->ISER[0] |= (1u<<29); /*Enable TIM3 interrupt in the NVIC table*/
				
				ptimer3->EGR |=0x1u; /*generate an UEV to update registers*/
				ptimer3->CR1 |= 0x1u; /*start the counter*/
			}
}


void encoder_handler3(void){
		//----GPIO configuration PB6 and PB7------------------------------------------------------------------------------
				/*default settings : pin config : push-pull , no pull-up pull-down; speed :low*/
			__HAL_RCC_GPIOB_CLK_ENABLE(); /*Enable the clock for GPIOB*/
			
			//Pin PB6 settings
			pgpiob = GPIOB;
			pgpiob->MODER |= GPIO_MODER_MODE6_1;  /*set GPIOB pin 6 in alternate mode*/
			pgpiob->AFR[0] |= (2u<<24); /*Set the alternate function AF2 corresponding to TIM4*/
		 
			//Pin PB7 settings
			pgpiob->MODER &= ~(3u<<14);  /*set GPIOB pin 7 in input mode*/
			
		//----Timer 4 configuration------------------------------------------------------------------------------------------
			if((ptimer4->CR1 & 0x1u) == 0u){
				__HAL_RCC_TIM4_CLK_ENABLE(); /*Enable the clock for TIMER 4*/
				ptimer4 = TIM4;
				
				ptimer4->PSC=3u; /*  CK_CNT = CK_PSC/4  = 4 MHz*/
				ptimer4->ARR=65535u; /* setting the max value of the counter to it's max 2^16*/
				
				//channel 1 settings
				ptimer4->CCMR1 |= 0x1u; /* setting input capture mode in TI1 channel 1 (connect TI1 to IC1)*/
				ptimer4->CCER |= 0x1u; /* enable input on TI1 */
				ptimer4->CCER &= ~(1u<<1); /* set TI1 input polarity to active high CC1NP/CC1P = 00  non-inverted/rising edge detection*/
				
				ptimer4->CR1 |= (1u<<2); /*set the URS bit : don't generate an interrupt when the counter overflow the ARR reg*/
				ptimer4->DIER |= (1u<<1); /* enable interrupt from TIM4 on channel1*/
				
				ptimer4->SMCR |= (1u<<2); /* enable Slave Reset mode : to reset the counter */
				ptimer4->SMCR |= (5u<<4); /* TS = 101 => select TI1FP1 filtered signal => configure the reset event on every rising edge of TI1 */
				
				NVIC->ISER[0] |= (1u<<30); /*Enable TIM4 interrupt in the NVIC table*/
				
				ptimer4->EGR |=0x1u; /*generate an UEV to update registers*/
				ptimer4->CR1 |= 0x1u; /*start the counter*/
			}
}


void encoder_handler1(void){
		//----GPIO configuration PA15 and PB3------------------------------------------------------------------------------
				/*default settings : pin config : push-pull , no pull-up pull-down; speed :low*/
			__HAL_RCC_GPIOB_CLK_ENABLE(); /*Enable the clock for GPIOB*/
			__HAL_RCC_GPIOA_CLK_ENABLE(); /*Enable the clock for GPIOA*/
			
			//Pin PA15 settings
			pgpioa = GPIOA;
			pgpioa->MODER |= GPIO_MODER_MODE15_1;  /*set GPIOA pin 15 in alternate mode*/
			pgpioa->AFR[1] |= (1u<<28); /*Set the alternate function AF1 corresponding to TIM2*/
		 
			//Pin PB3 settings
			pgpiob = GPIOB;
			pgpiob->MODER &= ~(3u<<6) ; /*set GPIOB pin 3 in input mode*/
			
		//----Timer 2 configuration------------------------------------------------------------------------------------------
			if((ptimer2->CR1 & 0x1u) == 0u){
				__HAL_RCC_TIM2_CLK_ENABLE(); /*Enable the clock for TIMER 2*/
				ptimer2 = TIM2;
				
				ptimer2->PSC=3u; /*  CK_CNT = CK_PSC/4  = 4 MHz*/
				ptimer2->ARR=65535u; /* setting the max value of the counter to its max 2^16*/
				
				//channel 1 settings
				ptimer2->CCMR1 |= 0x1u; /* setting input capture mode in TI1 channel 1 (connect TI1 to IC1)*/
				ptimer2->CCER |= 0x1u; /* enable input on TI1 */
				ptimer2->CCER &= ~(1u<<1); /* set TI1 input polarity to active high CC1NP/CC1P = 00  non-inverted/rising edge detection*/
				
				ptimer2->CR1 |= (1u<<2); /*set the URS bit : set the UIF flag only when the counter overflow the ARR reg*/
				ptimer2->DIER |= (1u<<1); /* enable interrupt from TIM2*/
				
				ptimer2->SMCR |= (1u<<2); /* enable Slave Reset mode : to reset the counter */
				ptimer2->SMCR |= (5u<<4); /* TS = 101 => select TI1FP1 filtered signal => configure the reset event on every rising edge of TI1 */
				
				NVIC->ISER[0] |= (1u<<28); /*Enable TIM2 interrupt*/
				
				ptimer2->EGR |=0x1u; /*generate an UEV to update registers*/
				ptimer2->CR1 |= 0x1u; /*start the counter*/
			}
}

/*Motor 1 
---------------------------------------------------------------------------------*/

void TIM2_IRQHandler(void){
	
		if((ptimer2->SR & 0x1u) == 0x1u){				/* verify if it’s a counter overflow interrupt */
				i1++;   														/* count how much the overflow occurred */
				count1 = i1*65535u + ptimer2->CCR1; /* correct the measured value by adding the counter max to the current value captured in the CCR register */
				ptimer2->SR &= ~(1u<<0); 						/* Clear interruption flag in Status Reg*/
		}
		else if((ptimer2->SR & (1u<<1)) == (1u<<1)){ /* Verify if it’s a capture interrupt */
		i1=0; 																	/* reset the overflow count */
		count1 = ptimer2->CCR1;  								/* Read the value of the counter captured in the CCR reg at the rising edge */
		dir1 = (pgpiob->IDR & (1u<<3)); 				/* read the input in PC8 : the second encoder signal, at the rising edge to know the direction of rotation*/
		ptimer2->SR &= ~(1u<<1); 								/* Clear interruption flag in status Reg*/
		}
		if(dir1 == cw1) pulse1++;								/* keep track of the number of pluses occurred*/
		else pulse1--;
		
		/*calcule de position*/
		
		if(dir1 == cw1){
				delta_r += 1/(3*R_robot) * 2pi/729.69f;   /*for motor 1 odometry calibration gives 729.69 impulsions for one revolution*/
				delta_x += -0.6666f * sin(delta_r) * circonference/729.69f;
				delta_y += 0.6666f * cos(delta_r) * circonference/729.69f;
		}
		else if(dir1 == ccw1){
				delta_r -= 1/(3*R_robot) * 2pi/729.69f;
				delta_x -= -0.6666f * sin(delta_r) * circonference/729.69f;
				delta_y -= 0.6666f * cos(delta_r) * circonference/729.69f;
		}
}

/*Motor 2 
---------------------------------------------------------------------------------*/

void TIM3_IRQHandler(void){

		if((ptimer3->SR & 0x1u) == 0x1u){				/* verify if it’s a counter overflow interrupt */
				i2++;   														/* count how much the overflow occurred */
				count2 = i2*65535u + ptimer3->CCR1; /* correct the measured value by adding the counter max to the current value captured in the CCR register */
				ptimer3->SR &= ~(1u<<0); 						/* Clear interruption flag in status Reg*/
		}
		else if((ptimer3->SR & (1u<<1)) == (1u<<1)){ /* Verify if it’s a capture interrupt */
		i2=0; 																	/* reset the overflow count */
		count2 = ptimer3->CCR1;  								/* Read the value of the counter captured in the CCR reg at the rising edge */
		dir2 = (pgpiob->IDR & (1u<<5));					/* read the input in PB5 : the second encoder signal, at the rising edge to know the direction of rotation*/
		ptimer3->SR &= ~(1u<<1);								/* Clear interruption flag in status Reg*/
		}
		if(dir2 == cw2) pulse2++;								/* keep track of the number of pluses occurred*/
		else pulse2--;
		
			/*calcule de position*/
		
		if(dir2 == cw2){
			delta_r += 1/(3*R_robot) * 2pi/730.24f; 			/*for motor 2 odometry calibration gives 730.24 impulsions for one revolution*/
			delta_x += (0.3333f*sin(delta_r)-0.5773f*cos(delta_r)) * circonference/730.24f;
			delta_y += (-0.3333f*cos(delta_r)-0.5773f*sin(delta_r)) * circonference/730.24f;
		}
		else if(dir2 == ccw2){
			delta_r -= 1/(3*R_robot) * 2pi/730.24f;
			delta_x -= (0.3333f*sin(delta_r)-0.5773f*cos(delta_r)) * circonference/730.24f;
			delta_y -= (-0.3333f*cos(delta_r)-0.5773f*sin(delta_r)) * circonference/730.24f;
		}
}

/*Motor 3 
---------------------------------------------------------------------------------*/
void TIM4_IRQHandler(void){

		if((ptimer4->SR & 0x1u) == 0x1u){				/* verify if it’s a counter overflow interrupt */
			i3++;   															/* count how much the overflow occurred */
				count3 = i3*65535u + ptimer4->CCR1; /* correct the measured value by adding the counter max to the current value captured in the CCR register */
				ptimer4->SR &= ~(1u<<0); 						/* Clear interruption flag in status Reg*/
		}
		else if((ptimer4->SR & (1u<<1)) == (1u<<1)){ /* Verify if it’s a capture interrupt */
		i3=0; 																	/* reset the overflow count */
		count3 = ptimer4->CCR1;  								/* Read the value of the counter captured in the CCR reg at the rising edge */
		dir3 = (pgpiob->IDR & (1u<<7));					/* read the input in PB7 : the second encoder signal, at the rising edge to know the direction of rotation*/
		ptimer4->SR &= ~(1u<<1); 								/* Clear interruption flag in status Reg*/
		}
		if(dir3 == cw3) pulse3++;								/* keep track of the number of pluses occurred*/
		else pulse3--;
		
		/*calcule de position*/
		
		if(dir3 == cw3){
			delta_r += 1/(3*R_robot) * 2pi/730.31f;  /*for motor 3 odometry calibration gives 730.31 impulsions for one revolution*/
			delta_x += (0.3333f*sin(delta_r)+0.5773f*cos(delta_r)) * circonference/730.31f;
			delta_y += (-0.3333f*cos(delta_r)+0.5773f*sin(delta_r)) * circonference/730.31f;
		}
		else if(dir3 == ccw3){
			delta_r -= 1/(3*R_robot) * 2pi/730.31f;
			delta_x -= (0.3333f*sin(delta_r)+0.5773f*cos(delta_r)) * circonference/730.31f;
			delta_y -= (-0.3333f*cos(delta_r)+0.5773f*sin(delta_r)) * circonference/730.31f;
		}
}
