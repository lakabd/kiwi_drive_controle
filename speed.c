//==========================================================================================================================
//	FILE : speed.c
//	AUTHOR : A.LAKBIR
//==========================================================================================================================
//	DESCRIPTION : API implementing a PI controller for speed control			
//	DETAIL : Configuring a sampling time T_e using TIM5 and implement the PI controller in the interruption routine
//
//==========================================================================================================================

#include "speed.h"

#define cw1 0u   // clockwise direction value for motor 1
#define ccw1 8u  // counterclockwise direction value for motor 1
#define cw2 0u   // clockwise direction value for motor 2
#define ccw2 32u // counterclockwise direction value for motor 2
#define cw3 128u // clockwise direction value for motor 3
#define ccw3 0u  // counterclockwise direction value for motor 3

extern uint32_t count1,count2,count3;
extern uint32_t dir1,dir2,dir3;
TIM_TypeDef *ptimer5;
float ref1=0.0f,mesure1=0.0f;
float ref2=0.0f,mesure2=0.0f;
float ref3=0.0f,mesure3=0.0f;
float u1=0.0f,Kp1=0.0f,Kd1=0.0f,Ki1=0.0f,sum1=0.0f,e1=0.0f;
float u2=0.0f,Kp2=0.0f,Kd2=0.0f,Ki2=0.0f,sum2=0.0f,e2=0.0f;
float u3=0.0f,Kp3=0.0f,Kd3=0.0f,Ki3=0.0f,sum3=0.0f,e3=0.0f;
float speed1,speed2,speed3;


/*this function converts speed from RPS to encoder impulsion per sec*/
uint16_t speed_to_count(float speed){ /*speed in RPS*/
	float  counter = ((1/speed)/699.0f)/0.00000025f;
	return (uint16_t)counter;
}

/*this function converts encoder impulsion per sec to speed in RPS */
float count_to_speed(uint32_t counter){ /*counter impulsion*/
	float speed = (float)(1.0f/(699.0f*0.00000025f*(float)counter));
	return speed;
}


/* run motors on the defined speed in RPS*/
void speed_def(float vitesse1,float vitesse2,float vitesse3){ // speed_def(speed_m1,speed_m2,speed_m3)
	
	if(vitesse1!=speed1)speed1=vitesse1;
	if(vitesse2!=speed2)speed2=vitesse2;
	if(vitesse3!=speed3)speed3=vitesse3;
	
	/*motor 1
	------------------------------------------------------------------------------------------------*/
	
	/*Set direction*/
	if(vitesse1<0) dir_conf1(1);
	else dir_conf1(0);
	
	/*call encoder handler to measure motor speed feedback*/
		encoder_handler1();
	/*set reference speed*/
		ref1= fabs(vitesse1);
		Kp1 = 35;
		/*output = output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start)*/
		/*maping speed [0.5 - 3.5] ----> Ki [1/6 - 1/2.8]   or map  [0, 3.5 - 0.5] ----> [0, 1/2.8 - 1/6]*/
		Ki1=(float) 1.0f/(1.0f/6.0f + (1.0f/4.0f-1.0f/6.0f)*(ref1 - 0.5f)/3.0f);
		
	/*handle motor 1 null speed*/
	if(vitesse1 == 0){
		Ki1=0;		/*disable integrator effect to prevent oscillations*/
		Kp1=70;
	}
	
	/*motor 2
	------------------------------------------------------------------------------------------------*/
	
	/*Set direction*/
	if(vitesse2<0) dir_conf2(1);
	else dir_conf2(0);

	/*call encoder handler to measure motor speed feedback*/
		encoder_handler2();
	/*set reference speed*/
		ref2= fabs(vitesse2);
		Kp2 = 35;
		/*output = output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start)*/
		/*maping speed [0.5 - 3.5] ----> Ki [1/6 - 1/2.8]   or map  [0, 3.5 - 0.5] ----> [0, 1/2.8 - 1/6]*/
		Ki2=(float) 1.0f/(1.0f/6.0f + (1.0f/4.0f-1.0f/6.0f)*(ref2 - 0.5f)/3.0f);
	
	/*handle motor 2 null speed*/
	if(vitesse2 == 0){
		Ki2=0; 		/*disable integrator effect to prevent oscillations*/
		Kp2=70;
	}
	
	/*motor 3
	------------------------------------------------------------------------------------------------*/

	/*define direction*/
	if(vitesse3<0) dir_conf3(1);
	else dir_conf3(0);

	/*call encoder handler to measure motor speed feedback*/
		encoder_handler3();
	/*set reference speed*/
		ref3= fabs(vitesse3);
		Kp3 = 35;
		/*output = output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start)*/
		/*maping speed [0.5 - 3.5] ----> Ki [1/6 - 1/2.8]   or map  [0, 3.5 - 0.5] ----> [0, 1/2.8 - 1/6]*/
		Ki3=(float) 1.0f/(1.0f/6.0f + (1.0f/4.0f-1.0f/6.0f)*(ref3 - 0.5f)/3.0f);
		
	/*handle motor 3 null speed*/
	if(vitesse3 == 0){
		Ki3=0; 		/*disable integrator effect to prevent oscillations*/
		Kp3=70;
	}

	/*configure timer 5 channel 1 in basic mode
	------------------------------------------------------------------------------------------------*/
	if((ptimer5->CR1 & 0x1u) == 0u){
	__HAL_RCC_TIM5_CLK_ENABLE(); /*Enable TIM5 clock*/
	ptimer5 = TIM5; 
	
	ptimer5->SMCR = 0x0u;/*disable slave mode to use internal clock*/
	ptimer5->PSC = 15999u; /*set the prescaller to 16KHz so the CK_CNT = 16MHz/16KHz = 1KHz*/
	ptimer5->ARR = 5u;/*set sampling time to 5 ms*/
	
	ptimer5->DIER |= 0x1u;/*enable channel 1 counter to generate interrupts*/
	NVIC->ISER[1] |= (1u<<18); /*Enable TIM5 interrupt in NVIC table*/
	NVIC->IP[50]=0x31u;
	ptimer5->CR1 |= 0x1u; /*start the counter*/
	}
}


void TIM5_IRQHandler(void){
	
		/*motor 1
		------------------------------------------------------------------------------------------------*/
	mesure1 = count_to_speed(count1);		/* current speed measurement */
	if(speed1>0){												/* if the reference speed is positive that means direction of rotation in clockwise */
		e1 = ref1 - mesure1;					
		if(dir1==ccw1) e1=ref1+mesure1;		/* if the current direction of rotation is counterclockwise change the error equation */
	}
	else{ 															/* if the reference speed is negative that means direction of rotation in counterclockwise */
		e1 = mesure1 - ref1; 		
		if(dir1==cw1) e1=-ref1-mesure1;		/* if the current direction of rotation is clockwise change the error equation */
	}
	sum1 += e1; 												/* integrate the error or accumulate the current error with the previous ones */
	u1 = Kp1*e1+Ki1*sum1; 							/* calculate the command by adding proportional term and integral term together */
	PWM_gen1((int32_t)u1); 							/* apply the command to the motor */
	
		/*motor 2
		------------------------------------------------------------------------------------------------*/

	mesure2 = count_to_speed(count2);		/* current speed measurement */
	if(speed2>0){												/* if the reference speed is positive that means direction of rotation in clockwise */
		e2 = ref2 - mesure2;					
		if(dir2==ccw2) e2=ref2+mesure2;		/* if the current direction of rotation is counterclockwise change the error equation */
	}
	else{ 															/* if the reference speed is negative that means direction of rotation in counterclockwise */
		e2 = mesure2 - ref2; 							
		if(dir2==cw2) e2=-ref2-mesure2;		/* if the current direction of rotation is clockwise change the error equation */
	}
	sum2 += e2; 												/* integrate the error or accumulate the current error with the previous ones */
	u2 = Kp2*e2+Ki2*sum2; 							/* calculate the command by adding proportional term and integral term together */
	PWM_gen2((int32_t)u2); 							/* apply the command to the motor */

	
		/*motor 3
		------------------------------------------------------------------------------------------------*/

	mesure3 = count_to_speed(count3);		/* current speed measurement */
	if(speed3>0){												/* if the reference speed is positive that means direction of rotation in clockwise */
		e3 = ref3 - mesure3;					
		if(dir3==ccw3) e3=ref3+mesure3;		/* if the current direction of rotation is counterclockwise change the error equation */
	}
	else{ 															/* if the reference speed is negative that means direction of rotation in counterclockwise */
		e3 = mesure3 - ref3; 					
		if(dir3==cw3) e3=-ref3-mesure3;		/* if the current direction of rotation is clockwise change the error equation */
	}
	sum3 += e3; 												/* integrate the error or accumulate the current error with the previous ones */
	u3 = Kp3*e3+Ki3*sum3; 							/* calculate the command by adding proportional term and integral term together */
	PWM_gen3((int32_t)u3); 							/* apply the command to the motor */

	/*------------------------------------------------------------------------------------------------*/
	
	ptimer5->SR &= ~(1u<<0); /*Clear interruption flag in status Reg*/
}
