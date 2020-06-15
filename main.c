#include "stm32f4xx_hal.h"              // Keil::Device:STM32Cube HAL:Common
#include "encoder.h"
#include "pwm.h"
#include "speed.h"
#include<math.h>


#define encoder_resolution 700.0f
#define circonference 188.5f
#define R_robot 110.0f

extern float delta_x, delta_y, delta_r;
TIM_TypeDef *ptimer6;
float w1=0.0f,w2=0.0f,w3=0.0f,ref_x=0.0f,ref_y=0.0f,ref_r=0.0f,X=0.0f, Y=0.0f,R=0.0f;
float angle_dir=0.0f;

void position_def(float x, float y,float r){
	delta_x=0;
	delta_y=0;
	delta_r=0;
	
	/*define direction --------------------------------------------*/
	angle_dir = (float) acos(x/sqrt(x*x+y*y)); /*calculate the angle between the x axis end the vector (x,y)*/
	/*scale the value of X and Y*/
	if(x!=0){
		X = x>0?1.0f:-1.0f;
		Y = (float) tan(angle_dir)/fabs(X);
	}
	else{
		X=0;
		Y=y>0?1.0f:-1.0f;
	}
	/*calcuate motors speed correpending to the direction (X,Y)*/
	w1 = Y;
	w2 = -0.866f*X - 0.5f*Y;
	w3 = 0.866f*X - 0.5f*Y;
	
	/*set speed*/
	speed_def(w1,w2,w3);
}

uint32_t ticks=0;

int main(){
	SystemCoreClockUpdate(); /*running on internal clock 16MHz*/
	ref_x=1000;
	ref_y=0;
	position_def(ref_x,ref_y,ref_r);

	while(1);
}
