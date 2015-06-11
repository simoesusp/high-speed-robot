#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <softPwm.h>

void setup(){
	wiringPiSetupGpio();
	
	pwmSetMode(PWM_MODE_MS);
	pwmSetRange(1024);
	pwmSetClock(375);
	pwmWrite(40,0);
	pwmWrite(45,1024);
}

int main(void){
	setup();
	while(1){
		//digitalWrite(40, HIGH);
		//digitalWrite(45, HIGH);
		//softPwmWrite(40, 50);
		//softPwmWrite(45, 100);
			/*if(digitalRead(4)==HIGH){
			printf("4");
		}
		if(digitalRead(5)==HIGH){
			printf("5");
		}*/
		
	}
}