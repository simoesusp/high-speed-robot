#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <softPwm.h>

void setup(){
	wiringPiSetup();
	pinMode(0, OUTPUT);
	pinMode(1, OUTPUT);
	pinMode(2, OUTPUT);
	pinMode(3, OUTPUT);
	pinMode(4, OUTPUT);
	pinMode(5, OUTPUT);
	pinMode(6, OUTPUT);
	pinMode(7, OUTPUT);
	//pinMode(40, OUTPUT);
	//pinMode(45, OUTPUT);
	
	digitalWrite(0, HIGH);
	digitalWrite(1, HIGH);
	digitalWrite(2, HIGH);
	digitalWrite(3, HIGH);
	digitalWrite(4, HIGH);
	digitalWrite(5, HIGH);
	digitalWrite(6, HIGH);
	digitalWrite(7, HIGH);
	/*pwmSetMode(PWM_MODE_MS);
	pwmSetRange(1024);
	pwmSetClock(375);
	pwmWrite(40,36);
	pwmWrite(45,128);*/
	/*softPwmCreate(40,0,200);
	softPwmWrite(40,195);
	softPwmCreate(45,0,200);
	softPwmWrite(45,195);*/
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