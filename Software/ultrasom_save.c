#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <string.h>
 
 
#define TRIG 17
#define ECHO 18 

#define MOTOR0 40
#define MOTOR1 45
 
void setup() {
        wiringPiSetupGpio();
        pinMode(TRIG, OUTPUT);
        pinMode(ECHO, INPUT);
 
        //TRIG pin must start LOW
        digitalWrite(TRIG, LOW);
        delay(30);
		
		//seta motor
		pwmSetMode(PWM_MODE_MS);
		pwmSetRange(1024);
		pwmSetClock(375);
		pwmWrite(MOTOR0,975);
		pwmWrite(MOTOR1,975);
}
 
int getCM() {		
		//Send trig pulse
        digitalWrite(TRIG, HIGH);
        delayMicroseconds(20);
        digitalWrite(TRIG, LOW);
 
        //Wait for echo start
        while(digitalRead(ECHO) == LOW);
 
        //Wait for echo end
        long startTime = micros();
        while(digitalRead(ECHO) == HIGH);
        long travelTime = micros() - startTime;
 
        //Get distance in cm
        int distance = travelTime / 58;
 
        return distance;
}
 
int main(void) {
		
		int distancia=100;
		int count=0;
		
		char str[20];
		printf("Nome do arquivo: ");
		fgets(str,20,stdin);
		if(str[strlen(str)-1]=='\n'){str[strlen(str)-1]='\0';}
		strcat(str,".txt");
		puts(str);
		
		FILE *fp;
		fp = fopen(str,"w");
		
		setup();
		
		pwmWrite(MOTOR0,965);
		pwmWrite(MOTOR1,964);
		
		while(count < 101){
			delay(100);
			distancia=getCM();
			printf("Distance: %dcm\n", distancia);
			fprintf(fp,"%d\n",distancia);
			count++;
		}
		fclose(fp);
        return 0;
}