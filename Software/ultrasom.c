#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <string.h>
 
 FILE *fp;
 
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
 
 
void getCM() {		
		//Send trig pulse
        digitalWrite(TRIG, HIGH);
        delayMicroseconds(20);
        digitalWrite(TRIG, LOW);
		long startTime = micros();
 
        //Wait for echo start
        while(digitalRead(ECHO) == LOW);
		long lowTime = micros() - startTime;
        //Wait for echo end
        
        while(digitalRead(ECHO) == HIGH);
        long travelTime = micros() - startTime -lowTime;
 
        //Get distance in cm
        int alto = travelTime / 58;
		int baixo = lowTime /58;
 
        printf("%d\t",alto);
		printf("%d\n",baixo);
		printf("\n");
		fprintf(fp,"%d\t",alto);
		fprintf(fp,"%d\n",baixo);
}
 
int main(void) {		
		setup();
		
		char str[20];
		printf("Nome do arquivo: ");
		fgets(str,20,stdin);
		if(str[strlen(str)-1]=='\n'){str[strlen(str)-1]='\0';}
		strcat(str,".txt");
		puts(str);
		
		fp = fopen(str,"w");
		
		while(1){
			delay(100);
			getCM(fp);
			pwmWrite(MOTOR0,965);
			pwmWrite(MOTOR1,964);
		}
        return 0;
}