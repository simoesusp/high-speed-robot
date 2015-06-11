#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <string.h>

//FILE *fp;
 
#define TRIG0 17
#define ECHO0 18
 
#define TRIG1 27
#define ECHO1 22 
 
#define TRIG2 23
#define ECHO2 24  

#define MOTOR0 40
#define MOTOR1 45
 
void setup() {
        wiringPiSetupGpio();
        pinMode(TRIG0, OUTPUT);
        pinMode(ECHO0, INPUT);
		pinMode(TRIG1, OUTPUT);
        pinMode(ECHO1, INPUT);
		pinMode(TRIG2, OUTPUT);
        pinMode(ECHO2, INPUT);
 
        //TRIG pin must start LOW
        digitalWrite(TRIG0, LOW);
		digitalWrite(TRIG1, LOW);
		digitalWrite(TRIG2, LOW);
        delay(30);
		
		//seta motor
		pwmSetMode(PWM_MODE_MS);
		pwmSetRange(1024);
		pwmSetClock(375);
		pwmWrite(MOTOR0,975);
		pwmWrite(MOTOR1,975);
}
 
void getCM(int TRIG, int ECHO) {		
		int count = 0;
		
		//Send trig pulse
        digitalWrite(TRIG, HIGH);
        delayMicroseconds(20);
        digitalWrite(TRIG, LOW);

        //Wait for echo start
		printf("ta baixo    ");
        while(digitalRead(ECHO) == LOW);
		long startTime = micros();
		long travelTime;
		printf("ta alto   ");
        
	  loop: while(digitalRead(ECHO) == HIGH){
		travelTime = micros() - startTime;
		if (travelTime >= 19140 ) goto sai;
	  }
		count=0;
		printf("passou   ");
		while(count<30){
			count++;
			if (digitalRead(ECHO) == HIGH) goto loop;
		}
	  sai:;
		
		//Get distance in cm
        int distance = travelTime / 58;
 
        printf("%d",distance);
		//printf("\n");
		//fprintf(fp,"%d\n",distance);
}
 
int main(void) {		
		setup();
		
		//pwmWrite(MOTOR0,965);
		//pwmWrite(MOTOR1,964);
		
		//char str[20];
		//printf("Nome do arquivo: ");
		//fgets(str,20,stdin);
		//if(str[strlen(str)-1]=='\n'){str[strlen(str)-1]='\0';}
		//strcat(str,".txt");
		//puts(str);
		
		//fp = fopen(str,"w");
		
		while(1){
			
			printf("\nSENSOR0: ");
			getCM(TRIG0, ECHO0);
			printf("\nSENSOR1: ");
			getCM(TRIG1, ECHO1);
			printf("\nSENSOR2: ");
			getCM(TRIG2, ECHO2);
			delayMicroseconds(2000000);
		}
        return 0;
}