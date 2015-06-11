#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <string.h>

//FILE *fp;
 
#define TRIG 23
#define ECHO 24 

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
		
		
		
		//Wait for echo end
        /*while(digitalRead(ECHO) == HIGH && count < 30){
			if(digitalRead(ECHO) == LOW){
				count++;
			}else{
				count=0;
			}
		}
        long travelTime = micros() - startTime;
		*/
		
		
		
		
        //Get distance in cm
        int distance = travelTime / 58;
 
        printf("%d\n",distance);
		printf("\n");
		//fprintf(fp,"%d\n",distance);
}
 
int main(void) {		
		setup();
		
		pwmWrite(MOTOR0,965);
		pwmWrite(MOTOR1,964);
		
		char str[20];
		printf("Nome do arquivo: ");
		fgets(str,20,stdin);
		if(str[strlen(str)-1]=='\n'){str[strlen(str)-1]='\0';}
		strcat(str,".txt");
		puts(str);
		
		//fp = fopen(str,"w");
		
		while(1){
			delay(100);
			getCM();
		}
        return 0;
}