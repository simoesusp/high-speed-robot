#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <string.h>
 
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
		
		pwmSetMode(PWM_MODE_MS);
		pwmSetRange(1024);
		pwmSetClock(375);
		pwmWrite(MOTOR0,975);
		pwmWrite(MOTOR1,975);
		
        pinMode(TRIG0, OUTPUT);
        pinMode(ECHO0, INPUT);
		
		pinMode(TRIG1, OUTPUT);
        pinMode(ECHO1, INPUT);
		
		pinMode(TRIG2, OUTPUT);
        pinMode(ECHO2, INPUT);
		
        //TRIG pin must start LOW
        digitalWrite(TRIG1, LOW);
		digitalWrite(TRIG2, LOW);
		digitalWrite(TRIG0, LOW);
        delay(30);
}
 
int getCM1() {		
		//Send trig pulse
        digitalWrite(TRIG1, HIGH);
        delayMicroseconds(20);
        digitalWrite(TRIG1, LOW);
 
        //Wait for echo start
        while(digitalRead(ECHO1) == LOW);
 
        //Wait for echo end
        long startTime = micros();
        long travelTime = micros() - startTime;
		while(digitalRead(ECHO1) == HIGH && travelTime <= 19140){
        travelTime = micros() - startTime;
		}
        //Get distance in cm
        int distance = travelTime / 58;
 
        return distance;
}

int getCM2() {		
		//Send trig pulse
        digitalWrite(TRIG2, HIGH);
        delayMicroseconds(20);
        digitalWrite(TRIG2, LOW);
 
        //Wait for echo start
        while(digitalRead(ECHO2) == LOW);
 
        //Wait for echo end
        long startTime = micros();
        long travelTime = micros() - startTime;
		while(digitalRead(ECHO2) == HIGH && travelTime <= 19140){
        travelTime = micros() - startTime;
		}
        //Get distance in cm
        int distance = travelTime / 58;
 
        return distance;
}

int getCM0() {		
		//Send trig pulse
        digitalWrite(TRIG0, HIGH);
        delayMicroseconds(20);
        digitalWrite(TRIG0, LOW);
 
        //Wait for echo start
        while(digitalRead(ECHO0) == LOW);
 
        //Wait for echo end
        long startTime = micros();
        long travelTime = micros() - startTime;
		while(digitalRead(ECHO0) == HIGH && travelTime <= 19140){
        travelTime = micros() - startTime;
		}
        //Get distance in cm
        int distance = travelTime / 58;
 
        return distance;
}
 
int main(void) {
		
		int distancia0=100, distancia1=100, distancia2=100;
		int m1=975, m0=975;
		
		char str[20];
		printf("Nome do arquivo: ");
		fgets(str,20,stdin);
		if(str[strlen(str)-1]=='\n'){str[strlen(str)-1]='\0';}
		strcat(str,".txt");
		puts(str);
		
		FILE *fp;
		fp = fopen(str,"w");
		
		setup();
		while(distancia1 > 10){
			
			distancia0=getCM0();
			delay(50);
			distancia1=getCM1();
			delay(50);
			distancia2=getCM2();
			delay(50);
			
			if (distancia1 < 25){
				m0=975;
				m1=975;
			}else if (distancia0 < 50 && distancia2 > 50){
				m0=970;
				m1=964;
			}else if (distancia2 < 50 && distancia0 > 50){
				m0=965;
				m1=970;
			}else if(distancia0 < 50 && distancia2 < 50){
				m0=970;
				m1=970;
			}else{
				m0=965;
				m1=964;
			}
			
			pwmWrite(MOTOR0,m0);
			pwmWrite(MOTOR1,m1);
			
			printf("Distance0: %dcm\n", distancia0);
			fprintf(fp,"%d\t",distancia0);
			printf("Distance1: %dcm\n", distancia1);
			fprintf(fp,"%d\t",distancia1);
			printf("Distance2: %dcm\n", distancia2);
			fprintf(fp,"%d\t",distancia2);
			printf("\n");
			fprintf(fp,"%d\t",m0);
			fprintf(fp,"%d\n",m1);
			
		}
		fclose(fp);
        return 0;
}
