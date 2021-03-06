#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <string.h>

FILE *fp;
 
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
 
void getCM1() {
		printf("sensor1\n");
		int count = 0;
		
		//Send trig pulse
        digitalWrite(TRIG1, HIGH);
        delayMicroseconds(20);
        digitalWrite(TRIG1, LOW);

        //Wait for echo start
		printf("ta baixo1    ");
        while(digitalRead(ECHO1) == LOW);
		long startTime = micros();
		long travelTime;
		printf("ta alto1   ");
        
	  loop: while(digitalRead(ECHO1) == HIGH){
		travelTime = micros() - startTime;
		if (travelTime >= 19140 ) goto sai;
	  }
		count=0;
		printf("passou1   ");
		while(count<30){
			count++;
			if (digitalRead(ECHO1) == HIGH) goto loop;
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
		//fprintf(fp,"%d\t",distance);
}

void getCM0() {		
		printf("sensor0\n");
		int count = 0;
		
		//Send trig pulse
        digitalWrite(TRIG0, HIGH);
        delayMicroseconds(20);
        digitalWrite(TRIG0, LOW);

        //Wait for echo start
		printf("ta baixo0    ");
        while(digitalRead(ECHO0) == LOW);
		long startTime = micros();
		long travelTime;
		printf("ta alto0   ");
        
	  loop: while(digitalRead(ECHO0) == HIGH){
		travelTime = micros() - startTime;
		if (travelTime >= 19140 ) goto sai;
	  }
		count=0;
		printf("passou0   ");
		while(count<30){
			count++;
			if (digitalRead(ECHO0) == HIGH) goto loop;
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
		//fprintf(fp,"%d\t",distance);
}

void getCM2() {		
		printf("sensor2\n");
		int count = 0;
		
		//Send trig pulse
        digitalWrite(TRIG2, HIGH);
        delayMicroseconds(20);
        digitalWrite(TRIG2, LOW);

        //Wait for echo start
		printf("ta baixo2    ");
        while(digitalRead(ECHO2) == LOW);
		long startTime = micros();
		long travelTime;
		printf("ta alto2   ");
        
	  loop: while(digitalRead(ECHO2) == HIGH){
		travelTime = micros() - startTime;
		if (travelTime >= 19140 ) goto sai;
	  }
		count=0;
		printf("passou2   ");
		while(count<30){
			count++;
			if (digitalRead(ECHO2) == HIGH) goto loop;
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
		
		//pwmWrite(MOTOR0,965);
		//pwmWrite(MOTOR1,964);
		
		char str[20];
		printf("Nome do arquivo: ");
		fgets(str,20,stdin);
		if(str[strlen(str)-1]=='\n'){str[strlen(str)-1]='\0';}
		strcat(str,".txt");
		puts(str);
		
		//fp = fopen(str,"w");
		
		while(1){
			delay(100);
			printf("main\n");
			getCM0();
			getCM1();
			//getCM2();
		}
        return 0;
}