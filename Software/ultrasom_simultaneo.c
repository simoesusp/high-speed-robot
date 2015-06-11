#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <string.h>
 
#define TRIG0 0
#define ECHO0 1 

#define TRIG1 2
#define ECHO1 3

#define TRIG2 4
#define ECHO2 5
 
void setup() {
        wiringPiSetup();
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
		digitalWrite(TRIG2, HIGH);
        delayMicroseconds(20);
        digitalWrite(TRIG1, LOW);
		digitalWrite(TRIG2, LOW);
 
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
		
		char str[20];
		printf("Nome do arquivo: ");
		fgets(str,20,stdin);
		if(str[strlen(str)-1]=='\n'){str[strlen(str)-1]='\0';}
		strcat(str,".txt");
		puts(str);
		
		FILE *fp;
		fp = fopen(str,"w");
		
		setup();
		while(distancia1 > 5 && distancia2 > 5){
			delay(2000);
			/*distancia0=getCM0();
			if (distancia0 >= 330){
				printf("Distance0: Out of Bounds\n");
				fprintf(fp,"OUT\t");
			}else{
				printf("Distance0: %dcm\n", distancia0);
				fprintf(fp,"%d\t",distancia0);
			}*/
			
			distancia1=getCM1();
			if (distancia1 >= 330){
				printf("Distance1: Out of Bounds\n");
				fprintf(fp,"OUT\t");
			}else{
				printf("Distance1: %dcm\n", distancia1);
				fprintf(fp,"%d\t",distancia1);
			}
			
			distancia2=getCM2();
			if (distancia2 >= 330){
				printf("Distance2: Out of Bounds\n");
				fprintf(fp,"OUT\n");
			}else{
				printf("Distance2: %dcm\n", distancia2);
				fprintf(fp,"%d\n",distancia2);
			}
		}
		fclose(fp);
        return 0;
}