#include <iostream>
#include <wiringPi.h>
#include <softPwm.h>

#define MOTOR0 6
#define MOTOR1 7

using namespace std;
int main(){
	wiringPiSetup();
	pinMode(MOTOR0 , OUTPUT);
	pinMode(MOTOR1 , OUTPUT);
	digitalWrite(MOTOR0 , LOW);
	digitalWrite(MOTOR1 , LOW);
	softPwmCreate(MOTOR0,0,200);
	softPwmCreate(MOTOR1,0,200);
	softPwmWrite(MOTOR0,195);
	softPwmWrite(MOTOR1,195);
	delay(2000);
	
	char ch;
	int i=195;
	
	while(1){
		cin>>ch;
		if(ch=='a' && i>180) i--;
		if(ch=='d' && i<195) i++;
		softPwmWrite(MOTOR0,i);
		softPwmWrite(MOTOR1,i);
		cout<<i<<endl;
	}
}