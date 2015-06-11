#include <iostream>
#include <wiringPi.h>
#include <softPwm.h>

#define MOTOR0 40
#define MOTOR1 45

using namespace std;
int main(){
	wiringPiSetupGpio();
	pwmSetMode(PWM_MODE_MS);
	pwmSetRange(1024);
	pwmSetClock(375);
	pwmWrite(MOTOR0,998);
	pwmWrite(MOTOR1,998);
	char ch;
	int i=978;
	
	while(1){
		cin>>ch;
		if(ch=='a' && i>800){ i--;}
		if(ch=='d' && i<1000){ i++;}
		pwmWrite(MOTOR0,i);
		pwmWrite(MOTOR1,i);
		cout<<i<<endl;
	}
}