/*
  Software serial receive commands from RaspberryPi and turn led on

 Receives from the hardware serial, sends to software serial.
 Receives from software serial, sends to hardware serial.
 stores in vector v the message
 sends message back to RaspberryPi via software serial
 checks if message = command 5 and turn on led for 5 sec

 The circuit:
 * RX is digital pin 10 (connect to TX of other device)
 * TX is digital pin 11 (connect to RX of other device)
 
 by Simoes
 based on Tom Igoe and Mikal Hart's example
*/

#define motorleft 9               // the pin that the LED is attached to
//#define motorright 10

#include <PWM.h>

int32_t frequency = 50;    //frequency (in Hz)
uint8_t pwm_value = 13;     // pwm_value  = Motor Stopped   :  Pulse Widh = 1ms

void setup()
{
  pinMode(13, OUTPUT);

  pinMode(motorleft, OUTPUT);

  // PWM Stuff !!
  //initialize all timers except for 0, to save time keeping functions
  InitTimersSafe(); 
  //sets the frequency for the specified pin
  bool success = SetPinFrequencySafe(motorleft, frequency);
  //if the pin frequency was set successfully, pin 13 turn on
  if(success)     digitalWrite(13, HIGH);    
  else            digitalWrite(13, LOW);
  
  pwmWrite(motorleft, pwm_value);  // The motors should be turned on with a 1ms pulse, otherwise you will get complaining beeps!!
  delay(5000);    // Time to turn motors on with low throtle 
  digitalWrite(13, LOW);


}

void loop() 
{
                     digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
                     
                     pwm_value = 26;
                     pwmWrite(motorleft, pwm_value);  // Maximum Speed  // pwmWrite(led, DUTY * 256 / 100);  // 

                      
                     delay(5000);              // wait for x seconds

                     pwm_value = 13;
                     pwmWrite(motorleft, pwm_value);  // Minimum Speed  // pwmWrite(led, DUTY * 256 / 100);  // 
                     
                     digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
                     delay(5000);              // wait for x seconds

}


