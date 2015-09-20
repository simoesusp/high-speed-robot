
/*                      DÚVIDAS
 * usar int ao inves de long na variavel distance
 * fazer a conta da distancia direto (ao invés de duas divisoes)
 */


//#include <pins_arduino.h>
//#include <wiring_private.h>
#include <Time.h>
#include <PWM.h>
//SoftwareSerial mySerial(10, 11); // RX, TX

#define leftmotor 3
#define rightmotor 9

#define minDist 150 // minimum obstacle distance from the robot allowed
#define warningDist 300

// maximum time, in microseconds, allowed for ultrasonic sensor reading

#define time_out 100000

// maximum time, in microseconds, allowed to wait for the previous pulse in sensors echoPin to end

#define prev_pulse_timeout 100000

#define E_L 0
#define E_M 1
#define E_F 2

#define Frente 3

#define D_L 4
#define D_M 5
#define D_F 6



// non PWM digital pins were chosen to be sensor pins.
#define trigPin 7
#define echoPin0 2
#define echoPin1 4
#define echoPin2 8
#define echoPin3 12
#define echoPin4 11

int32_t frequency = 200;    //frequency (in Hz)
uint8_t pwm_value = 52;     // pwm_value  = Motor Stopped   :  Pulse Widht = 1ns --> 1ms ??

uint8_t fullSpeed = 75;     // não tentei nada acima disso, pelas contas seria 104...
uint8_t avgSpeed = 65;
uint8_t lowSpeed = 60;
time_t t;
int T[32];                  // Truth Table

struct sensor_t
{
  uint8_t Bit;
  uint8_t port;
  bool data_available; // HIGH when distance is updated, LOW when distance isn't available yet.
  unsigned long duration; // pulse duration in echoPin.
  unsigned long distance; // obstacle distance calculated.

} USS[5];


//		FUNCTIONS DECLARATION

void SensorSettings();
int myPulseIn(uint8_t, uint8_t, unsigned long);
int SensorReading();
void SensorsDataSerialPrint();
int obstacleShape();
void speedControl(int);
void initTable(int *T);

void setup()
{
  //------------------------ Ultrasound Sensors Settings -----------

  //sets ultrasonic ranging module pins, all 5 sensors share trigPin.
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin0, INPUT);
  pinMode(echoPin1, INPUT);
  pinMode(echoPin2, INPUT);
  pinMode(echoPin3, INPUT);
  pinMode(echoPin4, INPUT);
  //---------------------------------------------------------------

  // Open hardware serial communications and wait for port to open:
  Serial.begin(9600);

  // set the data rate for the SoftwareSerial port
  //  mySerial.begin(9600);


  //sets LED pin            ----> desliguei o LED para usar a porta 13, só pra evitar usar as PWM para futuras aplicações ;)
  //pinMode(13, OUTPUT);

  //sets motors pins
  pinMode(leftmotor, OUTPUT);
  pinMode(rightmotor, OUTPUT);




  // --------------------- MOTORS SETTINGS -----------------------



  //initialize all timers but 0 to save time keeping functions
  InitTimersSafe();

  //sets frequency for specified pin
  bool success_1 = SetPinFrequencySafe(leftmotor, frequency);
  bool success_2 = SetPinFrequencySafe(rightmotor, frequency);
  /*
  	//if pin frequency was successfully set, turn LED on
  	if(success_1 & success_2)
  		digitalWrite(13, HIGH);
  	else
  		digitalWrite(13, LOW);
  */
  if (success_1 & success_2)
    Serial.println("OK.");
  else
    Serial.println("fail.");

  pwmWrite(leftmotor, pwm_value);   // pwmWrite(pin, DUTY * 255 / 100);
  pwmWrite(rightmotor, pwm_value);   // pwmWrite(pin, DUTY * 255 / 100);
  // Can use pwm_values between 13 (min speed) to 26 (max speed)
  // minimum value to turn motor on is ??, then it can be reduced up to 13

  SensorSettings();
  delay(5000);    // Time to turn motors on with low throtle
  //digitalWrite(13, LOW); // turns LED off
  //----------------------------------------------------------------

  t = now();
}

void loop()
{
  time_t aux;
  initTable(T);

  while (1)
  {
    // Initializing Variables
    //	register long duration, distance[5];
    register int obstacle;

    //delay(5000);
    obstacle = SensorReading();

    // Danger Zone
    if (obstacle < 0)
    {
      //   STOP !!!
      pwmWrite(leftmotor, pwm_value);
      pwmWrite(rightmotor, pwm_value);
    }

    // Warning Zone
    else if (obstacle > 0)
      speedControl(obstacle);

    // Safe Zone
    else
    {
      pwmWrite(leftmotor, fullSpeed);
      pwmWrite(rightmotor, fullSpeed);
    }
    aux = now() - t;
    /*
     if(aux > 15)
    	 setup();
      */
    SensorsDataSerialPrint();
    delay(1000);

  }
}

// incluir tratativas de erro?
void SensorSettings()
{
  USS[0].Bit = digitalPinToBitMask(echoPin0);
  USS[0].port = digitalPinToPort(echoPin0);

  USS[1].Bit = digitalPinToBitMask(echoPin1);
  USS[1].port = digitalPinToPort(echoPin1);

  USS[2].Bit = digitalPinToBitMask(echoPin2);
  USS[2].port = digitalPinToPort(echoPin2);

  USS[3].Bit = digitalPinToBitMask(echoPin3);
  USS[3].port = digitalPinToPort(echoPin3);

  USS[4].Bit = digitalPinToBitMask(echoPin4);
  USS[4].port = digitalPinToPort(echoPin4);

}

// guarantee that micros() won't overflow during execution!!!
int myPulseIn()
{
  unsigned long initTime = micros();

  boolean aux = 0;
  boolean previous_pulse[5]; // auxiliar variable that states if previous pulse has already ended or not.
  boolean reading[5];  // auxiliar variable that states if data is being read or not.

  reading[0] = 0;
  previous_pulse[0] = 0;

  reading[1] = 0;
  previous_pulse[1] = 0;

  reading[2] = 0;
  previous_pulse[2] = 0;

  reading[3] = 0;
  previous_pulse[3] = 0;

  reading[4] = 0;
  previous_pulse[4] = 0;

  /*
  	Cache the port and bit of the pins in order to speed up the
  	pulse width measuring loop and achieve finer resolution.
  	Calling digitalRead() instead yields much coarser resolution.

  */

  //	processing distance, so state is set to LOW and duration initialized 0.

  USS[0].data_available = LOW;
  USS[0].duration = 0;

  USS[1].data_available = LOW;
  USS[1].duration = 0;

  USS[2].data_available = LOW;
  USS[2].duration = 0;

  USS[3].data_available = LOW;
  USS[3].duration = 0;

  USS[4].data_available = LOW;
  USS[4].duration = 0;

  while (!aux)
  {

    if (!(USS[0].data_available)) // if data hasn't already been read
    {
      if (previous_pulse[0]) // if previous pulse has ended
      {

        if (reading[0]) // if data is being read
        {
          // if data has just finished being read
          if ((*portInputRegister(USS[0].port) & USS[0].Bit) != USS[0].Bit)
          {
            USS[0].duration = micros() - USS[0].duration;
            USS[0].data_available = 1;
          }
        }
        else // if data hasn't started to be read
        {
          // check if it has just started.
          if ((*portInputRegister(USS[0].port) & USS[0].Bit) == USS[0].Bit)
          {
            USS[0].duration = micros();
            reading[0] = 1;
          }
        }
      }
      else
      {
        // check if previous pulse has just ended and set previous_pulse if it has.
        if ((*portInputRegister(USS[0].port) & USS[0].Bit) != USS[0].Bit)
          previous_pulse[0] = 1;
      }
    }

    if (!(USS[1].data_available)) // if data hasn't already been read
    {
      if (previous_pulse[1]) // if previous pulse has ended
      {

        if (reading[1]) // if data is being read
        {
          // if data has just finished being read
          if ((*portInputRegister(USS[1].port) & USS[1].Bit) != USS[1].Bit)
          {
            USS[1].duration = micros() - USS[1].duration;
            USS[1].data_available = 1;
          }
        }
        else // if data hasn't started to be read
        {
          // check if it has just started.
          if ((*portInputRegister(USS[1].port) & USS[1].Bit) == USS[1].Bit)
          {
            USS[1].duration = micros();
            reading[1] = 1;
          }
        }
      }
      else
      {
        // check if previous pulse has just ended and set previous_pulse if it has.
        if ((*portInputRegister(USS[1].port) & USS[1].Bit) != USS[1].Bit)
          previous_pulse[1] = 1;
      }
    }

    if (!(USS[2].data_available)) // if data hasn't already been read
    {
      if (previous_pulse[2]) // if previous pulse has ended
      {

        if (reading[2]) // if data is being read
        {
          // if data has just finished being read
          if ((*portInputRegister(USS[2].port) & USS[2].Bit) != USS[2].Bit)
          {
            USS[2].duration = micros() - USS[2].duration;
            USS[2].data_available = 1;
          }
        }
        else // if data hasn't started to be read
        {
          // check if it has just started.
          if ((*portInputRegister(USS[2].port) & USS[2].Bit) == USS[2].Bit)
          {
            USS[2].duration = micros();
            reading[2] = 1;
          }
        }
      }
      else
      {
        // check if previous pulse has just ended and set previous_pulse if it has.
        if ((*portInputRegister(USS[2].port) & USS[2].Bit) != USS[2].Bit)
          previous_pulse[2] = 1;
      }
    }

    if (!(USS[3].data_available)) // if data hasn't already been read
    {
      if (previous_pulse[3]) // if previous pulse has ended
      {

        if (reading[3]) // if data is being read
        {
          // if data has just finished being read
          if ((*portInputRegister(USS[3].port) & USS[3].Bit) != USS[3].Bit)
          {
            USS[3].duration = micros() - USS[3].duration;
            USS[3].data_available = 1;
          }
        }
        else // if data hasn't started to be read
        {
          // check if it has just started.
          if ((*portInputRegister(USS[3].port) & USS[3].Bit) == USS[3].Bit)
          {
            USS[3].duration = micros();
            reading[3] = 1;
          }
        }
      }
      else
      {
        // check if previous pulse has just ended and set previous_pulse if it has.
        if ((*portInputRegister(USS[3].port) & USS[3].Bit) != USS[3].Bit)
          previous_pulse[3] = 1;
      }
    }

    if (!(USS[4].data_available)) // if data hasn't already been read
    {
      if (previous_pulse[4]) // if previous pulse has ended
      {

        if (reading[4]) // if data is being read
        {
          // if data has just finished being read
          if ((*portInputRegister(USS[4].port) & USS[4].Bit) != USS[4].Bit)
          {
            USS[4].duration = micros() - USS[0].duration;
            USS[4].data_available = 1;
          }
        }
        else // if data hasn't started to be read
        {
          // check if it has just started.
          if ((*portInputRegister(USS[4].port) & USS[4].Bit) == USS[4].Bit)
          {
            USS[4].duration = micros();
            reading[4] = 1;
          }
        }
      }
      else
      {
        // check if previous pulse has just ended and set previous_pulse if it has.
        if ((*portInputRegister(USS[4].port) & USS[4].Bit) != USS[4].Bit)
          previous_pulse[4] = 1;
      }
    }

    if (micros() > initTime + time_out)
      return 0;
    aux =  USS[0].data_available &  USS[1].data_available &  USS[2].data_available &  USS[3].data_available & USS[4].data_available;
  }
  return 1;
}


void initTable(int *T)
{
  T[0] = Frente;
  T[1] = E_L;
  T[2] = E_M;
  T[3] = E_M;
  T[4] = E_F;
  T[5] = E_F;
  T[6] = E_F;
  T[7] = E_F;
  T[8] = D_M;
  T[9] = D_M;
  T[10] = Frente;
  T[11] = E_M;
  T[12] = D_F;
  T[13] = E_F;
  T[14] = E_F;
  T[15] = E_F;
  T[16] = D_L;
  T[17] = Frente;
  T[18] = E_M;
  T[19] = E_M;
  T[20] = D_F;
  T[21] = E_F;
  T[22] = D_F;
  T[23] = E_F;
  T[24] = D_M;
  T[25] = D_M;
  T[26] = D_M;
  T[27] = Frente;
  T[28] = D_F;
  T[29] = D_F;
  T[30] = D_F;
  T[31] = E_F;
}


void speedControl(int obstacle)
{

  switch (T[obstacle])
  {
    case (E_L):
      pwmWrite(leftmotor, lowSpeed);
      pwmWrite(rightmotor, avgSpeed);
      break;
    case (E_M):
      pwmWrite(leftmotor, avgSpeed);
      pwmWrite(rightmotor, fullSpeed);
      break;
    case (E_F):
      pwmWrite(leftmotor, lowSpeed);
      pwmWrite(rightmotor, fullSpeed);
      break;

    case (D_L):
      pwmWrite(leftmotor, avgSpeed);
      pwmWrite(rightmotor, lowSpeed);
      break;

    case (D_M):
      pwmWrite(leftmotor, fullSpeed);
      pwmWrite(rightmotor, avgSpeed);
      break;

    case (D_F):
      pwmWrite(leftmotor, fullSpeed);
      pwmWrite(rightmotor, lowSpeed);
      break;

    case (Frente):
      pwmWrite(leftmotor, avgSpeed);
      pwmWrite(rightmotor, avgSpeed);
      break;
  }
}

int SensorReading()
{

  // gets sensor0 distance
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);

  myPulseIn(); // colocar tratativa de erro??

  USS[0].distance = (USS[0].duration / 2) / 29.1;

  USS[1].distance = (USS[1].duration / 2) / 29.1;

  USS[2].distance = (USS[2].duration / 2) / 29.1;

  USS[3].distance = (USS[3].duration / 2) / 29.1;

  USS[4].distance = (USS[4].duration / 2) / 29.1;


  if (USS[0].distance < minDist)
    return -1;

  if (USS[1].distance < minDist)
    return -1;

  if (USS[2].distance < minDist)
    return -1;

  if (USS[3].distance < minDist)
    return -1;

  if (USS[4].distance < minDist)
    return -1;

  return obstacleShape();
}


void SensorsDataSerialPrint()
{
  int i;

  for (i = 0; i < 5; i++)
  {
    if (USS[i].distance >= 600 || USS[i].distance <= 2)
      Serial.println("Out of range");
    else
    {
      Serial.print("Sensor 0: ");
      Serial.print(USS[0].distance);
      Serial.println(" cm");
    }
  }
  Serial.println(" ------------- ") ;
  //delay(50);
}


int obstacleShape()
{
  register int obstacle = 0;

  if ((USS[4].distance < warningDist) && (USS[4].distance > 2))
    obstacle += 1;

  if ((USS[3].distance < warningDist) && (USS[3].distance > 2))
    obstacle += 2;

  if ((USS[2].distance < warningDist) && (USS[2].distance > 2))
    obstacle += 4;

  if ((USS[1].distance < warningDist) && (USS[1].distance > 2))
    obstacle += 8;

  if ((USS[0].distance < warningDist) && (USS[0].distance > 2))
    obstacle += 16;

  return obstacle;
}

/*
          TABELA VERDADE

 T[00000] = F;
 T[00001] = E_L;
 T[00010] = E_M;
 T[00011] = E_M;
 T[00100] = E_F;
 T[00101] = E_F;
 T[00110] = E_F;
 T[00111] = E_F;
 T[01000] = D_M;
 T[01001] = D_M;
 T[01010] = F;
 T[01011] = E_M;
 T[01100] = D_F;
 T[01101] = E_F;
 T[01110] = E_F;
 T[01111] = E_F;
 T[10000] = D_L;
 T[10001] = F;
 T[10010] = E_M;
 T[10011] = E_M;
 T[10100] = D_F;
 T[10101] = E_F;
 T[10110] = D_F;
 T[10111] = E_F;
 T[11000] = D_M;
 T[11001] = D_M;
 T[11010] = D_M;
 T[11011] = F;
 T[11100] = D_F;
 T[11101] = D_F;
 T[11110] = D_F;
 T[11111] = E_F;
 */

