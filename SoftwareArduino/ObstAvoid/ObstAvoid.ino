// TODO referenciar direitinho as referencias

/* nRF2L01+ pins
   1 - GND
   2 - VCC 3.3V !!! NOT 5V
   3 - CE to Arduino pin 5
   4 - CSN to Arduino pin 6
   5 - SCK to Arduino pin 13
   6 - MOSI to Arduino pin 11
   7 - MISO to Arduino pin 12
   8 - UNUSED
   
 - V1.00 11/26/13
   Based on examples at http://www.bajdi.com/
   Questions: terry@yourduino.com */


#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Time.h>
#include <PWM.h>

/*=============================   DEFINES   ========================================== */
// ------- radio pins ----------------------------
#define CE_PIN   5
#define CSN_PIN 6
// -------- motor related defines -----------------
#define leftmotor 10 // left motor pin
#define rightmotor 3 // right motor pin
#define stop_pwm_l 110 // pwm in which left motor is stopped
#define stop_pwm_r 90  // pwm in which right motor is stopped
// --------- deadlines (in microsseconds)----------------------------
#define Ncommands 12
#define dt 1000 // time increment for the deadlock handling routine
#define t_min 10000
#define t_max 100000 
#define T_com 1000000 
#define T_start 1000000 
// TODO: ajeitar isso aqui!!!!
// maximum time (microseconds) allowed for ultrasonic sensor reading

//TODO: pq medir além da regiao de atenção??
#define time_out 40000 // USS reading deadline <=> 6.8m (datasheet fala que só vai até 4m...)
// maximum time (microseconds) allowed to wait for the previous pulse in sensors echoPin to end
// #define prev_pulse_timeout 7000 // previous pulse waiting deadline 

// NOT A DEADLINE:
#define MaxPayload 25 
#define n_max 10
// ---------- safety button pins -----------------
#define ON  3// always high
#define OFF 2 // always low 
#define state 4 // says if in trouble or safe!

// ----------- USS Pins --------------------------
              // Obs.: non PWM digital pins were chosen to be sensor pins.
#define trigPin 2
#define echoPin0 A0
#define echoPin1 A1 
#define echoPin2 A2
#define echoPin3 7 
#define echoPin4 8 

// ---------- obstacle avoidance related defines ---

//TODO: nao esquecer de arrumar de novo!!!!
#define minDist 50 // obstacle can't get any closer than minDist (in cm) from the robot
#define warningDist 100 // minDist < obstacle distance < warningDist => warning zone!


#define E_L 0 // turn softly to the left
#define E_M 1 // turn to the left
#define E_F 2 // turn abruptly to the right

#define Frente 3 // straight ahead

#define D_L 4 // turn softly to the right
#define D_M 5 // turn to the right
#define D_F 6 // turn abruptly to the right

/*======================================================================= */

/*========================= GLOBAL VARIABLES ============================================================*/

// -------- motor related variables -----------------
/*int32_t*/ int frequency_r = 400;    // right motor pwm_value  = Motor Stopped   :  Pulse Width = 1ns
/*uint8_t*/ int pwm_value_r = 330;    // right motor  pwm_value  = Motor Stopped   :  Pulse Width = 1ns
/*int32_t*/ int frequency_l = 400;    // left motor frequency (in Hz)
/*uint8_t*/ int pwm_value_l = 104;    // left motor  pwm_value  = Motor Stopped   :  Pulse Width = 1ns
bool left; // left motor on/off flag
bool right; // right motor on/off flag
bool speed; // if true, try to read pwm_value from remote control
bool frequency;// if true, try to read frequency from remote control

//TODO: why not make these defines???
uint8_t fullSpeed = 200;
uint8_t avgSpeed = 170;
uint8_t lowSpeed = 150;

// -------- communication related variables -----------------
const uint64_t pipes[2] = {0xDEDEDEDEE7LL, 0xDEDEDEDEE9LL}; // Define the transmit pipe (Obs.: "LL" => "LongLong" type)
RF24 radio(CE_PIN, CSN_PIN); // Create a Radio Object
bool reading; // reading/writing flag
char rcv[MaxPayload]; // message received
char msg[MaxPayload]; // used by remote control
char cmd[MaxPayload]; // used by the robot
char **intCmd; // internal commands
long randNumber;
unsigned long t;
//--------- UltraSonic Sensor type --------------------------
struct sensor_t
{
    uint8_t Bit;
    uint8_t port;
    bool data_available; // HIGH when distance is up to date, LOW when distance isn't available yet.
    unsigned long duration; // pulse duration in echoPin.
    unsigned long distance; // obstacle distance calculated.
} USS[5]; 


bool safety_button;

bool select = 1; // 0: master(remote control) ; 1: slave(robot) 

int *T;                  // Truth Table

/*=======================================================================================================*/

/*========================= FUNCTIONS  ============================================================*/
void autonomous();

// -------- communication oriented functions -------
void communication(bool mode);
bool Read_nonBlocking();
bool Read_blocking();
bool Write_nonBlocking(bool);
bool Write_blocking(bool);
void deadlockHandling(unsigned int);

//---------- string handling functions -------------
void concatenate(char *, char *);
void copyString(char *, char *);
bool isSubstring(char *, char *);
void write_command(char*);
int char2int(char*);
void int2char(char *o,int n);

void read_message(); // Serial or safety button reading for transmission
void setInternalCommands(char**); // robot commands
bool processing(); // processes robot received data
void action(int); // perform command
void status(); // selected motor(s), pwm modulation rates and frequencies
void start(); // turn selected motors on
bool checkButton(); // check safety button status

//--------- obstacle avoidance related functions ---
void ObstacleAvoid();
void SensorSettings();
int myPulseIn(uint8_t, uint8_t, unsigned long);
int SensorReading();
void SensorsDataPrint();
void SensorsDataSerialPrint();
int obstacleShape();
void speedControl(int);
void initTable(int *T);
/*=======================================================================================================*/

void setup()
{
    //------------------------ nRF24L01+ transceiver Settings -----------
	Serial.begin(9600);
	radio.begin();
	radio.enableAckPayload();
	radio.enableDynamicPayloads();
	radio.openReadingPipe(1,pipes[(int)!select]);
	radio.openWritingPipe(pipes[(int)select]);
    radio.setPALevel(RF24_PA_MAX);
	randomSeed(analogRead(0));
	reading = !select;

	if(select) // robot
    {
        //------------------------ Ultrasound Sensors Settings -----------

        //sets ultrasonic ranging module pins, all 5 sensors share trigPin.
        pinMode(trigPin, OUTPUT);
        pinMode(echoPin0, INPUT);
        pinMode(echoPin1, INPUT);
        pinMode(echoPin2, INPUT);
        pinMode(echoPin3, INPUT);
        pinMode(echoPin4, INPUT);
        SensorSettings();
        //---------------------------------------------------------------

        // --------------------- MOTORS SETTINGS -----------------------
        pinMode(leftmotor, OUTPUT);
        pinMode(rightmotor, OUTPUT);

        InitTimersSafe(); //initialize all timers except for 0, to save time keeping functions

        //sets the frequency for the specified pin
        bool success = SetPinFrequencySafe(leftmotor, frequency_l);
        success &= SetPinFrequencySafe(rightmotor, frequency_r);

        pwmWrite(leftmotor, pwm_value_l);   // pwmWrite(pin, DUTY * 255 / 100);  
        pwmWrite(rightmotor, pwm_value_r);   // pwmWrite(pin, DUTY * 255 / 100);  
        delay(1000); // TODO ve se da pra fazer isso mais rapido!
        pwmWrite(rightmotor, stop_pwm_r);   // pwmWrite(pin, DUTY * 255 / 100);  
        delay(1000);
        pwmWrite(rightmotor, pwm_value_r);   // pwmWrite(pin, DUTY * 255 / 100);  
        delay(1000);
        pwm_value_r = stop_pwm_r;
        pwmWrite(rightmotor, pwm_value_r);   // pwmWrite(pin, DUTY * 255 / 100);  

        int i;
        intCmd = (char**)malloc(Ncommands*sizeof(char*));
        for(i=0;i<Ncommands;i++)
            intCmd[i] = (char*)malloc(MaxPayload*sizeof(char));
        setInternalCommands(intCmd);
    }
    else // remote control
    {
        pinMode(ON, OUTPUT);
        pinMode(OFF, OUTPUT);
        pinMode(state, INPUT);

        digitalWrite(ON, HIGH);
        digitalWrite(OFF, LOW);
    }

    T = (int*)malloc(32*sizeof(int));
    initTable(T); // T: truth table
}

void loop()
{
    communication(select);
}

/*---------------------------------  OBSTACLE AVOIDANCE STUFF    -----------------------------------------------------------------------*/

void ObstacleAvoid()
{
    register int obstacle;

    obstacle = SensorReading();

    // Danger Zone
    if (obstacle < 0)
    {
        //   STOP !!!
        pwmWrite(leftmotor, stop_pwm_l);
        pwmWrite(rightmotor, stop_pwm_r);
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

/*TODO:
 *
 * check if it's possible to analyze all sensors at once:  *portInputRegister(USS[i].port)
 * guarantee that micros() won't overflow during execution!!!
 */
int myPulseIn()
{
    unsigned long initTime = micros();

    boolean finished = 0; // finished reading all sensors data flag
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

    while (!finished)
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
                        USS[4].duration = micros() - USS[4].duration;
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
        finished =  USS[0].data_available &  USS[1].data_available &  USS[2].data_available &  USS[3].data_available & USS[4].data_available;
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

//TODO: resolver problema e tirar acochambraçao!!!
    if (USS[0].distance < minDist)
    if (USS[0].distance > 10)
        return -1;

    if (USS[1].distance < minDist)
    if (USS[1].distance > 10)
        return -1;

    if (USS[2].distance < minDist)
    if (USS[2].distance > 10)
        return -1;

    if (USS[3].distance < minDist)
    if (USS[3].distance > 10)
        return -1;

    if (USS[4].distance < minDist)
    if (USS[4].distance > 10)
        return -1;

    return obstacleShape();
}

// returned value is seen as binary data: i-th bit represents the i-th sensor data (1 for warningDist, 0 otherwise)
//Obs.: 1st sensor is the leftmost while the 5th, is the rightmost; that's why the first sensor is USS[4]!!!
int obstacleShape()
{
    register int obstacle = 0;

    if ((USS[4].distance < warningDist) && (USS[4].distance > 2))
        obstacle += 1; // 1st bit <=> 1st sensor

    if ((USS[3].distance < warningDist) && (USS[3].distance > 2))
        obstacle += 2; // 2nd bit <=> 2nd sensor

    if ((USS[2].distance < warningDist) && (USS[2].distance > 2))
        obstacle += 4; // 3rd bit <=> 3rd sensor

    if ((USS[1].distance < warningDist) && (USS[1].distance > 2))
        obstacle += 8; // 4th bit <=> 4th sensor

    if ((USS[0].distance < warningDist) && (USS[0].distance > 2))
        obstacle += 16; // 5th bit <=> 5th sensor

    return obstacle;
}

/*--------------------------------------------------------------------------------------------------------------------------------------*/


/*==========================================  REMOTE CONTROL STUFF  ===================================================================*/ 

void status()
{
/*----------------------------------------- DATA FORMAT: ---------------------------------------------------------------------------
(bool)left:(bool)right:(bool)speed:(bool)frequency:(int32_t)frequency_l:(uint8_t)pwm_value_l;(int32_t)frequency_r:(uint8_t)pwm_value_r
----------------------------------------------------------------------------------------------------------------------------------- */	
	char status[MaxPayload], aux[6]; //
	int i;

	status[0] = '\0';
	status[0] = 48+left;
	status[1] = ':';
	status[2] = 48+right;
	status[3] = ':';
	status[4] = 48+speed;
	status[5] = ':';
	status[6] = 48+frequency;
	status[7] = ':';
	status[8] = '\0';

	int2char(aux,frequency_l);
	concatenate(status,aux);
	concatenate(status,":");

	int2char(aux,pwm_value_l);
	concatenate(status,aux);
	concatenate(status,":");

	int2char(aux,frequency_r);
	concatenate(status,aux);
	concatenate(status,":");

	int2char(aux,pwm_value_r);
	concatenate(status,aux);

	write_command(status);

}

void communication(bool mode)
{
	unsigned long T = micros();
	bool w = 0,r = 0, quit = 0;

	while( (micros() < T+T_com) | !(w&r)) 
	{	

		if(reading)
			if(Read_blocking())
			{
				r = 1;
				reading = 0;
				if(select)
					quit = processing(); // safety button status has been received
			}
			else;
		else
			if( quit || Write_blocking(mode) )
			{
				w = 1;
				reading = 1;
			}
	}
}


void action(int n)
{

	if(n < Ncommands)
		switch(n)
        {
            case 0: //	   left
                left = 1;
                right = 0;
                write_command("left motor!");
                break;

            case 1: //	   right
                left = 0;
                right = 1;
                write_command("right motor!");
                break;

            case 2: //	   both
                left = 1;
                right = 1;
                write_command("both motors!");
                break;

            case 3: //	   speed
                speed = 1;
                frequency = 0;
                write_command("enter speed value.");
                break;

            case 4: //	   start
                write_command("here we go!");
                start();
                break;

            case 5: //	   stop
                stop();
                write_command("stop!!!");
                break;

            case 6: //	   sweep
                sweep();
                break;

            case 7: //	  status 
                status();
                break;

            case 8: //	 frequency 
                speed = 0;
                frequency = 1;
                write_command("enter frequency value.");
                break;
            case 9: //	 safety button status 
                if(safety_button)
                    write_command("ON");
                else
                    write_command("OFF");
                break;
            case 10: // print USS data  
                SensorsDataPrint();
                break;
            case 11: //	obstacle avoidance 
                autonomous();
                break;
        }
}

void stop()
{
	if(right)
		pwmWrite(rightmotor, stop_pwm_r);  // pwmWrite(pin, DUTY * 255 / 100);

	if(left)
		pwmWrite(leftmotor, stop_pwm_l);
}

void sweep()
{
    bool ok = 0;
    int i = pwm_value_r, j = pwm_value_l;
    int r = pwm_value_r, l = pwm_value_l;

    pwm_value_r = stop_pwm_r;
    pwm_value_l = stop_pwm_l;

    while(!ok)
    {
        if(i > pwm_value_r || j > pwm_value_l)
        {
            if(i > pwm_value_r && j > pwm_value_l)
            {
                pwmWrite(rightmotor, pwm_value_r);
                pwmWrite(leftmotor, pwm_value_l);
                pwm_value_r++;
                pwm_value_l++;
            }
            if(i > pwm_value_r && !(j > pwm_value_l))
            {
                pwmWrite(rightmotor, pwm_value_r);
                pwm_value_r++;
            }
            if(!(i > pwm_value_r) && j > pwm_value_l)
            {
                pwmWrite(leftmotor, pwm_value_l);
                pwm_value_l++;
            }
            delay(50);
        }
        else
            ok = 1;
    }

    ok = 0;
    i = stop_pwm_r;
    j = stop_pwm_l;

    while(!ok)
    {
        if(i < pwm_value_r || j < pwm_value_l)
        {
            if(i < pwm_value_r && j < pwm_value_l)
            {
                pwmWrite(rightmotor, pwm_value_r);
                pwmWrite(leftmotor, pwm_value_l);
                pwm_value_r--;
                pwm_value_l--;
            }
            if(i < pwm_value_r && !(j < pwm_value_l))
            {
                pwmWrite(rightmotor, pwm_value_r);
                pwm_value_r--;
            }
            if(!(i < pwm_value_r) && j < pwm_value_l)
            {
                pwmWrite(leftmotor, pwm_value_l);
                pwm_value_l--;
            }
            delay(50);
        }
        else
            ok = 1;
    }
    pwm_value_r = r;
    pwm_value_l = l;
}

void start()
{

	reading = 1;
	radio.startListening();
	delay(1000);

	if(right)
		pwmWrite(rightmotor, pwm_value_r);  // pwmWrite(pin, DUTY * 255 / 100);

	if(left)
		pwmWrite(leftmotor, pwm_value_l);

    t = micros();
	if(right || left)
        while(micros() < t + T_start)
        {

            if(radio.available())
            {
                bool done = false;
                while (!done)
                    done = radio.read( rcv, sizeof(rcv) );
            }

            if(rcv[0] == '-')
            {
                break;
            }
            else if(rcv[0] == '+')
            {
                t = micros();
            }
            else
            {
                processing();
            }
            rcv[0] = '\0';
        }
		stop();
}

bool processing()
{
	int i;
	for(i = 0; i < Ncommands; i++) // have we just got a command?
		if(isSubstring(intCmd[i],rcv) == 1)
			break;

	if(i == Ncommands) // if not a command
    {
        if(speed)
        {
            int pwm = char2int(rcv);
            if(pwm > 0)
            {
                if(left)
                {
                    pwm_value_l = pwm;
                    speed = 0;
                    if(right)
                        concatenate(rcv," (both pwm)");
                    else
                        concatenate(rcv," (left pwm)");
                    write_command(rcv);
                }
                if(right)
                {
                    pwm_value_r = pwm;
                    speed = 0;
                    if(!left)
                    {
                        concatenate(rcv," (right pwm)");
                        write_command(rcv);
                    }
                }
            }
        }
        else if(frequency)
        {
            int f = char2int(rcv);
            if(f > 0) 
            {
                if(left)
                {
                    frequency_l = f;
                    frequency = 0;
                    SetPinFrequencySafe(leftmotor, frequency_l);
                    if(right)
                        concatenate(rcv," (freq_LnR)");
                    else
                        concatenate(rcv," (freq_L)");
                    write_command(rcv);
                }

                if(right)
                {
                    frequency_r = f;
                    frequency = 0;
                    SetPinFrequencySafe(rightmotor, frequency_r);
                    if(!left)
                    {
                        concatenate(rcv," (freq_R)");
                        write_command(rcv);
                    }
                }
            }
        }
        else if( (  (rcv[0]=='+' || rcv[0]=='-') /* && (rcv[1]=='\0') */ )  )  // safety button status message
        {	
            if(rcv[0] == '+')
                safety_button = 1;
            else
                safety_button = 0;
            //return 1;
        }
        else // nothing recognizable send it back
        {	
            write_command(rcv);
        }
    }
	else // perform command i
	{
		action(i);
	}

    return 0;
}

bool checkButton()
{
    if(Serial.available() == 0)
    {
        if(digitalRead(state) == HIGH)
        {
            msg[0] = '+';
            msg[1] = '\0';
            return 1;
        }
        else
        {
            msg[0] = '-';
            msg[1] = '\0';
            return 0;
        }

    }
}

void read_message()
{
    if(Serial.available() == 0 && msg[0] == '\0')
        checkButton();
    else
    {
        delay(1);
        int aux = 0;
        while(Serial.available() > 0)
            msg[aux++] = Serial.read();
        msg[aux] = '\0';
    }
}

void copyString(char *o, char *c)
{
    int i;
    for(i = 0; i < MaxPayload; i++)
    {
       c[i] = o[i];
        if(o[i] == '\0')
            break;
    }
}

void write_command(char *m)
{
	copyString(m,cmd);
}

// TODO: make it generic
int char2int(char *s)
{
    int r = 0;

    if(s[1] == '\0')
    {
        if(s[0] >= '0' && s[0] <= '9')
        {
            r = (s[0] - 48);
            return r;
        }
        return -1;
    }

    if(s[2] == '\0')
    {
        if(s[0] >= '0' && s[0] <= '9' && s[1] >= '0' && s[1] <= '9')
        {
            r = (s[1] - 48) + (s[0] - 48)*10;
            return r;
        }
        return -1;
    }

    if(s[3] == '\0')
    {
        if(s[0] >= '0' && s[0] <= '9' && s[1] >= '0' && s[1] <= '9' && s[2] >= '0' && s[2] <= '9')
        {
            r = (s[2] - 48) + (s[1] - 48)*10 + (s[0] - 48)*100 ;
            return r;
        }
        return -1;
    }

    return -2;

/*
    int i = 0,j, r = 0, aux;

    for(i = 0; i < MaxPayload; i++)
        if(s[i] == '\0')
            break;

    if(i == MaxPayload)
    {
        write_command("-3"); // number too big for this application
        return -3;
    }

    if(i == 0)
    {
        write_command("-2"); // empty string
        return -2;
    }

    for(j=0; j < i; j++)
    {
        //if((s[j] >= '0') && (s[j] <= '9'))
        if((s[j] > 47) && (s[j] < 57))
        {
            aux = (int) s[j];
            aux -= 48;
            aux *= (int) pow(10,i-1-j);
            r += aux;
            //r += (aux-48)*pow(10,i-j-1);	
        }
        else
        {
            int2char(s,i);
            concatenate(s,":error!");
            write_command(s);
            return -1;
        }
    }
    return r;
*/
}

void int2char(char *o,int n)
{
	int i = 1, aux = 10, j;
	if(n >= 0)
	{
		while(n/i > 9)
			i *=10;
		for(j = 0; i>0; j++)
		{
			aux = n/i;
			o[j] = aux+48;
			n -= aux*i;
			i /= 10;
		}
		o[j] = '\0';

	}
	else // negative values aren't useful for this application
		o[0] = '\0';
}

void concatenate(char *o, char *p)
{
	int i,j,k;
	for(i = 0; i < MaxPayload; i++)
		if(o[i] == '\0')
			break;

	for(j = 0; j < MaxPayload; j++)
		if(p[j] == '\0')
			break;

	if( i+j <= MaxPayload)
	{
		for(k = 0; k < j ;k++)
			o[i+k] = p[k];
		o[i+k] = '\0';
	}
}


void setInternalCommands(char**m)
{

	copyString("left",m[0]);
	copyString("right",m[1]);
	copyString("both",m[2]);
	copyString("speed",m[3]);
	copyString("start",m[4]);
	copyString("stop",m[5]);
	copyString("sweep",m[6]);
	copyString("status",m[7]);
	copyString("frequency",m[8]);
	copyString("safety button",m[9]);
	copyString("distances",m[10]);
	copyString("autonomous",m[11]);
	//copyString("",m[]);

}

bool isSubstring(char *a, char *b)
{
	int i;

	if((a[0] == '\0')||(b[0] == '\0'))
	{
		Serial.println("Empty string.");
		return 0;
	}	

	for(i = 0; i < MaxPayload; i++ )
	{
		if((a[i] == '\0')||(b[i] == '\0'))
			if((a[i] == '\0')&&(b[i] == '\0'))
				return 1;
			else
			{
				return 0;
			}
		if(a[i] != b[i])
			return 0;
	}
	return 1;
}

bool Read_blocking()
{
	radio.startListening();
	delay(1);
	bool r = Read_nonBlocking();
	unsigned int n = 0;
	t = micros();

	while(!r)
	{
		if(micros() > t+dt)
			deadlockHandling(n++);

		if(!reading)
			return 0;
		r = Read_nonBlocking();
	}
	return 1;
}

bool Write_blocking(bool mode) // mode 0: user written command; mode 1: std_command
{
    radio.stopListening();
    delay(1);
    bool w;
    unsigned int n = 0;

    if(!mode)
        read_message();
    w = Write_nonBlocking(mode);
    t = micros();
    while(!w)
    {
        if(micros() > t+dt)
            deadlockHandling(n++);

        if(reading)
            return 0;
        w = Write_nonBlocking(mode);
    }
    if(mode)
        cmd[0] = '\0';
    else
        msg[0] = '\0';

    return 1;
}

void deadlockHandling(unsigned int n)
{
/*
    if(!select)
    {
        Serial.print("Welcome to deadlockHandling function (");
        Serial.print(n);
        Serial.println(")");
    }
*/

    if(n < n_max)
    {
        t += (unsigned long)random(t_min+n*dt, t_max);
        if(t%2)
        {
            reading = !reading;
/*
            if(!select)
                Serial.println("toggle.");
*/
        }
    }
/*
    else
        if(!select)
            Serial.println("deadlock.");
*/

}

bool Read_nonBlocking() // tirei o startListening() daqui!!!!
{
	reading = 1;
	if(!radio.available())
		return 0;
	bool done = false;
	while (!done)
		done = radio.read( rcv, sizeof(rcv) );
	if(done)
    {
        if(!select)
        {
            Serial.print("Message received: ");
            Serial.println(rcv);
        }
        return 1;
    }
}


bool Write_nonBlocking(bool mode)
{
	bool check;
	reading = 0;

	if(mode)
	{
        if(cmd[0] != '\0')
            check = radio.write( cmd, sizeof(cmd));
        else 
            return 1;
	}
	else
	{
        if(msg[0] != '\0')
        {
            check = radio.write( msg, sizeof(msg));
        }
        else 
        {
            return 1;
        }
	}

	if(check)
		return 1;
	else
		return 0;
}

void autonomous()
{

    reading = 1;
    radio.startListening();
    delay(1000);

    t = micros();
    if(right || left)
        while(micros() < t + T_start)
        {

            if(radio.available())
            {
                bool done = false;
                while (!done)
                    done = radio.read( rcv, sizeof(rcv) );
            }

            if(rcv[0] == '-')
            {
                break;
            }
            else if(rcv[0] == '+')
            {
                t = micros();
            }
            else
            {
                ObstacleAvoid();
                //processing();
            }
            rcv[0] = '\0';
        }
    stop();
}

void SensorsDataPrint()
{
    int i;
    char status[MaxPayload], aux[6]; //
    status[0] = '\0';

    for (i = 0; i < 5; i++)
    {
        if (USS[i].distance >= 600 || USS[i].distance <= 2)
            concatenate(status,"OoR:");
        else
        {
            int2char(aux,USS[i].distance);
            concatenate(status,aux);
            concatenate(status,":");
        }
    }
    write_command(status);
}
