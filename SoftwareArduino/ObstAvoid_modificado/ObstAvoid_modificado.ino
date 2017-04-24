// Chapa de aço 30cm x 2cm 

// TODO referenciar direitinho as referencias


/*                      RELATED TO PWM LIBRARY USAGE 
Copyright (c) 2012 Sam Knight

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


/*                      RELATED TO RF24 LIBRARY USAGE 
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

  This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License
    version 2 as published by the Free Software Foundation.
*/




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
#include <nRF24L01.h> // acho que não está sendo utilizada!!!
#include <RF24.h>
#include <Time.h>
#include <PWM.h>

/*=============================   DEFINES   ========================================== */
#define Ncommands 16 // number of commands available
// ------- radio pins ----------------------------
#define CE_PIN   5
#define CSN_PIN 6
// -------- motor related defines -----------------
#define leftmotor 10 // left motor pin
#define rightmotor 3 // right motor pin

#define stop_pwm_l 110 // pwm in which left motor is stopped
#define stop_pwm_r 90  // pwm in which right motor is stopped
#define fullSpeed_l 170
#define fullSpeed_r 170
#define avgSpeed_l 140
#define avgSpeed_r 150
#define lowSpeed_l 120
#define lowSpeed_r 100

#define buffer_size 50

// --------- deadlines (in millisseconds)----------------------------
#define t_max 100  // TODO: give it a better name
#define t_start 500 // deadline for loss of radio connection when running - autonomous() as well as start() -
#define time_out 60 // USS reading deadline <=> 6.8m (datasheet fala que só vai até 4m...)

// RADIO RELATED

#define MaxPayload 24 // Maximum message size (32 bytes)
#define intCmd_size 9 // Bytes
// ---------- safety button pins -----------------
#define ON  9// always high
#define state 8 // says if in trouble or safe!
#define OFF 7 // always low 

// ----------- USS Pins --------------------------
              // Obs.: non PWM digital pins were chosen to be sensor pins.
#define trigPin 2
#define echoPin0 8
#define echoPin1 7
#define echoPin2 A0 
#define echoPin3 A1 
#define echoPin4 A2
// ---------- obstacle avoidance related defines ---

//TODO: nao esquecer de arrumar de novo!!!!
#define minDist 50 // obstacle can't get any closer than minDist (in cm) from the robot
#define warningDist 350 // minDist < obstacle distance < warningDist => warning zone!
#define outOfRange 400

#define E_L 0 // turn softly to the left
#define E_M 1 // turn to the left
#define E_F 2 // turn abruptly to the right
#define Frente 3 // straight ahead
#define D_L 4 // turn softly to the right
#define D_M 5 // turn to the right
#define D_F 6 // turn abruptly to the right
#define FullSpeed 7 // straight ahead

/*======================================================================= */

/*========================= GLOBAL VARIABLES ============================================================*/

unsigned int **buffer;

// -------- motor related variables -----------------
//TODO: maybe we could use uint8_t to save space
/*int32_t*/ int frequency_r = 400;    // right motor pwm_value  = Motor Stopped   :  Pulse Width = 1ns
/*int32_t*/ int frequency_l = 400;    // left motor frequency (in Hz)
/*uint8_t*/ int pwm_value_r = 330;    // right motor  pwm_value  = Motor Stopped   :  Pulse Width = 1ns
/*uint8_t*/ int pwm_value_l = 104;    // left motor  pwm_value  = Motor Stopped   :  Pulse Width = 1ns

bool left; // left motor on/off flag
bool right; // right motor on/off flag
bool speed; // if true, try to read pwm_value from remote control
bool frequency;// if true, try to read frequency from remote control
bool dyn = 0; // dynamical/fixed USS echo reading
bool all_sensors = 0;

// -------- communication related variables -----------------
const uint64_t pipe = 0xDEDEDEDEE7LL; // Define the transmit pipe (Obs.: "LL" => "LongLong" type)
RF24 radio(CE_PIN, CSN_PIN); // Create a Radio Object
char rcv[MaxPayload]; // message received
char cmd[MaxPayload]; // message transmitted
char copy[MaxPayload]; // used by the remote controller
char **intCmd; // internal commands
unsigned long t;
//--------- UltraSonic Sensor type --------------------------
struct sensor_t
{
    uint8_t Bit;
    uint8_t port;
    bool data_available; // HIGH => distance is up to date, LOW => reading data || no obstacle detected during last reading operation.
    unsigned long duration; // pulse duration in echoPin.
    unsigned int distance[5]; // current obstacle distance calculated.
    unsigned int farthest; // farthest obstacle distance calculated (10 measures iteration).
    unsigned int closest; // closest obstacle distance calculated (10 measures iteration).
    unsigned int mean; // mean obstacle distance calculated (10 measures iteration).
    short int pointer; // points to most recent data in circular vector distance.
} USS[5]; 

bool safety_button; // robot: button status 

    bool select = 1; // 0: master(remote control) ; 1: slave(robot) 

short int *T;                  // Truth Table

/*=======================================================================================================*/

/*========================= FUNCTIONS  ============================================================*/
void autonomous();
void logging();
void flushLogData(int); // TODO: find a better name for it.
// -------- communication oriented functions -------
void communication();
bool read_nonBlocking();
void read_Blocking();
bool write_nonBlocking();
void statusFeedback(char);
void writeSensorsData();

//---------- string handling functions -------------
void append(char *, char *);
void copyString(char *, char *);
bool isSubstring(char *, char *);
void write_ackPayload(char*);
int char2int(char*);
void int2char(char *o,int n);

void set_msg(); // transmits Serial.read() or safety button status - remote controller
void setInternalCommands(char**); // robot commands
bool processing(); // processes robot received data
void action(int); // perform received command - robot
void status(); // selected motor(s), pwm modulation rates and frequencies
void start(); // turn selected motors on
bool checkButton(); // check safety button status

//--------- obstacle avoidance related functions ---
void quiet();
void ObstacleAvoid();
void SensorSettings();
int myPulseIn(bool);
int SensorReading();
void getExtreamValues();
void SensorsDataPrint();
int obstacleShape();
void speedControl(int);
void initTable(short int *);
/*=======================================================================================================*/
 int mypow(int , int );

void setup()
{
    //------------------------ nRF24L01+ transceiver Settings -----------
	Serial.begin(115200);
	radio.begin();
    radio.setDataRate(RF24_250KBPS);
	radio.enableAckPayload();
    radio.setPALevel(RF24_PA_MAX);
    radio.setRetries(8,15);
    //---------------------------------------------------------------

	if(select) // robot
    {
        radio.openReadingPipe(1,pipe);
        radio.startListening();
        //------------------------ Ultrasound Sensors Settings -----------

        //sets ultrasonic ranging module pins, all 5 sensors share trigPin.
        pinMode(trigPin, OUTPUT);
        pinMode(echoPin0, INPUT);
        pinMode(echoPin1, INPUT);
        pinMode(echoPin2, INPUT);
        pinMode(echoPin3, INPUT);
        pinMode(echoPin4, INPUT);
        SensorSettings();
        // to avoid first mean values to be absurd take a few samples from the environment before running
        for(int i = 0; i < 10; i++)
            SensorReading();
        //---------------------------------------------------------------

        // --------------------- MOTORS SETTINGS -----------------------
        pinMode(leftmotor, OUTPUT);
        pinMode(rightmotor, OUTPUT);

        InitTimersSafe(); //initialize all timers except for 0, to save time keeping functions

        //sets the frequency for the specified pin
        SetPinFrequencySafe(leftmotor, frequency_l);
        SetPinFrequencySafe(rightmotor, frequency_r);

        pwmWrite(leftmotor, pwm_value_l);   // pwmWrite(pin, DUTY * 255 / 100);  
        pwmWrite(rightmotor, pwm_value_r);   // pwmWrite(pin, DUTY * 255 / 100);  
        delay(1); 
        pwmWrite(rightmotor, stop_pwm_r);   // pwmWrite(pin, DUTY * 255 / 100);  
        delay(1);
        pwmWrite(rightmotor, pwm_value_r);   // pwmWrite(pin, DUTY * 255 / 100);  
        delay(1);
        pwm_value_r = stop_pwm_r;
        pwmWrite(rightmotor, pwm_value_r);   // pwmWrite(pin, DUTY * 255 / 100);  

        // initialize command set
        intCmd = (char**)malloc(Ncommands*sizeof(char*));
        for(int i=0;i<Ncommands;i++)
            intCmd[i] = (char*)malloc(intCmd_size*sizeof(char));
        setInternalCommands(intCmd);

        buffer = (unsigned int**)malloc(buffer_size*sizeof(unsigned int*));
        for(int i = 0; i < buffer_size; i++)
            buffer[i] = (unsigned int*)malloc(5*sizeof(unsigned int));

        T = (short int*)malloc(32*sizeof(short int));
        initTable(T); // T: truth table
    }
    else // remote controller
    {
        radio.openWritingPipe(pipe);
        pinMode(ON, OUTPUT);
        pinMode(OFF, OUTPUT);
        pinMode(state, INPUT);

        digitalWrite(ON, HIGH);
        digitalWrite(OFF, LOW);

        copy[0] = '\0'; // TODO: is it really necessary???
    }



    t = millis();
}

void loop()
{
    communication();
}

/*---------------------------------  OBSTACLE AVOIDANCE STUFF    --------------------------------------------------------------------*/ 

void ObstacleAvoid()
{
    int obstacle;

    obstacle = SensorReading();

    if (obstacle < 0) // Danger Zone
    {
        //   STOP !!!
        stop();
    }
    else // Warning Zone and Safe Zone
        speedControl(obstacle);
}

// incluir tratativas de erro?
void SensorSettings()
{
    USS[0].Bit = digitalPinToBitMask(echoPin0);
    USS[0].port = digitalPinToPort(echoPin0);
    USS[0].pointer = 4;

    USS[1].Bit = digitalPinToBitMask(echoPin1);
    USS[1].port = digitalPinToPort(echoPin1);
    USS[1].pointer = 4;

    USS[2].Bit = digitalPinToBitMask(echoPin2);
    USS[2].port = digitalPinToPort(echoPin2);
    USS[2].pointer = 4;

    USS[3].Bit = digitalPinToBitMask(echoPin3);
    USS[3].port = digitalPinToPort(echoPin3);
    USS[3].pointer = 4;

    USS[4].Bit = digitalPinToBitMask(echoPin4);
    USS[4].port = digitalPinToPort(echoPin4);
    USS[4].pointer = 4;
    //TODO: change it back!!!
}


/*
       Cache the port and bit of the pins in order to speed up the
       pulse width measuring loop and achieve finer resolution.
       Calling digitalRead() instead yields much coarser resolution.
*/
int myPulseIn( bool dynamical_reading)
{
    /*TODO:
     * 1) check if it's possible to analyze all sensors at once:  *portInputRegister(USS[i].port)
     * 2) guarantee that micros() won't overflow during execution!!!
     */

    unsigned long initTime = millis();

    boolean finished = LOW; // finished reading all sensors data flag
    boolean previous_pulse[5]; // auxiliar variable that states if previous pulse has already ended or not.
    boolean reading[5];  // auxiliar variable that states if data is being read or not.

    for(int i = 0; i < 5; i++)
    {
        reading[i] = LOW;
        previous_pulse[i] = LOW;

        USS[i].data_available = LOW;
        USS[i].duration = 0;

        if(USS[i].pointer == 4)
            USS[i].pointer = 0;
        else
            USS[i].pointer = USS[i].pointer + 1;
    }

    // trigger
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    while (!finished) // wait for echo
    {
        for(int i = 0; i < 5; i++)
        {
            if (!(USS[i].data_available)) // if data hasn't already been read
            {
                if (previous_pulse[i]) // if previous pulse has ended
                {
                    if (reading[i]) // if data is being read
                    {
                        // if data has just finished being read
                        if ((*portInputRegister(USS[i].port) & USS[i].Bit) != USS[i].Bit)
                        {
                            USS[i].duration = micros() - USS[i].duration;
                            USS[i].distance[USS[i].pointer] = USS[i].duration / 58.2;
                            USS[i].data_available = HIGH;
                        }
                    }
                    else // if data hasn't started to be read
                    {
                        // check if it has just started.
                        if ((*portInputRegister(USS[i].port) & USS[i].Bit) == USS[i].Bit)
                        {
                            USS[i].duration = micros();
                            reading[i] = HIGH;
                        }
                    }
                }
                else
                {
                    // check if previous pulse has just ended and set previous_pulse if it has.
                    if ((*portInputRegister(USS[i].port) & USS[i].Bit) != USS[i].Bit)
                        previous_pulse[i] = HIGH;
                }
            }
        }

        if (millis() > initTime + time_out)
        {
            for(int i = 0; i < 5; i++) // if there is no echo, we assume there is nothing in front of us
            {
                if( !(USS[i].data_available) )
                    USS[i].distance[USS[i].pointer] = outOfRange;
            }
            return 0;
        }

        finished =  USS[0].data_available &  USS[1].data_available &  USS[2].data_available &  USS[3].data_available & USS[4].data_available;
    }

    if(!dynamical_reading)
        while(millis() < initTime + time_out) ; // TODO: wasted time, find out something useful to do here.

    return 1; // TODO: return 0 instead of 1!!!
}

/*
motor da direita sem torque
rodas sem aderência. Não usar X_F
*/

void initTable(short int *T)
{

    T[B00000] = Frente; // FullSpeed;

/**/T[B00001] = E_F; // E_L;  

    T[B00010] = E_F; // E_M;  
    T[B00011] = E_F; // E_M;  

/**/T[B00100] = E_F;  

    T[B00101] = E_F;  
    T[B00110] = E_F;  
    T[B00111] = E_F;  
    T[B01000] = D_F; // D_M;  
    T[B01001] = D_F; // D_M;  
    // T[B01010] = Frente;  
    T[B01010] = E_F;  
    T[B01011] = E_F; // E_M;  
    T[B01100] = D_F;  
    T[B01101] = E_F;  
    T[B01110] = E_F;  
    T[B01111] = E_F;  

/**/T[B10000] = D_F; // D_L;  

/**/T[B10001] = Frente;  

    T[B10010] = E_F; // E_M;  
    T[B10011] = E_F; // E_M;  

/**/T[B10100] = D_F;  

/**/T[B10101] = E_F;  

    //T[B10110] = D_F;  
    T[B10110] = E_F;  
    T[B10111] = E_F;  
    T[B11000] = D_F; // D_M;  
    T[B11001] = D_F; // D_M;  
    T[B11010] = D_F; // D_M;  
    T[B11011] = Frente;  
    T[B11100] = D_F;  
    T[B11101] = D_F;  
    T[B11110] = D_F;  
    T[B11111] = E_F;  
}

/*
    trocamos L por M.
    testar
    colocar M como F
*/

void speedControl(int obstacle)
{
//TODO: talvez compense tirar o switch e colocar if
    switch (T[obstacle])
    {
        case (E_L):
            pwmWrite(leftmotor, avgSpeed_l);
            pwmWrite(rightmotor, fullSpeed_r);
            break;
        case (E_M):
            pwmWrite(leftmotor, lowSpeed_l);
            pwmWrite(rightmotor, avgSpeed_r);
            break;
        case (E_F):
            pwmWrite(leftmotor, lowSpeed_l);
            pwmWrite(rightmotor, fullSpeed_r);
            break;

        case (D_L):
            pwmWrite(leftmotor, fullSpeed_l);
            pwmWrite(rightmotor, avgSpeed_r);
            break;

        case (D_M):
            pwmWrite(leftmotor, avgSpeed_l);
            pwmWrite(rightmotor, lowSpeed_r);
            break;

        case (D_F):
            pwmWrite(leftmotor, fullSpeed_l);
            pwmWrite(rightmotor, lowSpeed_r);
            break;

        case (Frente):
            pwmWrite(leftmotor, avgSpeed_l);
            pwmWrite(rightmotor, avgSpeed_r);
            break;

        case (FullSpeed):
            pwmWrite(leftmotor, fullSpeed_l);
            pwmWrite(rightmotor, fullSpeed_r);
            break;
        
        default: // not supposed to enter here
            write_ackPayload("ERROR!!!");
            stop();
            break;
    }
}

void getExtreamValues()
{
    for(int j = 0; j < 5; j++)
    {
        for(int i = 0; i < 5; i++)
        {
            if(j == 0)
            {
                USS[i].farthest = USS[i].distance[j];
                USS[i].closest = USS[i].distance[j];
                USS[i].mean = USS[i].distance[j];
            }
            else
            {
                if(USS[i].farthest < USS[i].distance[j])
                    USS[i].farthest = USS[i].distance[j];
                if(USS[i].closest > USS[i].distance[j])
                    USS[i].closest = USS[i].distance[j];
                USS[i].mean = USS[i].mean + USS[i].distance[j];
            }

            if(j == 4)
                USS[i].mean = ( USS[i].mean - USS[i].farthest - USS[i].closest ) / 3;
        }
    }
}

int SensorReading()
{
    myPulseIn(dyn);
    getExtreamValues();

    if(USS[0].mean < minDist)
        return -1;

    if(all_sensors)
    {
        if(USS[1].mean < minDist)
            return -1;
    }

    if(USS[2].mean < minDist)
        return -1;

    if(all_sensors)
    {
        if(USS[3].mean < minDist)
            return -1;
    }

    if(USS[4].mean < minDist)
        return -1;

    return obstacleShape();
}

int obstacleShape()
{
    int obstacle = 0;

    if (USS[0].mean < warningDist) 
        obstacle += B00001; // 1st bit <=> 1st sensor

    if(all_sensors)
    {
        if (USS[1].mean < warningDist) 
            obstacle += B00010; // 2nd bit <=> 2nd sensor
    }

    if (USS[2].mean < warningDist) 
        obstacle += B00100; // 3rd bit <=> 3rd sensor

    if(all_sensors)
    {
        if (USS[3].mean < warningDist) 
            obstacle += B01000; // 4th bit <=> 4th sensor
    }

    if (USS[4].mean < warningDist) 
        obstacle += B10000; // 5th bit <=> 5th sensor

    return obstacle;
}

/*-------------------------------------------------------------------------------------------------------------------------------------*/


/*==========================================  REMOTE CONTROL STUFF ===================================================================*/ 

void status()
{
/*----------------------------------------- DATA FORMAT: ---------------------------------------------------------------------------
(bool)left:(bool)right:(bool)speed:(bool)frequency:(int32_t)frequency_l:(uint8_t)pwm_value_l;(int32_t)frequency_r:(uint8_t)pwm_value_r
----------------------------------------------------------------------------------------------------------------------------------- */	
	char status[MaxPayload], aux[6]; //
	int i;

	status[0] = 48+left;
	status[1] = ' ';
	status[2] = '\0';

/*
	status[0] = 48+left;
	status[1] = 'L';
	status[2] = 48+frequency;
	status[3] = 'F';
	status[4] = 48+speed;
	status[5] = 'S';
	status[6] =  48+right;
	status[7] =  'R';
	status[8] = ':';
	status[9] = '\0';
*/

/*
	int2char(aux,frequency_l);
	append(status,aux);
	append(status,"F");
*/

	int2char(aux,pwm_value_l);
	append(status,aux);
	append(status,"S|");

	aux[0] = 48+right;
	aux[1] = ' ';
	aux[2] = '\0';
	append(status,aux);

/*
	int2char(aux,frequency_r);
	append(status,aux);
	append(status,"f");
*/

	int2char(aux,pwm_value_r);
	append(status,aux);
	append(status,"S ");

	aux[0] = 's';
	aux[1] = 48+speed;
	aux[2] = ' ';
	aux[3] = 'f';
	aux[4] = 48+frequency;
	aux[5] = '\0';
	append(status,aux);

	write_ackPayload(status);

}

void communication()
{
    if(select) // robot
    {
        if(!read_nonBlocking()) // if there is a message for us
            processing();
    }
    else // remote controller
    {
        if( write_nonBlocking() ) // failed to send?
           radio.write( cmd, sizeof(cmd) ); // try again. 
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
                write_ackPayload("left motor!");
                break;

            case 1: //	   right
                left = 0;
                right = 1;
                write_ackPayload("right motor!");
                break;

            case 2: //	   both
                left = 1;
                right = 1;
                write_ackPayload("both motors!");
                break;

            case 3: //	   speed
                speed = 1;
                frequency = 0;
                write_ackPayload("enter speed value.");
                break;

            case 4: //	   start
                write_ackPayload("here we go!");
                start();
                break;

            case 5: //	   stop
                stop();
                write_ackPayload("stop!!!");
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
                write_ackPayload("enter frequency value.");
                break;
            case 9: //	 safety button status 
                if(safety_button)
                    write_ackPayload("ON");
                else
                    write_ackPayload("OFF");
                break;
            case 10: // dist <=> print USS data  
                SensorsDataPrint();
                break;
            case 11: //	obstacle avoidance 
                write_ackPayload("autonomous navigation");
                autonomous();
                break;
            case 12: //	log
                logging();
                break;
                
            case 13: // test	
                test();
                break;
                
            case 14: // autonomous navigation without USS data feedback to remote controller
                quiet();
                break;

            case 15: // toggle the number of USS being used for decision making
                all_sensors = ! all_sensors;
                if(all_sensors)
                    write_ackPayload("using 5 USS.");
                else
                    write_ackPayload("using 3 USS.");
                break;
        }
}

void stop()
{
    pwmWrite(rightmotor, stop_pwm_r);  // pwmWrite(pin, DUTY * 255 / 100);
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
    char iteration = '0'; // check for packet loss
    bool safe = 0;

    t = millis();
    write_ackPayload("{");
    while(millis() < t + t_start)
    {
        if( SensorReading() < 0)
        {
            if(safe) // only write to PWM if its value will be changed
            {
                stop();
                safe = 0;
            }
        }
        else
        {
            if(!safe) // only write to PWM if its value will be changed
            {
                pwmWrite(rightmotor, pwm_value_r);
                pwmWrite(leftmotor, pwm_value_l);
                safe = 1;
            }

        }

        if(radio.available())
        {
            bool done = false;
            while (!done && millis() < t + t_start)
                done = radio.read( rcv, sizeof(rcv) );
        }

        if(rcv[0] == '-')
        {
            write_ackPayload("#");
            break;
        }

        if(rcv[0] == '+')
            t = millis();

        statusFeedback(iteration);
        rcv[0] = '\0';
        if (iteration == '9')
            iteration = '0';
        else 
            iteration++;
    }
    stop();
    write_ackPayload("#");
}

bool processing()
{
	int i;
	for(i = 0; i < Ncommands; i++) // search for a known command
		if(isSubstring(intCmd[i],rcv) == 1)
			break;

	if(i < Ncommands) // have we just got a command?
		action(i); // yes, perform i-th command 
	else // not a command, maybe it is data
    {
        int number = char2int(rcv);

        if( number > 0 ) 
        {
             if(speed)
            {
                if(left)
                {
                    pwm_value_l = number;
                    speed = 0;
                    if(right)
                        append(rcv," (both pwm)");
                    else
                        append(rcv," (left pwm)");
                    write_ackPayload(rcv);
                }
                if(right)
                {
                    pwm_value_r = number;
                    speed = 0;
                    if(!left)
                    {
                        append(rcv," (right pwm)");
                        write_ackPayload(rcv);
                    }
                }

                if( !(right || left) )
                    write_ackPayload("set for which motor?");

            }
            else if(frequency)
            {
                if(left)
                {
                    frequency_l = number;
                    frequency = 0;
                    SetPinFrequencySafe(leftmotor, frequency_l);
                    if(right)
                        append(rcv," (freq L and R)");
                    else
                        append(rcv," (freq L)");
                    write_ackPayload(rcv);
                }

                if(right)
                {
                    frequency_r = number;
                    frequency = 0;
                    SetPinFrequencySafe(rightmotor, frequency_r);
                    if(!left)
                    {
                        append(rcv," (freq R)");
                        write_ackPayload(rcv);
                    }
                }
                if( !(right || left) )
                    write_ackPayload("set for which motor?");
            }
            else // what is this number for?
                write_ackPayload(rcv); // send it back!
        }
        else if( (  (rcv[0]=='+' || rcv[0]=='-')  && (rcv[1]=='\0')  )  )  // have we received safety button status?
        {	
            radio.writeAckPayload( 1, cmd, sizeof(cmd) ); // resend previous message to remote controller, maybe the last one got lost...
            if(rcv[0] == '+')
                safety_button = 1;
            else
            {
                safety_button = 0;
                return 1;
            }
        }
        else // nothing recognizable, send it back
        {	
            write_ackPayload(rcv);
        }
    }

    return 0;
}

bool checkButton()
{
    if(digitalRead(state) == HIGH)
    {
        cmd[0] = '+';
        cmd[1] = '\0';
        return 1;
    }
    else
    {
        cmd[0] = '-';
        cmd[1] = '\0';
        return 0;
    }
}

void set_msg()
{
    // if(Serial.available() == 0 && cmd[0] == '\0')
    if( Serial.available() )
    {
        delay(1);
        int aux = 0;
        while(Serial.available() > 0)
            cmd[aux++] = Serial.read();
        cmd[aux] = '\0';

        if(aux != 0 && isSubstring("clear", cmd) )
        {
            for(int i = 0; i < 50; i++)
                Serial.println(" ");
            cmd[0] = '\0';
        }
    }
    else if(rcv[0] == '{') // metadata asking for safety button status
    {
            checkButton();
    }
    else if(millis() > t + t_max )
    {
        checkButton();
        t = millis();
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

void write_ackPayload(char *m)
{
    if( m[0] != '\0' )
    {
        copyString(m,cmd);
        radio.writeAckPayload( 1, cmd, sizeof(cmd) );
    }
}

int mypow(int base, int exponent)
{
    int result = 1;
    for(int i = 0; i < exponent; i++)
        result = result*base;
    return result;
}

// TODO: make it generic
int char2int(char *s)
{

    /*
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
     */

    int i = 0,j, r = 0, aux;

    for(i = 0; i < MaxPayload; i++)
    {
        if( !((s[i] >= '0') && (s[i] <= '9')) ) // is it a number?
        {
            if(s[i] == '\0')
                break;
            else
                return -1;
        }
    }

    if(i == MaxPayload) // number too big for this application
        return -3;

    if(i == 0) // empty string
        return -2;

    for(j=0; j < i; j++)
    {
        aux = (int) s[j];
        aux -= 48;
        aux *= (int) mypow(10,i-1-j);
        r += aux;
    }
    return r;
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

void append(char *o, char *p)
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
	copyString("freq",m[8]);
	copyString("button",m[9]);
	copyString("dist",m[10]);
	copyString("auto",m[11]);
	copyString("log",m[12]);
	copyString("test",m[13]);
	copyString("quiet",m[14]);

	copyString("USS",m[15]);
}

bool isSubstring(char *a, char *b)
{
	int i;

	if((a[0] == '\0')||(b[0] == '\0'))
	{
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

bool read_nonBlocking() // robot
{
	if(radio.available())
    {
        bool done = false;
        while (!done)
            done = radio.read( rcv, sizeof(rcv) );
        return 0; // success
    }
    return 1; // nothing to be read 
}

void read_Blocking() // robot
{
    while(!radio.available());
    bool done = false;
    while (!done)
        done = radio.read( rcv, sizeof(rcv) );
}

bool write_nonBlocking()
{
    bool check;
    if( radio.isAckPayloadAvailable() ) // should I do it outside this function??
    {
      radio.read(&rcv,sizeof(rcv));
      if(!isSubstring(copy,rcv) && rcv[0] != '\0' && rcv[0] != '#') // is it something new?
      {
          Serial.println(rcv);
          copyString(rcv,copy);
      }
    }

    set_msg();
    check = radio.write( cmd, sizeof(cmd));
    cmd[0] = '\0';

    return !check; // return 0 if successful.
}

void statusFeedback(char iteration)
{
    int OS = obstacleShape();
    char aux[15];

/*
     aux[0] = '{';
     aux[1] = iteration;
     aux[2] = '}';
     aux[3] = '\0';
     append(cmd,aux);
     append(cmd," ");
*/
     aux[0] = '{';
     aux[1] = '\0';
     append(cmd,aux);

    writeSensorsData();

    // Move
    if( (USS[0].mean < minDist)||(USS[0].mean < minDist)||(USS[2].mean < minDist)||(USS[3].mean < minDist)||(USS[4].mean < minDist) )
        append(cmd,"STP ");
    else
    {
        if(OS > 0)
            switch (T[OS])
            {
                case (E_L):
                    append(cmd,"EL ");
                    break;
                case (E_M):
                    append(cmd,"EM ");
                    break;
                case (E_F):
                    append(cmd,"EF ");
                    break;
                case (D_L):
                    append(cmd,"DL ");
                    break;
                case (D_M):
                    append(cmd,"DM ");
                    break;
                case (D_F):
                    append(cmd,"DF ");
                    break;
                case (Frente):
                    append(cmd,"Fr ");
                    break;
                case (FullSpeed):
                    append(cmd,"FS ");
                    break;
            }
        else
            append(cmd,"OR ");
    }

/*
    // Obstacle Shape
    if(OS >= 16)
    {
        OS = OS-16;
        if(USS[4].mean < minDist)
            append(cmd,"!");
        else
            append(cmd,"1");
    }
    else
        append(cmd,"0");

    if(OS >= 8)
    {
        OS = OS-8;
        if(USS[3].mean < minDist)
            append(cmd,"!");
        else
            append(cmd,"1");
    }
    else
        append(cmd,"0");

    if(OS >= 4)
    {
        OS = OS-4;
        if(USS[2].mean < minDist)
            append(cmd,"!");
        else
            append(cmd,"1");
    }
    else
        append(cmd,"0");

    if(OS >= 2)
    {
        OS = OS-2;
        if(USS[1].mean < minDist)
            append(cmd,"!");
        else
            append(cmd,"1");
    }
    else
        append(cmd,"0");

    if(OS >= 1)
    {
        OS = OS-1;
        if(USS[0].mean < minDist)
            append(cmd,"!");
        else
            append(cmd,"1");
    }
    else
        append(cmd,"0");
*/


    radio.writeAckPayload(1,cmd,sizeof(cmd));
    rcv[0] = '\0';
    cmd[0] = '\0';
    aux[0] = '\0';
}

// ser mais rigoroso ao parar o carrinho. Verificar se é somente um sensor com medidas erradas.
void autonomous()
{
    char status[MaxPayload], iteration = '0'; // check for packet loss
    status[0] = '\0';

    t = millis();
    write_ackPayload("{");
    while(millis() < t + t_start)
    {
        ObstacleAvoid();

        if(radio.available())
        {
            bool done = false;
            while (!done && millis() < t + t_start)
                done = radio.read( rcv, sizeof(rcv) );

            if(rcv[0] == '-')
            {
                stop();
                write_ackPayload("#");
                break;
            }

            if(rcv[0] == '+')
                t = millis();

            statusFeedback(iteration);
            rcv[0] = '\0';
            if (iteration == '9')
                iteration = '0';
            else 
                iteration++;
        }

    }
    stop();
    write_ackPayload("#");
}


void quiet()
{
    t = millis();
    while(millis() < t + t_start)
    {
        ObstacleAvoid();

        if(radio.available())
        {
            bool done = false;
            while (!done && millis() < t + t_start)
                done = radio.read( rcv, sizeof(rcv) );

            if(rcv[0] == '-')
            {
                stop();
                write_ackPayload("#");
                break;
            }

            if(rcv[0] == '+')
                t = millis();
        }

    }
    stop();
    write_ackPayload("#");
}

void logging()
{
    t = millis();
    while(millis() < t + t_start)
    {
        for(int i = 0; i < buffer_size; i++)
        {
            ObstacleAvoid();
            buffer[i][0] = USS[0].mean;
            buffer[i][1] = USS[1].mean;
            buffer[i][2] = USS[2].mean;
            buffer[i][3] = USS[3].mean;
            buffer[i][4] = USS[4].mean;
        }
        stop();

        flushLogData(0);
        for(int i = 1; i < buffer_size; i++)
        {
            read_Blocking();
            flushLogData(i);

            if(rcv[0] == '-')
            {
                write_ackPayload("#");
                stop();
                break;
            }

            if(rcv[0] == '+')
                t = millis();
        }

//TODO: check if indeed necessary...
        if(radio.available())
        {
            bool done = false;
            while (!done && millis() < t + t_start)
                done = radio.read( rcv, sizeof(rcv) );

            if(rcv[0] == '-')
            {
                stop();
                write_ackPayload("#");
                break;
            }

            if(rcv[0] == '+')
                t = millis();
        }
    }
    stop();
}

void flushLogData(int N_buffer)
{
    char aux[5];
    cmd[0] = '{'; 
    cmd[1] = '\0';
    int2char(aux,N_buffer);
    append(cmd,aux);
    append(cmd,"} ");
    int2char(aux,buffer[N_buffer][4]);
    append(cmd,aux);
    append(cmd,"|");
    int2char(aux,buffer[N_buffer][3]);
    append(cmd,aux);
    append(cmd,"|");
    int2char(aux,buffer[N_buffer][2]);
    append(cmd,aux);
    append(cmd,"|");
    int2char(aux,buffer[N_buffer][1]);
    append(cmd,aux);
    append(cmd,"|");
    int2char(aux,buffer[N_buffer][0]);
    append(cmd,aux);
    radio.writeAckPayload( 1, cmd, sizeof(cmd) );
}

void writeSensorsData()
{
    char aux[6];

    int2char(aux,USS[4].mean);
    append(cmd,aux);
    append(cmd,"|");

    int2char(aux,USS[3].mean);
    append(cmd,aux);

    append(cmd,"|");
    int2char(aux,USS[2].mean);
    append(cmd,aux);
    append(cmd,"|");

    int2char(aux,USS[1].mean);
    append(cmd,aux);

    append(cmd,"|");
    int2char(aux,USS[0].mean);
    append(cmd,aux);
    /*
    append(cmd,":");
    int2char(aux,USS[0].distance[USS[0].pointer]);
    append(cmd,aux);
    */

    append(cmd," ");
}

void SensorsDataPrint()
{
    int i;
    char status[MaxPayload], aux[6],iteration = '0';
    status[0] = '\0';

    myPulseIn(dyn);
    getExtreamValues();
    t = millis();
    while(millis() < t + t_start)
    {
        status[0] = '{';
        status[1] = iteration;
        status[2] = '}';
        status[3] = '\0';

        for (i = 0; i < 5; i++)
        {
            int2char(aux,USS[4-i].distance[USS[4-i].pointer]);
            //int2char(aux,USS[4-i].mean);
            append(status,aux);
            if(i<4)
                append(status,"|");
        }

        read_nonBlocking();
        if(rcv[0] == '-')
        {
            write_ackPayload("#");
            break;
        }
        if(rcv[0] == '+')
            t = millis();

        write_ackPayload(status);
        myPulseIn(dyn);
        getExtreamValues();
        rcv[0] = '\0';
        aux[0] = '\0';

        if (iteration == '9')
            iteration = '0';
        else 
            iteration++;
    }
}

void test()
{
    t = millis();
    while(millis() < t + t_start)
    {

        for(int i = 1; i < buffer_size; i = i+2)
        {
            SensorReading();
            buffer[i-1][0] = USS[0].distance[USS[0].pointer];
            buffer[i-1][1] = USS[1].distance[USS[1].pointer];
            buffer[i-1][2] = USS[2].distance[USS[2].pointer];
            buffer[i-1][3] = USS[3].distance[USS[3].pointer];
            buffer[i-1][4] = USS[4].distance[USS[4].pointer];

            buffer[i][0] = USS[0].mean;
            buffer[i][1] = USS[1].mean;
            buffer[i][2] = USS[2].mean;
            buffer[i][3] = USS[3].mean;
            buffer[i][4] = USS[4].mean;
        }

        flushLogData(0);

        for(int i = 1; i < buffer_size; i++)
        {
            read_Blocking();
            flushLogData(i);

            if(rcv[0] == '-')
            {
                write_ackPayload("#");
                break;
            }

            if(rcv[0] == '+')
                t = millis();
        }
    }
}
