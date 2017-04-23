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
#include <RF24.h>
#include <Time.h>
#include <PWM.h>

#include "string_handling.h"

/*=============================   DEFINES   ========================================== */
#define Ncommands 11 // number of commands available
// ------- radio pins ----------------------------
#define CE_PIN   5
#define CSN_PIN 6
// -------- motor related defines -----------------
#define leftmotor 10 // left motor pin
#define rightmotor 3 // right motor pin

#define stop_pwm_l 110 // pwm in which left motor is stopped
#define stop_pwm_r 90  // pwm in which right motor is stopped

int fullSpeed_l =  170;
int fullSpeed_r =  170;
int avgSpeed_l =  140;
int avgSpeed_r =  150;
int lowSpeed_l =  120;
int lowSpeed_r =  100;

#define buffer_size 50

// --------- deadlines (in millisseconds)----------------------------
#define t_max 75  // TODO: give it a better name
#define t_start 500 // deadline for loss of radio connection when running - autonomous() as well as start() -
#define time_out 30 // USS reading deadline <=> 6.8m (datasheet fala que só vai até 4m...)

// RADIO RELATED

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
#define minDist 50 // obstacle can't get any closer than minDist (in cm) from the robot
#define warningDist 350 // minDist < obstacle distance < warningDist => warning zone!
#define outOfRange 400

//TODO: nao esquecer de arrumar de novo!!!!

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
int frequency_r = 400;    // right motor pwm_value  = Motor Stopped   :  Pulse Width = 1ns
int frequency_l = 400;    // left motor frequency (in Hz)
int pwm_value_r = 330;    // right motor  pwm_value  = Motor Stopped   :  Pulse Width = 1ns
int pwm_value_l = 104;    // left motor  pwm_value  = Motor Stopped   :  Pulse Width = 1ns

bool dyn = 1; // dynamical/fixed USS echo reading
bool all_sensors = 0;

int obstacle_avoidance_method = 0;
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

    bool select = 0; // 0: master(remote control) ; 1: slave(robot) 

short int *T;                  // Truth Table

/*=======================================================================================================*/

/*========================= FUNCTIONS  ============================================================*/

void ObstacleAvoidanceTechnique(int obstacle_avoidance_method, int obstacle);


void autonomous();
// -------- communication oriented functions -------
void communication();
bool read_nonBlocking();
void read_Blocking();
void statusFeedback(char);
void writeSensorsData();

//---------- string handling functions -------------
bool ToBeSubstituted(char*);

void set_msg(); // transmits Serial.read() or safety button status - remote controller
void setInternalCommands(char**); // robot commands
bool processing(); // processes robot received data
void action(int); // perform received command - robot
void status(); // selected motor(s), pwm modulation rates and frequencies
void start(); // turn selected motors on
bool checkButton(); // check safety button status

//--------- obstacle avoidance related functions ---
void ObstacleAvoid();
void SensorSettings();
int myPulseIn(bool);
int SensorReading();
void getExtreamValues();
void printSensorsData();
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
    radio.setDataRate(RF24_1MBPS);
	radio.enableAckPayload();
    radio.setPALevel(RF24_PA_MAX);
    radio.setRetries(4,15);
    //---------------------------------------------------------------

	if(select) // robot
    {
        radio.openWritingPipe(pipe);
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
        radio.openReadingPipe(1,pipe);
        radio.startListening();
        pinMode(ON, OUTPUT);
        pinMode(OFF, OUTPUT);
        pinMode(state, INPUT);

        digitalWrite(ON, HIGH);
        digitalWrite(OFF, LOW);

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
        ObstacleAvoidanceTechnique(obstacle_avoidance_method, obstacle);
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
        //	processing distance, so state is set to LOW and duration initialized 0.
        USS[i].data_available = LOW;
        USS[i].duration = 0;
        // adjusting pointer
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

    // wait for echo
    while (!finished)
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
            // if there is no echo, we assume there is nothing in front of us
            for(int i = 0; i < 5; i++)
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
    T[B00000] = FullSpeed;

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
            ToBeSubstituted("ERROR!!!");
            stop();
            break;
    }
}

void getExtreamValues()
{
    // 0
    for(int j = 0; j< 5; j++)
        for(int i = 0; i< 5; i++)
        {
            USS[i].farthest = USS[i].distance[j];
            USS[i].closest = USS[i].distance[j];
            USS[i].mean = USS[i].distance[j];
        }

    for(int i = 0; i< 5; i++)
        USS[i].mean = ( USS[i].mean - USS[i].farthest - USS[i].closest ) / 3;
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
frequency:(int32_t)frequency_l:(uint8_t)pwm_value_l;(int32_t)frequency_r:(uint8_t)pwm_value_r
----------------------------------------------------------------------------------------------------------------------------------- */	
	char status[MaxPayload], aux[6]; //
	int i;

	status[0] = '\0';

	int2char(aux,pwm_value_l);
	append(status,aux);
	append(status,"S|");

	aux[0] = '\0';
	append(status,aux);

	int2char(aux,pwm_value_r);
	append(status,aux);
	append(status,"S ");

	aux[0] = '\0';
	append(status,aux);

	ToBeSubstituted(status);

}

void communication()
{
    if(select) // robot
    {
        if( radio.isAckPayloadAvailable() ) // should I do it outside this function??
        {
            radio.read(rcv,sizeof(rcv));
            processing();
        }
        else
        {
            ToBeSubstituted("!");
        }

    }
    else // remote controller
    {
        read_nonBlocking();
        set_msg();
    }
}

void action(int n)
{

	if(n < Ncommands)
		switch(n)
        {
            case 0: //	   left
                break;

            case 1: //	   right
                break;

            case 2: //	   both
                break;

            case 3: //	   speed
                break;

            case 4: //	   start
                ToBeSubstituted("here we go!");
                start();
                break;

            case 5: //	   stop
                stop();
                ToBeSubstituted("stop!!!");
                break;

            case 6: //	   sweep
                break;

            case 7: //	  status 
                status();
                break;

            case 8: //	 frequency 
                break;
            case 9: //	 safety button status 
                if(safety_button)
                    ToBeSubstituted("ON");
                else
                    ToBeSubstituted("OFF");
                break;
            case 10: // dist <=> print USS data  
                printSensorsData();
                break;
            case 11: //	obstacle avoidance 
                ToBeSubstituted("autonomous navigation");
                autonomous();
                break;
            case 12: //	log
                break;
                
            case 13: // test	
                break;
                
            case 14: // autonomous navigation without USS data feedback to remote controller
                break;

            case 15: // toggle the number of USS being used for decision making
                all_sensors = ! all_sensors;
                if(all_sensors)
                    ToBeSubstituted("using 5 USS.");
                else
                    ToBeSubstituted("using 3 USS.");
                break;
        }
}

void stop()
{
    pwmWrite(rightmotor, stop_pwm_r);  // pwmWrite(pin, DUTY * 255 / 100);
    pwmWrite(leftmotor, stop_pwm_l);
}

void start()
{
    char iteration = '0'; // check for packet loss
    bool safe = 0;

    t = millis();
    ToBeSubstituted("{");
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

        if(radio.isAckPayloadAvailable())
        {
            bool done = false;
            while (!done && millis() < t + t_start)
                done = radio.read( rcv, sizeof(rcv) );
        }

        if(rcv[0] == '-')
        {
            ToBeSubstituted("#");
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
    ToBeSubstituted("#");
}

bool processing()
{

	int i;
	for(i = 0; i < Ncommands; i++) // search for a known command
		if(isSubstring(intCmd[i],rcv) == 1)
			break;

	if(i < Ncommands) // have we just got a command?
		action(i); // yes, perform i-th command 
    else if(((rcv[0]=='+' || rcv[0]=='-')  && (rcv[1]=='\0')))  // have we received safety button status?
    {	
        if(rcv[0] == '+')
            safety_button = 1;
        else
        {
            safety_button = 0;
            return 1;
        }
    }
    else
        ToBeSubstituted(rcv);
    return 0;
}

bool checkButton()
{
    t = millis();
    if(digitalRead(state) == HIGH)
    {
        cmd[0] = '+';
        cmd[1] = '\0';
        radio.writeAckPayload(1,cmd,sizeof(cmd));
        return 1;
    }
    else
    {
        cmd[0] = '-';
        cmd[1] = '\0';
        radio.writeAckPayload(1,cmd,sizeof(cmd));
        return 0;
    }
}

void set_msg()
{
    if( Serial.available() )
    {
        delay(5);
        int aux = 0;
        while(Serial.available() > 0)
            if(aux < MaxPayload)
                cmd[aux++] = Serial.read();
            else
                break;
        cmd[aux] = '\0';

        if(aux != 0 && isSubstring("clear", cmd) )
        {
            for(int i = 0; i < 50; i++)
                Serial.println(" ");
            cmd[0] = '\0';
        }
        else if(cmd[0] != '\0')
            radio.writeAckPayload(1,cmd,sizeof(cmd));
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
	copyString("USS",m[12]);
}


bool read_nonBlocking() // remote controller
{
	if(radio.available())
    {
        bool done = false;
        while (!done)
            done = radio.read( rcv, sizeof(rcv) );

        if(rcv[0] != '!') // if robot not waiting for command
            Serial.println(rcv);

        return 0; // success
    } 
    else if(millis() > t + t_max)
        checkButton();

    return 1; // nothing to be read 
}

void read_Blocking() // robot
{
    while(!radio.available());
    bool done = false;
    while (!done)
        done = radio.read( rcv, sizeof(rcv) );
}


void statusFeedback(char iteration)
{
    int OS = obstacleShape();
    char aux[15];

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
            }
        else
            append(cmd,"OR ");
    }

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
    ToBeSubstituted("{");
    while(millis() < t + t_start)
    {
        ObstacleAvoid();

        if(radio.isAckPayloadAvailable())
        {
            bool done = false;
            while (!done && millis() < t + t_start)
                done = radio.read( rcv, sizeof(rcv) );

            if(rcv[0] == '-')
            {
                stop();
                ToBeSubstituted("#");
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
    ToBeSubstituted("#");
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
    append(cmd," ");
}

void printSensorsData()
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
            int2char(aux,USS[i].distance[USS[i].pointer]);
            //int2char(aux,USS[i].mean);
            append(status,aux);
            if(i<4)
                append(status,"|");
        }

        while(!ToBeSubstituted(status));
        if(radio.isAckPayloadAvailable());
        {
            bool done = false;
            while (!done)
                done = radio.read( rcv, sizeof(rcv) );

            if(rcv[0] == '-')
            {
                stop();
                ToBeSubstituted("#");
                break;
            } 
            else if(rcv[0] == '+')
            {
                t = millis();
            }

            rcv[0] = '\0';
        }

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

void ObstacleAvoidanceTechnique(int obstacle_avoidance_method, int obstacle)
{
    switch(obstacle_avoidance_method)
    {
        case 0:
            speedControl(obstacle);
            break;
        case 1:
            proportionalController();
            break;
        default:
            speedControl(obstacle);
    }
}

void proportionalController()
{
    /*
                            TODO
        o sensor com menor valor lido sera levado em consideração, não a média
        usa 3 pra esquerda e 2 pra direita
        ACERTAR O PRINT DOS DADOS PARA QUE ELE RETORNE AS VELOCIDADES
    */
    char status[20], aux[5];
    int leftSpeed = ( ( (USS[0].mean + USS[1].mean)/2 - 100 )*(avgSpeed_l - 120) )/(400 - 100);
    int rightSpeed = ( ( (USS[3].mean + USS[4].mean)/2 - 100 )*(avgSpeed_r  - 100) )/(400 - 100);

    pwmWrite(leftmotor, leftSpeed);
    pwmWrite(rightmotor, rightSpeed);

	int2char(aux,leftSpeed);
    append(status,aux);
	int2char(aux,rightSpeed);
    append(status,aux);
    ToBeSubstituted(status);

}

bool ToBeSubstituted(char *m)
{
    if( m[0] != '\0' )
    {
        bool check;
        copyString(m,cmd);
        check = radio.write(cmd, sizeof(cmd) );
        return check;
    }
    return 0;
}
