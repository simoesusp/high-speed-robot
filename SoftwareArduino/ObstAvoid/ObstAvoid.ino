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
#define Ncommands 8 // number of commands available
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


// --------- deadlines (in millisseconds)----------------------------
#define t_max 100  // TODO: give it a better name
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


// -------- motor related variables -----------------
//TODO: maybe we could use uint8_t to save space
int frequency_r = 400;    // right motor pwm_value  = Motor Stopped   :  Pulse Width = 1ns
int frequency_l = 400;    // left motor frequency (in Hz)
int pwm_value_r = 330;    // right motor  pwm_value  = Motor Stopped   :  Pulse Width = 1ns
int pwm_value_l = 104;    // left motor  pwm_value  = Motor Stopped   :  Pulse Width = 1ns

bool dyn = 1; // dynamical/fixed USS echo reading
bool all_sensors = 1;

int obstacle_avoidance_method = 0;
// -------- communication related variables -----------------
const uint64_t pipe = 0xDEDEDEDEE7LL; // Define the transmit pipe (Obs.: "LL" => "LongLong" type)
RF24 radio(CE_PIN, CSN_PIN); // Create a Radio Object
char rcv[MaxPayload]; // message received
char cmd[MaxPayload]; // message transmitted
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

struct button_t
{
    uint8_t Bit;
    uint8_t port;
}Button;

bool safety_button; // robot: button status 

    bool select = 1; // 0: master(remote control) ; 1: slave(robot) 

short int *T;                  // Truth Table

/*=======================================================================================================*/

/*========================= FUNCTIONS  ============================================================*/

void ObstacleAvoidanceTechnique(int obstacle_avoidance_method, int obstacle);
void pwm_write(int , int );


void autonomous();
// -------- communication oriented functions -------
void communication();
bool read_nonBlocking();
void read_blocking();
bool write_nonBlocking();
void statusFeedback(char);
void writeSensorsData();

//---------- string handling functions -------------
void write_ackPayload(char*);

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
    radio.setDataRate(RF24_2MBPS);
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

        pwm_write(leftmotor, pwm_value_l);   // pwmWrite(pin, DUTY * 255 / 100);  
        pwm_write(rightmotor, pwm_value_r);   // pwmWrite(pin, DUTY * 255 / 100);  
        delay(1); 
        pwm_write(rightmotor, stop_pwm_r);   // pwmWrite(pin, DUTY * 255 / 100);  
        delay(1);
        pwm_write(rightmotor, pwm_value_r);   // pwmWrite(pin, DUTY * 255 / 100);  
        delay(1);
        pwm_value_r = stop_pwm_r;
        pwm_write(rightmotor, pwm_value_r);   // pwmWrite(pin, DUTY * 255 / 100);  

        // initialize command set
        intCmd = (char**)malloc(Ncommands*sizeof(char*));
        for(int i=0;i<Ncommands;i++)
            intCmd[i] = (char*)malloc(intCmd_size*sizeof(char));
        setInternalCommands(intCmd);

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

        Button.Bit = digitalPinToBitMask(state);
        Button.port  = digitalPinToPort(state); 
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
    switch (T[obstacle])
    {
        case (E_L):
            pwm_write(leftmotor, avgSpeed_l);
            pwm_write(rightmotor, fullSpeed_r);
            break;
        case (E_M):
            pwm_write(leftmotor, lowSpeed_l);
            pwm_write(rightmotor, avgSpeed_r);
            break;
        case (E_F):
            pwm_write(leftmotor, lowSpeed_l);
            pwm_write(rightmotor, fullSpeed_r);
            break;

        case (D_L):
            pwm_write(leftmotor, fullSpeed_l);
            pwm_write(rightmotor, avgSpeed_r);
            break;

        case (D_M):
            pwm_write(leftmotor, avgSpeed_l);
            pwm_write(rightmotor, lowSpeed_r);
            break;

        case (D_F):
            pwm_write(leftmotor, fullSpeed_l);
            pwm_write(rightmotor, lowSpeed_r);
            break;

        case (Frente):
            pwm_write(leftmotor, avgSpeed_l);
            pwm_write(rightmotor, avgSpeed_r);
            break;

        case (FullSpeed):
            pwm_write(leftmotor, fullSpeed_l);
            pwm_write(rightmotor, fullSpeed_r);
            break;
        
        default: // not supposed to enter here
            write_ackPayload("ERROR!!!");
            stop();
            break;
    }
}

void getExtreamValues()
{
    for(int i = 0; i< 5; i++)
    {
        USS[i].farthest = USS[i].distance[0];
        USS[i].closest = USS[i].distance[0];
        USS[i].mean = USS[i].distance[0];
    }

    for(int j = 1; j< 5; j++)
        for(int i = 0; i< 5; i++)
        {
            if( USS[i].farthest < USS[i].distance[j])
                 USS[i].farthest = USS[i].distance[j];
            if(USS[i].closest > USS[i].distance[j])
                USS[i].closest = USS[i].distance[j];
            USS[i].mean += USS[i].distance[j];
        }

    for(int i = 0; i< 5; i++)
        USS[i].mean = ( USS[i].mean - USS[i].farthest - USS[i].closest ) / 3;
}

int SensorReading()
{
    myPulseIn(dyn);
    getExtreamValues();

    if((USS[0].mean < minDist) || (USS[2].mean < minDist) || (USS[4].mean < minDist))
        return -1;

    if(all_sensors)
    {
        if((USS[1].mean < minDist) ||(USS[3].mean < minDist))
            return -1;
    }

    return obstacleShape();
}

int obstacleShape()
{
    int obstacle = 0;

    if (USS[0].mean < warningDist) 
        obstacle += B00001; // 1st bit <=> 1st sensor

    if(all_sensors)
        if (USS[1].mean < warningDist) 
            obstacle += B00010; // 2nd bit <=> 2nd sensor

    if (USS[2].mean < warningDist) 
        obstacle += B00100; // 3rd bit <=> 3rd sensor

    if(all_sensors)
        if (USS[3].mean < warningDist) 
            obstacle += B01000; // 4th bit <=> 4th sensor

    if (USS[4].mean < warningDist) 
        obstacle += B10000; // 5th bit <=> 5th sensor

    return obstacle;
}

/*-------------------------------------------------------------------------------------------------------------------------------------*/


/*========================================  REMOTE CONTROL STUFF ===================================================================*/ 

void status()
{

	char status[MaxPayload], aux[6]; //
	int i;

	status[0] = '\0';

	int2char(aux,lowSpeed_l);
	append(status,aux);
	append(status,"|");

	aux[0] = '\0';
	int2char(aux,avgSpeed_l);
	append(status,aux);
	append(status,"|");

	int2char(aux,fullSpeed_l);
	append(status,aux);
	append(status,"&&");

	int2char(aux,lowSpeed_r);
	append(status,aux);
	append(status,"|");

	aux[0] = '\0';
	int2char(aux,avgSpeed_r);
	append(status,aux);
	append(status,"|");

	int2char(aux,fullSpeed_r);
	append(status,aux);
	aux[0] = '\0';
	append(status,aux);

	write_ackPayload(status);
}

void communication()
{
    if(select) // robot
    {
        read_blocking();
        processing();
    }
    else // remote controller
        if( write_nonBlocking() )
            Serial.println("fail");
}


void action(int n)
{

	if(n < Ncommands)
		switch(n)
        {
            case 0: //	  start 
                if( parse_command() )
                    write_ackPayload("invalid syntax");
                else
                    status();
                break;

            case 1: //	  start 
                write_ackPayload("here we go!");
                start();
                break;

            case 2: //	  status 
                status();
                break;

            case 3: //	 safety button status 
                if(safety_button)
                    write_ackPayload("ON");
                else
                    write_ackPayload("OFF");
                break;

            case 4: // display USS reading
                printSensorsData();
                break;

            case 5: //	obstacle avoidance 
                write_ackPayload("autonomous navigation");
                autonomous();
                break;

            case 6: // toggle the number of USS being used for decision making
                all_sensors = ! all_sensors;
                if(all_sensors)
                    write_ackPayload("using 5 USS.");
                else
                    write_ackPayload("using 3 USS.");
                break;
            case 7: // change obstacle avoidance method
                if( obstacle_avoidance_method == 1)
                {
                    obstacle_avoidance_method = 0;
                    write_ackPayload("default.");
                }
                else
                {
                    obstacle_avoidance_method = 1;
                    write_ackPayload("proportional controller");
                }
                break;
        }
}

void stop()
{
    pwm_write(rightmotor, stop_pwm_r);  // pwmWrite(pin, DUTY * 255 / 100);
    pwm_write(leftmotor, stop_pwm_l);
}

void start()
{
    char iteration = 'a'; // check for packet loss
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
                pwm_write(rightmotor, pwm_value_r);
                pwm_write(leftmotor, pwm_value_l);
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
        if (iteration == 'z')
            iteration = 'a';
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
            if(isEqual(intCmd[i],rcv) == 1 || i == 0)
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
        write_ackPayload(rcv);
    return 0;
}

bool checkButton()
{
    //if (*portInputRegister(Button.port) & Button.Bit)
    if(digitalRead(state))
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
    cmd[0] = '\0';
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
    }
    else if(millis() > t + t_max )
    {
        checkButton();
        t = millis();
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

void setInternalCommands(char**m)
{

	copyString(":",m[0]);
	copyString("start",m[1]);
	copyString("status",m[2]);
	copyString("button",m[3]);
	copyString("dist",m[4]);
	copyString("auto",m[5]);
	copyString("USS",m[6]);
	copyString("nav",m[7]);
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

void read_blocking() // robot
{
    while(!radio.available());
    bool done = false;
    while (!done)
        done = radio.read( rcv, sizeof(rcv) );
}

bool write_nonBlocking()
{
    bool check = 1;
    if( radio.isAckPayloadAvailable() ) // should I do it outside this function??
    {
      // radio.read(&rcv,sizeof(rcv));
      // {
        bool done = 0;
        while (!done)
            done = radio.read( rcv, sizeof(rcv) );
          Serial.println(rcv);
      // }
    }

    set_msg();
    if(cmd[0] != '\0')
        check = radio.write( cmd, sizeof(cmd));

    return !check; // return 0 if successful.
}

void statusFeedback(char iteration)
{
    char aux[5];
    int percentualSpeed;
    cmd[0] = '\0';
    aux[0] = '\0';

     aux[0] = '{';
     aux[1] = iteration;
     aux[2] = '}';
     aux[3] = '\0';
     append(cmd,aux);

    writeSensorsData();

    if(pwm_value_l > stop_pwm_l && pwm_value_r > stop_pwm_r)
    {
        append(cmd," L");
        percentualSpeed = 100*(pwm_value_l - lowSpeed_l)/(fullSpeed_l - lowSpeed_l);
        int2char(aux,percentualSpeed);
        append(cmd,aux);
        append(cmd, "R");
        percentualSpeed = 100*(pwm_value_r - lowSpeed_r)/(fullSpeed_r - lowSpeed_r);
        int2char(aux,percentualSpeed);
        append(cmd,aux);
    }
    else 
        append(cmd," STP");
/*
    append(cmd," L");
    int2char(aux,pwm_value_l);
    append(cmd,aux);
    append(cmd, "R");
    int2char(aux,pwm_value_r);
    append(cmd,aux);
*/
    
    write_ackPayload(cmd);
}

void autonomous()
{
    char status[MaxPayload], iteration = 'a'; // check for packet loss
    status[0] = '\0';

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
                break;

            if(rcv[0] == '+')
                t = millis();

            statusFeedback(iteration);
            rcv[0] = '\0';
            if (iteration == 'z')
                iteration = 'a';
            else 
                iteration++;
        }

    }
    stop();
    if(rcv[0] == '-')
        write_ackPayload("button is OFF");
    else
        write_ackPayload("time out");
}


void writeSensorsData()
{
    char aux[6];

    for(int i = 0; i < 5; i++)
    {
        int2char(aux,USS[i].mean);
        append(cmd,aux);
        if(i < 4)
            append(cmd,"|");
        else
            append(cmd," ");
    }
}

void printSensorsData()
{
    int i;
    char status[MaxPayload], aux[6],iteration = 'a';
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
            append(status,aux);
            if(i<4)
                append(status,"|");
        }

        read_nonBlocking();
        if(rcv[0] == '-')
            break;
        else if(rcv[0] == '+')
            t = millis();

        write_ackPayload(status);
        myPulseIn(dyn);
        getExtreamValues();
        rcv[0] = '\0';
        aux[0] = '\0';

        if (iteration == 'z')
            iteration = 'a';
        else 
            iteration++;
    }
    if(rcv[0] == '-')
        write_ackPayload("button is OFF");
    else
        write_ackPayload("time out");
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
    int obst_to_the_left, obst_to_the_right, leftSpeed, rightSpeed;
/*
    int leftSpeed = ( ( (USS[0].mean + USS[1].mean)/2 - 100 )*(avgSpeed_l - 120) )/(400 - 100);
    int rightSpeed = ( ( (USS[3].mean + USS[4].mean)/2 - 100 )*(avgSpeed_r  - 100) )/(400 - 100);
*/
    if(USS[0].mean < USS[1].mean && USS[0].mean < USS[2].mean)
        obst_to_the_right = USS[0].mean;
    else if(USS[1].mean < USS[0].mean && USS[1].mean < USS[2].mean)
        obst_to_the_right = USS[1].mean;
    else
        obst_to_the_right = USS[2].mean;

    if(USS[3].mean < USS[4].mean)
        obst_to_the_left = USS[3].mean;
    else
        obst_to_the_left = USS[4].mean;
    
    leftSpeed = lowSpeed_l + (fullSpeed_l - lowSpeed_l)*(obst_to_the_right - minDist)/(outOfRange - minDist);
    rightSpeed = lowSpeed_r + (fullSpeed_r - lowSpeed_r)*(obst_to_the_left - minDist)/(outOfRange - minDist);

    pwm_write(leftmotor, leftSpeed);
    pwm_write(rightmotor, rightSpeed);
}


bool parse_command() // expected command format  => ':l[0-999]a[0-999]f[0-999]|l[0-999]a[0-999]f[0-999]'
{
    char preset_speeds[4];
    preset_speeds[0] = 'l'; // low_speed
    preset_speeds[1] = 'a'; // avg_speed
    preset_speeds[2] = 'f'; // full_speed

    int new_speed[6];

    int next = 0;
    if(rcv[1+next] == 'l')
    {
        char aux[4];
        aux[0] = rcv[2+next];
        aux[1] = rcv[3+next];
        aux[2] = rcv[4+next];
        aux[3] = '\0';
        new_speed[0] = char2int(aux);
    }
    else
        return 1;

    next = 4;
    if(rcv[1+next] == 'a')
    {
        char aux[4];
        aux[0] = rcv[2+next];
        aux[1] = rcv[3+next];
        aux[2] = rcv[4+next];
        aux[3] = '\0';
        new_speed[1] = char2int(aux);
    }
    else
        return 1;

    next = 8;
    if(rcv[1+next] == 'f')
    {
        char aux[4];
        aux[0] = rcv[2+next];
        aux[1] = rcv[3+next];
        aux[2] = rcv[4+next];
        aux[3] = '\0';
        new_speed[2] = char2int(aux);
    }
    else
        return 1;

    if(rcv[13] != '|')
        return 1;

    next = 13;
    if(rcv[1+next] == 'l')
    {
        char aux[4];
        aux[0] = rcv[2+next];
        aux[1] = rcv[3+next];
        aux[2] = rcv[4+next];
        aux[3] = '\0';
        new_speed[3] = char2int(aux);
    }
    else
        return 1;

    next = 17;
    if(rcv[1+next] == 'a')
    {
        char aux[4];
        aux[0] = rcv[2+next];
        aux[1] = rcv[3+next];
        aux[2] = rcv[4+next];
        aux[3] = '\0';
        new_speed[4] = char2int(aux);
    }
    else
        return 1;

    next = 21;
    if(rcv[1+next] == 'f')
    {
        char aux[4];
        aux[0] = rcv[2+next];
        aux[1] = rcv[3+next];
        aux[2] = rcv[4+next];
        aux[3] = '\0';
        new_speed[5] = char2int(aux);
    }
    else
        return 1;

    // valid syntax, let's write to the real variables in charge of motor speed
    lowSpeed_l = new_speed[0];
    avgSpeed_l = new_speed[1];
    fullSpeed_l = new_speed[2];
    lowSpeed_r = new_speed[3];
    avgSpeed_r = new_speed[4];
    fullSpeed_r = new_speed[5];

    return 0;
}


void pwm_write(int whichmotor, int speed)
{
    pwmWrite(whichmotor,speed);
    if(whichmotor == leftmotor)
        pwm_value_l = speed;
    else if(whichmotor == rightmotor)
        pwm_value_r = speed;

}
