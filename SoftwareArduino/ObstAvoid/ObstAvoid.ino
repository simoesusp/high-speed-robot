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
#define Ncommands 13 // number of commands available
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
#define dt 1 // time increment for the deadlock handling routine
#define t_min 1
#define t_max 100 
#define t_com 500 // deadline for performing communication (snd & rcv)
#define t_start 1000 // deadline for loss of radio connection when running - autonomous() as well as start() -
#define n_max 10 // number of attemptives to handle deadlock

#define time_out 30 // USS reading deadline <=> 6.8m (datasheet fala que só vai até 4m...)
// maximum time (milliseconds) allowed to wait for the previous pulse in sensors echoPin to end

// RADIO RELATED

#define MaxPayload 32 // Maximum message size (32 bytes)
#define intCmd_size 9
// ---------- safety button pins -----------------
#define ON  2// always high
#define OFF 4 // always low 
#define state 3 // says if in trouble or safe!

// ----------- USS Pins --------------------------
              // Obs.: non PWM digital pins were chosen to be sensor pins.
#define trigPin 2
#define echoPin0 8
#define echoPin1 7
#define echoPin2 A2
#define echoPin3 A1 
#define echoPin4 A0 
// ---------- obstacle avoidance related defines ---

//TODO: nao esquecer de arrumar de novo!!!!
#define minDist 150 // obstacle can't get any closer than minDist (in cm) from the robot
#define warningDist 250 // minDist < obstacle distance < warningDist => warning zone!
#define outOfRange 400


#define E_L 0 // turn softly to the left
#define E_M 1 // turn to the left
#define E_F 2 // turn abruptly to the right
#define Frente 3 // straight ahead
#define D_L 4 // turn softly to the right
#define D_M 5 // turn to the right
#define D_F 6 // turn abruptly to the right

/*======================================================================= */

/*========================= GLOBAL VARIABLES ============================================================*/

unsigned int **buffer;

// -------- motor related variables -----------------
/*int32_t*/ int frequency_r = 400;    // right motor pwm_value  = Motor Stopped   :  Pulse Width = 1ns
/*int32_t*/ int frequency_l = 400;    // left motor frequency (in Hz)
/*uint8_t*/ int pwm_value_r = 330;    // right motor  pwm_value  = Motor Stopped   :  Pulse Width = 1ns
/*uint8_t*/ int pwm_value_l = 104;    // left motor  pwm_value  = Motor Stopped   :  Pulse Width = 1ns

bool left; // left motor on/off flag
bool right; // right motor on/off flag
bool speed; // if true, try to read pwm_value from remote control
bool frequency;// if true, try to read frequency from remote control
bool logging_flag;

// -------- communication related variables -----------------
const uint64_t pipes[2] = {0xDEDEDEDEE7LL, 0xDEDEDEDEE9LL}; // Define the transmit pipe (Obs.: "LL" => "LongLong" type)
RF24 radio(CE_PIN, CSN_PIN); // Create a Radio Object
bool reading; // reading/writing flag
char rcv[MaxPayload]; // message received
char msg[MaxPayload]; // used by remote control
char cmd[MaxPayload]; // used by the robot
char **intCmd; // internal commands
long randNumber; // used by deadlock handling function
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

bool safety_button;

bool select = 1; // 0: master(remote control) ; 1: slave(robot) 

short int *T;                  // Truth Table

/*=======================================================================================================*/

/*========================= FUNCTIONS  ============================================================*/
void autonomous();
void logging();
void flushLogData(int);
// -------- communication oriented functions -------
void communication(bool mode);
bool Read_nonBlocking();
bool Read_blocking();
bool Write_nonBlocking(bool);
bool Write_blocking(bool);
void deadlockHandling(unsigned int);
void statusFeedback(char);
void writeSensorsData();

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
int myPulseIn();
int SensorReading();
void measuring();
void getExtreamValues();
void SensorsDataPrint();
void SensorsDataSerialPrint();
int obstacleShape();
void speedControl(int);
void initTable(short int *);
/*=======================================================================================================*/
// int mypow(int , int );

void setup()
{
    //------------------------ nRF24L01+ transceiver Settings -----------
	//Serial.begin(9600);
	Serial.begin(115200);
	radio.begin();
    radio.setRetries(1,15);
    radio.setDataRate(RF24_250KBPS);
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

        logging_flag = LOW;
    }
    else // remote control
    {
        pinMode(ON, OUTPUT);
        pinMode(OFF, OUTPUT);
        pinMode(state, INPUT);

        digitalWrite(ON, HIGH);
        digitalWrite(OFF, LOW);
    }

    T = (short int*)malloc(32*sizeof(short int));
    initTable(T); // T: truth table
}

void loop()
{
    communication(select);
}

/*---------------------------------  OBSTACLE AVOIDANCE STUFF    --------------------------------------------------------------------*/ 

void ObstacleAvoid()
{
    int obstacle;

    obstacle = SensorReading();

    // Danger Zone
    if (obstacle < 0)
    {
        //   STOP !!!
        stop();
    }

    // Warning Zone
    if (obstacle > 0)
        speedControl(obstacle);

    // Safe Zone
    if(obstacle == 0) // TODO: maybe it could be incorporated in the last if...
    {
        pwmWrite(leftmotor, fullSpeed_l);
        pwmWrite(rightmotor, fullSpeed_r);
    }
}

// incluir tratativas de erro?
void SensorSettings()
{
    USS[0].Bit = digitalPinToBitMask(echoPin0);
    USS[0].port = digitalPinToPort(echoPin0);
    USS[0].pointer = 0;

    USS[1].Bit = digitalPinToBitMask(echoPin1);
    USS[1].port = digitalPinToPort(echoPin1);
    USS[1].pointer = 0;

    USS[2].Bit = digitalPinToBitMask(echoPin2);
    USS[2].port = digitalPinToPort(echoPin2);
    USS[2].pointer = 0;

    USS[3].Bit = digitalPinToBitMask(echoPin3);
    USS[3].port = digitalPinToPort(echoPin3);
    USS[3].pointer = 0;

    USS[4].Bit = digitalPinToBitMask(echoPin4);
    USS[4].port = digitalPinToPort(echoPin4);
    USS[4].pointer = 0;

    for(int i = 0; i < 5; i++)
        for(int j = 0; j < 5; j++)
            USS[i].distance[j] = 0;
}


/*
       Cache the port and bit of the pins in order to speed up the
       pulse width measuring loop and achieve finer resolution.
       Calling digitalRead() instead yields much coarser resolution.
*/
int myPulseIn()
{
/*TODO:
 * 1) check if it's possible to analyze all sensors at once:  *portInputRegister(USS[i].port)
 * 2) guarantee that micros() won't overflow during execution!!!
 */
    unsigned long initTime = millis();

    boolean finished = LOW; // finished reading all sensors data flag
    boolean previous_pulse[5]; // auxiliar variable that states if previous pulse has already ended or not.
    boolean reading[5];  // auxiliar variable that states if data is being read or not.

    reading[0] = LOW;
    previous_pulse[0] = LOW;

    reading[1] = LOW;
    previous_pulse[1] = LOW;

    reading[2] = LOW;
    previous_pulse[2] = LOW;

    reading[3] = LOW;
    previous_pulse[3] = LOW;

    reading[4] = LOW;
    previous_pulse[4] = LOW;


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

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

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
                        USS[0].data_available = HIGH;
                    }
                }
                else // if data hasn't started to be read
                {
                    // check if it has just started.
                    if ((*portInputRegister(USS[0].port) & USS[0].Bit) == USS[0].Bit)
                    {
                        USS[0].duration = micros();
                        reading[0] = HIGH;
                    }
                }
            }
            else
            {
                // check if previous pulse has just ended and set previous_pulse if it has.
                if ((*portInputRegister(USS[0].port) & USS[0].Bit) != USS[0].Bit)
                    previous_pulse[0] = HIGH;
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
                        USS[1].data_available = HIGH;
                    }
                }
                else // if data hasn't started to be read
                {
                    // check if it has just started.
                    if ((*portInputRegister(USS[1].port) & USS[1].Bit) == USS[1].Bit)
                    {
                        USS[1].duration = micros();
                        reading[1] = HIGH;
                    }
                }
            }
            else
            {
                // check if previous pulse has just ended and set previous_pulse if it has.
                if ((*portInputRegister(USS[1].port) & USS[1].Bit) != USS[1].Bit)
                    previous_pulse[1] = HIGH;
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
                        USS[2].data_available = HIGH;
                    }
                }
                else // if data hasn't started to be read
                {
                    // check if it has just started.
                    if ((*portInputRegister(USS[2].port) & USS[2].Bit) == USS[2].Bit)
                    {
                        USS[2].duration = micros();
                        reading[2] = HIGH;
                    }
                }
            }
            else
            {
                // check if previous pulse has just ended and set previous_pulse if it has.
                if ((*portInputRegister(USS[2].port) & USS[2].Bit) != USS[2].Bit)
                    previous_pulse[2] = HIGH;
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
                        USS[3].data_available = HIGH;
                    }
                }
                else // if data hasn't started to be read
                {
                    // check if it has just started.
                    if ((*portInputRegister(USS[3].port) & USS[3].Bit) == USS[3].Bit)
                    {
                        USS[3].duration = micros();
                        reading[3] = HIGH;
                    }
                }
            }
            else
            {
                // check if previous pulse has just ended and set previous_pulse if it has.
                if ((*portInputRegister(USS[3].port) & USS[3].Bit) != USS[3].Bit)
                    previous_pulse[3] = HIGH;
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
                        USS[4].data_available = HIGH;
                    }
                }
                else // if data hasn't started to be read
                {
                    // check if it has just started.
                    if ((*portInputRegister(USS[4].port) & USS[4].Bit) == USS[4].Bit)
                    {
                        USS[4].duration = micros();
                        reading[4] = HIGH;
                    }
                }
            }
            else
            {
                // check if previous pulse has just ended and set previous_pulse if it has.
                if ((*portInputRegister(USS[4].port) & USS[4].Bit) != USS[4].Bit)
                    previous_pulse[4] = HIGH;
            }
        }

        if (millis() > initTime + time_out)
            return 0;
        finished =  USS[0].data_available &  USS[1].data_available &  USS[2].data_available &  USS[3].data_available & USS[4].data_available;
    }

    while(millis() < initTime + time_out) ; // TODO: wasted time, find out something useful to do here.

    return 1;
}

/*
motor da direita sem torque
rodas sem aderência. Não usar X_F
*/

void initTable(short int *T)
{
    T[B00000] = Frente; // doesnt happen!!!
    T[B00001] = E_L;  
    T[B00010] = E_M;  
    T[B00011] = E_M;  
    T[B00100] = E_F;  
    T[B00101] = E_F;  
    T[B00110] = E_F;  
    T[B00111] = E_F;  
    T[B01000] = D_M;  
    T[B01001] = D_M;  
    // T[B01010] = Frente;  
    T[B01010] = E_F;  
    T[B01011] = E_M;  
    T[B01100] = D_F;  
    T[B01101] = E_F;  
    T[B01110] = E_F;  
    T[B01111] = E_F;  
    T[B10000] = D_L;  
    T[B10001] = Frente;  
    T[B10010] = E_M;  
    T[B10011] = E_M;  
    T[B10100] = D_F;  
    T[B10101] = E_F;  
    //T[B10110] = D_F;  
    T[B10110] = E_F;  
    T[B10111] = E_F;  
    T[B11000] = D_M;  
    T[B11001] = D_M;  
    T[B11010] = D_M;  
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
            /*
            pwmWrite(leftmotor, fullSpeed_l);
            pwmWrite(rightmotor, fullSpeed_r);
            */
            break;
    }
}

void getExtreamValues()
{
    // 0
    USS[0].farthest = USS[0].distance[0];
    USS[0].closest = USS[0].distance[0];
    USS[0].mean = USS[0].distance[0];

    USS[1].farthest = USS[1].distance[0];
    USS[1].closest = USS[1].distance[0];
    USS[1].mean = USS[1].distance[0];

    USS[2].farthest = USS[2].distance[0];
    USS[2].closest = USS[2].distance[0];
    USS[2].mean = USS[2].distance[0];

    USS[3].farthest = USS[3].distance[0];
    USS[3].closest = USS[3].distance[0];
    USS[3].mean = USS[3].distance[0];

    USS[4].farthest = USS[4].distance[0];
    USS[4].closest = USS[4].distance[0];
    USS[4].mean = USS[4].distance[0];

    // 1
    if(USS[0].farthest < USS[0].distance[1])
        USS[0].farthest = USS[0].distance[1];
    if(USS[0].closest > USS[0].distance[1])
        USS[0].closest = USS[0].distance[1];
    USS[0].mean = USS[0].mean + USS[0].distance[1];

    if(USS[1].farthest < USS[1].distance[1])
        USS[1].farthest = USS[1].distance[1];
    if(USS[1].closest > USS[1].distance[1])
        USS[1].closest = USS[1].distance[1];
    USS[1].mean = USS[1].mean + USS[1].distance[1];

    if(USS[2].farthest < USS[2].distance[1])
        USS[2].farthest = USS[2].distance[1];
    if(USS[2].closest > USS[2].distance[1])
        USS[2].closest = USS[2].distance[1];
    USS[2].mean = USS[2].mean + USS[2].distance[1];

    if(USS[3].farthest < USS[3].distance[1])
        USS[3].farthest = USS[3].distance[1];
    if(USS[3].closest > USS[3].distance[1])
        USS[3].closest = USS[3].distance[1];
    USS[3].mean = USS[3].mean + USS[3].distance[1];

    if(USS[4].farthest < USS[4].distance[1])
        USS[4].farthest = USS[4].distance[1];
    if(USS[4].closest > USS[4].distance[1])
        USS[4].closest = USS[4].distance[1];
    USS[4].mean = USS[4].mean + USS[4].distance[1];

    // 2
    if(USS[0].farthest < USS[0].distance[2])
        USS[0].farthest = USS[0].distance[2];
    if(USS[0].closest > USS[0].distance[2])
        USS[0].closest = USS[0].distance[2];
    USS[0].mean = USS[0].mean + USS[0].distance[2];

    if(USS[1].farthest < USS[1].distance[2])
        USS[1].farthest = USS[1].distance[2];
    if(USS[1].closest > USS[1].distance[2])
        USS[1].closest = USS[1].distance[2];
    USS[1].mean = USS[1].mean + USS[1].distance[2];

    if(USS[2].farthest < USS[2].distance[2])
        USS[2].farthest = USS[2].distance[2];
    if(USS[2].closest > USS[2].distance[2])
        USS[2].closest = USS[2].distance[2];
    USS[2].mean = USS[2].mean + USS[2].distance[2];

    if(USS[3].farthest < USS[3].distance[2])
        USS[3].farthest = USS[3].distance[2];
    if(USS[3].closest > USS[3].distance[2])
        USS[3].closest = USS[3].distance[2];
    USS[3].mean = USS[3].mean + USS[3].distance[2];

    if(USS[4].farthest < USS[4].distance[2])
        USS[4].farthest = USS[4].distance[2];
    if(USS[4].closest > USS[4].distance[2])
        USS[4].closest =+ USS[4].distance[2];
    USS[4].mean = USS[4].mean + USS[4].distance[2];

    // 3
    if(USS[0].farthest < USS[0].distance[3])
        USS[0].farthest = USS[0].distance[3];
    if(USS[0].closest > USS[0].distance[3])
        USS[0].closest = USS[0].distance[3];
    USS[0].mean = USS[0].mean + USS[0].distance[3];

    if(USS[1].farthest < USS[1].distance[3])
        USS[1].farthest = USS[1].distance[3];
    if(USS[1].closest > USS[1].distance[3])
        USS[1].closest = USS[1].distance[3];
    USS[1].mean = USS[1].mean + USS[1].distance[3];

    if(USS[2].farthest < USS[2].distance[3])
        USS[2].farthest = USS[2].distance[3];
    if(USS[2].closest > USS[2].distance[3])
        USS[2].closest = USS[2].distance[3];
    USS[2].mean = USS[2].mean + USS[2].distance[3];

    if(USS[3].farthest < USS[3].distance[3])
        USS[3].farthest = USS[3].distance[3];
    if(USS[3].closest > USS[3].distance[3])
        USS[3].closest = USS[3].distance[3];
    USS[3].mean = USS[3].mean + USS[3].distance[3];

    if(USS[4].farthest < USS[4].distance[3])
        USS[4].farthest = USS[4].distance[3];
    if(USS[4].closest > USS[4].distance[3])
        USS[4].closest =+ USS[4].distance[3];
    USS[4].mean = USS[4].mean + USS[4].distance[3];

    // 4
    if(USS[0].farthest < USS[0].distance[4])
        USS[0].farthest = USS[0].distance[4];
    if(USS[0].closest > USS[0].distance[4])
        USS[0].closest = USS[0].distance[4];
    USS[0].mean = USS[0].mean + USS[0].distance[4];

    if(USS[1].farthest < USS[1].distance[4])
        USS[1].farthest = USS[1].distance[4];
    if(USS[1].closest > USS[1].distance[4])
        USS[1].closest = USS[1].distance[4];
    USS[1].mean = USS[1].mean + USS[1].distance[4];

    if(USS[2].farthest < USS[2].distance[4])
        USS[2].farthest = USS[2].distance[4];
    if(USS[2].closest > USS[2].distance[4])
        USS[2].closest = USS[2].distance[4];
    USS[2].mean = USS[2].mean + USS[2].distance[4];

    if(USS[3].farthest < USS[3].distance[4])
        USS[3].farthest = USS[3].distance[4];
    if(USS[3].closest > USS[3].distance[4])
        USS[3].closest = USS[3].distance[4];
    USS[3].mean = USS[3].mean + USS[3].distance[4];

    if(USS[4].farthest < USS[4].distance[4])
        USS[4].farthest = USS[4].distance[4];
    if(USS[4].closest > USS[4].distance[4])
        USS[4].closest = USS[4].distance[4];
    USS[4].mean = USS[4].mean + USS[4].distance[4];


    USS[0].mean = ( USS[0].mean - USS[0].farthest - USS[0].closest ) / 3;

    USS[1].mean = ( USS[1].mean - USS[1].farthest - USS[1].closest ) / 3;

    USS[2].mean = ( USS[2].mean - USS[2].farthest - USS[2].closest ) / 3;

    USS[3].mean = ( USS[3].mean - USS[3].farthest - USS[3].closest ) / 3;

    USS[4].mean = ( USS[4].mean - USS[4].farthest - USS[4].closest ) / 3;
}

void measuring()
{
    if( myPulseIn() )
    {
        USS[0].distance[USS[0].pointer] = USS[0].duration / 58.2;
        USS[1].distance[USS[1].pointer] = USS[1].duration / 58.2;
        USS[2].distance[USS[2].pointer] = USS[2].duration / 58.2;
        USS[3].distance[USS[3].pointer] = USS[3].duration / 58.2;
        USS[4].distance[USS[4].pointer] = USS[4].duration / 58.2;
    }
    else
    {
        if(USS[0].data_available)
            USS[0].distance[USS[0].pointer] = USS[0].duration / 58.2;
        else
            USS[0].distance[USS[0].pointer] = outOfRange;

        if(USS[1].data_available)
            USS[1].distance[USS[1].pointer] = USS[1].duration / 58.2;
        else
            USS[1].distance[USS[1].pointer] = outOfRange;

        if(USS[2].data_available)
            USS[2].distance[USS[2].pointer] = USS[2].duration / 58.2;
        else
            USS[2].distance[USS[2].pointer] = outOfRange;

        if(USS[3].data_available)
            USS[3].distance[USS[3].pointer] = USS[3].duration / 58.2;
        else
            USS[3].distance[USS[3].pointer] = outOfRange;

        if(USS[4].data_available)
            USS[4].distance[USS[4].pointer] = USS[4].duration / 58.2;
        else
            USS[4].distance[USS[4].pointer] = outOfRange;
    }

    if(USS[0].pointer == 4)
        USS[0].pointer = 0;
    else
        USS[0].pointer = USS[0].pointer + 1;

    if(USS[1].pointer == 4)
        USS[1].pointer = 0;
    else
        USS[1].pointer = USS[1].pointer + 1;

    if(USS[2].pointer == 4)
        USS[2].pointer = 0;
    else
        USS[2].pointer = USS[2].pointer + 1;

    if(USS[3].pointer == 4)
        USS[3].pointer = 0;
    else
        USS[3].pointer = USS[3].pointer + 1;

    if(USS[4].pointer == 4)
        USS[4].pointer = 0;
    else
        USS[4].pointer = USS[4].pointer + 1;
}

int SensorReading()
{
    measuring();
    getExtreamValues();

    if(USS[0].mean < minDist)
        return -1;

    if(USS[1].mean < minDist)
        return -1;

    if(USS[2].mean < minDist)
        return -1;

    if(USS[3].mean < minDist)
        return -1;

    if(USS[4].mean < minDist)
        return -1;

    return obstacleShape();
}

int obstacleShape()
{
    int obstacle = 0;

    if (USS[0].mean < warningDist) 
        obstacle += B00001; // 1st bit <=> 1st sensor

    if (USS[1].mean < warningDist) 
        obstacle += B00010; // 2nd bit <=> 2nd sensor

    if (USS[2].mean < warningDist) 
        obstacle += B00100; // 3rd bit <=> 3rd sensor

    if (USS[3].mean < warningDist) 
        obstacle += B01000; // 4th bit <=> 4th sensor

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
	status[1] = 'L';
	status[2] = 48+right;
	status[3] = 'R';
	status[4] = 48+speed;
	status[5] = 'S';
	status[6] = 48+frequency;
	status[7] = 'f';
	status[8] = '|';
	status[9] = '\0';

	int2char(aux,frequency_l);
	concatenate(status,aux);
	concatenate(status,"fL");

	int2char(aux,pwm_value_l);
	concatenate(status,aux);
	concatenate(status,"SL");

	int2char(aux,frequency_r);
	concatenate(status,aux);
	concatenate(status,"fR");

	int2char(aux,pwm_value_r);
	concatenate(status,aux);
	concatenate(status,"SR");

	write_command(status);

}

void communication(bool mode)
{
	unsigned long t_aux = millis();
	bool w = 0,r = 0, quit = 0;

	while( (millis() < t_aux+t_com) && !(w&r) ) 
    {	
        if(reading)
        {
            if(Read_blocking())
            {
                r = 1;
                reading = 0;
                if(select)
                    quit = processing(); // safety button status has been received
            }
        }
        else
        {
            if( quit || Write_blocking(mode) )
            {
                w = 1;
                reading = 1;
            }
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
                Write_nonBlocking(select);
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
                write_command("autonomous navigation");
                Write_nonBlocking(select);
                autonomous();
                break;
            case 12: //	log
                logging();
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
    reading = 1;
    radio.startListening();
    delay(1000);

    t = millis();
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
            break;

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
}

bool processing()
{
	int i;
	for(i = 0; i < Ncommands; i++) // have we just got a command?
		if(isSubstring(intCmd[i],rcv) == 1)
			break;

	if(i == Ncommands) // if not a command
    {
        if(logging_flag)
        {
            flushLogData(char2int(rcv));
        }
        else if(speed)
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
                        concatenate(rcv," (freq L and R)");
                    else
                        concatenate(rcv," (freq L)");
                    write_command(rcv);
                }

                if(right)
                {
                    frequency_r = f;
                    frequency = 0;
                    SetPinFrequencySafe(rightmotor, frequency_r);
                    if(!left)
                    {
                        concatenate(rcv," (freq R)");
                        write_command(rcv);
                    }
                }
            }
        }
        else if( (  (rcv[0]=='+' || rcv[0]=='-')  && (rcv[1]=='\0')  )  )  // safety button status message
        {	
            if(rcv[0] == '+')
                safety_button = 1;
            else
            {
                safety_button = 0;
                return 1;
            }
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

void read_message()
{
    if(Serial.available() == 0 && msg[0] == '\0')
        checkButton();
    else
    {
        delay(100);
        int aux = 0;
        while(Serial.available() > 0)
            msg[aux++] = Serial.read();
        msg[aux] = '\0';
    }

    if(rcv[0] == '{') // if log data, ask for the next
    {
        char aux[4];
        
        if(rcv[2] == '}')
        {
            aux[0] = rcv[1];
            aux[1] = '\0';
        }
        if(rcv[3] == '}')
        {
            aux[0] = rcv[1];
            aux[1] = rcv[2];
            aux[2] = '\0';
        }
        if(rcv[4] == '}')
        {
            aux[0] = rcv[1];
            aux[1] = rcv[2];
            aux[2] = rcv[3];
            aux[3] = '\0';
        }
        int2char(msg,char2int(aux)+1);
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

/*
int mypow(int base, int exponent)
{
    int result = 1;
    for(int i = 0; i < exponent; i++)
        result = result*base;
    return result;
}
*/

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
            aux *= (int) mypow(10,i-1-j);
            r += aux;
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
	copyString("freq",m[8]);
	copyString("button",m[9]);
	copyString("dist",m[10]);
	copyString("auto",m[11]);
	copyString("log",m[12]);

}

bool isSubstring(char *a, char *b)
{
	int i;

	if((a[0] == '\0')||(b[0] == '\0'))
	{
        if(!select)
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
	t = millis();

	while(!r)
	{
		if(millis() > t+dt)
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
    t = millis();
    while(!w)
    {
        if(millis() > t+dt)
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
    if(n < n_max)
    {
        t += (unsigned long)random(t_min, t_max);
        if(t%2)
        {
            reading = !reading;
        }
    }
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
            // Serial.print("Message received: ");
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

    return check;

/* stupidity:

	if(check)
		return 1;
	else
		return 0;
*/
}

void statusFeedback(char iteration)
{
    int OS = obstacleShape();
    char aux[15];
    radio.stopListening();

    // Safety Button
    if(rcv[0] != '\0')
        concatenate(cmd,rcv);
    else
        concatenate(cmd,"x");

    aux[0] = ' ';
    aux[1] = '[';
    aux[2] = iteration;
    aux[3] = ']';
    aux[4] = '\0';

    concatenate(cmd,aux);

    concatenate(cmd," ");
    writeSensorsData();

    // Move
    if( (USS[0].mean < minDist)||(USS[0].mean < minDist)||(USS[2].mean < minDist)||(USS[3].mean < minDist)||(USS[4].mean < minDist) )
        concatenate(cmd,"STP ");
    else
    {
        if(OS > 0)
            switch (T[OS])
            {
                case (E_L):
                    concatenate(cmd,"E_L ");
                    break;
                case (E_M):
                    concatenate(cmd,"E_M ");
                    break;
                case (E_F):
                    concatenate(cmd,"E_F ");
                    break;
                case (D_L):
                    concatenate(cmd,"D_L ");
                    break;
                case (D_M):
                    concatenate(cmd,"D_M ");
                    break;
                case (D_F):
                    concatenate(cmd,"D_F ");
                    break;
                case (Frente):
                    concatenate(cmd,"Fre ");
                    break;
            }
        else
            concatenate(cmd,"OoR ");
    }

/*
    // Obstacle Shape
    if(OS >= 16)
    {
        OS = OS-16;
        if(USS[4].mean < minDist)
            concatenate(cmd,"!");
        else
            concatenate(cmd,"1");
    }
    else
        concatenate(cmd,"0");

    if(OS >= 8)
    {
        OS = OS-8;
        if(USS[3].mean < minDist)
            concatenate(cmd,"!");
        else
            concatenate(cmd,"1");
    }
    else
        concatenate(cmd,"0");

    if(OS >= 4)
    {
        OS = OS-4;
        if(USS[2].mean < minDist)
            concatenate(cmd,"!");
        else
            concatenate(cmd,"1");
    }
    else
        concatenate(cmd,"0");

    if(OS >= 2)
    {
        OS = OS-2;
        if(USS[1].mean < minDist)
            concatenate(cmd,"!");
        else
            concatenate(cmd,"1");
    }
    else
        concatenate(cmd,"0");

    if(OS >= 1)
    {
        OS = OS-1;
        if(USS[0].mean < minDist)
            concatenate(cmd,"!");
        else
            concatenate(cmd,"1");
    }
    else
        concatenate(cmd,"0");
*/

    Write_nonBlocking(select);
    radio.startListening();

    rcv[0] = '\0';
    cmd[0] = '\0';
    aux[0] = '\0';
}

// ser mais rigoroso ao parar o carrinho. Verificar se é somente um sensor com medidas erradas.
void autonomous()
{
    char status[MaxPayload], iteration = '0'; // check for packet loss
    status[0] = '\0';

    reading = 1;
    radio.startListening();
    delay(1000);

    t = millis();
    while(millis() < t + t_start)
    {
        ObstacleAvoid();

        if (iteration == '9')
            iteration = '0';
        else 
            iteration++;

        if(radio.available())
        {
            bool done = false;
            while( (!done) && (millis() < t + t_start) ) // checa deadline por preciosismo...
                done = radio.read( rcv, sizeof(rcv) );
        }
        if(rcv[0] == '-')
            break;
        if(rcv[0] == '+')
            t = millis();
        statusFeedback(iteration);
    }
    stop();
}

void logging()
{
    radio.stopListening();
    for(int i = 0; i < buffer_size; i++)
    {
        ObstacleAvoid();
        buffer[i][0] = USS[0].mean;
        buffer[i][1] = USS[1].mean;
        buffer[i][2] = USS[2].mean;
        buffer[i][3] = USS[3].mean;
        buffer[i][4] = USS[4].mean;
    }

    flushLogData(0);
    logging_flag = HIGH;
}

void flushLogData(int N_buffer)
{
    if(N_buffer >= 0)
    {
        if(N_buffer < buffer_size)
        {
            char aux[5];
            cmd[0] = '{'; // is it really necessary??
            cmd[1] = '\0'; // is it really necessary??
            int2char(aux,N_buffer);
            concatenate(cmd,aux);
            concatenate(cmd,"} ");
            int2char(aux,buffer[N_buffer][4]);
            concatenate(cmd,aux);
            concatenate(cmd," | ");
            int2char(aux,buffer[N_buffer][3]);
            concatenate(cmd,aux);
            concatenate(cmd," | ");
            int2char(aux,buffer[N_buffer][2]);
            concatenate(cmd,aux);
            concatenate(cmd," | ");
            int2char(aux,buffer[N_buffer][1]);
            concatenate(cmd,aux);
            concatenate(cmd," | ");
            int2char(aux,buffer[N_buffer][0]);
            concatenate(cmd,aux);
        }
        else
        {
            logging_flag = LOW;
        }
    }
}

void writeSensorsData()
{
    char aux[6];

    int2char(aux,USS[4].mean);
    concatenate(cmd,aux);
    concatenate(cmd,"|");
    int2char(aux,USS[3].mean);
    concatenate(cmd,aux);
    concatenate(cmd,"|");
    int2char(aux,USS[2].mean);
    concatenate(cmd,aux);
    concatenate(cmd,"|");
    int2char(aux,USS[1].mean);
    concatenate(cmd,aux);
    concatenate(cmd,"|");
    int2char(aux,USS[0].mean);
    concatenate(cmd,aux);
    concatenate(cmd," ");
}

void SensorsDataPrint()
{
    int i;
    char status[MaxPayload], aux[6],iteration = '0';
    status[0] = '\0';

    reading = 1;
    radio.startListening();
    measuring();
    getExtreamValues();


    t = millis();
    while(millis() < t + t_start)
    {
        status[0] = '[';
        status[1] = iteration;
        status[2] = ']';
        status[3] = '\0';

        for (i = 0; i < 5; i++)
        {
            int2char(aux,USS[4-i].mean);
            concatenate(status,aux);
            if(i<4)
                concatenate(status,"|");
        }
        if(radio.available())
        {
            bool done = false;
            while( (!done) && (millis() < t + t_start) ) // checa deadline por preciosismo...
                done = radio.read( rcv, sizeof(rcv) );
        }
        if(rcv[0] == '-')
            break;
        if(rcv[0] == '+')
            t = millis();

        write_command(status);
        radio.stopListening();
        Write_nonBlocking(select);
        radio.startListening();
        measuring();
        getExtreamValues();
        rcv[0] = '\0';
        aux[0] = '\0';

        if (iteration == '9')
            iteration = '0';
        else 
            iteration++;
    }
}


