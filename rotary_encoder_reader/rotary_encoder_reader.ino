/*
    Read optical incremental rotary encoder output.
    This code was modified from https://gist.github.com/kinverarity1/8ca395b85bb63e1e7471d7db0284c30b

    Serial output columns:

    1. Absolute encoder position (pulses, integer) from startup
    2. Change in encoder position from trial start (pulses, integer)
    3. Encoder position at trial start (degrees, float)
    4. Change in motor shaft position (degrees, float)
    5. Time since trial start in (microseconds, integer)
    6. Motor Shaft Speed (degrees per second, float)

    This code tracks absolute position of the chair in pulses from the time of startup, and converts that to
    relative position and velocity in degrees based on the current position and time at the start of each trial.

    volatile directive is used for variables that are modified in the
    interrupt service routines.

*/

// Pin definitions.
// - enc_a is ENC Signal A line (Arduino digital pin 2)
// - enc_b is ENC Signal B line (Arduino digital pin 3)
#define ENC_A  2
#define ENC_B  3
#define triggerPinRE 12  //trigger/enable line from the motor driver Arduino
// #define trialcountpin 13 //this pin will change state every X trials to help us keep count

// Main loop refresh period.
#define REFRESH_MS  50

// Main serial data connection to computer.
#define BAUD_RATE   115200

// Encoder signal line states
volatile boolean state_a = 0;
volatile boolean state_b = 0;

// Encoder position
volatile int enc_pos = 0;
int enc_pos_prev = 0;
int enc_pos_change = 0;
int enc_pos_trialstart = 0;
float abs_pos_degrees = 0;
float rel_pos_degrees = 0;
float rel_pos_prev = 0;
float rel_pos_change = 0;
float abs_pos_trialstart = 0;


float REratio = (200.0 / 268.7);  //ratio of circumference of rotary encoder wheel to motor shaft
//the rotary encoder has a 200 mm circumference
float REppr = 1000.0; //the rotary encoder has 1000 pulses/revolution

// Timing
unsigned long micros_current = 0;
unsigned long micros_prev = 0;
unsigned long micros_trialstart = 0;
long micros_change = 0;

// keep track of enable signal
int readEnable = LOW;
int readPriorState = LOW;
int trial = 0;  // note that rather than send the trial number over (which will take ~8 pins), we will just count enables

//keep track of whether we are actively printing the encoder output or not
bool REenabled;
bool REhold;
unsigned long REholdstime;
unsigned long REholdctime;
long REholdetime;


void setup()
{
    pinMode(ENC_A, INPUT);
    pinMode(ENC_B, INPUT);

    pinMode(triggerPinRE, INPUT);

    state_a = (boolean) digitalRead(ENC_A);
    state_b = (boolean) digitalRead(ENC_B);

    readEnable = digitalRead(triggerPinRE);

    attachInterrupt(0, interrupt_enc_a, CHANGE);
    attachInterrupt(1, interrupt_enc_b, CHANGE);

    micros_prev = micros();
    micros_trialstart = micros();

    rel_pos_degrees = 0;
    abs_pos_trialstart = abs_pos_degrees;
    enc_pos_trialstart = enc_pos;
    rel_pos_prev = 0;
    rel_pos_change = 0;

    Serial.begin(BAUD_RATE);

    Serial.println("Ready to begin recording.");

    REenabled = 0;
}


void loop()
{

    int n;
    float bounded_trialstart;
    unsigned long etime;

    readEnable = digitalRead(triggerPinRE);
    if (readPriorState == LOW && readEnable == HIGH) // if the enable signal just went high (START RECORDING)
    {

        //reset trial-based tracking variables back to 0
        rel_pos_degrees = 0;
        abs_pos_trialstart = abs_pos_degrees;
        enc_pos_trialstart = enc_pos;
        rel_pos_prev = 0;
        rel_pos_change = 0;

        // Timing
        micros_trialstart = micros();
        etime = 0;

        //enable updates
        REenabled = true;
        REhold = false;

        trial = trial + 1;
        Serial.print("\n");
        Serial.print("<-*-*- TRIAL "); Serial.print(trial); Serial.println(" -*-*->");

        readPriorState = readEnable;
    }
    else if (readPriorState == HIGH && readEnable == LOW) // if the enable signal just went low (STOP RECORDING)
    {
        //REenabled = false; // stop updating the position information
        REhold = true;
        REholdstime = micros(); //start a timer to delay when recordin shuts off
        readPriorState = readEnable; // stop
    }
    else if (REhold)
    {
        REholdctime = micros();
        if (REholdctime < REholdstime) {
            REholdetime = REholdctime + (4294967295 - REholdstime);
        } else {
            REholdetime = REholdctime - REholdstime;
        }

        if (REholdetime > 1000000)  //the timer has expired, stop recording
        {
            REhold = false;
            REenabled = false; // stop updating the position information
        }
      
    }
    

    //REenabled = 1;


    // Calculate change in encoder position.
    //enc_pos_change = enc_pos - enc_pos_prev;
    //enc_pos_change = abs(enc_pos_change);
    //abs_pos_degrees = enc_pos * .00074074 * 360; //(1/1350)
    abs_pos_degrees = -1 * enc_pos * (360.0 / REppr) * REratio; // number of pulses * (deg/pulse) * (motor rotation / encoder rotation)


    if (REenabled)
    {

        // Calculate elapsed time
        micros_current = micros();
        if (micros_current < micros_prev) {
            micros_change = micros_current + (4294967295 - micros_prev);
        } else {
            micros_change = micros_current - micros_prev;
        }
        //calculate elapsed time since trial start
        if (micros_current < micros_trialstart) {
            etime = micros_current + (4294967295 - micros_trialstart);
        } else {
            etime = micros_current - micros_trialstart;
        }
        
        //calculate the relative position change (pulses) since trial start
        enc_pos_change = enc_pos - enc_pos_trialstart;

        // Calculate relative position change (degrees) since trial start
        rel_pos_degrees = abs_pos_degrees - abs_pos_trialstart; // this is bounded to [-360 360] by the motor_driver code
        rel_pos_change = abs(rel_pos_degrees - rel_pos_prev);

        //compute bounded absolute position at trial start
        if (abs_pos_trialstart >= 0)
        {
            n = floor(abs_pos_trialstart / 360);
            bounded_trialstart = abs_pos_trialstart - 360 * n;
        }
        else if (abs_pos_trialstart < 0)
        {
            n = floor(-abs_pos_trialstart / 360);
            bounded_trialstart = (360 + abs_pos_trialstart + (360 * n));
        }

        // Emit data
        Serial.print(enc_pos);
        Serial.print("\t");
        Serial.print(enc_pos_change);
        Serial.print("\t");
        Serial.print(bounded_trialstart, 3);
        Serial.print("\t");
        Serial.print(abs_pos_degrees, 3);
        Serial.print("\t");
        Serial.print(etime);
        Serial.print("\t");
        Serial.print(rel_pos_change / (micros_change / 1e6), 3);
        //Serial.print(enc_pos);
        //Serial.print("\t");
        //Serial.print(enc_pos_change);
        //Serial.print("\t");
        //Serial.print(micros_current);
        //Serial.print("\t");
        //Serial.print(micros_change);
        //Serial.print("\t");
        //Serial.print(enc_pos_change / (micros_change / 1e6));
        Serial.print("\n");

        micros_prev = micros_current;
        rel_pos_prev = rel_pos_degrees;

    }

    //enc_pos_prev = enc_pos;


    delay(REFRESH_MS);
}


// Detect pulses from depth encoder.

void interrupt_enc_a()
{
    if (!state_a) {
        state_b ? enc_pos++ : enc_pos--;
    }
    state_a = !state_a;
}

void interrupt_enc_b()
{
    state_b = !state_b;
}
