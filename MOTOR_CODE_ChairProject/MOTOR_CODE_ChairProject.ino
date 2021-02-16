/* MOTOR_CODE_ChairProject
   -*- mode: C++ -*-

  ***PURPOSE***
    This program is intended to control the vestibular rotating chair. 
    Prompt use for input, rotate to desired endpoint, display relevant information.
    
    2021 - Evan C. Snow (For use in Erica Barhost-Cates PhD. and Aaron Wong PhD. study on Localization)
           with additional mods by Aaron L. Wong

  ***SCHEMATIC***
    ~Arduino~
    DIGITAL pin 6: PWM (+) to microstepper
    DIGITAL pin 7: DIR (+) to microstepper
    GND: PUL (-), DIR(-)

    ~Microstepper~        _
    Port 1: DIR (-)     |
    Port 2: DIR (+)     | Logic Pins to Arduino
    Port 3: PUL (-)     |
    Port 4: PUL (+)    _|
    Port 5: ENA (-)     | // Not used //
    Port 6: ENA (+)    _| ~Can be used for Enabling/Disabling microstepper from Arduino HIGH/LOW DIGITAL pin output~
                _
    Switch 1: ON     |
    Switch 2: ON     | Current Tweaking Switches: 4.28A (REF), 5.14A (PEAK)
    Switch 3: OFF   _|
    Switch 4: ON    _| Full vs Half Current Switch: FULL
    Switch 5: ON     |
    Switch 6: ON     | Microstepping Set Switches: 6400 steps/rev
    Switch 7: OFF    |
    Switch 8: ON    _|
                  _
    Port 7: A (+)      |
    Port 8: A (-)      | Output to NEMA 34 stepper motor
    Port 9: B (+)      |
    Port 10: B (-)    _|
    Port 11: V (+)     | Thick Black Wire [+] HIGH VOLTAGE (~36VDC)     --DO NOT TOUCH THESE WIRES IT WILL HURT--
    Port 12: V (-)    _| Thick White Wire [-] HIGH VOLTAGE (~36V)

    ~Power Supply~             _
    Port 1: Lead (Black)        |
    Port 2: Neutral (White)     | Power cord to power strip to wall. [120VAC]
    Port 3: Ground (Green)______|
    Port 4:                |
    Port 5: V [-]          |
    Port 6: //Not used     | HIGH VOLTAGE [36VDC] LOOP TO MICROSTEPPER
    Port 7: V [+]          |
    Port 8: //Not used     |
    Port 9: //Not used    _|

  ***INSTRUCTIONS***

    -CONNECT "Motor Arduino" (Arduino Uno COMX) where X is the COM port number, and open Tools > Serial Monitor. Once clicking 
    this the Arduino board will reboot (This can be a soft reset i.e. close Serial Monitor and open Serial Monitor to effectively
    RESET).

    In the Serial Monitor, set the Baud rate to match the rate specified in the code (nominally 115200), and choose the option to "No Line Ending".
    Also make suer that the "Autoscroll" and "TimeStamp" options are turned on. Close and Reopen the Serial Monitor or Clear the console if desired.

    -Follow instructions on Serial Monitor. There is a field on top to send Serial data to the Arduino board. Click SEND button or hit the 'Return' key 
    once done typing in desired input command.

    -When finished, click anywhere in the Serial Monitor output and ctr + A then ctr + C to copy all outputs.

    -Paste in a .txt file or whatever type of file you want the data in

    -BOOM, you did it!


  ***NOTES***
    To add AccelStepper library, Go to Tools > Manage Libraries. Search for AccelStepper and install the latest version.

    To communicate with the secondary Arduino (which is reading the rotary encoder), connect the two arduinos via the
    GND pin and one of the digital pins to act as an enable line/trigger signal. Here we use pin 12 as defined by triggerPinRE.
    Note the Rotary Encoder does not actually receive trial number information, it counts trials independently based on
    the number of detected triggers.

    For audio, see https://www.arduino.cc/reference/en/language/functions/advanced-io/tone/

*/

#include <AccelStepper.h>
#include "pitches.h"
//#include <avr/wdt.h> //for library containing watchdog

// Initialize the library with number of interface pins
AccelStepper stepper(1, 6, 7); // 1 is mode, 6 is PWM (+) pin and 7 is dir (+) connection, (-) counterparts to GND

float movementSteps = 0;
bool inputReceived = 0;
float inputMove = 0;
float oneRevSteps = 51200;

//trial counter
int trial = 1;

//variables to trigger the second Arduino reading the rotary encoder
const int triggerPinRE = 12;
int triggerPinREState = LOW;

//variables to support the rotate-pause-rotate_back paradigm
unsigned long timeStoppedMoving = 0; //time the chair finished rotating
const int timePause = 3000;          // time in milliseconds to pause before rotating the chair back
int movePhase = 0;                  //keep track of whether we are moving outward or back

//serial Baud rate - this must match what is selected in the Serial Monitor
#define BAUD_RATE 115200
//38400;

//variables to support tone
int noteDur = 300;
#define TONE_PIN 13


void setup()
{
    Serial.begin(BAUD_RATE);             //Initialize serial communication between Arduino UNO and USB port on Windows computer

    pinMode(triggerPinRE, OUTPUT);    //Initialize digital trigger pin to talk between microcontrollers

    // Stepper Parameters
    stepper.setMaxSpeed(8000);
    stepper.setAcceleration(1000);
    //oneRevSteps = 6400;             //One full rotation is 6400 steps

    InitializeProgram();
}


//recall that we can also use a void serialEvent() {} loop to capture serial data as well
void loop()
{
    Serial.print("\n");
    Serial.print("<-*-*- TRIAL "); Serial.print(trial); Serial.println(" -*-*->");
    delay(1000);
    inputReceived = false;  // set at zero for every iteration to pop back into the while loop
    inputMove = 0;      // set at zero to begin every iteration... also helps to make sure there are no accidental movements
    movePhase = 0;

    //Get input for movement
    GetInput();

    //Move the chair: rotate outward
    //if (movePhase == 1) {
        MoveChair(inputMove);
    //}

    //pause at the end of the rotation and beep
    //if (movePhase == 2) {
        delay(timePause/2); //this will work, but note it is blocking!
        noTone(TONE_PIN);
        tone(TONE_PIN, NOTE_G5, noteDur);
        delay(timePause/2); //this will work, but note it is blocking!
        movePhase = 3;
    //}

    //rotate the chair back to the home position and beep
    //if (movePhase == 3) {
        MoveChair(-inputMove);
    //}

    //Record patient output
    RecordOutput();

    trial++;    // increase trial iteration
    delay(1000);
}

//-------FUNCTIONS-------\\

void InitializeProgram() {
    //Give time for power supply to reach stable voltage
    Serial.println(""); //clear Serial buffer
    Serial.print("Initializing");
    delay(1000); Serial.print("."); delay(1000); Serial.print("."); delay(1000); Serial.println(".");

    // set the LED with the ledState of the variable:
    triggerPinREState = LOW;        //initialize the trigger signal LOW from the motor digital pin to the rotary encoder digital pin using digitalRead(Pin, State [HIGH, LOW])
    digitalWrite(triggerPinRE, triggerPinREState);  //Make sure digital pin 12 initializes to LOW


    //Give information about readiness of program and feedback to the user.
    Serial.print("The current location is: "); DegreeCalculate(); Serial.println(" degrees.");
    Serial.println("Enter anything in the serial monitor and click SEND to begin the main program.");
    while (Serial.available() == 0) {}
    Serial.println("Begin testing protocol.");
    delay(500);
}

void GetInput() {
    while (inputReceived == false) {

        //There's a bug EXCLUSIVE to the first iteration that skips the next serial call, so this keeps unexpected stuff from happeing on first iteration
        if (trial == 1) {
            while (Serial.available() == 0) {} //hold at this point in the code until user clicks SEND on Serial Monitor
            inputMove = Serial.parseInt();    //store input
        }
        Serial.println("How far would you like to move RELATIVE to the current position? [CCW (+) CW(-)]");
        Serial.println("(Please input a non-zero value from -360 to 360 degrees and click SEND.)");  //prompt user for input
        while (Serial.available() == 0) {} //hold at this point in the code until user clicks SEND on Serial Monitor
        inputMove = Serial.parseInt();    //store input

        //movePhase = 0;

        if (((inputMove >= -360) && (inputMove <= 360)) && (inputMove != 0)) {
            //Serial.print("Moving "); Serial.print(inputMove);
            //Serial.print(" degrees. Starting Position: ");
            //DegreeCalculate(); Serial.print(" degrees. Ending Position: ");
            //DegreeCalculate2(); Serial.println(" degrees.");

            delay(1000);
            Serial.println("--- BEGINNING MOVEMENT ---");

            // set the LED with the ledState of the variable:
            triggerPinREState = HIGH;        //trigger a signal from the motor digital pin to the rotary encoder digital pin using digitalRead(Pin, State [HIGH, LOW])
            digitalWrite(triggerPinRE, triggerPinREState);  //Set digital pin 12 to HIGH
            inputReceived = true;
            
            noTone(TONE_PIN);
            tone(TONE_PIN, NOTE_C5, noteDur);
            delay(500);

            movePhase = 1;
        }
        else {
            Serial.println("You did not enter a non-zero value between -360 and 360 degrees.");
            delay(1000);
        }
    }
}

void MoveChair(float MoveAngle) {


    //Serial.print("oneRevSteps: "); Serial.println(oneRevSteps);

    movementSteps = (float)(oneRevSteps * (MoveAngle / 360));
    Serial.print("inputMove: "); Serial.print(inputMove); Serial.println(" degrees");
    Serial.print("movementSteps: "); Serial.print(movementSteps); Serial.println(" steps");
    Serial.print("StartingPosition: "); DegreeCalculate(); Serial.println(" degrees");
    Serial.print("EndingPosition: "); DegreeCalculate2(); Serial.println(" degrees");
    movementSteps += stepper.currentPosition();
    //Serial.print("movementSteps: "); Serial.println(movementSteps);

    stepper.moveTo(movementSteps);
    while (stepper.distanceToGo() != 0) { //wait until movement is finished... making sure there is not skip
        stepper.run();
    }
    if (stepper.distanceToGo() == 0 && movePhase == 1) {
        Serial.println("--- FINISHED OUTWARD MOVEMENT ---");
        Serial.print("The current location is: "); DegreeCalculate(); Serial.println(" degrees.");
        movePhase = 2;
        timeStoppedMoving = micros();
    }
    else if (stepper.distanceToGo() == 0 && movePhase == 3) {
        Serial.println("--- FINISHED RETURN MOVEMENT ---");
        triggerPinREState = LOW;
        digitalWrite(triggerPinRE, triggerPinREState);
        Serial.print("The current location is: "); DegreeCalculate(); Serial.println(" degrees.");
        movePhase = 0;
        noTone(TONE_PIN);
        tone(TONE_PIN, NOTE_C6, noteDur);
    }
}

void RecordOutput() {
    Serial.println("What was the Patient response?");
    while (Serial.available() == 0) {} //hold at this point in the code until user clicks SEND on Serial Monitor
    int patientResponse = Serial.parseInt();    //store patient response integer
    Serial.print("patientResponse:  "); Serial.print(patientResponse); Serial.println(" degrees.");
}

void DegreeCalculate() {
    float degreesAt = 0;
    degreesAt = (float)stepper.currentPosition() / oneRevSteps * 360;

    if (degreesAt >= 0) {
        int n = floor(degreesAt / 360);
        degreesAt = degreesAt - 360 * n;
        Serial.print(degreesAt);
    }
    else if (degreesAt < 0) {
        int n = floor(-degreesAt / 360);
        degreesAt = (360 + degreesAt + (360 * n));
        Serial.print(degreesAt);
    }
    return;
}

void DegreeCalculate2() {
    float degreesAt2 = 0;
    degreesAt2 = (float)stepper.currentPosition() / oneRevSteps * 360;
    degreesAt2 += inputMove;
    if (degreesAt2 >= 0) {
        int n = floor(degreesAt2 / 360);
        degreesAt2 = degreesAt2 - 360 * n;
        Serial.print(degreesAt2);
    }
    else if (degreesAt2 < 0) {
        int n = floor(-degreesAt2 / 360);
        degreesAt2 = (360 + degreesAt2 + (360 * n));
        Serial.print(degreesAt2);
    }
    return;
}

/*  POSSIBLE SETUP IF WANT TO SAVE VARIABLE... I JUST SET UP A GLOBAL VARIABLE TO UPDATE (plenty of resources left)

    int degreeCalculate(){
    float degreesAt = 0;
    degreesAt = (float)stepper.currentPosition()/1600*180;
    if (degreesAt >= 0){
    int n = floor(degreesAt/360);
    degreesAt = degreesAt-360*n;
    return degreesAt;
    }
    else if (degreesAt < 0){
    int n = floor(-degreesAt/360);
    degreesAt = (360+degreesAt+(360*n));
    return degreesAt;
    }
    }
*/
