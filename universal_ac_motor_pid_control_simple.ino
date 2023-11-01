#include <avr/io.h>
#include <avr/interrupt.h>
#include <PID_v1.h>

#define TACHO 3            // tacho signals input pin
#define DETECT 2           // zero cross detect pin
#define GATE A3            // TRIAC gate pin
#define BUTTON 4           // on/off button pin
#define RELAY 5            // relay pin
#define PULSE 2            // number of triac trigger pulse width counts. One count is 16 microseconds
#define TACHOPULSES 4      // number of pulses per revolution 
#define LEVELRPM A5       // Niveau de RPM souhaité

unsigned int RPM;                   // real rpm variable
unsigned int count;                 // tacho pulses count variable
unsigned int lastcount = 0;         // additional tacho pulses count variable
unsigned long lastcounttime = 0;
unsigned long lastflash;
unsigned long lastpiddelay = 0;
unsigned long previousMillis = 0;
unsigned long lastDebounceTime = 0;

int levelread = 0;                  // variable to store the value read on analog input for desired rpm

const int sampleRate = 1;           // Variable that determines how fast our PID loop
const int rpmcorrection = 00;       // sito kazkodel reikia, kad realus rpm atitiktu matuojamus
const int protection = 3000;        // protection will switch on when real rpm exceeds desired by value
const int debounceDelay = 50;       // the debounce time; 
const int minoutputlimit = 80;      // limit of PID output
const int maxoutputlimit = 540;     // limit of PID output
const int mindimminglimit = 20;     // the shortest delay before triac fires
const int maxdimminglimit = 625;    // for 60Hz will be 520
const int risetime = 100;           // RPM rise time delay in microseconds (risetime x RPM)
int desiredRPM = 0;        // ENTER DESIRED RPM HERE

int dimming = 540;                  // this should be the same as maxoutputlimit
int tempcounter = 200;

byte relayState = LOW;              // the current state of the relay pin
byte buttonState;                   // the current reading from the input pin
byte lastButtonState = HIGH;        // the previous reading from the input pin

bool loopflag = false;              // flag for soft start
bool startflag = false;             // flag for motor start delay
bool runflag = false;               // flag for motor running state

double Setpoint, Input, Output;       // define PID variables
double sKp = 0.1, sKi = 0.2, sKd = 0; // PID tuning parameters for starting motor
double rKp = 0.25, rKi = 1, rKd = 0;  // PID tuning parameters for runnig motor

//PID myPID(&Input, &Output, &Setpoint, sKp, sKi, sKd, DIRECT); // define PID variables and parameters
PID myPID(&Input, &Output, &Setpoint, sKp, sKi, sKd, DIRECT); // define PID variables and parameters

int nbinterrupt = 0;

void setup() {
  Serial.begin(19200);
  // set up pins
  pinMode(BUTTON, INPUT);             // set the button pin
  pinMode(RELAY, OUTPUT);             // set the relay  pin
  pinMode(DETECT, INPUT);             // set the zero cross detect pin
  pinMode(GATE, OUTPUT);              // set the TRIAC gate control pin
  pinMode(TACHO, INPUT);              // set the tacho pulses detect pin
  digitalWrite(BUTTON, HIGH);         // turn on pullup resistors
  digitalWrite(RELAY, relayState);    // initialize relay output

  Input = 200;                        // asiign initial value for PID
  Setpoint = 200;                     // asiign initial value for PID

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(minoutputlimit, maxoutputlimit);
  myPID.SetSampleTime(sampleRate);    // Sets the sample rate

  // set up Timer1
  OCR1A = 100;                        // comparator overflow set to 100.
  TIMSK1 = 0x03;                      // enable comparator A and overflow interrupts : timer interruption mask = 00000011
                                      // CS10=0 CS11=0 => division timer by 64 => 4µS
  TCCR1A = 0x00;                      //  CS12=0, 0CS10=0 CS11=0 => timer disabled 
                                      // timer control registers set for 
  TCCR1B = 0x00;                      // CS12=0, 0CS10=0 CS11=0 => timer disabled
                                      //  normal operation, timer disabled

  // set up zero crossing interrupt IRQ0 on pin 2.
  // set up tacho sensor interrupt IRQ1 on pin3
  attachInterrupt(0, zeroCrossingInterrupt, RISING);
  attachInterrupt(1, tacho, FALLING);
}

// Interrupt Service Routines
// on zero crossing detction : 
// - start timer
// - set OCR1A to the duration desired
void zeroCrossingInterrupt() { // zero cross detect

  TCCR1B = 0x04;               // start timer with divide by 256 input
  TCNT1 = 0;                   // reset timer - count from zero
  OCR1A = dimming;             // set the compare register brightness desired.
}
// Losqu'n atteint OCR1A, 
// commence le décompte à 65536 - 2 
ISR(TIMER1_COMPA_vect) {       // comparator match
  if (startflag == true) {     // flag for start up delay
    digitalWrite(GATE, HIGH);  // set TRIAC gate to high
    TCNT1 = 65536 - PULSE;     // trigger pulse width
  }
}

ISR(TIMER1_OVF_vect) {         // timer1 overflow
  digitalWrite(GATE, LOW);     // turn off TRIAC gate
  TCCR1B = 0x00;               // disable timer stops unintended triggers
}

// RPM counting routine
void tacho() {
  //nbinterrupt++;
  count++;
  unsigned long tachotime = micros() - lastflash;
  float time_in_sec  = ((float)tachotime ) / 1000000;
  float prerpm = 60 / time_in_sec;
  RPM = prerpm / TACHOPULSES;
  lastflash = micros();
}

void loop() {
  levelread = analogRead(LEVELRPM);  // read the input pin
  Serial.print(desiredRPM);          // debug value
  Serial.print('\t');
  Serial.print(RPM);          // debug value
  Serial.print('\t');
  Serial.print(Setpoint);
  Serial.print('\t');
  Serial.println(dimming);
  // Serial.print("Int ");  
 //Serial.println(nbinterrupt);  
  desiredRPM = levelread*4; 
  // check the start / stop button state
  int reading = digitalRead(BUTTON);  // read the state of the switch into a local variable:
  if (reading != lastButtonState) {   // If the switch changed, due to noise or pressing
    lastDebounceTime = millis();      // reset the debouncing timer
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    //Serial.println(reading); 
    if (reading != buttonState) {     // if the button state has changed:
      buttonState = reading;
      if (buttonState == LOW) {       // only toggle the relay if the new button state is LOW
        relayState = !relayState;
        if (relayState == HIGH) {
          loopflag = true;
          digitalWrite(RELAY, relayState); // set the Relay:
          delay (300);                     // delay to prevent sparks on relay contacts
          startflag = true;                // flag to start motor
        }
        if (relayState == LOW) {
          Setpoint = 200;
          Input = 200;
          runflag = false;
          startflag = false;
          delay (300);                     // delay to prevent sparks on relay contacts
          digitalWrite(RELAY, relayState); // set the Relay:
        }
      }
    }
  }
  lastButtonState = reading;               // save the reading. Next time through the loop, it'll be the lastButtonState:

  //soft start
  if (loopflag == true) {
    myPID.SetTunings(sKp, sKi, sKd);        // Set the PID gain constants and start
    int i = (desiredRPM - tempcounter);
    for (int j = 1; j <= i; j++) {
      Input = RPM;
      Setpoint = tempcounter;
      myPID.Compute();
      // mise à jour de l'amortissement 
      dimming = map(Output, minoutputlimit, maxoutputlimit, maxoutputlimit, minoutputlimit); // inverse the output
      dimming = constrain(dimming, mindimminglimit, maxdimminglimit);     // check that dimming is in 20-625 range
      tempcounter++;
      delayMicroseconds (risetime);
    }
    if (tempcounter >= desiredRPM) {
      lastcounttime = millis();
      lastpiddelay = millis();
      loopflag = false;
      runflag = true;
      tempcounter = 100;
    }
  }

  // normal motor running state
  if (relayState == HIGH && loopflag == false) {
    unsigned long piddelay = millis();

    if ((piddelay - lastpiddelay) > 1000) {     // delay to switch PID values. Prevents hard start
      myPID.SetTunings(rKp, rKi, rKd);          // Set the PID gain constants and start
      lastpiddelay = millis();
    }

    Input = RPM;
    Setpoint = desiredRPM;
    myPID.Compute();
    dimming = map(Output, minoutputlimit, maxoutputlimit, maxoutputlimit, minoutputlimit); // reverse the output
    dimming = constrain(dimming, mindimminglimit, maxdimminglimit);     // check that dimming is in 20-625 range
  }

  // diagnose a fault and turn on protection
  unsigned long counttime = millis();
  if (counttime - lastcounttime >= 1000) {
    if (count == 0 && relayState == HIGH && runflag == true) {
      startflag = false;            // flag to turn off triac before relay turns off
      delay (300);                  // delay to prevent sparks on relay contacts
      digitalWrite(RELAY, LOW);
      relayState = LOW;
      Serial.println("protection on"); 
    }
    lastcount = count;
    count = 0;
    lastcounttime = millis();
  }

  //reset rpm after motor stops
  if (count == 0 && relayState == LOW) {
    RPM = 0;
  }

//  // protection against high rpm. i e triac damage
//  if (relayState == HIGH && RPM > desiredRPM + protection) {
//    startflag = false;            // flag to turn off triac before relay turns off
//    delay (300);                  // delay to prevent sparks on relay contacts
//    digitalWrite(RELAY, LOW);
//    relayState = LOW;
//     Serial.print("hight rpm protection active\t");          // debug value
//     Serial.print(RPM);          // debug value
//     Serial.print("\t");          // debug value
//     Serial.println(desiredRPM + protection);          // debug value
//  }
}
