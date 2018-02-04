// libraries
#include "repeatEvery.h"
#include "Timer1Fast.h"
// hardware configuration constants
const byte INpin=9;    // main PWM output to IN of IR2184
const byte SDpin=10;   // SD/ == Enable pin of IR2184. High, Low or PWM depending on mode
const byte ledPin=13;  // on-board LED 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("\n PWM test sketch");
  // set pin D10 to Output, Low 
  pinMode (SDpin, OUTPUT);
  digitalWrite (SDpin, LOW);
  // set pin D9 to Output, Low 
  pinMode (INpin , OUTPUT);
  digitalWrite (INpin , LOW);
  // set pin D13 to Output, Low 
  pinMode (ledPin , OUTPUT);
  digitalWrite (ledPin , LOW);
  Timer1Fast.initializeFast(20);
  Timer1Fast.setPeriodMicroseconds(20);
  Timer1Fast.setPwmDuty(INpin, 32000);
  Timer1Fast.setPwmDuty(SDpin, 40000);
  Serial.print ("F_CPU=");
  Serial.print (F_CPU);
  Serial.print (" MICROSECONDS_PER_SECOND=");
  Serial.println (MICROSECONDS_PER_SECOND);

  
}

void loop() {
  // put your main code here, to run repeatedly:
  repeatEvery(10000, countLoops);
  repeatEvery(1000, flashLed);
}

void setPeriod(int period){
//  Timer1Fast.setPeriodMicroseconds(period/16);
  while (period%16) {
//    Timer1Fast.incrementPeriod();
    -- period;
  }
}

void setAtime(unsigned long Atime, int period){ // clock cycles, takes any integer values
  unsigned long duty = (Atime<<16)/period;
//  Timer1Fast.setPwmDuty(INpin, duty);
}

void setBtime(unsigned long Btime, int period){ // clock cycles, takes any integer values
  unsigned long duty = (Btime<<16)/period;
//  Timer1Fast.setPwmDuty(SDpin, duty);
}

unsigned long countLoops (bool flag, unsigned long & Time){
  // counts the total executions of loops() between printouts
  static unsigned long lastTime;
  static unsigned long loopCounter=0;
  if (flag) {
    lastTime=Time;
    Serial.print ("\nPerforming countLoops printout function at time ");
    Serial.print((float)millis()/1000,3);
    Serial.print (" loop counter= ");
    Serial.print(loopCounter);
    loopCounter=0;
    return loopCounter;
  }
  ++loopCounter;
  Time = lastTime;
}

unsigned long flashLed (bool flag, unsigned long & Time){
  // flashes on-board LED 
  static unsigned long lastTime;
  static unsigned long loopCounter=0;
  static bool ledState = 0;
  if (flag) {
    lastTime=Time;
    digitalWrite (ledPin , ledState);
    ledState = !ledState;
  }
  ++loopCounter;
  Time = lastTime;
}

