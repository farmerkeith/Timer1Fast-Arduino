// libraries
#include "repeatEvery.h"
#include "Timer1Fast.h"
// hardware configuration constants
const byte INpin=9;    // main PWM output to IN of IR2184
const byte SDpin=10;   // SD/ == Enable pin of IR2184. High, Low or PWM depending on mode

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
  Timer1Fast.initializeFast(15);
  Timer1Fast.setPeriodMicroseconds(10);
  Timer1Fast.setPwmDuty(INpin, 32000);
  Timer1Fast.setPwmDuty(SDpin, 40000);

  
}

void loop() {
  // put your main code here, to run repeatedly:
  repeatEvery(10000, countLoops);
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

