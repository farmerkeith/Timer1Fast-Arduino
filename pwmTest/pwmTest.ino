// libraries
#include "repeatEvery.h"
#include "Timer1Fast.h"
#include "inspector.h"
// hardware configuration constants
const byte INpin=9;    // main PWM output to IN of IR2184
const byte SDpin=10;   // SD/ == Enable pin of IR2184. High, Low or PWM depending on mode
const byte ledPin=13;  // on-board LED 

void setup() {
  Serial.begin(115200);
  Serial.println("\n PWM test sketch");
  // set pin D10 to Output, Low 
//  pinMode (SDpin, OUTPUT);
//  digitalWrite (SDpin, LOW);
  // set pin D9 to Output, Low 
//  pinMode (INpin , OUTPUT);
//  digitalWrite (INpin , LOW);
  // set pin D13 to Output, Low 
  pinMode (ledPin , OUTPUT);
  digitalWrite (ledPin , LOW);
//  Timer1Fast.dummy();
  Timer1Fast.initializeFastCycles(65558);
//  Serial.print ("\n TCCR1A="); Serial.print(TCCR1A, BIN);
//  inspector.all();
//  Timer1Fast.initializeFast(20);
//  clock_select_bits = Timer1Fast.clock_select_bits;
//  Timer1Fast.setPeriodMicroseconds(20);
  Timer1Fast.setPwmDuty(INpin, 32000);
  Timer1Fast.setPwmDuty(SDpin, 40000);
  Timer1Fast.startPwm(INpin, 32000, 0);
  Timer1Fast.startPwm(SDpin, 40000, 0);
  Serial.print ("\n Timer1Fast clock_select_bits="); Serial.print (Timer1Fast.clock_select_bits);
  Serial.print ("\n Timer1Fast prescaler_value="); Serial.print (Timer1Fast.prescaler_value);
  Serial.print ("\n Timer1Fast desired_pwm_period="); Serial.print (Timer1Fast.desired_pwm_period);
  Serial.print ("\n Timer1Fast actual_pwm_period="); Serial.print (Timer1Fast.actual_pwm_period);
  Serial.print ("\n Timer1Fast absolute_pwm_duty_pb1="); Serial.print (Timer1Fast.absolute_pwm_duty_pb1);
  Serial.print ("\n Timer1Fast absolute_pwm_duty_pb2="); Serial.print (Timer1Fast.absolute_pwm_duty_pb2);
  Serial.print ("\n DDRB="); Serial.print(DDRB, BIN);
  Serial.print ("\n ICR1="); Serial.print(ICR1);
  Serial.print (" OCR1A="); Serial.print(OCR1A);
  Serial.print (" OCR1B="); Serial.print(OCR1B);
  Serial.print ("\n TCNT1="); Serial.print(TCNT1);
  Serial.print ("\n TCCR1A="); Serial.print(TCCR1A, BIN);
  Serial.print (" TCCR1B="); Serial.print(TCCR1B,BIN);
  Serial.print (" TCCR1C="); Serial.print(TCCR1C,BIN);
  Serial.print ("\n TIFR1="); Serial.print(TIFR1, BIN);
  Serial.print (" TIMSK1="); Serial.print(TIMSK1, BIN);
  Serial.print (" SREG="); Serial.print(SREG, BIN);
  Serial.println();   
//  Timer1Fast.setPeriodClockCycles(250);
//  Timer1Fast.setPeriodMicroseconds(10);
  Serial.print ("\n ICR1="); Serial.print(ICR1);
  Serial.print (" OCR1A="); Serial.print(OCR1A);
  Serial.print (" OCR1B="); Serial.print(OCR1B);
  Serial.println();   
  
/*
  byte te = 0;
  te=Timer1Fast.writeBit (te, 0, 1);
  te=Timer1Fast.writeBit (te, 3, 1);
  Serial.println(inspector.readBit(te,0));
  Serial.println(inspector.readBit(te,1));
  Serial.println(inspector.readBit(te,2));
  Serial.println(inspector.readBit(te,3));
*/

  inspector.all();
/*  
  inspector.wgm();
  inspector.com1a();
  inspector.com1b();
  inspector.clock_select();
  Serial.print ("\n TIMSK1="); Serial.print(TIMSK1, BIN);
  inspector.timsk1();
  inspector.tifr1();
  inspector.globalInterrupt();
*/
//  Timer1Fast.tccr1a = TCCR1A;
//  TCCR1A = Timer1Fast.tccr1a;
//    Timer1Fast.wgm1 = 0;
//    byte wg = Timer1Fast.tccr1a; 
//    byte wg = Timer1Fast.wgm1; 
//    Timer1Fast.tccr1a.wgm101 = 2;
//    Timer1Fast.tccr1b.wgm123 = 3;
//    Serial.print ("\n wg="); Serial.print(wg, BIN);
//    Serial.print (" wgm123="); Serial.print(Timer1Fast.tccr1b.wgm123, BIN);
//    Serial.print (" wgm101="); Serial.print(Timer1Fast.tccr1a.wgm101, BIN);
//    Serial.print (" tccr1a="); Serial.print(Timer1Fast.tccr1a, BIN);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  repeatEvery(8000, countLoops);
  repeatEvery(4200, flashLed);
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
    Serial.println(loopCounter);
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
    // Serial.print (" TCNT1="); Serial.print(TCNT1);
    int static counter =0;
    counter ++;
    if (counter>100) counter=1;
    Serial.print ("\n counter="); Serial.print(counter);
//    long us = (long)counter*counter*counter*20;
//    Serial.print ("\n period us="); Serial.print(us);
      Timer1Fast.decrementPeriod();
//      Timer1Fast.incrementPeriod();
//    Timer1Fast.setPeriodMicroseconds(us);
//    if (ledState) Timer1Fast.setPeriodMicroseconds(us);
    Timer1Fast.incrementPwmDuty(INpin);
    Timer1Fast.incrementPwmDuty(SDpin);
    
//    Serial.print ("\n period us="); Serial.print(us);
//    Serial.print (" clocks="); Serial.print(us*16);
    Serial.print ("\n ICR1="); Serial.print(ICR1);
    Serial.print (" OCR1A="); Serial.print(OCR1A);
    Serial.print (" OCR1B="); Serial.print(OCR1B);
    Serial.print (" TCCR1B="); Serial.print(TCCR1B);
    Serial.print ("\n Timer1Fast.desired_pwm_period="); 
    Serial.print(Timer1Fast.desired_pwm_period);
    Serial.print ("\n Timer1Fast.actual_pwm_period="); 
    Serial.print(Timer1Fast.actual_pwm_period);
    inspector.clock_select();
    
//    inspector.all();
    Serial.println();   
  }
  ++loopCounter;
  Time = lastTime;
}


