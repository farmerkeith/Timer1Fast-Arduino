// libraries
#include "repeatEvery.h"
#include "Timer1Fast.h"
// hardware configuration constants
const byte INpin=9;    // main PWM output to IN of IR2184
const byte SDpin=10;   // SD/ == Enable pin of IR2184. High, Low or PWM depending on mode
const byte ledPin=13;  // on-board LED 

// extern uint8_t clock_select_bits;
// extern uint16_t prescaler_value;
// extern uint16_t desired_pwm_period;
// extern uint16_t actual_pwm_period;
// extern uint16_t absolute_pwm_duty_pb1;
// extern uint16_t absolute_pwm_duty_pb2;



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
//  Timer1Fast.dummy();
  Timer1Fast.initializeFastCycles(320);
//  Timer1Fast.initializeFast(20);
//  clock_select_bits = Timer1Fast.clock_select_bits;
//  Timer1Fast.setPeriodMicroseconds(20);
  Timer1Fast.setPwmDuty(INpin, 32000);
  Timer1Fast.setPwmDuty(SDpin, 40000);
  Serial.print (" Timer1Fast clock_select_bits="); Serial.print (Timer1Fast.clock_select_bits);
  Serial.print (" Timer1Fast prescaler_value="); Serial.print (Timer1Fast.prescaler_value);
  Serial.print (" Timer1Fast desired_pwm_period="); Serial.print (Timer1Fast.desired_pwm_period);
  Serial.print (" Timer1Fast actual_pwm_period="); Serial.print (Timer1Fast.actual_pwm_period);
  Serial.print (" Timer1Fast absolute_pwm_duty_pb1="); Serial.print (Timer1Fast.absolute_pwm_duty_pb1);
  Serial.print (" Timer1Fast absolute_pwm_duty_pb2="); Serial.print (Timer1Fast.absolute_pwm_duty_pb2);
  Serial.print ("\n DDRB="); Serial.print(DDRB, BIN);
  Serial.print ("\n ICR1="); Serial.print(ICR1);
  Serial.print (" OCR1A="); Serial.print(OCR1A);
  Serial.print (" OCR1B="); Serial.print(OCR1B);
  Serial.print ("\n TCNT1="); Serial.print(TCNT1);
  Serial.print ("\n TCCR1A="); Serial.print(TCCR1A);
  Serial.print (" TCCR1B="); Serial.print(TCCR1B);
  Serial.print (" TCCR1C="); Serial.print(TCCR1C);
  Serial.print ("\n TIFR1="); Serial.print(TIFR1);
  Serial.print (" SREG="); Serial.print(SREG);
  Serial.println();   
  Timer1Fast.setPeriodClockCycles(150);
//  Timer1Fast.setPeriodMicroseconds(10);
  Serial.print ("\n ICR1="); Serial.print(ICR1);
  Serial.print (" OCR1A="); Serial.print(OCR1A);
  Serial.print (" OCR1B="); Serial.print(OCR1B);
  Serial.println();   
  
}

void loop() {
  // put your main code here, to run repeatedly:
  repeatEvery(8000, countLoops);
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
    long us = counter*200;
    Timer1Fast.setPeriodMicroseconds(us);
    Timer1Fast.incrementPwmDuty(INpin);
    Timer1Fast.incrementPwmDuty(SDpin);
    
    Serial.print ("\n period us="); Serial.print(us);
    Serial.print (" clocks="); Serial.print(us*16);
    Serial.print (" ICR1="); Serial.print(ICR1);
    Serial.print (" OCR1A="); Serial.print(OCR1A);
    Serial.print (" OCR1B="); Serial.print(OCR1B);
    Serial.print (" TCCR1B="); Serial.print(TCCR1B);
    Serial.println();   

  }
  ++loopCounter;
  Time = lastTime;
}

