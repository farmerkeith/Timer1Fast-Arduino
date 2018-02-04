/* Timer1 library for AVR microcontrollers, offering Fast PWM (mode 14).
  Rewritten from scratch by Jude Hungerford with reference to the work of 
  Jesse Tane, Jérôme Despatis, Michael Polli, Lex Talionis and Andrew Richards.
    Copyright (C) 2015  Jude Hungerford

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef TIMERONE_FAST_cpp
#define TIMERONE_FAST_cpp

#include "Timer1Fast.h"

// TimerOneFast Timer1Fast;	// instance of wrapper object


void TimerOneFast::initializeFastCycles(unsigned long clock_cycles){

//  void initialize (uint32_t period) {

  // set pin mode of outputs
  DDRB = _BV(PORTB1) | _BV(PORTB2);

  // WGM mode 14: Fast PWM with TOP set by ICR1
  TCCR1A = _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12);

  // timer Compare Output Mode: clear on Compare Match, set at Bottom
  // (non-inverting mode, rising edges synchronised)
  TCCR1A &= ~(_BV(COM1A0)); // for port PB1, timer 1A
  TCCR1A |= _BV(COM1A1);
  TCCR1A &= ~(_BV(COM1B0)); // for port PB2, timer 1B
  TCCR1A |= _BV(COM1B1);
  setPeriodClockCycles(clock_cycles);
  resume();
}


// set PWM period in CPU clock cycles
void  TimerOneFast::setPeriodClockCycles(unsigned long clock_cycles){
  set_period_clock_cycles_common(clock_cycles);
  update_period_immediate();
}

void  TimerOneFast::resume(){
  // ToDo
}

// common function used by delayed and immediate PWM period-setting functions
void  TimerOneFast::set_period_clock_cycles_common(unsigned long clock_cycles){
  uint32_t cycles = clock_cycles; 
  if (cycles < maximum_period) {
    clock_select_bits = _BV(CS10); // no prescaler
    prescaler_value = 1;
  }
  else if ((cycles >>=3) <= maximum_period) {
    clock_select_bits = _BV(CS11); // prescaler set to 8
    prescaler_value = 8;
  }
  else if ((cycles >>=3) <= maximum_period) {
    clock_select_bits = _BV(CS11) | _BV(CS10); //prescaler set to 64
    prescaler_value = 64;
  }
  else if ((cycles >>=2) <= maximum_period) {
    clock_select_bits = _BV(CS12); //prescaler set to 256
    prescaler_value = 256;
  }
  // ToDo: remove these 2 tests and just set prescaler to 1024
  else if ((cycles >>=2) <= maximum_period) {
    clock_select_bits = _BV(CS12) | _BV(CS10); //prescaler set to 1024
    prescaler_value = 1024;
  }
  else { // out of bounds, prescaler set to maximum
    cycles = maximum_period;
    clock_select_bits = _BV(CS12) | _BV(CS10);
    prescaler_value = 1024;
  }
  desired_pwm_period = cycles; 
}

void TimerOneFast::update_period_immediate(){
  uint8_t old_sreg = SREG;
  cli(); // Disable interrupts for 16 bit register access
  ICR1 = actual_pwm_period = desired_pwm_period; // ICR1 is TOP, Fast PWM
  SREG = old_sreg;
  correct_duty_after_changing_period();
  resume(); // ensure clock select bits are updated
}

void TimerOneFast::correct_duty_after_changing_period(){
  // ToDo;
}


    
void TimerOneFast::initializeFast(uint32_t microseconds) {
  initialize (microseconds);
}

void TimerOneFast::setPeriodMicroseconds(uint32_t microseconds) {
  set_period_microseconds_delayed(microseconds);
}

void TimerOneFast::incrementPeriod() {
  increment_period();
}

void TimerOneFast::decrementPeriod() {
  decrement_period();
}

void TimerOneFast::setPwmDuty(uint8_t pin, uint32_t duty) {
  set_pwm_duty(pin, duty);
}

void TimerOneFast::incrementPwmDuty(uint8_t pin) {
  increment_pwm_duty(pin);
}

void TimerOneFast::decrementPwmDuty(uint8_t pin) {
  decrement_pwm_duty(pin);
}

void TimerOneFast::startPwm(uint8_t pin, uint32_t duty, uint32_t microseconds, bool invert) {
  pwm(pin, duty, microseconds, invert);
}
void TimerOneFast::startPwm(uint8_t pin, uint32_t duty, uint32_t microseconds) {
  pwm(pin, duty, microseconds, 0);
}


void TimerOneFast::disablePwm(uint8_t pin) {
  disable_pwm(pin);
}

/* The following does not work, use underlying Timer1FastBase equivalents
void TimerOneFast::attachUserInterrupt(void (*interrupt)()) {
  attach_user_interrupt(interrupt);
}

void TimerOneFast::detachUserInterrupt() {
  detach_user_interrupt();
}
*/

#endif
