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

TimerOneFast Timer1Fast;	// instance of wrapper object

// extern uint8_t clock_select_bits;
// extern uint16_t prescaler_value;
// extern uint16_t desired_pwm_period;
// extern uint16_t actual_pwm_period;
// extern uint16_t absolute_pwm_duty_pb1;
// extern uint16_t absolute_pwm_duty_pb2;

void TimerOneFast::initializeFast(uint32_t microseconds) {
//  initialize (microseconds);
  uint32_t cycles = (F_CPU / microseconds_per_second) * microseconds;
  initializeFastCycles(cycles);
}

byte TimerOneFast::writeBit (byte reg, byte bitPos, bool val){
  if (val) return reg |= 1<<bitPos;
  else return reg &= ~(1<<bitPos);
}


void TimerOneFast::initializeFastCycles(unsigned long clock_cycles){
  // set pin mode of outputs
  DDRB = writeBit (DDRB, PORTB1,1);
  DDRB = writeBit (DDRB, PORTB2,1);
//  DDRB = _BV(PORTB1) | _BV(PORTB2);

  // WGM mode 14: Fast PWM with TOP set by ICR1
  TCCR1A = writeBit (TCCR1A, WGM11,1);
  TCCR1B = writeBit (TCCR1B, WGM13,1);
  TCCR1B = writeBit (TCCR1B, WGM12,1);

//  TCCR1A = _BV(WGM11);
//  TCCR1B = _BV(WGM13) | _BV(WGM12);

  // timer Compare Output Mode: clear on Compare Match, set at Bottom
  // (non-inverting mode, rising edges synchronised)
  TCCR1A = writeBit (TCCR1A, COM1A0,0); // for port PB1, timer 1A
  TCCR1A = writeBit (TCCR1A, COM1A1,1); // 
//  TCCR1A &= ~(_BV(COM1A0)); // for port PB1, timer 1A
//  TCCR1A |= _BV(COM1A1);
  TCCR1A = writeBit (TCCR1A, COM1B0,0); // for port PB1, timer 1B
  TCCR1A = writeBit (TCCR1A, COM1B1,1); // 
//  TCCR1A &= ~(_BV(COM1B0)); // for port PB2, timer 1B
//  TCCR1A |= _BV(COM1B1);
  set_period_clock_cycles_common(clock_cycles);
  update_period_immediate();
  // was  set_period_clock_cycles(clock_cycles);
  resume();
}

void TimerOneFast::setPeriodMicroseconds(uint32_t microseconds) {
  // ToDo
  uint32_t cycles = (F_CPU / microseconds_per_second) * microseconds;
  setPeriodClockCycles(cycles);
}

void  TimerOneFast::setPeriodClockCycles(unsigned long clock_cycles){
  // ToDo - in progress
  set_period_clock_cycles_common(clock_cycles);
  attach_library_interrupt (update_period_callback); // delay the update until overflow
    // update_period_callback is a function that sets ICR1 to the new period (and then checks duty)
    // attach_library_interrupt is a function that takes a pointer as its argument 
    // (in this case the pointer is to the function update_period_callback)
    // and writes it into the pointer to the function library_callback
    // and then sets the bool variable library_callback_enabled = 1
}

/* Interrupt handling functions
 ******************************
 */

//  Called upon Timer1 overflow if TOIE1 is set
ISR(TIMER1_OVF_vect) // see pages 65, 136 and 140 of datasheet. 
    //  TOV1 is set at TOP, triggering interrupt if TOIE1 is set in the TIMSK register
{ 
  uint8_t local_old_sreg = SREG; 
  cli();
  Timer1Fast.isr_callback(); 
  SREG = local_old_sreg;
} 

// wraps the user_callback and library_callback in a single function
void TimerOneFast::isr_callback_wrapper() {
  if (Timer1Fast.user_callback_enabled) {
    Timer1Fast.user_callback();
  }
  if (Timer1Fast.library_callback_enabled) {
    Timer1Fast.library_callback();
  }
}

// disable the library-specified interrupt (and disable real interrupt if neither user- nor library- interrupt enabled)
void TimerOneFast::detach_library_interrupt() {
  library_callback_enabled = 0;
  if (!(user_callback_enabled)) {
    detach_interrupt();
  }
}

// disable the actual overflow interrupt
void TimerOneFast::detach_interrupt() {
  isr_callback_enabled = 0;
  TIMSK1 &= ~_BV(TOIE1); // clear the timer overflow interrupt enable bit
}

void TimerOneFast::incrementPeriod() {
  increment_period();
}

void TimerOneFast::increment_period() {
  uint32_t next_cycles = desired_pwm_period + 1;
  if (next_cycles <= maximum_period); // OK, nothing to change in clock_select_bits
  else if (clock_select_bits == _BV(CS10)) { // there was no prescaler
    next_cycles >>= 3; // divide next_cycles by 8
    clock_select_bits = _BV(CS11); // set prescaler to 8
    prescaler_value = 8;
  }
  else if (clock_select_bits == _BV(CS11)) { // prescaler was set to 8
    next_cycles >>= 3; // divide next_cycles by 8
    clock_select_bits = _BV(CS10) | _BV(CS11); // set prescaler to 64
    prescaler_value = 64;
  }
  else if (clock_select_bits == _BV(CS10) | _BV(CS11)) { // prescaler was 64
    next_cycles >>= 2; // divide next_cycles by 4
    clock_select_bits = _BV(CS12); // set prescaler to 256
    prescaler_value = 256;
  }
  else if (clock_select_bits == _BV(CS12)) { // prescaler was 256
    next_cycles >>= 2; // divide next_cycles by 4
    clock_select_bits = _BV(CS10) | _BV(CS12); // set prescaler to 1024
    prescaler_value = 1024;
  }
  else { // out of bounds, set period to maximum
    next_cycles = maximum_period;
    //prescaler_value = 1024;
    // no need to change prescaler, it will only fall through to 
    // this case if prescaler == 1024 and next_cycles >= RESOLUTION
  }
  desired_pwm_period = next_cycles;
  //update_period_immediate();
  attach_library_interrupt (update_period_callback); // delay the update until overflow
  // clock prescaler will be updated by the callback
}

void TimerOneFast::decrementPeriod() {
  decrement_period();
}

void TimerOneFast::decrement_period() {
  uint32_t next_cycles = desired_pwm_period;
  uint16_t min_cycles;

  // check the smallest value for next_cycles before we should change prescaler
  if (prescaler_value == 1) min_cycles = minimum_period; // 3 is the minimum for TOP.
  else if (prescaler_value == 8) min_cycles = resolution >> 3;
  else if (prescaler_value == 64) min_cycles = resolution >> 3;
  else if (prescaler_value == 256) min_cycles = resolution >> 2;
  else if (prescaler_value == 1024) min_cycles = resolution >> 2;

  if (next_cycles >= min_cycles) next_cycles -= 1; // nothing to change in clock_select_bits, just decrement next_cycles.
  else if (clock_select_bits == _BV(CS10)) { // there was no prescaler
    next_cycles = minimum_period; // cycles < 3 is invalid, so 3 is the minimum.
  }
  else if (clock_select_bits == _BV(CS11)) { // prescaler was set to 8
    next_cycles = maximum_period; // set next_cycles to maximum
    clock_select_bits = _BV(CS10); // remove prescaler
  }
  else if (clock_select_bits == _BV(CS10) | _BV(CS11)) { // prescaler was 64
    next_cycles = maximum_period; // set next_cycles to maximum
    clock_select_bits = _BV(CS11); // set prescaler to 8
  }
  else if (clock_select_bits == _BV(CS12)) { // prescaler was 256
    next_cycles = maximum_period; // set next_cycles to maximum
    clock_select_bits = _BV(CS10) | _BV(CS11); // set prescaler to 64
  }
  else { // prescaler was 1024
    next_cycles = maximum_period; // set next_cycles to maximum
    clock_select_bits = _BV(CS12); // set prescaler to 256
  }
  desired_pwm_period = next_cycles;
  attach_library_interrupt (update_period_callback); // delay the update until overflow
  // clock prescaler will be updated by the callback
}

void TimerOneFast::setPwmDuty(uint8_t pin, uint32_t duty) {
  set_pwm_duty(pin, duty);
}

// Note that this considers pins 1 and 9 to be the same pin, just like the
// original Timer1 library.
void TimerOneFast::set_pwm_duty(uint8_t pin, uint32_t duty) {
  uint32_t duty_cycle = actual_pwm_period;
  uint8_t old_sreg;
  duty_cycle *= duty;
  duty_cycle >>= 16;
  if (duty_cycle < minimum_duty) duty_cycle = minimum_duty;
  if (duty_cycle > actual_pwm_period - minimum_duty) 
    duty_cycle = actual_pwm_period - minimum_duty;
  // disable interrupts and set OCR1A or OCR1B
  if(pin == 1 || pin == 9) {
    old_sreg = SREG;
    cli();
    OCR1A = duty_cycle;
    SREG = old_sreg;
    proportional_pwm_duty_pb1 = duty;
  }
  else if(pin == 2 || pin == 10) {
    old_sreg = SREG;
    cli();
    OCR1B = duty_cycle;
    SREG = old_sreg;
    proportional_pwm_duty_pb2 = duty;
  }
  // pwm duty was last set using proportional value
  pwm_duty_set_proportional = 1; 
  // Record the selected duty_cycle so we don't have to disable interrupts
  // to read the value when incrementing or decrementing.
  if(pin == 1 || pin == 9) {
    absolute_pwm_duty_pb1 = duty_cycle;
  }
  else if(pin == 2 || pin == 10) {
    absolute_pwm_duty_pb2 = duty_cycle;
  }
  resume(); // ensure clock is running with correct clock_select_bits
}

void TimerOneFast::incrementPwmDuty(uint8_t pin) {
  increment_pwm_duty(pin);
}

// increment pwm duty for appropriate pin
void TimerOneFast::increment_pwm_duty(uint8_t pin) {
  if (pin == 1 || pin == 9) {
    increment_absolute_pwm_duty_pb1();
  }
  else if (pin == 2 || pin == 10) {
    increment_absolute_pwm_duty_pb2();
  }
  // pwm duty was last set using absolute value, not proportional
  pwm_duty_set_proportional = 0;
}

// increment pb1 duty with bounds checking
// I'm assuming setting duty == period is invalid, must be at most period - 1
// Repetition is accepted to minimize time with interrupts disabled.
void TimerOneFast::increment_absolute_pwm_duty_pb1 () {
  if (absolute_pwm_duty_pb1 < (actual_pwm_period - minimum_duty)) {
    absolute_pwm_duty_pb1++;
    update_actual_absolute_pwm_duty_pb1();
  }
}

// increment pb2 duty with bounds checking
void TimerOneFast::increment_absolute_pwm_duty_pb2 () {
  if (absolute_pwm_duty_pb2 < (actual_pwm_period - minimum_duty)) {
    absolute_pwm_duty_pb2++;
    update_actual_absolute_pwm_duty_pb2();
  }
}

void TimerOneFast::decrementPwmDuty(uint8_t pin) {
  decrement_pwm_duty(pin);
}

// decrement pwm duty for appropriate pin
void TimerOneFast::decrement_pwm_duty(uint8_t pin) {
  if (pin == 1 || pin == 9) {
    decrement_absolute_pwm_duty_pb1();
  }
  else if (pin == 2 || pin == 10) {
    decrement_absolute_pwm_duty_pb2();
  }
  // pwm duty was last set using absolute value, not proportional
  pwm_duty_set_proportional = 0;
}

// decrement pb1 duty with bounds checking
// I'm assuming duty == 0 is invalid, must be at least 1
void TimerOneFast::decrement_absolute_pwm_duty_pb1 () {
  if (absolute_pwm_duty_pb1 > minimum_duty) {
    absolute_pwm_duty_pb1--;
    update_actual_absolute_pwm_duty_pb1();
  }
}

// decrement pb2 duty with bounds checking
void TimerOneFast::decrement_absolute_pwm_duty_pb2 () {
  if (absolute_pwm_duty_pb2 > minimum_duty) {
    absolute_pwm_duty_pb2--;
    update_actual_absolute_pwm_duty_pb2();
  }
}

void TimerOneFast::startPwm(uint8_t pin, uint32_t duty, uint32_t microseconds, bool invert) {
  pwm(pin, duty, microseconds, invert);
}
void TimerOneFast::startPwm(uint8_t pin, uint32_t duty, uint32_t microseconds) {
  pwm(pin, duty, microseconds, 0);
}

// set PWM signal on selected pin with selected duty and period in microseconds
// (but if microseconds == 0, don't change period). See above note about pin
// naming systems.
// Note that initialize() must still be called to set WGM bits.
// KH bool invert added with logic to invert pwm output if invert==1
void TimerOneFast::pwm(uint8_t pin, uint32_t duty, uint32_t microseconds, bool invert) {
  if (microseconds > 0) set_period_microseconds_delayed (microseconds);
  if (pin == 1 || pin == 9) { 
    DDRB |= _BV(PORTB1);  // PB1 set to output 
    if (invert){
      // PB1 in inverting PWM mode:
      TCCR1A |= _BV(COM1A0);  // COM1A0 high
      TCCR1A |= _BV(COM1A1);  // COM1A1 high
    } else {
      // PB1 in non-inverting PWM mode:
      TCCR1A &= ~(_BV(COM1A0)); // COM1A0 low
      TCCR1A |= _BV(COM1A1);  // COM1A1 high
    }
  }
  else if (pin == 2 || pin == 10) { 
    DDRB |= _BV(PORTB2);  // PB2 set to output 
    if (invert) {
      // PB2 in inverting PWM mode:
      TCCR1A |= _BV(COM1B0);  // COM1B0 high
      TCCR1A |= _BV(COM1B1);  // COM1B1 high
    } else {
      // PB2 in non-inverting PWM mode:
      TCCR1A &= ~(_BV(COM1B0)); // COM1B0 low
      TCCR1A |= _BV(COM1B1);  // COM1B1 high
    }
  }
  set_pwm_duty (pin, duty);
  resume(); 
}

void TimerOneFast::disablePwm(uint8_t pin) {
  disable_pwm(pin);
}

// disables the PWM signal on the selected pin
void TimerOneFast::disable_pwm(uint8_t pin) {
  if (pin == 1 || pin == 9)
    TCCR1A &= ~(_BV(COM1A1)); // disable PB0
  else if (pin == 2 || pin == 10)
    TCCR1A &= ~(_BV(COM1B1)); // disable PB1
}



// --------------------------
// -----------------------------------------------------------------------------


/* The following does not work, use underlying Timer1FastBase equivalents
void TimerOneFast::attachUserInterrupt(void (*interrupt)()) {
  attach_user_interrupt(interrupt);
}

void TimerOneFast::detachUserInterrupt() {
  detach_user_interrupt();
}
*/
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
//  ::clock_select_bits = clock_select_bits;
//  ::prescaler_value = prescaler_value;
//  ::desired_pwm_period = desired_pwm_period;
}

void TimerOneFast::update_period_immediate(){
  uint8_t old_sreg = SREG;
  cli(); // Disable interrupts for 16 bit register access
  ICR1 = actual_pwm_period = desired_pwm_period; // ICR1 is TOP, Fast PWM
//  ::actual_pwm_period = actual_pwm_period;
  SREG = old_sreg;
  correct_duty_after_changing_period();
  resume(); // ensure clock select bits are updated
}

// attach interrupt to library_callback and ensure that isr_callback is active
void TimerOneFast::attach_library_interrupt(void (*callback)()) {
  library_callback = callback; // do this first so there's no danger of calling a null function pointer
  library_callback_enabled = 1; // mark the fact that the library callback is enabled
  // attach the isr_callback_wrapper to the real overflow interrupt
  attach_interrupt (isr_callback_wrapper);
}

// used to update actual_pwm_period and ICR1 at the appropriate time to avoid
// the glitches which could occur when updating ICR1 mid-period.
void static TimerOneFast::update_period_callback() {
  // set ICR1 and actual_pwm_period
  uint8_t old_sreg = SREG;
  Timer1Fast.actual_pwm_period = Timer1Fast.desired_pwm_period; // ICR1 is TOP, Fast PWM
  cli(); // Disable interrupts for 16 bit register access
  ICR1 = Timer1Fast.desired_pwm_period; // ICR1 is TOP, Fast PWM
  SREG = old_sreg;
  Timer1Fast.detach_library_interrupt();

  // ensure that duties are still within bounds after changing period
  Timer1Fast.check_pwm_duty_bounds_pb1();
  Timer1Fast.check_pwm_duty_bounds_pb2();

  // make sure duties are still within bounds
  Timer1Fast.correct_duty_after_changing_period();

  // make sure the clock is running with appropriate clock_select_bits
  Timer1Fast.resume(); // use this function to avoid duplication
}

// attaches the supplied function to the Timer1 overflow interrupt
void TimerOneFast::attach_interrupt(void (*isr)()) {
  isr_callback = isr;
  isr_callback_enabled = 1;
  TIMSK1 |= _BV(TOIE1); // set timer overflow interrupt enable bit
  resume(); 
}

void TimerOneFast::correct_duty_after_changing_period(){
  check_pwm_duty_bounds_pb1();
  check_pwm_duty_bounds_pb2();
}

// ensure that absolute_pwm_duty_pb1 is within bounds
void TimerOneFast::check_pwm_duty_bounds_pb1(){
  if (absolute_pwm_duty_pb1 > (actual_pwm_period - minimum_duty)) {
    absolute_pwm_duty_pb1 = actual_pwm_period - minimum_duty;
    update_actual_absolute_pwm_duty_pb1();
  }
  else if (absolute_pwm_duty_pb1 < minimum_duty) { // 3 not minimum_duty in original
    absolute_pwm_duty_pb1 = minimum_duty;
    update_actual_absolute_pwm_duty_pb1();
  }
}

// ensure that absolute_pwm_duty_pb2 is within bounds
void TimerOneFast::check_pwm_duty_bounds_pb2(){
  if (absolute_pwm_duty_pb2 > (actual_pwm_period - minimum_duty)) {
    absolute_pwm_duty_pb2 = actual_pwm_period - minimum_duty;
    update_actual_absolute_pwm_duty_pb2();
  }
  else if (absolute_pwm_duty_pb2 < minimum_duty) { // 3 not minimum_duty in original
    absolute_pwm_duty_pb2 = minimum_duty;
    update_actual_absolute_pwm_duty_pb2();
  }
}

// disable interrupts and set OCR1A = absolute_pwm_duty_pb1
// this is just to remove duplication in all the places this is needed
void TimerOneFast::update_actual_absolute_pwm_duty_pb1(){
  uint8_t old_sreg = SREG;
  cli();
  OCR1A = absolute_pwm_duty_pb1;
//  ::absolute_pwm_duty_pb1 = absolute_pwm_duty_pb1;
  SREG = old_sreg;
}

// disable interrupts and set OCR1B = absolute_pwm_duty_pb2
void TimerOneFast::update_actual_absolute_pwm_duty_pb2(){
  uint8_t old_sreg = SREG;
  cli();
  OCR1B = absolute_pwm_duty_pb2;
//  ::absolute_pwm_duty_pb2 = absolute_pwm_duty_pb2;
  SREG = old_sreg;
}

// set clock running with previously-specified clock_select_bits
void  TimerOneFast::resume(){
  TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
  TCCR1B |= clock_select_bits;
}



#endif
