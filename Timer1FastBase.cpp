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


#ifndef TIMERONE_FAST_BASE
#define TIMERONE_FAST_BASE

#include "Timer1FastBase.h"

/* Function prototypes for helper functions */
void update_period_callback();
void correct_duty_after_changing_period ();
void update_actual_absolute_pwm_duty_pb1 ();
void update_actual_absolute_pwm_duty_pb2 ();
void set_period_microseconds(uint32_t microseconds);
void set_period_microseconds_common(uint32_t microseconds);
void attach_library_interrupt(void (*interrupt)());
void attach_interrupt(void (*isr)());
void detach_library_interrupt();
void detach_interrupt();
void resume();

/* Callback function prototypes */
void (*isr_callback)();
void (*user_callback)();
void (*library_callback)();
void isr_callback_wrapper();


/* Global variables */
uint8_t clock_select_bits;
uint8_t isr_callback_enabled = 0;
uint8_t user_callback_enabled = 0;
uint8_t library_callback_enabled = 0;
uint16_t desired_pwm_period;
uint16_t actual_pwm_period;
uint16_t absolute_pwm_duty_pb1;
uint16_t absolute_pwm_duty_pb2;
uint16_t proportional_pwm_duty_pb1;
uint16_t proportional_pwm_duty_pb2;
uint16_t prescaler_value;
uint8_t pwm_duty_set_proportional;


/* Core functions
 ****************
 */

// initialize the PWM pin settings, set the period in microseconds and 
// set clock running. 
void initialize (uint32_t period) {

  // set pin mode of outputs
  DDRB = _BV(PORTB1) | _BV(PORTB2);

  // WGM mode 14: Fast PWM with TOP set by ICR1
  TCCR1A = _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12);

  // timer Compare Output Mode: clear on Compare Match, set at Bottom
  // (non-inverting mode, rising edges synchronised)
  TCCR1A &= ~(_BV(COM1A0)); // orginal version
  TCCR1A |= _BV(COM1A1);
//  TCCR1A &= ~(_BV(COM1A0)); // original version (mistake? should be B0??)
  TCCR1A &= ~(_BV(COM1B0)); // corrected mistake? made to be B0??
  TCCR1A |= _BV(COM1B1);

  set_period_microseconds(period);


  resume();
}

// set clock running with previously-specified clock_select_bits
void resume() {
  TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
  TCCR1B |= clock_select_bits;
}

// set PWM signal on selected pin with selected duty and period in microseconds
// (but if microseconds == 0, don't change period). See above note about pin
// naming systems.
// Note that initialize() must still be called to set WGM bits.
// KH bool invert added with logic to invert pwm output if invert==1
void pwm(uint8_t pin, uint32_t duty, uint32_t microseconds, bool invert) {
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

/* Period-setting functions
 **************************
 */

// common function used by delayed and immediate PWM period-setting functions
void set_period_microseconds_common(uint32_t microseconds) {
  uint32_t cycles = (F_CPU / MICROSECONDS_PER_SECOND) * microseconds;
  if (cycles < MAXIMUM_PERIOD) {
    clock_select_bits = _BV(CS10); // no prescaler
    prescaler_value = 1;
  }

  else if ((cycles >>=3) <= MAXIMUM_PERIOD) {
    clock_select_bits = _BV(CS11); // prescaler set to 8
    prescaler_value = 8;
  }

  else if ((cycles >>=3) <= MAXIMUM_PERIOD) {
    clock_select_bits = _BV(CS11) | _BV(CS10); //prescaler set to 64
    prescaler_value = 64;
  }

  else if ((cycles >>=2) <= MAXIMUM_PERIOD) {
    clock_select_bits = _BV(CS12); //prescaler set to 256
    prescaler_value = 256;
  }

  else if ((cycles >>=2) <= MAXIMUM_PERIOD) {
    clock_select_bits = _BV(CS12) | _BV(CS10); //prescaler set to 1024
    prescaler_value = 1024;
  }

  else { // out of bounds, prescaler set to maximum
    cycles = MAXIMUM_PERIOD;
    clock_select_bits = _BV(CS12) | _BV(CS10);
    prescaler_value = 1024;
  }

  desired_pwm_period = cycles; 
}

// calculates the appropriate clock_select_bits and desired_pwm_period, but
// delays the actual update until timer overflow to avoid glitches.
void set_period_microseconds_delayed(uint32_t microseconds) {
  set_period_microseconds_common(microseconds);
  attach_library_interrupt (update_period_callback); // delay the update until overflow
}

void update_period_immediate() {
  uint8_t old_sreg = SREG;
  cli(); // Disable interrupts for 16 bit register access
  ICR1 = actual_pwm_period = desired_pwm_period; // ICR1 is TOP, Fast PWM
  SREG = old_sreg;
  correct_duty_after_changing_period();
  resume(); // ensure clock select bits are updated
}

// set PWM period in microseconds
void set_period_microseconds(uint32_t microseconds) {
  set_period_microseconds_common(microseconds);
  update_period_immediate();
}

void increment_period() {
  uint32_t next_cycles = desired_pwm_period + 1;
  if (next_cycles <= MAXIMUM_PERIOD); // OK, nothing to change in clock_select_bits
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
    next_cycles = MAXIMUM_PERIOD;
    //prescaler_value = 1024;
    // no need to change prescaler, it will only fall through to 
    // this case if prescaler == 1024 and next_cycles >= RESOLUTION
  }

  desired_pwm_period = next_cycles;
  //update_period_immediate();
  attach_library_interrupt (update_period_callback); // delay the update until overflow
  // clock prescaler will be updated by the callback
}

void decrement_period() {
  uint32_t next_cycles = desired_pwm_period;
  uint16_t min_cycles;

  // check the smallest value for next_cycles before we should change prescaler
  if (prescaler_value == 1) min_cycles = MINIMUM_PERIOD; // 3 is the minimum for TOP.
  else if (prescaler_value == 8) min_cycles = RESOLUTION >> 3;
  else if (prescaler_value == 64) min_cycles = RESOLUTION >> 3;
  else if (prescaler_value == 256) min_cycles = RESOLUTION >> 2;
  else if (prescaler_value == 1024) min_cycles = RESOLUTION >> 2;

  if (next_cycles >= min_cycles) next_cycles -= 1; // nothing to change in clock_select_bits, just decrement next_cycles.
  else if (clock_select_bits == _BV(CS10)) { // there was no prescaler
    next_cycles = MINIMUM_PERIOD; // cycles < 3 is invalid, so 3 is the minimum.
  }
  else if (clock_select_bits == _BV(CS11)) { // prescaler was set to 8
    next_cycles = MAXIMUM_PERIOD; // set next_cycles to maximum
    clock_select_bits = _BV(CS10); // remove prescaler
  }
  else if (clock_select_bits == _BV(CS10) | _BV(CS11)) { // prescaler was 64
    next_cycles = MAXIMUM_PERIOD; // set next_cycles to maximum
    clock_select_bits = _BV(CS11); // set prescaler to 8
  }
  else if (clock_select_bits == _BV(CS12)) { // prescaler was 256
    next_cycles = MAXIMUM_PERIOD; // set next_cycles to maximum
    clock_select_bits = _BV(CS10) | _BV(CS11); // set prescaler to 64
  }
  else { // prescaler was 1024
    next_cycles = MAXIMUM_PERIOD; // set next_cycles to maximum
    clock_select_bits = _BV(CS12); // set prescaler to 256
  }

  desired_pwm_period = next_cycles;
  attach_library_interrupt (update_period_callback); // delay the update until overflow
  // clock prescaler will be updated by the callback
}

/* Duty-setting functions
 ************************
 */

// Note that this considers pins 1 and 9 to be the same pin, just like the
// original Timer1 library.
void set_pwm_duty(uint8_t pin, uint32_t duty) {

  uint32_t duty_cycle = actual_pwm_period;
  uint8_t old_sreg;
  
  duty_cycle *= duty;
  duty_cycle >>= 16;

  if (duty_cycle < MINIMUM_DUTY) 
    duty_cycle = MINIMUM_DUTY;
  if (duty_cycle > actual_pwm_period - MINIMUM_DUTY) 
    duty_cycle = actual_pwm_period - MINIMUM_DUTY;

  // disable interrupts and set OCR1A or OCR1B
  if(pin == 1 || pin == 9)
  {
    old_sreg = SREG;
    cli();
    OCR1A = duty_cycle;
    SREG = old_sreg;
    proportional_pwm_duty_pb1 = duty;
  }
  else if(pin == 2 || pin == 10) 
  {
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
  if(pin == 1 || pin == 9)
  {
    absolute_pwm_duty_pb1 = duty_cycle;
  }
  else if(pin == 2 || pin == 10) 
  {
    absolute_pwm_duty_pb2 = duty_cycle;
  }
  resume(); // ensure clock is running with correct clock_select_bits

}

// increment pb1 duty with bounds checking
// I'm assuming setting duty == period is invalid, must be at most period - 1
// Repetition is accepted to minimize time with interrupts disabled.
void increment_absolute_pwm_duty_pb1 () {
  if (absolute_pwm_duty_pb1 < (actual_pwm_period - MINIMUM_DUTY)) {
    absolute_pwm_duty_pb1++;
    update_actual_absolute_pwm_duty_pb1();
  }
}

// increment pb2 duty with bounds checking
void increment_absolute_pwm_duty_pb2 () {
  if (absolute_pwm_duty_pb2 < (actual_pwm_period - MINIMUM_DUTY)) {
    absolute_pwm_duty_pb2++;
    update_actual_absolute_pwm_duty_pb2();
  }
}

// decrement pb1 duty with bounds checking
// I'm assuming duty == 0 is invalid, must be at least 1
void decrement_absolute_pwm_duty_pb1 () {
  if (absolute_pwm_duty_pb1 > MINIMUM_DUTY) {
    absolute_pwm_duty_pb1--;
    update_actual_absolute_pwm_duty_pb1();
  }
}

// decrement pb2 duty with bounds checking
void decrement_absolute_pwm_duty_pb2 () {
  if (absolute_pwm_duty_pb2 > MINIMUM_DUTY) {
    absolute_pwm_duty_pb2--;
    update_actual_absolute_pwm_duty_pb2();
  }
}

// increment pwm duty for appropriate pin
void increment_pwm_duty(uint8_t pin) {
  if (pin == 1 || pin == 9) {
    increment_absolute_pwm_duty_pb1();
  }
  else if (pin == 2 || pin == 10) {
    increment_absolute_pwm_duty_pb2();
  }
  // pwm duty was last set using absolute value, not proportional
  pwm_duty_set_proportional = 0;
}

// decrement pwm duty for appropriate pin
void decrement_pwm_duty(uint8_t pin) {
  if (pin == 1 || pin == 9) {
    decrement_absolute_pwm_duty_pb1();
  }
  else if (pin == 2 || pin == 10) {
    decrement_absolute_pwm_duty_pb2();
  }
  // pwm duty was last set using absolute value, not proportional
  pwm_duty_set_proportional = 0;
}

// disables the PWM signal on the selected pin
void disable_pwm(uint8_t pin) {
  if (pin == 1 || pin == 9)
    TCCR1A &= ~(_BV(COM1A1)); // disable PB0
  else if (pin == 2 || pin == 10)
    TCCR1A &= ~(_BV(COM1B1)); // disable PB1
}

/* PWM Duty bounds checking functions
 ************************************
 */

// disable interrupts and set OCR1A = absolute_pwm_duty_pb1
// this is just to remove duplication in all the places this is needed
void update_actual_absolute_pwm_duty_pb1 () {
  uint8_t old_sreg = SREG;
  cli();
  OCR1A = absolute_pwm_duty_pb1;
  SREG = old_sreg;
}

// disable interrupts and set OCR1B = absolute_pwm_duty_pb2
void update_actual_absolute_pwm_duty_pb2 () {
  uint8_t old_sreg = SREG;
  cli();
  OCR1B = absolute_pwm_duty_pb2;
  SREG = old_sreg;
}

// ensure that absolute_pwm_duty_pb1 is within bounds
void check_pwm_duty_bounds_pb1 () {
  if (absolute_pwm_duty_pb1 > (actual_pwm_period - MINIMUM_DUTY)) {
    absolute_pwm_duty_pb1 = actual_pwm_period - MINIMUM_DUTY;
    update_actual_absolute_pwm_duty_pb1();
  }
  else if (absolute_pwm_duty_pb1 < 3) {
    absolute_pwm_duty_pb1 = 3;
    update_actual_absolute_pwm_duty_pb1();
  }
  
}

// ensure that absolute_pwm_duty_pb2 is within bounds
void check_pwm_duty_bounds_pb2 () {
  if (absolute_pwm_duty_pb2 > (actual_pwm_period - MINIMUM_DUTY)) {
    absolute_pwm_duty_pb2 = actual_pwm_period - MINIMUM_DUTY;
    update_actual_absolute_pwm_duty_pb2();
  }
  else if (absolute_pwm_duty_pb2 < MINIMUM_DUTY) {
    absolute_pwm_duty_pb2 = MINIMUM_DUTY;
    update_actual_absolute_pwm_duty_pb2();
  }
  
}

void correct_duty_after_changing_period () {
    check_pwm_duty_bounds_pb1();
    check_pwm_duty_bounds_pb2();
}

/* Interrupt handling functions
 ******************************
 */

// Called upon Timer1 overflow if TOIE1 is set
ISR(TIMER1_OVF_vect) // see pages 65, 136 and 140 of datasheet. TOV1 is set at TOP, triggering interrupt if TOIE1 is set in the TIMSK register
{ 
  uint8_t local_old_sreg = SREG; 
  cli();
  isr_callback(); 
  SREG = local_old_sreg;
} 

// attach interrupt to user_callback and ensure that isr_callback is active
void attach_user_interrupt(void (*callback)()) {
  user_callback = callback; // do this first so there's no danger of calling a null function pointer
  user_callback_enabled = 1; // mark the fact that the library callback is enabled
  attach_interrupt (isr_callback_wrapper);
}

// attach interrupt to library_callback and ensure that isr_callback is active
void attach_library_interrupt(void (*callback)()) {
  library_callback = callback; // do this first so there's no danger of calling a null function pointer
  library_callback_enabled = 1; // mark the fact that the library callback is enabled
  // attach the isr_callback_wrapper to the real overflow interrupt
  attach_interrupt (isr_callback_wrapper);
}

// wraps the user_callback and library_callback in a single function
void isr_callback_wrapper() {
  if (user_callback_enabled) {
    user_callback();
  }
  if (library_callback_enabled) {
    library_callback();
  }
}

// attaches the supplied function to the Timer1 overflow interrupt
void attach_interrupt(void (*isr)()) {
  isr_callback = isr;
  isr_callback_enabled = 1;
  TIMSK1 |= _BV(TOIE1); // set timer overflow interrupt enable bit
  resume(); 
}

// disable the user-specified interrupt (and disable real interrupt if neither user- nor library- interrupt enabled)
void detach_user_interrupt() {
  user_callback_enabled = 0;
  if (!(library_callback_enabled)) {
    detach_interrupt();
  }
}

// disable the library-specified interrupt (and disable real interrupt if neither user- nor library- interrupt enabled)
void detach_library_interrupt() {
  library_callback_enabled = 0;
  if (!(user_callback_enabled)) {
    detach_interrupt();
  }
}

// disable the actual overflow interrupt
void detach_interrupt() {
  isr_callback_enabled = 0;
  TIMSK1 &= ~_BV(TOIE1); // clear the timer overflow interrupt enable bit
}

// used to update actual_pwm_period and ICR1 at the appropriate time to avoid
// the glitches which could occur when updating ICR1 mid-period.
void update_period_callback() {

  // set ICR1 and actual_pwm_period
  uint8_t old_sreg = SREG;
  cli(); // Disable interrupts for 16 bit register access
  ICR1 = actual_pwm_period = desired_pwm_period; // ICR1 is TOP, Fast PWM
  SREG = old_sreg;
  detach_library_interrupt();

  // ensure that duties are still within bounds after changing period
  check_pwm_duty_bounds_pb1();
  check_pwm_duty_bounds_pb2();

  // make sure duties are still within bounds
  correct_duty_after_changing_period();

  // make sure the clock is running with appropriate clock_select_bits
  resume(); // use this function to avoid duplication

}


// start(), stop() and read() have not been added because the MPPT code does not seem to use them.

#endif
