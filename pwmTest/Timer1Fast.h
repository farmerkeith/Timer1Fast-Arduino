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

#ifndef TIMERONE_FAST_h
#define TIMERONE_FAST_h

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

// #include "Timer1FastBase.h"

class TimerOneFast {
  public:
    void (*user_callback)();

    // initializeFast configures pin modes and registers.
    // the 'clock_cycles' argument sets the period in CPU clock cycles.
    void initializeFastCycles(unsigned long clock_cycles);                
    // the 'microseconds' argument sets the period in microseconds.
    void initializeFast(unsigned long microseconds);                      

    // setPeriodClockCycles sets the PWM period in CPU clock cycles. 
    void  setPeriodClockCycles(unsigned long clock_cycles);               
    
    // setPeriodMicroseconds sets the PWM period in microseconds. 
    // period is 250ns to 4096 us in steps of 62.5 ns
    // then 4096 us to 32.768 ms in steps of 0.5 us
    // then 32.768 ms to 262.144 ms in steps of 4 us
    // then 262.144 ms to 1048.576 ms in steps of 16 us
    // then 1048.576 ms to 4194.304 ms in steps of 64 us
    // max period is 4,194,304 us = 67,108,864 clock cycles
    void setPeriodMicroseconds(unsigned long microseconds);

    /* incrementPeriod increments the PWM period by the smallest value
     * possible with the current clock prescaler.
     */
    void incrementPeriod();

    /* decrementPeriod decrements the PWM period by the smallest value
     * possible with the current clock prescaler.
     */
    void decrementPeriod();

    /* setPwmDuty sets the duty cycle for the specified pin (digital 9 or 10). 
     * The 'duty' argument is a 16-bit number, with 0xFFFF specifying fully 
     * on, and 0x0000 specifying fully off.
     */
    void setPwmDuty(uint8_t pin, unsigned long duty);

    /* incrementPwmDuty increments the PWM duty by the smallest value
     * possible with the current clock prescaler.
     */
    void incrementPwmDuty(uint8_t pin);

    /* decrementPwmDuty decrements the PWM duty by the smallest value
     * possible with the current clock prescaler.
     */
    void decrementPwmDuty(uint8_t pin);

    /* startPwm sets the PWM signal running on 'pin' with duty cycle 'duty'
     * and period 'microseconds'. startPwm also sets the WGM bits.
     * extra parameter "bool invert" to control inverted output
     */
    void startPwm(uint8_t pin, uint32_t duty, uint32_t microseconds, bool invert);
    void startPwm(uint8_t pin, uint32_t duty, uint32_t microseconds);

    /* disablePwm disables the PWM signal for the specified pin.
     */
    void disablePwm(uint8_t pin);
    
  private:
    // functions
    void update_period_immediate();
    void correct_duty_after_changing_period();
    void check_pwm_duty_bounds_pb1();
    void check_pwm_duty_bounds_pb2();
    void update_actual_absolute_pwm_duty_pb1();
    void update_actual_absolute_pwm_duty_pb2();
    void resume();
    unsigned long set_clock_select_bits(unsigned long cycles);

    // functions supporting interrupts
  public:
    void (*isr_callback)();
    byte writeBit (byte reg, byte bitPos, bool val);
    bool readBit (byte reg, byte bitPos);
  private:
    void attach_library_interrupt(void (*interrupt)());
    void static update_period_callback(); 
    void (*library_callback)();
    void static isr_callback_wrapper();
    void attach_interrupt(void (*isr)());
    void detach_library_interrupt();
    void detach_interrupt();
    void increment_absolute_pwm_duty_pb1 ();
    void increment_absolute_pwm_duty_pb2 ();
    void decrement_absolute_pwm_duty_pb1 ();
    void decrement_absolute_pwm_duty_pb2 ();
    void setWgm(byte val);
    void setCom1A(byte val);
    void setCom1B(byte val);
    void setPinMode(uint8_t pin, bool invert);
    
// variables and constants
    const uint32_t maximum_period = 65535; // 0xFFFF
    const uint32_t minimum_period = 3;
    const uint32_t minimum_duty = 0;
    const uint32_t resolution = 65536;
    const uint32_t microseconds_per_second = 1000000; 
    uint8_t library_callback_enabled = 0;
    uint8_t user_callback_enabled = 0;
    uint8_t isr_callback_enabled = 0;
    uint16_t proportional_pwm_duty_pb1;
    uint16_t proportional_pwm_duty_pb2;
    uint8_t pwm_duty_set_proportional;

  public:
    uint8_t clock_select_bits;
    uint16_t prescaler_value;
    uint16_t desired_pwm_period=320;
    uint16_t actual_pwm_period=0;
    uint16_t absolute_pwm_duty_pb1;
    uint16_t absolute_pwm_duty_pb2;
};

extern TimerOneFast Timer1Fast;

#endif
