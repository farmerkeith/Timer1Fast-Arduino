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

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

#include "Timer1FastBase.h"

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
     * and period 'microseconds'. Note that you still have to call
     * initializeFast first in order to set the WGM bits.
     * KH added extra parameter "bool invert" to control inverted output
     */
    void startPwm(uint8_t pin, uint32_t duty, uint32_t microseconds, bool invert);
    void startPwm(uint8_t pin, uint32_t duty, uint32_t microseconds);

    /* disablePwm disables the PWM signal for the specified pin.
     */
    void disablePwm(uint8_t pin);
  private:
    // functions
    void set_period_clock_cycles_common(unsigned long clock_cycles);
    void update_period_immediate();
    void correct_duty_after_changing_period();
    void resume();
    // variables and constants
    const uint32_t maximum_period = 65535; // 0xFFFF
    uint8_t clock_select_bits;
    uint16_t prescaler_value;
    uint16_t desired_pwm_period;
    uint16_t actual_pwm_period;

};

// TimerOneFast Timer1Fast;
extern TimerOneFast Timer1Fast;

#endif
