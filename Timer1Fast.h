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

#include <Timer1FastBase.h>

class TimerOneFast {
  public:

    /* initializeFast configures pin modes and registers.
     * the 'microseconds' argument sets the period in microseconds.
     */
    void initializeFast(uint32_t microseconds);
    
    /* setPeriodMicroseconds sets the PWM period in microseconds. 
     */
    void setPeriodMicroseconds(uint32_t microseconds);

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
    void setPwmDuty(uint8_t pin, uint32_t duty);

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
     * Specifying 0 for microseconds will leave the period unchanged.
     */
    void startPwm(uint8_t pin, uint32_t duty, uint32_t microseconds);

    /* disbalePwm disables the PWM signal for the specified pin.
     */
    void disablePwm(uint8_t pin);

    /* resumePwm resumes the PWM signal for the specified pin.
     */
    void resumePwm(uint8_t pin);

    /* this user_callback should not be used. Instead, look at these two
     * functions, specified in Timer1FastBase.h:
     * void attach_user_interrupt(void (*interrupt)());
     * void detach_user_interrupt();
     */
    void (*user_callback)();

};

extern TimerOneFast Timer1Fast;

#endif
