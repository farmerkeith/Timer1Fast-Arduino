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

#include <Timer1Fast.h>

TimerOneFast Timer1Fast;	// instance of wrapper object

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
