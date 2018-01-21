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

#ifndef TIMERONE_FAST_BASE_h
#define TIMERONE_FAST_BASE_h

#define MICROSECONDS_PER_SECOND 1000000
#define RESOLUTION 65536    // Timer1 is 16 bit
#define MINIMUM_PERIOD 3
#define MAXIMUM_PERIOD 65535 // 0xFFFF
#define MINIMUM_DUTY 0

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

// These functions can be called indirectly from Timer1Fast.h, but see below
void initialize (uint32_t period);
void set_period_microseconds_delayed(uint32_t microseconds);
void increment_period();
void decrement_period();
void set_pwm_duty(uint8_t pin, uint32_t duty);
void increment_pwm_duty(uint8_t pin);
void decrement_pwm_duty(uint8_t pin);
void pwm(uint8_t pin, uint32_t duty, uint32_t microseconds, bool invert);
void disable_pwm(uint8_t pin);

// These two functions must be called directly from this library, if used.
void attach_user_interrupt(void (*interrupt)());
void detach_user_interrupt();

#endif
