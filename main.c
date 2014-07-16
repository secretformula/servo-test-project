/*
 * The following is a code test by:
 * Nigil Lee <nigil.lee@gmail.com>
 *
 * The code is meant to convert PWM encoded servo motor signals to
 * direction and step  stepper motor signaling
 *
 * Theory:
 * 	* To perform the capture of the input pulse length I will use the input capture
 * 	  function of Timer 1
 * 	* I will then use Timer 0 to generate the output PWM waveform to control the
 * 	  step signal and hence the step velocity via...
 * 	  			fstep * (stepAngleDegree / 360) * pi * 25.4cm = velocity
 * 	* The output from pulse length will be noisy so need to create bins of range to assign
 * 	  a certain speed for predictable results
 *
 * FCLK: 20MHz
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdint.h>
#include <stdlib.h>
#include "compare_values.h"

#define INITIAL_TIMER0_PRESCALE 50

// Cycle counting
#define CYCLES_PER_MS 20000L // FCLK * 1e-3
#define MS_TO_CYCLES(ms) (ms * CYCLES_PER_MS)

#define NUMBER_BINS 200
#define BIN_WIDTH (CYCLES_PER_MS / NUMBER_BINS)

// Pins and Ports
#define SERVO_DDR DDRD
#define SERVO_PORT PORTD
#define SERVO_DIR_PIN PIND7
#define SERVO_STEP_PIN PIND6

// Status Flag masks
#define FOUND_PULSE 0x1

static volatile uint16_t firstEdgeCycle;
static volatile uint16_t lastEdgeCycle;
static volatile uint8_t statusFlags;

int main() {
	// Setup timer 0 to be ready to run
	// TCCR0A |= _BV(COM0A0); // Enable toggle on compare match
	TCCR0A |= _BV(WGM01); // Enable CTC mode
	TCCR0B |= _BV(CS00); // Enable - no prescale
	OCR0A |= INITIAL_TIMER0_PRESCALE;

	// Setup timer 1 to wait for input
	TCCR1A = 0x0; // Normal mode
	TCCR1B |= _BV(ICNC1); // Enable noise cancel
	TCCR1B |= _BV(ICES1); // Enable rising edge input trigger
	TCCR1B |= _BV(CS10); // Enable - no prescale
	TIMSK1 |= _BV(ICIE1); // Enable interrupt for input

	// Setup ports
	SERVO_DDR |= _BV(SERVO_DIR_PIN);
	SERVO_DDR |= _BV(SERVO_STEP_PIN);

	sei(); // Enable global interrupts

	while(1) {
		if(statusFlags & FOUND_PULSE) {
			uint16_t pulseLength;

			// Calulate pulse length
			if(lastEdgeCycle < firstEdgeCycle) {
				// Integer overflow occurred
				pulseLength = (UINT16_MAX - firstEdgeCycle) + lastEdgeCycle;
			}
			else {
				pulseLength = lastEdgeCycle - firstEdgeCycle;
			}

			if(pulseLength < MS_TO_CYCLES(1)) {
				pulseLength = MS_TO_CYCLES(1);
			}
			else if(pulseLength > MS_TO_CYCLES(2)) {
				pulseLength = MS_TO_CYCLES(2) - 1;
			}

			// Bin the pulseLength as actual pulse length will most likely vary by a few cycles
			uint8_t bin;
			for(bin = 0; bin < NUMBER_BINS; bin++) {
				uint16_t low = CYCLES_PER_MS + (BIN_WIDTH * bin);
				uint16_t high = CYCLES_PER_MS + (BIN_WIDTH * (bin + 1));
				if(pulseLength >= low && pulseLength < high) {
					break;
				}
			}

			// Convert bins into more meaningful motor speed
			int8_t motorVelocity;
			if(bin == 99 || bin == 100) {
				// Stopped
				motorVelocity = 0;
			}
			else if(bin < 99) {
				// Backwards
				motorVelocity = -99 + bin;
				SERVO_PORT &= ~_BV(SERVO_DIR_PIN);
			}
			else {
				// Forwards
				motorVelocity = bin - 100;
				SERVO_PORT |= _BV(SERVO_DIR_PIN);
			}

			motorVelocity = abs(motorVelocity);

			/*
			 * We need to convert motor speed into a frequency output via timer 0
			 * Fstep * (stepAngle / 360) * pi * 25.4cm = velocity
			 * Based on a minimum pulse hold time of 4.5uS (I will use 5uS) maximum step
			 * frequency is 200kHz
			 */
			if(motorVelocity == 0) {
				TCCR0A &= ~_BV(COM0A0); // Disable PWM output
				SERVO_PORT &= ~_BV(SERVO_STEP_PIN); // Ensure step is low
			}
			else {
				// We want to generate a wide range of frequencies so we will need
				// to use different prescaler values
				TCCR0A |= _BV(COM0A0); // Enable output compare B
				if(motorVelocity == 1 || motorVelocity == 2) {
					TCCR0B |= _BV(CS01) | _BV(CS00); // prescale = 64
				}
				else if(motorVelocity <= 20) {
					TCCR0B &= ~(_BV(CS00) | _BV(CS01) | _BV(CS02));
					TCCR0B |= _BV(CS01); // prescale = 8
				}
				else {
					TCCR0B &= ~(_BV(CS00) | _BV(CS01) | _BV(CS02));
					TCCR0B |= _BV(CS00); // prescale = 1
				}
				OCR0A = compareVals[motorVelocity - 1]; // Write CTC value from look up table
			}
			statusFlags &= ~FOUND_PULSE;
		}
		else {
			// Idle other (could have other processes here)
			set_sleep_mode(SLEEP_MODE_IDLE);
			sleep_mode();
		}
	}

	return 0;
}

ISR(TIMER1_CAPT_vect) {
	TCCR1B ^= _BV(ICES1); // Toggle edge trigger
	if(TCCR1B & _BV(ICES1)) {
		// This is the falling edge
		lastEdgeCycle = ICR1;
		statusFlags |= FOUND_PULSE;
	}
	else {
		// This is rising edge
		firstEdgeCycle = ICR1;

	}
}
