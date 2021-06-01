/*
 * sx1508b.c
 *
 *  Created on: May 4, 2021
 *      Author: michael
 */

/*************** INCLUDE ***************/
#include "sx1508b.h"
#include "sx1508b_core.h"
#include "sx1508b_default.h"

#include <string.h>

/*************** GLOBAL VARIABLE ***************/
sx1508b_communication_t sx1508b_comm;
/**
 * Save the pointer of the write and read function
 * that the user defined. Also modify the config struct
 * of the user with the default variable.
 *
 * @param  setup   user defined setup struct (ptr)
 * @retval       	interface status (MANDATORY: return 0 -> no Error) TODO
 *
 */
void sx1508b_setup_communication(sx1508b_t *configs) {

	/* Save pointer to function globally */
	sx1508b_comm.read = configs->comm.read;
	sx1508b_comm.write = configs->comm.write;

	// TODO: Implement return if successful
}

/**
 * General register read command
 *
 * @param				uint8_t reg: address to the register
 * @param				uint8_t* reg: ptr to get the read value
 *
 */
void sx1508b_read_reg(uint8_t reg, uint8_t *value) {
	sx1508b_comm.read(reg, value, 1);
}

/**
 * General register write command
 *
 * @param				uint8_t reg: address to the register
 * @param				uint8_t* reg: ptr to get the read value
 *
 */
void sx1508b_write_reg(uint8_t reg, uint8_t *value) {
	sx1508b_comm.write(reg, value, 1);
}

/**
 * activates the internal oscillator (2MHz base clock) and puts fOSCOUT=clock/(2^(divider-1)) onto its output pin
 *
 * @param divider		4 bit value that divides the base clock of 2MHz to define fOSCOUT
 */
void sx1508b_configure_clock(uint8_t divider) {

	// divider must be a 4 bit value
	divider = divider & 0b1111;

	// clock X -- activate the internal clock signal
	// bit 7   -- unused								-- 0
	// bit 6:5 -- Oscillator frequency (fOSC) source	-- internal
	// bit 4   -- OSCIO pin function (Cf. §4.7)			-- output pin
	// bit 3:0 -- fOSCOUT = fOSC/(2^(RegClock[3:0]-1))
	uint8_t reg = (0b0101 << 4) | divider;
	sx1508b_comm.write(SX1508B_REG_CLOCK, &reg, 1);
}

/**
 * configures the PWM clock frequency using fPWM=clock/(2^(divider-1))
 *
 * @param divider		3 bit value that divides the base clock of 2MHz to define fPWM
 */
void sx1508b_configure_pwm(uint8_t divider) {

	// divider must be a 3 bit value
	divider = divider & 0b111;

	// read previous configuration
	uint8_t currentSettings = 0;
	sx1508b_comm.read(SX1508B_REG_MISC, &currentSettings, 1);

	// set pwm clock and mode
	// bit 7   -- LED Driver mode for Bank B ‘s fading capable IOs (IO7)	-- linear
	// bit 6:4 -- 0: OFF, else: ClkX = fOSC/(2^(RegMisc[6:4]-1))
	// bit 3   -- LED Driver mode for Bank A ‘s fading capable IOs (IO3)	-- linear
	// bit 2:0 -- keep value
	uint8_t reg = (currentSettings & 0b00000111) | (divider << 4);
	sx1508b_comm.write(SX1508B_REG_MISC, &reg, 1);
}

/**
 * configures the specified pin as a PWM pin. pull up or pull down resistors can be selected
 *
 * @param pinNumber		the number of the pin in 0:7 that is used as a PWM pin
 * @param pullUp		0: no pullup resistor, 1: use pullup resistor
 * @param pullDown		0: nu pulldown resistor, 1: use pulldown resistor
 */
void sx1508b_configure_pwm_pin(uint8_t pinNumber, uint8_t pullUp,
		uint8_t pullDown) {

	// pullUp and pullDown must be one bit
	pullUp = pullUp & 0b1;
	pullDown = pullDown & 0b1;

	// disable input buffer for specified pin
	uint8_t currentSettings = 0;
	sx1508b_comm.read(SX1508B_REG_INPUT_DISABLE, &currentSettings, 1);
	uint8_t reg = currentSettings | (0b1 << pinNumber);
	sx1508b_comm.write(SX1508B_REG_INPUT_DISABLE, &reg, 1);

	// update pull up for specified pin
	currentSettings = 0;
	sx1508b_comm.read(SX1508B_REG_PULL_UP, &currentSettings, 1);
	reg = (currentSettings & (0b11111111 ^ (0b1 << pinNumber)))
			| (pullUp << pinNumber);
	sx1508b_comm.write(SX1508B_REG_PULL_UP, &reg, 1);

	// update pull down for specified pin
	currentSettings = 0;
	sx1508b_comm.read(SX1508B_REG_PULL_DOWN, &currentSettings, 1);
	reg = (currentSettings & (0b11111111 ^ (0b1 << pinNumber)))
			| (pullDown << pinNumber);
	sx1508b_comm.write(SX1508B_REG_PULL_DOWN, &reg, 1);

	// enable open drain for specified pin
	currentSettings = 0;
	sx1508b_comm.read(SX1508B_REG_OPEN_DRAIN, &currentSettings, 1);
	reg = currentSettings | (0b1 << pinNumber);
	sx1508b_comm.write(SX1508B_REG_OPEN_DRAIN, &reg, 1);

	// configure specified pin as output pin
	sx1508b_set_pin_direction(pinNumber, 0);

	// enable LED driver for specified pin
	currentSettings = 0;
	sx1508b_comm.read(SX1508B_REG_LED_DRIVER_ENABLE, &currentSettings, 1);
	reg = currentSettings | (0b1 << pinNumber);
	sx1508b_comm.write(SX1508B_REG_LED_DRIVER_ENABLE, &reg, 1);

	// clear data bit on specified pin to enable PWM
	currentSettings = 0;
	sx1508b_comm.read(SX1508B_REG_DATA, &currentSettings, 1);
	reg = currentSettings & (0b11111111 ^ (0b1 << pinNumber));
	sx1508b_comm.write(SX1508B_REG_DATA, &reg, 1);
}

/**
 * sets the PWM intensity given as a float in [0,1] to the specified pin.
 * the intensity is dependent on a configured pull up or pull down resistor
 *
 * @param pinNumber		the number of the pin in 0:7 whose intensity should be changed
 * @param intensity		floating point number in [0,1] corresponding to the duty cycle of the PWM
 */
void sx1508b_set_pwm_intensity(uint8_t pinNumber, float intensity) {

	uint8_t result = intensity * 255;
	uint8_t reg = 0;

	switch (pinNumber) {
	case 0:
		reg = SX1508B_REG_ION0;
		break;
	case 1:
		reg = SX1508B_REG_ION1;
		break;
	case 2:
		reg = SX1508B_REG_ION2;
		break;
	case 3:
		reg = SX1508B_REG_ION3;
		break;
	case 4:
		reg = SX1508B_REG_ION4;
		break;
	case 5:
		reg = SX1508B_REG_ION5;
		break;
	case 6:
		reg = SX1508B_REG_ION6;
		break;
	case 7:
		reg = SX1508B_REG_ION7;
		break;
	default:
		return;
	}

	// set the intensity for the specified pin
	sx1508b_comm.write(reg, &result, 1);
}

/**
 * sets the value of one pin in 0:7
 *
 * @param pinNumber		the number of the pin to update its value
 * @param value			the value to write onto the specified pin
 */
void sx1508b_set_pin(uint8_t pinNumber, uint8_t value) {

	// value must be 1 bit
	value = value & 0b1;

	// update the register value
	uint8_t currentSettings = 0;
	sx1508b_comm.read(SX1508B_REG_DATA, &currentSettings, 1);
	uint8_t reg = (currentSettings & (0b11111111 ^ (0b1 << pinNumber))) | (value << pinNumber);
	sx1508b_comm.write(SX1508B_REG_DATA, &reg, 1);
}

/**
 * reads the value of the specified pin and writes it into the given data buffer
 *
 * @param pinNumber		number of the pin, whose value is read out
 * @param data			buffer where the value of the pin is stored into
 */
void sx1508b_read_pin(uint8_t pinNumber, uint8_t *data) {

	// update the register value
	uint8_t currentSettings = 0;
	sx1508b_comm.read(SX1508B_REG_DATA, &currentSettings, 1);
	//TODO: doublecheck if this pointer access is correct
	*data = (currentSettings & (0b00000000 ^ (0b1 << pinNumber))) >> pinNumber;
}

/**
 * toggles the value of one pin in 0:7 between the output value 0 and 1
 *
 * @param pinNumber		the number of the pin to toggle its value
 */
void sx1508b_toggle_pin(uint8_t pinNumber) {

	// update the register value
	uint8_t currentSettings = 0;
	sx1508b_comm.read(SX1508B_REG_DATA, &currentSettings, 1);
	uint8_t reg = currentSettings ^ (0b00000000 ^ (0b1 << pinNumber));
	sx1508b_comm.write(SX1508B_REG_DATA, &reg, 1);
}

/**
 * defines the direction of the specified pin
 * 0: pin is an output pin
 * 1: pin is an input pin
 *
 * @param pinNumber		the number of the pin to update its direction
 * @param value			the direction, 0: output, 1: input
 */
void sx1508b_set_pin_direction(uint8_t pinNumber, uint8_t value) {

	// value must be 1 bit
	value = value & 0b1;

	// update the register value
	uint8_t currentSettings = 0;
	sx1508b_comm.read(SX1508B_REG_DIR, &currentSettings, 1);
	uint8_t reg = currentSettings & (0b11111111 ^ (value << pinNumber));
	sx1508b_comm.write(SX1508B_REG_DIR, &reg, 1);
}
