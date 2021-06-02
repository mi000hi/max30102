/*
 * max30102.c
 *
 *  Created on: June 5, 2021
 *      Author: michael
 */

/*************** INCLUDE ***************/
#include "max30102.h"

#include <string.h>

/*************** GLOBAL VARIABLE ***************/
max30102_communication_t max30102_comm;

/**
 * Save the pointer of the write and read function
 * that the user defined. Also modify the config struct
 * of the user with the default variable.
 *
 * @param  setup   user defined setup struct (ptr)
 * @retval       	interface status (MANDATORY: return 0 -> no Error) TODO
 *
 */
void max30102_setup_communication(max30102_t *configs) {

	/* Save pointer to function globally */
	max30102_comm.read = configs->comm.read;
	max30102_comm.write = configs->comm.write;

	// TODO: Implement return if successful
}

/**
 * General register read command
 *
 * @param				uint8_t reg: address to the register
 * @param				uint8_t* reg: ptr to get the read value
 *
 */
void max30102_read_reg(uint8_t reg, uint8_t *value) {
	max30102_comm.read(reg, value, 1);
}

/**
 * General register write command
 *
 * @param				uint8_t reg: address to the register
 * @param				uint8_t* reg: ptr to get the read value
 *
 */
void max30102_write_reg(uint8_t reg, uint8_t *value) {
	max30102_comm.write(reg, value, 1);
}

/**
 * writes the specified bit value to a single bit in a register while leaving the
 * other bits untouched.
 *
 * @param reg		address of the register where we want to change a bit
 * @param bitNumber	the number of the bit to change its value, 0: lsb, 1: msb
 * @param value		the value that the new bit should have, only 1 bit
 */
void max30102_setSingleBit(uint8_t reg, uint8_t bitNumber, uint8_t value) {
	// make sure value is only one bit
	value = value & 0b1;

	// read the old register value
	uint8_t data;
	max30102_comm.read(reg, &data, 1);

	// write the new register value
	data = (data & (0xFF ^ (0b1 << bitNumber))) | (value << bitNumber);
	max30102_comm.write(reg, &data, 1);
}

/**
 * clears the registers that will be used
 */
void max30102_setupForMeasurement() {
	// clear registers that will store measurement information
	uint8_t data = 0x00;
	max30102_comm.write(MAX30102_REG_FIFO_WRITE_PTR, &data, 1);
	max30102_comm.write(MAX30102_REG_OVERFLOW_COUNTER, &data, 1);
	max30102_comm.write(MAX30102_REG_FIFO_READ_PTR, &data, 1);

	// set pulse width to 411us
	data = 0x00;
	max30102_comm.read(MAX30102_REG_SPO2_CONF, &data, 1);
	data = (data & 0b11111100) | 0b11;
	max30102_comm.write(MAX30102_REG_SPO2_CONF, &data, 1);

	// set LED current to 12.6mA
	data = 0x3F;
	max30102_comm.write(MAX30102_REG_LED1_AMP, &data, 1); // red led
	max30102_comm.write(MAX30102_REG_LED2_AMP, &data, 1); // ir led

}

void max30102_setMeasurementMode(uint8_t mode) {
	// make sure the mode is only 3 bits
	mode = mode & 0b111;

	// update the measurement mode
	uint8_t data;
	max30102_comm.read(MAX30102_REG_MODE_CONF, &data, 1);
	data = (data & 0b11000000) | mode;
	max30102_comm.write(MAX30102_REG_MODE_CONF, &data, 1);
}

/**
 * starts the measurement of the heart rate
 */
void max30102_startHeartrateMeasurement() {
	// enter the heartrate measure mode
	max30102_setMeasurementMode(MAX30102_MODE_HEART_RATE);

	// activate A_FULL_EN interrupt
	// TODO: maybe need to do before setting the mode
	max30102_setSingleBit(MAX30102_REG_INT_EN_1, 7, 1);
}

/**
 * stops the measurement of the heart rate
 */
void max30102_stopHeartrateMeasurement() {
	// deactivate A_FULL_EN interrupt
	max30102_setSingleBit(MAX30102_REG_INT_EN_1, 7, 0);
}

/**
 * waits until the FIFO queue as almost full, and then reads all 32 bytes
 *
 * @param buf		the buffer where the data is written to
 */
void max30102_waitAndGetHeartrateSamples(uint8_t *buf) {

	// wait for the A_FULL interrupt
	uint8_t data;
	while(1) {
		max30102_comm.readInterruptPin(&data);

		// break if we have an interrupt
		// TODO: check if interrupt is of type A_FULL, only needed if other interrupts are active
		if(data == 0) break;

		// TODO: change/remove delay time
		HAL_Delay(1);
	}

	// read the samples from the FIFO
	// TODO: reads the whole fifo, might need/want to read less bytes
	max30102_comm.read(MAX30102_REG_FIFO_DATA, buf, 32);

}
