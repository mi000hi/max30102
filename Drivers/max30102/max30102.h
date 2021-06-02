/*
 * sx1508b.h
 *
 *  Created on: May 4, 2021
 *      Author: michael
 */

#ifndef MAX30102_MAX30102_H_
#define MAX30102_MAX30102_H_

#include <stdint.h>		//may not be ideal depending on linking of implementation, maybe add as requirement?

/* Read and write */

#define MAX30102_ADDRESS_BASE 			0b1010111	/**< device address **/

/* Mode Control */
#define MAX30102_MODE_HEART_RATE		0b010		/**< only red led active **/
#define MAX30102_MODE_SPO2				0b011		/**< only ir led active **/
#define MAX30102_MODE_MULTI_LED			0b111		/**< both red and ir leds active **/


/*********** REGISTERS ***********/

#define MAX30102_REG_INT_STAT_1					0x00	/**< interrupt status register 1 **/
#define MAX30102_REG_INT_STAT_2					0x01	/**< interrupt status register 2 **/
#define MAX30102_REG_INT_EN_1					0x02	/**< interrupt enable register 1 **/
#define MAX30102_REG_INT_EN_2					0x03	/**< interrupt enable register 2 **/

#define MAX30102_REG_FIFO_WRITE_PTR				0x04	/**< fifo write pointer **/
#define MAX30102_REG_OVERFLOW_COUNTER			0x05	/**< overflow counter **/
#define MAX30102_REG_FIFO_READ_PTR				0x06	/**< fifo read pointer **/
#define MAX30102_REG_FIFO_DATA					0x07	/**< fifo data register **/

#define MAX30102_REG_FIFO_CONF					0x08	/**< fifo configuration **/
#define MAX30102_REG_MODE_CONF					0x09	/**< mode configuration **/
#define MAX30102_REG_SPO2_CONF					0x0A	/**< spo2 configuration **/
#define MAX30102_REG_LED1_AMP					0x0C	/**< led pulse amplitude configuration, msb **/
#define MAX30102_REG_LED2_AMP					0x0D	/**< led pulse amplitude configuration, lsb **/
#define MAX30102_REG_MULTI_LED_MODE_CTRL_MSB	0x11	/**< multi led mode control register, msb **/
#define MAX30102_REG_MULTI_LED_MODE_CTRL_LSB	0x12	/**< multi led mode control register, lsb **/

#define MAX30102_REG_TEMP_INT					0x1F	/**< die temperature integer **/
#define MAX30102_REG_TEMP_FRAC					0x20	/**< die temperature fraction **/
#define MAX30102_REG_TEMP_EN					0x21	/**< die temperature configuration **/

#define MAX30102_REG_REVISION_ID				0xFE	/**< revision ID register **/
#define MAX30102_REG_PART_ID					0xFF	/**< part ID register **/

/*********** STRUCTS ***********/

typedef struct {
	void (*write)(uint8_t reg, uint8_t *buf, uint8_t bytes); /**< User write function **/
	void (*read)(uint8_t reg, uint8_t *buf, uint8_t bytes); /**< User read function **/
	void (*readInterruptPin)(uint8_t *buf); /**< User interrupt pin read function **/
} max30102_communication_t;

typedef struct {
	max30102_communication_t comm;
//  sx1508b_configuration_t config;
} max30102_t;

/*********** Functions declaration ***********/

/**
 * Save the pointer of the write and read function
 * that the user defined. Also modify the config struct
 * of the user with the default variable.
 *
 * @param  configs   	user defined setup struct (ptr)
 * @retval       		interface status (MANDATORY: return 0 -> no Error) TODO
 *
 **/
void max30102_setup_communication(max30102_t *configs);

/**
 * writes the specified bit value to a single bit in a register while leaving the
 * other bits untouched.
 *
 * @param reg		address of the register where we want to change a bit
 * @param bitNumber	the number of the bit to change its value, 0: lsb, 1: msb
 * @param value		the value that the new bit should have, only 1 bit
 */
void max30102_setSingleBit(uint8_t reg, uint8_t bitNumber, uint8_t value);

void max30102_setupForMeasurement();

void max30102_setMeasurementMode(uint8_t mode);

/**
 * starts the measurement of the heart rate
 */
void max30102_startHeartrateMeasurement();

/**
 * stops the measurement of the heart rate
 */
void max30102_stopHeartrateMeasurement();

/**
 * waits until the FIFO queue as almost full, and then reads all 32 bytes
 *
 * @param buf		the buffer where the data is written to
 */
void max30102_waitAndGetHeartrateSamples(uint8_t *buf);

#endif /* MAX30102_MAX30102_H_ */
