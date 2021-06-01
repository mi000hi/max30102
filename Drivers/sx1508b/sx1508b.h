/*
 * sx1508b.h
 *
 *  Created on: May 4, 2021
 *      Author: michael
 */

#ifndef SX1508B_SX1508B_H_
#define SX1508B_SX1508B_H_

#include <stdint.h>		//may not be ideal depending on linking of implementation, maybe add as requirement?

/* Read and write */

#define SX1508B_ADDRESS_BASE 			0b01000	/**< device address without ADDR[1:0] **/

/*! @} */

/*! \addtogroup lps22hb_registers
 *  LPS22HB registers
 *  @{
 */

/*********** REGISTERS ***********/

#define SX1508B_REG_INPUT_DISABLE		0x00	/**< input buffer disable register **/
#define SX1508B_REG_LONG_SLEW			0x01	/**< output buffer long slew register **/
#define SX1508B_REG_LOW_DRIVE			0x02	/**< output buffer low drive register **/

#define SX1508B_REG_PULL_UP				0x03	/**< pull up register **/
#define SX1508B_REG_PULL_DOWN			0x04	/**< pull down register **/

#define SX1508B_REG_OPEN_DRAIN			0x05	/**< open drain register **/
#define SX1508B_REG_POLARITY			0x06	/**< polarity register **/

#define SX1508B_REG_DIR					0x07	/**< direction register **/
#define SX1508B_REG_DATA				0x08	/**< data register **/
#define SX1508B_REG_INTERRUPT_MASK		0x09	/**< interrupt mask register **/

#define SX1508B_REG_SENSE_HIGH			0x0A	/**< sense register for i/o[7:4] **/
#define SX1508B_REG_SENSE_LOW			0x0B	/**< sense register for i/o[3:0] **/
#define SX1508B_REG_INTERRUPT_SOURCE	0x0C	/**< interrupt source register **/
#define SX1508B_REG_EVENT_STATUS		0x0D	/**< event status register **/
#define SX1508B_REG_LEVEL_SHIFTER		0x0E	/**< level shifter register **/
#define SX1508B_REG_CLOCK				0x0F	/**< clock management register **/
#define SX1508B_REG_MISC				0x10	/**< miscellaneous device settings register **/
#define SX1508B_REG_LED_DRIVER_ENABLE	0x11	/**< led driver enable register **/

#define SX1508B_REG_DEBOUNCE_CONFIG		0x12	/**< debounce configuration register **/
#define SX1508B_REG_DEBOUNCE_ENABLE		0x13	/**< debounce enable register **/

#define SX1508B_REG_KEY_CONFIG			0x14	/**< key scan configuration register **/
#define SX1508B_REG_KEY_DATA			0x15	/**< key value **/

#define SX1508B_REG_ION0				0x16	/**< ON intensity register for i/o[0] **/
#define SX1508B_REG_ION1				0x17	/**< ON intensity register for i/o[1] **/
#define SX1508B_REG_TON2				0x18	/**< ON time register for i/o[2] **/
#define SX1508B_REG_ION2				0x19	/**< ON intensity register for i/o[2] **/
#define SX1508B_REG_OFF2				0x1A	/**< OFF time/intensity register for i/o[2] **/
#define SX1508B_REG_TON3				0x1B	/**< ON time register for i/o[3] **/
#define SX1508B_REG_ION3				0x1C	/**< ON intensity register for i/o[3] **/
#define SX1508B_REG_OFF3				0x1D	/**< OFF time/intensity register for i/o[3] **/
#define SX1508B_REG_TRISE3				0x1E	/**< fade in register for i/o[3] **/
#define SX1508B_REG_TFALL3				0x1F	/**< fade out register for i/o[3] **/

#define SX1508B_REG_ION4				0x20	/**< ON intensity register for i/o[4] **/
#define SX1508B_REG_ION5				0x21	/**< ON intensity register for i/o[5] **/
#define SX1508B_REG_TON6				0x22	/**< ON time register for i/o[6] **/
#define SX1508B_REG_ION6				0x23	/**< ON intensity register for i/o[6] **/
#define SX1508B_REG_OFF6				0x24	/**< OFF time/intensity register for i/o[6] **/
#define SX1508B_REG_TON7				0x25	/**< OFF time register for i/o[7] **/
#define SX1508B_REG_ION7				0x26	/**< ON intensity register for i/o[7] **/
#define SX1508B_REG_OFF7				0x27	/**< OFF time/intensity register for i/o[7] **/
#define SX1508B_REG_TRISE7				0x28	/**< fade in register for i/o[7] **/
#define SX1508B_REG_TFALL7				0x29	/**< fade out register for i/o[7] **/

#define SX1508B_REG_HIGH_INPUT			0x2A	/**< high input enable register **/

#define SX1508B_REG_RESET				0x7D	/**< software reset register **/

#define SX1508B_REG_TEST1				0x7E	/**< test register **/
#define SX1508B_REG_TEST2				0x7F	/**< test register **/

/*********** STRUCTS ***********/

typedef struct {
	void (*write)(uint8_t reg, uint8_t *buf, uint8_t bytes); /**< User write function **/
	void (*read)(uint8_t reg, uint8_t *buf, uint8_t bytes); /**< User read function **/
} sx1508b_communication_t;

typedef struct {
	sx1508b_communication_t comm;
//  sx1508b_configuration_t config;
} sx1508b_t;

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
void sx1508b_setup_communication(sx1508b_t *configs);

/**
 * activates the internal oscillator (2MHz base clock) and puts fOSCOUT=clock/(2^(divider-1)) onto its output pin
 *
 * @param divider		4 bit value that divides the base clock of 2MHz to define fOSCOUT
 */
void sx1508b_configure_clock(uint8_t divider);

/**
 * configures the PWM clock frequency using fPWM=clock/(2^(divider-1))
 *
 * @param divider		3 bit value that divides the base clock of 2MHz to define fPWM
 */
void sx1508b_configure_pwm(uint8_t divider);

/**
 * configures the specified pin as a PWM pin. pull up or pull down resistors can be selected
 *
 * @param pinNumber		the number of the pin in 0:7 that is used as a PWM pin
 * @param pullUp		0: no pullup resistor, 1: use pullup resistor
 * @param pullDown		0: nu pulldown resistor, 1: use pulldown resistor
 */
void sx1508b_configure_pwm_pin(uint8_t pinNumber, uint8_t pullUp,
		uint8_t pullDown);

/**
 * sets the PWM intensity given as a float in [0,1] to the specified pin.
 * the intensity is dependent on a configured pull up or pull down resistor
 *
 * @param pinNumber		the number of the pin in 0:7 whose intensity should be changed
 * @param intensity		floating point number in [0,1] corresponding to the duty cycle of the PWM
 */
void sx1508b_set_pwm_intensity(uint8_t pinNumber, float intensity);

/**
 * sets the value of one pin in 0:7
 *
 * @param pinNumber		the number of the pin to update its value
 * @param value			the value to write onto the specified pin
 */
void sx1508b_set_pin(uint8_t pinNumber, uint8_t value);

/**
 * reads the value of the specified pin and writes it into the given data buffer
 *
 * @param pinNumber		number of the pin, whose value is read out
 * @param data			buffer where the value of the pin is stored into
 */
void sx1508b_read_pin(uint8_t pinNumber, uint8_t *data);

/**
 * toggles the value of one pin in 0:7 between the output value 0 and 1
 *
 * @param pinNumber		the number of the pin to toggle its value
 */
void sx1508b_toggle_pin(uint8_t pinNumber) ;

/**
 * defines the direction of the specified pin
 * 0: pin is an output pin
 * 1: pin is an intput pin
 *
 * @param pinNumber		the number of the pin to update its direction
 * @param value			the direction, 0: output, 1: input
 */
void sx1508b_set_pin_direction(uint8_t pinNumber, uint8_t value);

#endif /* SX1508B_SX1508B_H_ */
