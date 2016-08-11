/* File: "TLC59116.h"
 * Desc: Header file for TI rgb led driver I2C Commands and Functions
*/

//Includes
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include <stdlib.h>

//Defines
/* Registers */
#define TLC59116_REG_GPIO_A				0x00
#define TLC59116_REG_GPIO_B				0x01
#define TLC59116_REG_PRESCALE0			0x02
#define TLC59116_REG_PWM0				0x03
#define TLC59116_REG_PRESCALE1			0x04
#define TLC59116_REG_PWM1				0x05
#define TLC59116_REG_MUX0				0x06
#define TLC59116_REG_MUX1				0x07
#define TLC59116_REG_MUX2				0x08
#define TLC59116_REG_MUX3				0x09

/* Bit description for TLC59116_REG_MUX0 ~ 3 */
#define TLC59116_GPIO_IN				0x00
#define TLC59116_GPIO_OUT_HIGH			0x00
#define TLC59116_GPIO_OUT_LOW			0x01
#define TLC59116_DIM_PWM0				0x02
#define TLC59116_DIM_PWM1				0x03

#define TLC59116_NUM_PWMS				2

//Types
enum TLC59116_pwm_output {
	TLC59116_PWM_OUT0,
	TLC59116_PWM_OUT1,
	TLC59116_PWM_OUT2,
	TLC59116_PWM_OUT3,
	TLC59116_PWM_OUT4,
	TLC59116_PWM_OUT5,
	TLC59116_PWM_OUT6,
	TLC59116_PWM_OUT7,
	TLC59116_PWM_OUT8,
	TLC59116_PWM_OUT9,
	TLC59116_PWM_OUT10,
	TLC59116_PWM_OUT11,
	TLC59116_PWM_OUT12,
	TLC59116_PWM_OUT13,
	TLC59116_PWM_OUT14,
	TLC59116_PWM_OUT15,
};


unsigned char TxData[BUFFERSIZE];           // Stores data bytes to be TX'd
unsigned char RxData[BUFFERSIZE];           // Stores data bytes that are RX'd


//Function Declarations
unsigned int transBytes2UnsignedInt(unsigned char msb, unsigned char lsb);
void TLC59116_read(unsigned char cmd, unsigned int bytes);
void TLC59116_cmdWrite(unsigned char cmd, unsigned char data);
void TLC59116_blockWrite(unsigned char *buffer, unsigned int length);
void TLC59116_error(void);

/*
 * Function below based on TLC59116 Datasheet standard data commands for TLC59116
 */
void TLC59116_init(void);
void TLC59116_reset(void);
void TLC59116_enable_outputs(bool yes, bool with_delay);
void TLC59116_modify_control_register(byte register_num, byte value);
void TLC59116_modify_control_register_bits(byte register_num, byte mask, byte bits);
void TLC59116_set_outputs(word pattern, word which);
void TLC59116_group_pwm(word bit_pattern, byte brightness);
void TLC59116_group_blink(word bit_pattern, int blink_delay, int on_ratio);
void TLC59116_update_registers(const byte want[], byte start_r, byte end_r);
void TLC59116_set_milliamps(byte ma, int Rext);
void TLC59116_setPWM(int duty_cycle_percent, int channel);






