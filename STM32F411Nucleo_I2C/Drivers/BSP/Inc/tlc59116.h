/* File: "TLC59116.h"
 * Desc: Header file for TI rgb led driver I2C Commands and Functions
*/
#ifndef TLC59116_h
#define TLC59116_h

/********************************Includes*********************************/
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include <stdlib.h>

/********************************Define***********************************/
#define TLC59116BaseAddr					(0x60<<1) // 0b110xxxx0 device address with write bit 0
#define TLC59116Channels					16
#define Control_Register_Max				(0x1E<<1) // NB, no 0x1F !
#define Control_Register_Min				0
/********************************Typedef**********************************/
typedef uint8_t byte;
typedef uint16_t word;


/********************************Constants********************************/

static const int Rext_Min = 156; // in ohms, gives 120ma at reset
static const byte TLC59116Reset_Addr = 		TLC59116BaseAddr + (0x0B<<1); // +0b1011
static const byte TLC59116Reset_Byte1 = 	0xA5;
static const byte TLC59116Reset_Byte2 = 	0x5A;
static const word TLC59116Reset_Bytes =  	0xA55A;

//Register Addresses (7 bits shifted to left by 1)
static const byte MODE1_Register = 			(0x00);
static const byte MODE2_Register = 			(0x01);
static const byte PWM0_Register = 			(0x02);
static const byte GRPPWM_Register = 		(0x12); // aka blink-duty-cycle-register
static const byte GRPFREQ_Register = 		(0x13); // aka blink-length 0=~24hz, 255=~10sec
static const byte LEDOUT0_Register = 		(0x14);
static const byte SUBADR1_Register = 		(0x18);
static const byte AllCall_Addr_Register = 	(0x1B);
static const byte IREF_Register = 			(0x1C);
static const byte EFLAG1_Register = 		(0x1D);
static const byte EFLAG2_Register = 		(0x1E);


//Address
static const byte AllCall_Addr = 			TLC59116BaseAddr + (0x08<<1); // +0b1000 Programmable Enabled at on/reset
static const byte SUBADR1 = 				TLC59116BaseAddr + (0x09<<1); // +0b1001 Programmable Disabled at on/reset
static const byte SUBADR2 = 				TLC59116BaseAddr + (0x0A<<1); // +0b1010 Programmable Disabled at on/reset
static const byte SUBADR3 = 				TLC59116BaseAddr + (0x0C<<1); // +0b1100 Programmable Disabled at on/reset + 0x0D .. 0x0F
// The byte(s) sent for Reset_Addr are not increment+address, it's a special mode
static const byte Max_Addr = 				TLC59116BaseAddr + (0xF<<1);   // 14 typically available
  

// Auto-increment mode bits
// default is Auto_none, same register can be written to multiple times
static const byte Auto_All = 				0b10000000; // 0..Control_Register_Max
static const byte Auto_PWM = 				0b10100000; // PWM0_Register .. (Channels-1)
static const byte Auto_GRP = 				0b11000000; // GRPPWM..GRPFREQ
static const byte Auto_PWM_GRP = 			0b11100000; // PWM0_Register..n, GRPPWM..GRPFREQ

//Masks
static const byte IREF_CM_mask = 			(1<<7);
static const byte IREF_HC_mask = 			1<<6;
static const byte IREF_CC_mask = 			0b111111; // 6 bits
static const byte MODE1_OSC_mask = 			0b10000;
static const byte MODE1_SUB1_mask = 		0b1000;
static const byte MODE1_SUB2_mask = 		0b100;
static const byte MODE1_SUB3_mask = 		0b10;
static const byte MODE1_ALLCALL_mask = 		0b1;

//Mode setting bits
static const byte MODE2_DMBLNK = 			1 << 5; // 0 = group dimming, 1 = group blinking
static const byte MODE2_EFCLR = 			1 << 7; // 0 to enable, 1 to clear
static const byte MODE2_OCH = 				1 << 3; // 0 = "latch" on Stop, 1 = latch on ACK
static const byte LEDOUTx_Max = 			TLC59116Channels-1; // 0..15
const static byte LEDOUT_Mask = 			0b11; // 2 bits per led
const static byte LEDOUT_PWM = 				0b10;
const static byte LEDOUT_PWM_ALL = 			0b10101010; //all LED in led bank pwm controlled
const static byte LEDOUT_GRPPWM = 			0b11; // also "group blink" when mode2[dmblnk] = 1
const static byte LEDOUT_DigitalOn = 		0b01;
const static byte LEDOUT_DigitalOff = 		0b00;


unsigned char TxData[I2C_BUFFERSIZE];           // Stores data bytes to be TX'd
unsigned char RxData[I2C_BUFFERSIZE];           // Stores data bytes that are RX'd

// Typedefs

typedef struct tlc59116_register_t {
	byte address;
	byte value;
};

typedef struct tlc59116_register_controller_t {
	byte initialized;
	byte device_address;
	byte channels;
	byte reset_address;
	byte reset_byte[2];
	tlc59116_register_t mode[2];
	tlc59116_register_t pwm[16];
	tlc59116_register_t group_pwm;
	tlc59116_register_t group_freq;
	tlc59116_register_t ledout[4];
	tlc59116_register_t subaddr[3];
	tlc59116_register_t all_call;
	tlc59116_register_t iref;
	tlc59116_register_t eflag[2];
};


// I2C Function Declarations
int TLC59116_i2c_write(unsigned char addr, unsigned char *buffer, int length);
int TLC59116_i2c_scan(void);
void TLC59116_read(unsigned char cmd, unsigned int bytes);
int TLC59116_blockWrite(unsigned char *buffer, unsigned int length);
void TLC59116_error(void);

// Generic functions
void TLC59116_init_register_controller(void);
void TLC59116_reset_shadow_registers(void);
byte TLC59116_set_with_mask(byte was, byte mask, byte new_bits);

// Control Function below based on TLC59116 Datasheet standard data commands for TLC59116
int TLC59116_scan(void);
int TLC59116_init(void);
int TLC59116_reset(void);
void TLC59116_enable_outputs(int yes, int with_delay);
void TLC59116_enable_pwm_outputs(void);
void TLC59116_modify_register(byte register_num, byte value);
void TLC59116_modify_register_bits(byte register_num, byte mask, byte bits);
void TLC59116_set_register(byte register_num, byte data);
void TLC59116_set_output(int led_num, int offon, byte pwm);
void TLC59116_set_outputs(byte ch_states[], byte ch_pwm[]);
void TLC59116_LEDx_set_mode(byte registers[], byte to_what, word which);
void TLC59116_group_pwm(word bit_pattern, byte brightness);
void TLC59116_group_blink(word bit_pattern, int blink_delay, int on_ratio);
void TLC59116_update_registers(const byte want[], byte start_r, byte end_r);
void TLC59116_set_milliamps(byte ma, int Rext);
void TLC59116_setPWM(int duty_cycle_percent, int channel);

#endif //TLC59116_h







