/*
 * TLC59116.c
 * Desc: TI 16 channel pwm driver code
 */

/*******************************Includes*********************************************/
#include "tlc59116.h"
#include <string.h>

/*******************************Variables********************************************/
uint8_t Power_Up_Register_Values[Control_Register_Max+1];
uint8_t shadow_registers[Control_Register_Max+1];

tlc59116_register_controller_t tlcHandler;


/*******************************Function Definitions*********************************/

int TLC59116_i2c_write(unsigned char addr, unsigned char *buffer, int length){
	//write to any addr of i2c
	 if(HAL_I2C_Master_Transmit(&hi2c1, addr, buffer, length, 0xFFFF) != HAL_OK){
		  TLC59116_error();
		  return 0;
	  }
	  else{
		  return 1;
	  }

}
int TLC59116_i2c_scan(void){
	  //Don't send any data just see if dev responds when you request a write
	  if(HAL_I2C_Master_Transmit(&hi2c1, TLC59116BaseAddr, NULL, 0, 0xFFFF) != HAL_OK){
		  return 0;
	  }
	  else{
		  return 1;
	  }
}

void TLC59116_read(unsigned char cmd, unsigned int bytes)
{
  unsigned char tx[1];

  tx[0] = cmd;
  if(HAL_I2C_Master_Transmit(&hi2c1, TLC59116BaseAddr, tx, 1, 0xFFFF) != HAL_OK){
	  TLC59116_error();
  }
  if(HAL_I2C_Master_Receive(&hi2c1, TLC59116BaseAddr, RxData, bytes, 0xFFFF) != HAL_OK){
	  TLC59116_error();
  }

}


int TLC59116_blockWrite(unsigned char *buffer, unsigned int length)
{
  if(HAL_I2C_Master_Transmit(&hi2c1, TLC59116BaseAddr, buffer, length, 0xFFFF) != HAL_OK){
	  TLC59116_error();
	  return 0;
  }
  else{
	  return 1;
  }
}

void TLC59116_error(void)
{
	//TODO: ?
}

void TLC59116_setPWM(int duty_cycle_percent, int channel){
	//TODO: ?
}

/************************ TLC59116 Generic Functions ******************************/
void TLC59116_init_register_controller(void){
	tlcHandler.initialized = 1;
	tlcHandler.error = NOERROR;
	//Slave Device I2C Address and Reset
	tlcHandler.device_address = TLC59116BaseAddr;
	tlcHandler.reset_address = TLC59116Reset_Addr;
	tlcHandler.reset_byte[0] = TLC59116Reset_Byte1;
	tlcHandler.reset_byte[1] = TLC59116Reset_Byte2;

	//MODE Registers
	tlcHandler.mode[0].address = MODE1_Register;
	tlcHandler.mode[1].address = MODE2_Register;

	//PWM Registers
	tlcHandler.channels = Channels;
	tlcHandler.pwm[0].address = PWM0_Register;
	tlcHandler.pwm[0].value = 0x00;
	tlcHandler.pwm[1].address = PWM0_Register + 1;
	tlcHandler.pwm[1].value = 0x00;
	tlcHandler.pwm[2].address = PWM0_Register + 2;
	tlcHandler.pwm[2].value = 0x00;
	tlcHandler.pwm[3].address = PWM0_Register + 3;
	tlcHandler.pwm[3].value = 0x00;
	tlcHandler.pwm[4].address = PWM0_Register + 4;
	tlcHandler.pwm[4].value = 0x00;
	tlcHandler.pwm[5].address = PWM0_Register + 5;
	tlcHandler.pwm[5].value = 0x00;
	tlcHandler.pwm[6].address = PWM0_Register + 6;
	tlcHandler.pwm[6].value = 0x00;
	tlcHandler.pwm[7].address = PWM0_Register + 7;
	tlcHandler.pwm[7].value = 0x00;
	tlcHandler.pwm[8].address = PWM0_Register + 8;
	tlcHandler.pwm[8].value = 0x00;
	tlcHandler.pwm[9].address = PWM0_Register + 9;
	tlcHandler.pwm[9].value = 0x00;
	tlcHandler.pwm[10].address = PWM0_Register + 10;
	tlcHandler.pwm[10].value = 0x00;
	tlcHandler.pwm[11].address = PWM0_Register + 11;
	tlcHandler.pwm[11].value = 0x00;
	tlcHandler.pwm[12].address = PWM0_Register + 12;
	tlcHandler.pwm[12].value = 0x00;
	tlcHandler.pwm[13].address = PWM0_Register + 13;
	tlcHandler.pwm[13].value = 0x00;
	tlcHandler.pwm[14].address = PWM0_Register + 14;
	tlcHandler.pwm[14].value = 0x00;
	tlcHandler.pwm[15].address = PWM0_Register + 15;
	tlcHandler.pwm[15].value = 0x00;

	//Group PWM and Frequency Registers
	tlcHandler.group_pwm.address = GRPPWM_Register;
	tlcHandler.group_pwm.value = 0x00;
	tlcHandler.group_freq.address = GRPFREQ_Register;
	tlcHandler.group_freq.value = 0x00;

	//LED Bank Registers
	tlcHandler.ledout[0].address = LEDOUT0_Register;
	tlcHandler.ledout[0].value = 0x00;
	tlcHandler.ledout[1].address = LEDOUT0_Register + 1;
	tlcHandler.ledout[1].value = 0x00;
	tlcHandler.ledout[2].address = LEDOUT0_Register + 2;
	tlcHandler.ledout[2].value = 0x00;
	tlcHandler.ledout[3].address = LEDOUT0_Register + 3;
	tlcHandler.ledout[3].value = 0x00;

	//Subaddress Registers
	tlcHandler.subaddr[0].address = SUBADR1_Register;
	tlcHandler.subaddr[0].value = 0x00;
	tlcHandler.subaddr[1].address = SUBADR2_Register;
	tlcHandler.subaddr[1].value = 0x00;
	tlcHandler.subaddr[2].address = SUBADR3_Register;
	tlcHandler.subaddr[2].value = 0x00;

	//Other Registers
	tlcHandler.all_call.address = AllCall_Register;
	tlcHandler.all_call.value = 0x00;
	tlcHandler.iref.address = IREF_Register;
	tlcHandler.iref.value = 0x00;
	tlcHandler.eflag[0].address = EFLAG1_Register;
	tlcHandler.eflag[0].value = 0x00;
	tlcHandler.eflag[1].address = EFLAG2_Register;
	tlcHandler.eflag[1].value = 0x00;



}

void TLC59116_reset_shadow_registers(void) {
	//resets saved register values to their defaults
    memcpy(shadow_registers, Power_Up_Register_Values, Control_Register_Max);
}

byte TLC59116_set_with_mask(byte was, byte mask, byte new_bits) { // only set the bits marked by mask
      byte to_change = mask & new_bits; // sanity
      byte unchanged = ~mask & was; // the bits of interest are 0 here
      byte new_value = unchanged | to_change;
      return new_value;
}


/************************TLC59116 Control Functions *******************************/
/*
 * Note: scans for a valid TLC59116 i2c device
 */
int TLC59116_scan(void) {
  // this code lifted & adapted from Nick Gammon (written 20th April 2011)
  // http://www.gammon.com.au/forum/?id=10896&reply=6#reply6

  int device_ct = 0;
  byte base_addr = tlcHandler.device_address;
  for (byte addr = base_addr; addr <= Max_Addr; addr++) {

    // yup, just "ping"
    int stat = TLC59116_i2c_scan();

    if (stat) {
      if (addr == tlcHandler.all_call.address) { continue; } //AllCall_Addr, skipped
      if (addr == tlcHandler.reset_address) { continue; } //Reset_Addr, skipped
		device_ct++;
      } // end of good response

    } // end of for loop

  return device_ct;
}

/*
 * Note: scans for a valid TLC59116 i2c device and if found resets the device / enables led pwm outputs
 */
int TLC59116_init(void) {
	TLC59116_init_register_controller();
	  int dev_count = TLC59116_scan();
	  if (dev_count){
		  return TLC59116_reset(); // does enable
	  }
	  else{
		  //TODO: handler no device found error
		  tlcHandler.error = NODEVICE;
	  }

	  return 0;
}

/*
 * Note: resets the tlcHandler structure, sends reset command to device, and enables led pwm outputs
 */
int TLC59116_reset(void){
	uint8_t data[2];

	//resets the saved register values to default
	TLC59116_init_register_controller();

	data[0] = tlcHandler.reset_byte[0];
	data[1] = tlcHandler.reset_byte[1];
	byte reset_addr = tlcHandler.reset_address;
	int length = 2;

	int rez = TLC59116_i2c_write(reset_addr, data, length);
	if (!rez)  {
		//TODO: handler failure to reset
		return rez;
	}

	//Reset worked so disable outputs
	int en = 1;
	int timeout = 0;
	TLC59116_enable_outputs(en, timeout);
	TLC59116_enable_pwm_outputs();
	return rez;
}

/*
 * Note: turn on or off the oscillators that control the led output
 */
void TLC59116_enable_outputs(int yes, int with_delay){
	if (yes) {
		TLC59116_modify_register_bits(&tlcHandler.mode[0], MODE1_OSC_mask, 0x00); // bits off is osc on
		if (with_delay){} //TODO: add a delay?
	}
	else {
		TLC59116_modify_register_bits(&tlcHandler.mode[0], MODE1_OSC_mask, MODE1_OSC_mask); // bits on is osc off
	}
	TLC59116_set_register(&tlcHandler.mode[1]); //send via i2c

	return ;
}

/*
 * Note: Enables leds to be controlled by pwm registers
 */
void TLC59116_enable_pwm_outputs(void){
	for(int i = 0; i<4; i++){
		TLC59116_modify_register(&tlcHandler.ledout[i], LEDOUT_PWM_ALL); //update register handler
		TLC59116_set_register(&tlcHandler.ledout[i]); //send to device via i2c
	}
}

/*
 * Note: Set led output to pwm value
 */
void TLC59116_set_output(tlc59116_register_t * ledPwmReg, byte pwm){
	TLC59116_modify_register(ledPwmReg, pwm);
	TLC59116_set_register(ledPwmReg);
}

/*
 * Note: set all led outputs at once based on pwm value array (16 values)
 */
void TLC59116_set_outputs(byte pwm_values[]){
	for(int i = 0; i<tlcHandler.channels; i++){
		TLC59116_modify_register(&tlcHandler.pwm[i], pwm_values[i]); //this just updates the pwm handler values
	}
	TLC59116_set_pwm_registers(); //after modifying pwm handler values structure send to device via i2c
	return ;
}

/*
 * Note: turn on group pwm control
 */
void TLC59116_group_pwm(word bit_pattern, byte brightness) {
  return ;
}

/*
 * turn on group blink
 */
void TLC59116_group_blink(word bit_pattern, int blink_delay, int on_ratio) {
  return ;
}

/*
 * Note: configure the constant current output setting
 */
void TLC59116_set_milliamps(byte ma, int Rext) {
  return ;
}


/*
 * Updates the tlcHandler structure register value but does not send to device
 */
void TLC59116_modify_register(tlc59116_register_t * reg, byte value) {
	if (reg->value != value) {
		reg->value = value;
	}
}

void TLC59116_modify_register_bits(tlc59116_register_t * reg, byte mask, byte bits){
	byte new_value = TLC59116_set_with_mask(reg->value, mask, bits);

	if (reg->address < PWM0_Register || reg->address > 0x17) {
		//TODO: ?
	}
	TLC59116_modify_register(reg, new_value);

	return ;
}

/*
 * Note: sends the values stored in the tlcHandler structure register to the device via i2c
 */
void TLC59116_set_register(tlc59116_register_t * reg) {
	//TODO: make sure valid control register
	unsigned char buffer[2];
	buffer[0] = reg->address;
	buffer[1] = reg->value;
	TLC59116_blockWrite(buffer, 2); //update control register setting
	return ;
}

/*
 * Note: sends all the values stored in the tlcHandler structure pwm registers to the device via i2c
 */
void TLC59116_set_pwm_registers(void){
	byte buffer[Channels+1];
	buffer[0] = tlcHandler.pwm[0].address | Auto_PWM; //tell device to auto write increment pwm register only
	for(int i = 1; i<=Channels; i++){
		buffer[i] = tlcHandler.pwm[i].value;
	}
	TLC59116_blockWrite(buffer, Channels+1); //update control register setting
}




