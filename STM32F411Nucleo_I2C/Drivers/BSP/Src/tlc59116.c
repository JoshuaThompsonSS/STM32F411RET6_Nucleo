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

void TLC59116_cmdWrite(unsigned char cmd, unsigned char data)
{
  unsigned char tx[2];
  tx[0] = cmd;
  tx[1] = data;
  if(HAL_I2C_Master_Transmit(&hi2c1, TLC59116BaseAddr, tx, 2, 0xFFFF) != HAL_OK){
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
int TLC59116_scan(void) {
  // this code lifted & adapted from Nick Gammon (written 20th April 2011)
  // http://www.gammon.com.au/forum/?id=10896&reply=6#reply6

  int device_ct = 0;
  for (byte addr = TLC59116BaseAddr; addr <= Max_Addr; addr++) {

    // yup, just "ping"
    int stat = TLC59116_i2c_scan();

    if (stat) {
      if (addr == AllCall_Addr) { continue; } //AllCall_Addr, skipped
      if (addr == Reset_Addr) { continue; } //Reset_Addr, skipped
		device_ct++;
      } // end of good response

    } // end of for loop

  return device_ct;
}
  
int TLC59116_init(void) {
	  int dev_count = TLC59116_scan();
	  if (dev_count){
		  return TLC59116_reset(); // does enable
	  }

	  return 0;
}

int TLC59116_reset(void){
	uint8_t data[2];
	data[0] = Reset_Bytes>>8;
	data[1] = Reset_Bytes & 0xFF;
	int length = 2;

	int rez = TLC59116_i2c_write(Reset_Addr, data, length);
	if (!rez)  {
		//TODO: handler failure to reset
		return rez;
	}

	//resets the saved register values to default
	TLC59116_reset_shadow_registers();

	//Reset worked so disable outputs
	int en = 0;
	int timeout = 0;
	TLC59116_enable_outputs(en, timeout);
	return rez;
}


void TLC59116_enable_outputs(int yes, int with_delay){
	if (yes) {
		TLC59116_modify_control_register_bits(MODE1_Register,MODE1_OSC_mask, 0x00); // bits off is osc on
		if (with_delay){} //TODO: add a delay?
	}
	else {
		TLC59116_modify_control_register_bits(MODE1_Register,MODE1_OSC_mask, MODE1_OSC_mask); // bits on is osc off
	}

	return ;
}


void TLC59116_modify_control_register(byte register_num, byte value) {
	if (shadow_registers[register_num] != value) {
		shadow_registers[register_num] = value;
		TLC59116_control_register(register_num, value);
	}
}

void TLC59116_modify_control_register_bits(byte register_num, byte mask, byte bits){
	byte new_value = TLC59116_set_with_mask(shadow_registers[register_num], mask, bits);

	if (register_num < PWM0_Register || register_num > 0x17) {
		//TODO: ?
	}
	TLC59116_modify_control_register(register_num, new_value);

	return ;
}

void TLC59116_control_register(byte register_num, byte data) {
	//TODO: make sure valid control register
	unsigned char buffer[1];
	buffer[0] = data;
	TLC59116_blockWrite(buffer, 1); //update control register setting

	return ;
}
void TLC59116_set(int led_num, int offon) {
	word bits = 1 << led_num;
	word pattern = offon ? bits : ~bits;
	TLC59116_set_outputs(pattern, bits);
	return;
}

void TLC59116_set_outputs(word pattern, word which){
	// Only change bits marked in which: to bits in pattern

	// We'll make the desired ledoutx register set
	byte new_ledx[4];
	// need initial value for later comparison
	memcpy(new_ledx, &(this->shadow_registers[LEDOUT0_Register]), 4);

	// count through LED nums, starting from max (backwards is easier)

	for(byte ledx_i=15; ; ledx_i--) {
		if (0x8000 & which) {
		  new_ledx[ledx_i / 4] = LEDx_set_mode(new_ledx[ledx_i / 4],ledx_i, (pattern & 0x8000) ? LEDOUT_DigitalOn : LEDOUT_DigitalOff);
		}
		pattern <<= 1;
		which <<= 1;

		if (ledx_i==0) break; // can't detect < 0 on an unsigned!
	}

	update_registers(new_ledx, LEDOUT0_Register, LEDOUTx_Register(15));
	return ;
}

void TLC59116_LEDx_set_mode(byte registers[], byte to_what, word which) {
	// count through LED nums, starting from max (backwards is easier)

	for(byte ledx_i=15; ; ledx_i--) {
		if (0x8000 & which) {
			registers[ledx_i / 4] = LEDx_set_mode(registers[ledx_i / 4], ledx_i, to_what);
		}
		which <<= 1;

		if (ledx_i==0) break; // can't detect < 0 on an unsigned!
	}
}

void TLC59116_update_registers(const byte want[], byte start_r, byte end_r) {
	// Update only the registers that need it
	// 'want' has to be (shadow_registers & new_value)
	// want[0] is register_value[start_r], i.e. a subset of the full register set

	// now find the changed ones
	//  best case is: no writes
	//  2nd best case is some subrange

	const byte *want_fullset = want - start_r; // now want[i] matches shadow_register[i]

	// First change...
	bool has_change_first = false;
	byte change_first_r; // a register num

	for (change_first_r = start_r; change_first_r <= end_r; change_first_r++) {
		if (want_fullset[change_first_r] != shadow_registers[change_first_r]) {
				has_change_first = true; // found it
				break;
		}
	}

	// Write the data if any changed

	if (!has_change_first) { // might be "none changed"
		//TODO: ?
	}
	else {
			// Find last change
			byte change_last_r; // a register num
			for (change_last_r = end_r; change_last_r >= change_first_r; change_last_r--) {
				if (want_fullset[change_last_r] != shadow_registers[change_last_r]) {
					break; // found it
				}
		}

		// We have a first..last, so write them
		_begin_trans(Auto_All, change_first_r);
		// TLC59116Warn("  ");
		i2cbus.write(&want_fullset[change_first_r], change_last_r-change_first_r+1);
		_end_trans();
		// update shadow
		memcpy(&shadow_registers[change_first_r], &want_fullset[change_first_r], change_last_r-change_first_r+1);
		// FIXME: propagate shadow
	}
}




void TLC59116_group_pwm(word bit_pattern, byte brightness) {
  return ;
}

void TLC59116_group_blink(word bit_pattern, int blink_delay, int on_ratio) {
  return ;
}

void TLC59116_update_registers(const byte want[], byte start_r, byte end_r) {
	return ;
}

void TLC59116_set_milliamps(byte ma, int Rext) {
  return ;
}


