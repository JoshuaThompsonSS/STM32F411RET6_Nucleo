/*
 * TLC59116.c
 * Desc: TI 16 channel pwm driver code
 */

//Includes
#include "TLC59116.h"


unsigned int transBytes2UnsignedInt(unsigned char msb, unsigned char lsb)
{
  unsigned int tmp;

  tmp = ((msb << 8) & 0xFF00);
  return ((unsigned int)(tmp + lsb) & 0x0000FFFF);
}

void TLC59116_read(unsigned char cmd, unsigned int bytes)
{
  unsigned char tx[1];

  tx[0] = cmd;
  if(HAL_I2C_Master_Transmit(&hi2c1, I2CSLAVEADDR, tx, 1, 0xFFFF) != HAL_OK){
	  TLC59116_error();
  }
  if(HAL_I2C_Master_Receive(&hi2c1, I2CSLAVEADDR, RxData, bytes, 0xFFFF) != HAL_OK){
	  TLC59116_error();
  }

}

void TLC59116_cmdWrite(unsigned char cmd, unsigned char data)
{
  unsigned char tx[2];
  tx[0] = cmd;
  tx[1] = data;
  if(HAL_I2C_Master_Transmit(&hi2c1, I2CSLAVEADDR, tx, 2, 0xFFFF) != HAL_OK){
	  TLC59116_error();
  }

}

void TLC59116_blockWrite(unsigned char *buffer, unsigned int length)
{
  if(HAL_I2C_Master_Transmit(&hi2c1, I2CSLAVEADDR, buffer, length, 0xFFFF) != HAL_OK){
	  TLC59116_error();
  }
}

void TLC59116_error(void)
{
	int error = 1;
}

void TLC59116_setPWM(int duty_cycle_percent, int channel){
	//TODO: ?
}

/************************TLC59116 Control Functions *******************************/
void TLC59116_init() {
  return ;
}


int TLC59116Manager::reset() {
  return 0;
}

void TLC59116_enable_outputs(bool yes, bool with_delay) {
  return ;
}

void TLC59116_modify_control_register(byte register_num, byte value) {
	return ;
}

void TLC59116_modify_control_register(byte register_num, byte mask, byte bits) {
  return ;
}

void TLC59116_set_outputs(word pattern, word which) {
  // Only change bits marked in which: to bits in pattern
  return ;
}

void TLC59116_set_outputs(byte led_num_start, byte led_num_end, const byte brightness[]) {
  return ;
}

void TLC59116_group_pwm(word bit_pattern, byte brightness) {
  return 
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


