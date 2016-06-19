/*
 * bq27542.c
 * Desc: TI Fuel Gauge driver code
 */

//Includes
#include "bq27542.h"


unsigned int transBytes2UnsignedInt(unsigned char msb, unsigned char lsb)
{
  unsigned int tmp;

  tmp = ((msb << 8) & 0xFF00);
  return ((unsigned int)(tmp + lsb) & 0x0000FFFF);
}

void BQ27542_read(unsigned char cmd, unsigned int bytes)
{
  unsigned char tx[1];

  tx[0] = cmd;
  HAL_I2C_Master_Transmit(&hi2c1, I2CSLAVEADDR, tx, 1, 0xFFFF);
  HAL_I2C_Master_Receive(&hi2c1, I2CSLAVEADDR, RxData, bytes, 0xFFFF);

}

void BQ27542_cmdWrite(unsigned char cmd, unsigned char data)
{
  unsigned char tx[2];
  tx[0] = cmd;
  tx[1] = data;
  HAL_I2C_Master_Transmit(&hi2c1, I2CSLAVEADDR, tx, 2, 0xFFFF);

}

void BQ27542_blockWrite(unsigned char *buffer, unsigned int length)
{
  HAL_I2C_Master_Transmit(&hi2c1, I2CSLAVEADDR, buffer, length, 0xFFFF);
}

void BQ27542_error(void)
{

}

uint16_t BQ27542_getTemperature(void){
	// Read Temperature (units = 0.1K)
	BQ27542_read(bq27542CMD_TEMP_LSB, 2);
	temperature = transBytes2UnsignedInt(RxData[1], RxData[0]);
	return temperature;
}

uint16_t BQ27542_getDeviceType(void){
	// Read Device Type
	TxData[0] = bq27542CMD_CNTL_LSB;
	TxData[1] = bq27542CMD_CNTL_DTYPE_LSB;
	TxData[2] = bq27542CMD_CNTL_DTYPE_MSB;
	BQ27542_blockWrite(TxData, 3);
	//wait_sec(0.01);
	BQ27542_read(bq27542CMD_CNTL_LSB, 2);
	uint16_t devType = transBytes2UnsignedInt(RxData[1], RxData[0]);
	return devType;
}

