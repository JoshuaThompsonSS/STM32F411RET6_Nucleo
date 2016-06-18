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


