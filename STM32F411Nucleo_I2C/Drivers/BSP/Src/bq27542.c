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

uint16_t BQ27542_getUnfilteredSOC(void){
	//units: %
	int soc;

	//TODO: Josh - develop code
	return soc;
}

uint16_t BQ27542_getVoltage(void){
	//units: mV
	int mV;

	//TODO: Josh - develop code
	return mV;
}

uint16_t BQ27542_getNomAvailableCapacity(void){
	//units: mAh
	int nomCap;

	//TODO: Josh - develop code
	return nomCap;
}

uint16_t BQ27542_getFullAvailableCapacity(void){
	//units: mAh
	int fullCap;
	//TODO: Josh - develop code

	return fullCap;
}

uint16_t BQ27542_getRemainingCapacity(void){
	//units: mAh
	int remCap;

	//TODO: Josh - develop code

	return remCap;
}

uint16_t BQ27542_getFullChargeCapacity(void){
	//units: mAh
	int fullChrgCap;

	//TODO: Josh - develop code

	return fullChrgCap;
}

uint16_t BQ27542_getAverageCurrent(void){
	//units: mA
	int aveCurrent;

	//TODO: Josh - develop code

	return aveCurrent;
}

uint16_t BQ27542_getTimeToEmpty(void){
	//units: min
	int timeToEmpty;

	//TODO: Josh - develop code
	return timeToEmpty;
}

uint16_t BQ27542_getFullChargeCapacityFiltered(void){
	//units: mAh
	int fullChrgCapFilt;

	//TODO: Josh - develop code

	return fullChrgCapFilt;
}

uint16_t BQ27542_getFullChargeCapacityUnfiltered(void){
	//units: mAh
	int fullChrgCapUnfilt;

	//TODO: Josh - develop code
	return fullChrgCapUnfilt;
}

uint16_t BQ27542_getImax(void){
	//units: mAh
	int imax;

	//TODO: Josh - develop code
	return imax;
}

uint16_t BQ27542_getRemainingCapacityUnfiltered(void){
	//units: mAh
	int remCapUnfilt;

	//TODO: Josh - develop code

	return remCapUnfilt;
}

uint16_t BQ27542_getRemainingCapacityFiltered(void){
	//units: mAh
	int remCapFilt;

	//TODO: Josh - develop code

	return remCapFilt;
}

uint16_t BQ27542_setBTPSOC1(int mAh){
	//units: mAh
	int btpsoc1;

	//TODO: Josh - develop code
	return btpsoc1;
}

uint16_t BQ27542_clearBTPSOC1(void){
	//units: mAh
	int btpsoc1;

	//TODO: Josh - develop  code
	return btpsoc1;
}
uint16_t BQ27542_getInternalTemperature(void){
	//units: 0.1 deg K
	int intTemp;
	//TODO: Josh - develop  code
	return intTemp;
}

uint16_t BQ27542_getCycleCount(void){
	//units: count
	int cycleCnt;
	//TODO: Josh - develop  code

	return cycleCnt;
}

uint16_t BQ27542_getStateOfCharge(void){
	//units: %
	int soc;
	//TODO: Josh - develop  code

	return soc;
}

uint16_t BQ27542_getStateOfHealth(void){
	//units % / num
	int soh;

	//TODO: Josh - develop  code
	return soh;
}

uint16_t BQ27542_getChargingVoltage(void){
	//units: mV
	int chrgVolt;
	//TODO: Josh - develop  code

	return chrgVolt;
}

uint16_t BQ27542_getChargingCurrent(void){
	//units: mA
	int chrgCurrent;

	//TODO: Josh - develop  code
	return chrgCurrent;
}

uint16_t BQ27542_getPassedCharge(void){
	//units: mAh
	int passedChrg;
	//TODO: Josh - develop  code

	return passedChrg;
}

uint16_t BQ27542_getDOD0(void){
	//units: hex
	int dod0;
	//TODO: Josh - develop  code

	return dod0;
}

uint16_t BQ27542_getSelfDischargeCurrent(void){
	//units: mA
	int disCurrent;

	//TODO: Josh - develop  code
	return disCurrent;
}

