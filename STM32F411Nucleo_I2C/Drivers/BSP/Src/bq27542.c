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
  if(HAL_I2C_Master_Transmit(&hi2c1, I2CSLAVEADDR, tx, 1, 0xFFFF) != HAL_OK){
	  BQ27542_error();
  }
  if(HAL_I2C_Master_Receive(&hi2c1, I2CSLAVEADDR, RxData, bytes, 0xFFFF) != HAL_OK){
	  BQ27542_error();
  }

}

void BQ27542_cmdWrite(unsigned char cmd, unsigned char data)
{
  unsigned char tx[2];
  tx[0] = cmd;
  tx[1] = data;
  if(HAL_I2C_Master_Transmit(&hi2c1, I2CSLAVEADDR, tx, 2, 0xFFFF) != HAL_OK){
	  BQ27542_error();
  }

}

void BQ27542_blockWrite(unsigned char *buffer, unsigned int length)
{
  if(HAL_I2C_Master_Transmit(&hi2c1, I2CSLAVEADDR, buffer, length, 0xFFFF) != HAL_OK){
	  BQ27542_error();
  }
}

void BQ27542_error(void)
{
	int error = 1;
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
	BQ27542_read(bq27542CMD_USOC_LSB, 2);
	soc = transBytes2UnsignedInt(RxData[1], RxData[0]);

	return soc;
}

uint16_t BQ27542_getVoltage(void){
	//units: mV
	int mV; //bq27542CMD_VOLT_LSB
	BQ27542_read(bq27542CMD_VOLT_LSB, 2);
	mV = transBytes2UnsignedInt(RxData[1], RxData[0]);
	return mV;
}

uint16_t BQ27542_getNomAvailableCapacity(void){
	//units: mAh
	int nomCap;

	BQ27542_read(bq27542CMD_NAC_LSB, 2);
	nomCap = transBytes2UnsignedInt(RxData[1], RxData[0]);
	return nomCap;
}

uint16_t BQ27542_getFullAvailableCapacity(void){
	//units: mAh
	int fullCap;
	BQ27542_read(bq27542CMD_FAC_LSB, 2);
	fullCap = transBytes2UnsignedInt(RxData[1], RxData[0]);
	return fullCap;
}

uint16_t BQ27542_getRemainingCapacity(void){
	//units: mAh
	int remCap;
	BQ27542_read(bq27542CMD_RMC_LSB, 2);
	remCap = transBytes2UnsignedInt(RxData[1], RxData[0]);

	return remCap;
}

uint16_t BQ27542_getFullChargeCapacity(void){
	//units: mAh
	int fullChrgCap;
	BQ27542_read(bq27542CMD_FCC_LSB, 2);
	fullChrgCap = transBytes2UnsignedInt(RxData[1], RxData[0]);
	return fullChrgCap;
}

uint16_t BQ27542_getAverageCurrent(void){
	//units: mA
	int aveCurrent;
	BQ27542_read(bq27542CMD_AI_LSB, 2);
	aveCurrent = transBytes2UnsignedInt(RxData[1], RxData[0]);
	return aveCurrent;
}

uint16_t BQ27542_getTimeToEmpty(void){
	//units: min
	int timeToEmpty;
	BQ27542_read(bq27542CMD_TTE_LSB, 2);
	timeToEmpty = transBytes2UnsignedInt(RxData[1], RxData[0]);
	return timeToEmpty;
}

uint16_t BQ27542_getFullChargeCapacityFiltered(void){
	//units: mAh
	int fullChrgCapFilt;

	BQ27542_read(bq27542CMD_FCCF_LSB, 2);
	fullChrgCapFilt = transBytes2UnsignedInt(RxData[1], RxData[0]);

	return fullChrgCapFilt;
}

uint16_t BQ27542_getFullChargeCapacityUnfiltered(void){
	//units: mAh
	int fullChrgCapUnfilt;

	BQ27542_read(bq27542CMD_FCCU_LSB, 2);
	fullChrgCapUnfilt = transBytes2UnsignedInt(RxData[1], RxData[0]);
	return fullChrgCapUnfilt;
}

uint16_t BQ27542_getImax(void){
	//units: mAh
	int imax;

	BQ27542_read(bq27542CMD_IMAX_LSB, 2);
	imax = transBytes2UnsignedInt(RxData[1], RxData[0]);
	return imax;
}

uint16_t BQ27542_getRemainingCapacityUnfiltered(void){
	//units: mAh
	int remCapUnfilt;

	BQ27542_read(bq27542CMD_RCU_LSB, 2);
	remCapUnfilt = transBytes2UnsignedInt(RxData[1], RxData[0]);

	return remCapUnfilt;
}

uint16_t BQ27542_getRemainingCapacityFiltered(void){
	//units: mAh
	int remCapFilt;

	BQ27542_read(bq27542CMD_RCF_LSB, 2);
	remCapFilt = transBytes2UnsignedInt(RxData[1], RxData[0]);

	return remCapFilt;
}

uint16_t BQ27542_setBTPSOC1(int mAh){
	//units: mAh
	int btpsoc1;
	//TODO: Josh - need to write command
	//BQ27542_read(bq27542CMD_BTP1S_LSB, 2);
	//btpsoc1 = transBytes2UnsignedInt(RxData[1], RxData[0]);
	return btpsoc1;
}

uint16_t BQ27542_clearBTPSOC1(void){
	//units: mAh
	int btpsoc1;
	//TODO: Josh - need to sent write command to clear
	//BQ27542_read(bq27542CMD_BTP1C_LSB, 2);
	//btpsoc1 = transBytes2UnsignedInt(RxData[1], RxData[0]);
	return btpsoc1;
}
uint16_t BQ27542_getInternalTemperature(void){
	//units: 0.1 deg K
	int intTemp;
	BQ27542_read(bq27542CMD_ITMP_LSB, 2);
	intTemp = transBytes2UnsignedInt(RxData[1], RxData[0]);
	return intTemp;
}

uint16_t BQ27542_getCycleCount(void){
	//units: count
	int cycleCnt;
	BQ27542_read(bq27542CMD_CC_LSB, 2);
	cycleCnt = transBytes2UnsignedInt(RxData[1], RxData[0]);

	return cycleCnt;
}

uint16_t BQ27542_getStateOfCharge(void){
	//units: %
	int soc;
	BQ27542_read(bq27542CMD_SOC_LSB, 2);
	soc = transBytes2UnsignedInt(RxData[1], RxData[0]);

	return soc;
}

uint16_t BQ27542_getStateOfHealth(void){
	//units % / num
	int soh;
	BQ27542_read(bq27542CMD_SOH_LSB, 2);
	soh = transBytes2UnsignedInt(RxData[1], RxData[0]);
	return soh;
}

uint16_t BQ27542_getChargingVoltage(void){
	//units: mV
	int chrgVolt;
	BQ27542_read(bq27542CMD_CVLT_LSB, 2);
	chrgVolt = transBytes2UnsignedInt(RxData[1], RxData[0]);//TODO: Josh - develop  code

	return chrgVolt;
}

uint16_t BQ27542_getChargingCurrent(void){
	//units: mA
	int chrgCurrent;

	BQ27542_read(bq27542CMD_CCRNT_LSB, 2);
	chrgCurrent = transBytes2UnsignedInt(RxData[1], RxData[0]);
	return chrgCurrent;
}

uint16_t BQ27542_getPassedCharge(void){
	//units: mAh
	int passedChrg;
	BQ27542_read(bq27542CMD_PCHRG_LSB, 2);
	passedChrg = transBytes2UnsignedInt(RxData[1], RxData[0]);//TODO: Josh - develop  code

	return passedChrg;
}

uint16_t BQ27542_getDOD0(void){
	//units: hex
	int dod0;
	BQ27542_read(bq27542CMD_DOD0_LSB, 2);
	dod0 = transBytes2UnsignedInt(RxData[1], RxData[0]);

	return dod0;
}

uint16_t BQ27542_getSelfDischargeCurrent(void){
	//units: mA
	int disCurrent;

	BQ27542_read(bq27542CMD_SDC_LSB, 2);
	disCurrent = transBytes2UnsignedInt(RxData[1], RxData[0]);
	return disCurrent;
}

