/* File: "bq27542.h"
 * Desc: Header file for TI Fuel Gauge I2C Commands and Functions
*/

//Includes
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include <stdlib.h>

//I2C Register Constants
//Control Command
#define bq27542CMD_CNTL_LSB  	0x00
#define bq27542CMD_CNTL_MSB  	0x01
//Start Control Subcommands
#define bq27542CMD_CNTL_DTYPE_MSB   0x00
#define bq27542CMD_CNTL_DTYPE_LSB   0x01
#define bq27542CMD_CNTL_FWVER_MSB	0x00
#define bq27542CMD_CNTL_FWVER_LSB	0x02
#define bq27542CMD_CNTL_HWVER_MSB	0x00
#define bq27542CMD_CNTL_HWVER_LSB	0x03
//End Control subcommands

#define bq27542CMD_AR_LSB    	0x02
#define bq27542CMD_AR_MSB   	0x03
#define bq27542CMD_ARTTE_LSB 	0x04
#define bq27542CMD_ARTTE_MSB 	0x05
#define bq27542CMD_TEMP_LSB  	0x06
#define bq27542CMD_TEMP_MSB 	0x07
#define bq27542CMD_VOLT_LSB  	0x08
#define bq27542CMD_VOLT_MSB  	0x09
#define bq27542CMD_FLAGS_LSB 	0x0A
#define bq27542CMD_FLAGS_MSB 	0x0B
#define bq27542CMD_NAC_LSB   	0x0C
#define bq27542CMD_NAC_MSB   	0x0D
#define bq27542CMD_FAC_LSB   	0x0E
#define bq27542CMD_FAC_MSB   	0x0F
#define bq27542CMD_RM_LSB    	0x10
#define bq27542CMD_RM_MSB    	0x11
#define bq27542CMD_FCC_LSB   	0x12
#define bq27542CMD_FCC_MSB   	0x13
#define bq27542CMD_AI_LSB    	0x14
#define bq27542CMD_AI_MSB    	0x15
#define bq27542CMD_TTE_LSB   	0x16
#define bq27542CMD_TTE_MSB   	0x17
#define bq27542CMD_TTF_LSB   	0x18
#define bq27542CMD_TTF_MSB   	0x19
#define bq27542CMD_SI_LSB    	0x1A
#define bq27542CMD_SI_MSB    	0x1B
#define bq27542CMD_STTE_LSB  	0x1C
#define bq27542CMD_STTE_MSB  	0x1D
#define bq27542CMD_MLI_LSB   	0x1E
#define bq27542CMD_MLI_MSB   	0x1F
#define bq27542CMD_MLTTE_LSB 	0x20
#define bq27542CMD_MLTTE_MSB 	0x21
#define bq27542CMD_AE_LSB    	0x22
#define bq27542CMD_AE_MSB    	0x23
#define bq27542CMD_AP_LSB    	0x24
#define bq27542CMD_AP_MSB    	0x25
#define bq27542CMD_TTECP_LSB 	0x26
#define bq27542CMD_TTECP_MSB 	0x27
#define bq27542CMD_RSVD_LSB  	0x28
#define bq27542CMD_RSVD_MSB  	0x29
#define bq27542CMD_CC_LSB    	0x2A
#define bq27542CMD_CC_MSB    	0x2B
#define bq27542CMD_SOC_LSB   	0x2C
#define bq27542CMD_SOC_MSB   	0x2D


#define ATRATE_MA            -100           // USER CONFIG: AtRate setting (mA)
#define I2CSLAVEADDR         0xAA           // 0xAA = 0x55 << 1 = 7-bit slave address
#define BUFFERSIZE             32           // # of bytes for Tx & Rx buffers
/*
extern UINT8 Message[RANDMESGNUMBYTES]; // random message sent to the bq device
extern UINT8 Key[SECRETKEYNUMBYTES]; // secret key - should match contents of bq
extern UINT32 Digest_32[5]; // Result of SHA1/HMAC obtained by MCU stored here
extern UINT32 H[5];

UINT8 DeviceID[DEVICEIDNUMBYTES];           // Stores the Device ID data
UINT8 Digest[DIGESTNUMBYTES];               // SHA1 response from the bq27542
*/

unsigned char TxData[BUFFERSIZE];           // Stores data bytes to be TX'd
unsigned char RxData[BUFFERSIZE];           // Stores data bytes that are RX'd
unsigned int  temperature;                  // Stores temperature
unsigned int  voltage;                      // Stores voltage
  signed int  atrate;                       // Stores AtRate
unsigned int  artte;                        // Stores AtRate Time to Empty
unsigned int  soc;                          // Stores State of Charge
unsigned int  dcap;                         // Stores Design Capacity
unsigned int  dnamelen;                     // Stores Device Name Length


//Function Declarations
unsigned int transBytes2UnsignedInt(unsigned char msb, unsigned char lsb);
void BQ27542_read(unsigned char cmd, unsigned int bytes);
void BQ27542_cmdWrite(unsigned char cmd, unsigned char data);
void BQ27542_blockWrite(unsigned char *buffer, unsigned int length);
void BQ27542_error(void);

/*
 * Function below based on BQ27542 Datasheet standard data commands (http://www.ti.com/lit/ds/symlink/bq27542-g1.pdf)
 */

uint16_t BQ27542_getDeviceType(void); //0x0542 is the expected result
uint16_t BQ27542_getTemperature(void); //0.1 deg K
uint16_t BQ27542_getUnfilteredSOC(void); //%
uint16_t BQ27542_getVoltage(void); //mV
uint16_t BQ27542_getNomAvailableCapacity(void); //mAh
uint16_t BQ27542_getFullAvailableCapacity(void); //mAh
uint16_t BQ27542_getRemainingCapacity(void); //mAh
uint16_t BQ27542_getFullChargeCapacity(void); //mAh
uint16_t BQ27542_getAverageCurrent(void); //mA
uint16_t BQ27542_getTimeToEmpty(void); //min
uint16_t BQ27542_getFullChargeCapacityFiltered(void); //mAh
uint16_t BQ27542_getFullChargeCapacityUnfiltered(void); //mAh
uint16_t BQ27542_getImax(void); //mAh
uint16_t BQ27542_getRemainingCapacityUnfiltered(void); //mAh
uint16_t BQ27542_getRemainingCapacityFiltered(void); //mAh
uint16_t BQ27542_setBTPSOC1(int mAh); //mAh
uint16_t BQ27542_clearBTPSOC1(void); //mAh
uint16_t BQ27542_getInternalTemperature(void); // 0.1 deg K
uint16_t BQ27542_getCycleCount(void); //count
uint16_t BQ27542_getStateOfCharge(void); //%
uint16_t BQ27542_getStateOfHealth(void); //% / num
uint16_t BQ27542_getChargingVoltage(void); //mV
uint16_t BQ27542_getChargingCurrent(void); //mA
uint16_t BQ27542_getPassedCharge(void); //mAh
uint16_t BQ27542_getDOD0(void); //hex
uint16_t BQ27542_getSelfDischargeCurrent(void); //mA





