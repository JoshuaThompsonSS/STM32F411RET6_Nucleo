/*
 * FUEL_GAUGE.h
 *
 */

#ifndef _FUEL_GAUGE_H_
#define _FUEL_GAUGE_H_
#include "stm32f4xx_hal.h"


/*
 * ------Constants and Variable Declarations
*/





/*
 * --------Function Declarations
 */
void FUEL_GAUGE_InitConfigs(int rgbnum);
void FUEL_GAUGE_Init(int rgbnum);
void FUEL_GAUGE_Reset(int rgbnum);
void FUEL_GAUGE_Start(int rgbnum);
void FUEL_GAUGE_Stop(int rgbnum);
void FUEL_GAUGE_DeInit(int rgbnum);
void FUEL_GAUGE_PowerDown(int rgbnum);
void FUEL_GAUGE_ExitShutdown(void);
void FUEL_GAUGE_EnterShutdown(void);
void FUEL_GAUGE_ErrorHandler(void);


#endif /* _FUEL_GAUGE_H_ */
