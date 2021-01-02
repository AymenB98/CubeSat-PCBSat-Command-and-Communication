/** ============================================================================
 *  @file       CubeSat_nortos.h
 *
 *  @brief      Header file for CubeSat CC1310
 *
 *  The rfEasyLinkRx_nortos.h header file should be included in an application as
 *  follows:
 *  @code
 *  #include "CubeSat_nortos.h"
 *  @endcode
 *
 *  ============================================================================
 */

#ifndef CUBESAT_NORTOS_H_
#define CUBESAT_NORTOS_H_

// Standard C Libraries
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

// TI Drivers
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/SD.h>
#include <ti/display/Display.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/interrupt.h)

// Board Header files
#include "Board.h"

// Application Header files
#include "smartrf_settings/smartrf_settings.h"

// EasyLink API Header files
#include "easylink/EasyLink.h"

void driverSetup();
bool sdSetup(int8_t rssi, uint8_t errorCode);
bool sdWrite(SD_Handle sdHandle, int_fast8_t result, bool sdFailure);
void commandRx();
void groundStationAckTx();
void commandTx();
void femtosatAckRx();
void dataTx();
bool isPacketCorrect(EasyLink_RxPacket *rxp, EasyLink_TxPacket *txp);

#endif /* CUBESAT_NORTOS_H_ */
