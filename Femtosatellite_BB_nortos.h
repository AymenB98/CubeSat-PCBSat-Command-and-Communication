/** ============================================================================
 *  @file       Femtosatellite_BB_nortos.h
 *
 *  @brief      Header file for femtosatellite CC1310
 *
 *  The rfEasyLinkRxBB_nortos.h header file should be included in an application as
 *  follows:
 *  @code
 *  #include "Femtosatellite_BB_nortos.h"
 *  @endcode
 *
 *  ============================================================================
 */


#ifndef FEMTOSATELLITE_BB_NORTOS_H_
#define FEMTOSATELLITE_BB_NORTOS_H_

// Standard C Libraries
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>

/* TI Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/Power.h>
#include <ti/display/Display.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/interrupt.h)

/* Board Header files */
#include "Board.h"

/* Application Header files */
#include "smartrf_settings/smartrf_settings.h"

/* EasyLink API Header files */
#include "easylink/EasyLink.h"

void ledSetup();
void displaySetup();
void commandRx();
void ackTx();
void dummyCommand(uint8_t commandID, uint8_t rxPacket[30], uint8_t sleepTime);
void displayQuat(uint8_t rxPacket[30]);

#endif /* FEMTOSATELLITE_BB_NORTOS_H_ */
