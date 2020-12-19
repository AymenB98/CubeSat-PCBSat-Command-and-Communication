/** ============================================================================
 *  @file       rfEasyLinkRxBB_nortos.h
 *
 *  @brief      Header file for femtosatellite CC1310
 *
 *  The rfEasyLinkRxBB_nortos.h header file should be included in an application as
 *  follows:
 *  @code
 *  #include "rfEasyLinkRxBB_nortos.h"
 *  @endcode
 *
 *  ============================================================================
 */


#ifndef RFEASYLINKRXBB_NORTOS_H_
#define RFEASYLINKRXBB_NORTOS_H_

void ledSetup();
void displaySetup();
void commandRx();
void ackTx();
void dummyCommand(uint8_t commandID, uint8_t rxPacket[30], uint8_t sleepTime);
void displayQuat(uint8_t rxPacket[30]);

#endif /* RFEASYLINKRXBB_NORTOS_H_ */
