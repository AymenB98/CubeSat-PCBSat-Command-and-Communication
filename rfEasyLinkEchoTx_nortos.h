/** ============================================================================
 *  @file       rfEasyLinkTx_nortos.h
 *
 *  @brief      CC1310 LaunchPad Board Specific header file.
 *
 *  The rfEasyLinkTx_nortos.h header file should be included in an application as
 *  follows:
 *  @code
 *  #include "rfEasyLinkTx_nortos.h"
 *  @endcode
 *
 *  ============================================================================
 */

#ifndef RFEASYLINKECHOTX_NORTOS_H_
#define RFEASYLINKECHOTX_NORTOS_H_

// Function prototypes
void displaySetup();
void timerSetup();
void timerStart();
void timerEnd();
void rfPacketSetup();
void femtosatStatusDisplay(uint8_t femtoRssi, uint8_t statusByte, uint8_t femtoAddr);
void cubeSatTx();
bool cubeSatAckRx();
void dataRx(bool ackFlag);

#endif /* RFEASYLINKECHOTX_NORTOS_H_ */
