/** ============================================================================
 *  @file       rfEasyLinkRx_nortos.h
 *
 *  @brief      Header file for CubeSat CC1310
 *
 *  The rfEasyLinkRx_nortos.h header file should be included in an application as
 *  follows:
 *  @code
 *  #include "rfEasyLinkRx_nortos.h"
 *  @endcode
 *
 *  ============================================================================
 */

#ifndef RFEASYLINKRX_NORTOS_H_
#define RFEASYLINKRX_NORTOS_H_

void driverSetup();
bool sdSetup(int8_t rssi, uint8_t errorCode);
bool sdWrite(SD_Handle sdHandle, int_fast8_t result, bool sdFailure);
void commandRx();
void groundStationAckTx();
void commandTx();
void femtosatAckRx();
void dataTx();
bool isPacketCorrect(EasyLink_RxPacket *rxp, EasyLink_TxPacket *txp);

#endif /* RFEASYLINKRX_NORTOS_H_ */
