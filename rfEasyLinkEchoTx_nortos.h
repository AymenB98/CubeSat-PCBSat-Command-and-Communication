/** ============================================================================
 *  @file       rfEasyLinkTx_nortos.h
 *
 *  @brief      Header file for ground station CC1310
 *
 *  The rfEasyLinkTx_nortos.h header file should be included in an application
 *  as follows:
 *  @code
 *  #include "rfEasyLinkTx_nortos.h"
 *  @endcode
 *
 *  ============================================================================
 */

#ifndef RFEASYLINKECHOTX_NORTOS_H_
#define RFEASYLINKECHOTX_NORTOS_H_

//! \brief User entry options
typedef enum
{
    NUMBER_OF_COMMANDS = 0, /*!< Number of commands sent to femtosat */
    COMMAND_SLEEP_TIME = 1, /*!< The sleep time in between each command */
    FEMTOSAT_ADDRESS = 2 /*!< Selected femtosat address */
} Entry_Type;

// Function prototypes
void displaySetup();
void uartDisplaySetup();
char userInput(char *message, int messageSize);
bool inputErrors(char userEntry, Entry_Type entryType);
void userEntryCompile();
void uartWriteSimple(UART_Handle uartHandle, char *string, int stringSize);
void uartReadSimple(int status);
void timerSetup();
void timerStart();
void timerEnd();
void rfPacketSetup(uint8_t commandNumber, uint8_t sleepTime, uint8_t femtoAddress);
void femtosatStatusDisplay(uint8_t femtoRssi, uint8_t statusByte);
void cubeSatTx();
bool cubeSatAckRx();
void dataRx(bool ackFlag);

#endif /* RFEASYLINKECHOTX_NORTOS_H_ */
