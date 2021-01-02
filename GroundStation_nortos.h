/** ============================================================================
 *  @file       GroundStation_nortos.h
 *
 *  @brief      Header file for ground station CC1310
 *
 *  The rfEasyLinkTx_nortos.h header file should be included in an application
 *  as follows:
 *  @code
 *  #include "GroundStation_nortos.h"
 *  @endcode
 *
 *  ============================================================================
 */

#ifndef GROUNDSTATION_NORTOS_H_
#define GROUNDSTATION_NORTOS_H_

#include <stdio.h>
#include <stdlib.h>

#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/timer/GPTimerCC26XX.h>
#include <ti/display/Display.h>
#include <ti/devices/DeviceFamily.h>
#include <ti/drivers/UART.h>
#include <unistd.h>
// Define macros for constants used throughout code

#define PAYLOAD_LENGTH   30 /*!< Number of data bytes being transmitted/received */
#define RX_TIMEOUT   500 /*!< Time (ms) until RX operations timeout */
#define CUBESAT_ADDRESS     0xCC /*!< CubeSat address used for address filtering */
#define TIMING_TEST     0 /*!< Change to 1 when timing of code is to be performed */

// Define the command ID for each command
#define COMMAND_QUAT     0x1 /*!< Display quaternion sent from ground station to femtosat */
#define COMMAND_GLED     0x2 /*!< Set green LED high for two seconds */
#define COMMAND_THREE   0x3 /*!< Empty command */

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

#endif /* GROUNDSTATION_NORTOS_H_ */
