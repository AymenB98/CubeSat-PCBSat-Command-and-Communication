/*
 * Copyright (c) 2017-2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/** ============================================================================
 *  @file       GroundStation_nortos.c
 *
 *  @brief      Source file that allows communication between ground station
 *  and CubeSat.
 *
 *  @author     Aymen Benylles
 *  @date       30/12/2020
 *
 *  ============================================================================
 */
#include <GroundStation_nortos.h>
// Standard C Libraries
#include <stdio.h>
#include <stdlib.h>

/* TI-RTOS Header files */
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/timer/GPTimerCC26XX.h>
#include <ti/display/Display.h>
#include <ti/devices/DeviceFamily.h>
#include <ti/drivers/UART.h>
#include <unistd.h>

// Driverlib APIs
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)

// Board Header files
#include "Board.h"

// Application Header files
#include "smartrf_settings/smartrf_settings.h"

// EasyLink API Header files
#include "easylink/EasyLink.h"


EasyLink_Params easyLinkParams;

uint32_t absTime; /*!< Variable the RF core uses to time commands */

EasyLink_TxPacket txPacket = {{0}, 0, 0, {0}}; /*!< Packet transmitted to CubeSat */
EasyLink_RxPacket rxPacket = {{0}, 0, 0, 0, 0, {0}}; /*!< Packet received from CubeSat */

EasyLink_Status result; /*!< Status of each RF command */

uint8_t femtoAddr; /*!< Address of femtosatellite message is being sent to */
bool receptionFlag; /*!< Flag raised if femtosatellite has received commands */

// Driver handles
static PIN_Handle pinHandle;
static PIN_State pinState;
static Display_Handle display;
static GPTimerCC26XX_Handle timerHandle;
static UART_Handle uartHandle;
static UART_Params uartParams;

// Timer parameters
static GPTimerCC26XX_Params timerParams;
static GPTimerCC26XX_Value timerValue;

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config pinTable[] = {
    Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_PIN_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

/**
 *  @brief  Setup display driver
 *
 *  @return none
 *
 */
void displaySetup()
{
    Display_init();

    /* Open the display for output */
    display = Display_open(Display_Type_UART, NULL);
    if (display == NULL)
    {
        /* Failed to open display driver */
        while (1);
    }
}

/**
 *  @brief  Setup UART display driver
 *
 *  @return none
 *
 */
void uartDisplaySetup()
{
    UART_init();
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;

    uartHandle = UART_open(Board_UART0, &uartParams);
    if(uartHandle == NULL)
    {
        while(1);
    }
}

/**
 *  @brief  Setup GP timer
 *
 *  @return none
 *
 */
void timerSetup()
{
    char timerError[] = "Error opening GP Timer.\n";
    GPTimerCC26XX_Params_init(&timerParams);
    timerParams.width = GPT_CONFIG_32BIT;
    timerParams.direction = GPTimerCC26XX_DIRECTION_UP;
    timerParams.debugStallMode = GPTimerCC26XX_DEBUG_STALL_OFF;

    timerHandle = GPTimerCC26XX_open(Board_GPTIMER0A, &timerParams);
    if(timerHandle == NULL)
    {
        uartWriteSimple(uartHandle, timerError, sizeof(timerError));
    }
}

/**
 *  @brief  Start the timer
 *
 *  @return none
 *
 */
void timerStart()
{
    GPTimerCC26XX_start(timerHandle);
    if(timerHandle == NULL)
    {
        Display_printf(display, 0, 0, "Error starting GP Timer.\n");
    }
}

/**
 *  @brief  Stop the timer and close the driver
 *  This also displays the timer value in milliseconds.
 *
 *  @return none
 *
 */
void timerEnd()
{
    timerValue = GPTimerCC26XX_getValue(timerHandle);
    double clockFreq = 48000000;
    float timerValueSeconds = timerValue / clockFreq;
    float timerValueMs = timerValueSeconds * 1000;
    Display_printf(display, 0, 0, "Timer value: %fms \n", timerValueMs);
    GPTimerCC26XX_stop(timerHandle);
    GPTimerCC26XX_close(timerHandle);
    if(timerHandle == NULL)
    {
        Display_printf(display, 0, 0, "Failed to halt GP Timer.\n");
    }
}

/**
 *  @brief  Handle errors in user entry
 *
 *  @param userEntry    Character entered by user
 *  @param entryType    Type of data entered by user
 *
 *  @return userErrorFlag   Flag raised when user makes an invalid input
 *
 */
bool inputErrors(char userEntry, Entry_Type entryType)
{
    bool userErrorFlag = false;
    switch(entryType)
    {
    case NUMBER_OF_COMMANDS:
        if((userEntry > 51) | (userEntry < 49))
        {
            Display_printf(display, 0, 0, "Invalid number of commands.\n");
            Display_printf(display, 0, 0, "Please pick a number between 1 and 3.\n");
            Display_printf(display, 0, 0, "Restart the device to send command(s).\n");
            userErrorFlag = true;
        }
        break;
    case COMMAND_SLEEP_TIME:
        if(userEntry > 57)
        {
            Display_printf(display, 0, 0, "Invalid sleep time.\n");
            Display_printf(display, 0, 0, "Please declare a sleep time between 0 and 9s.\n");
            Display_printf(display, 0, 0, "Restart the device to send command(s).\n");
            userErrorFlag = true;
        }
        break;
    case FEMTOSAT_ADDRESS:
        if(((int)userEntry < 97) | ((int)userEntry > 101))
        {
            Display_printf(display, 0, 0, "Invalid femtosat address.\n");
            Display_printf(display, 0, 0, "Please enter a letter from the list of options.\n");
            Display_printf(display, 0, 0, "Restart the device to send command(s).\n");
            userErrorFlag = true;
        }
        break;
    default:
        break;
    }

    return userErrorFlag;
}

/**
 *  @brief  Perform UART write with error checking
 *
 *  @return none
 *
 */
void uartWriteSimple(UART_Handle uartHandle, char *string, int stringSize)
{
    UART_write(uartHandle, string, stringSize);
    if(uartHandle == NULL)
    {
        while(1);
    }
}

/**
 *  @brief  Perform UART read with error checking
 *
 *  @param  status      Status of read operation
 *
 *  @return none
 *
 */
void uartReadSimple(int status)
{
    if(status == NULL)
    {
        while(1);
    }
}

/**
 *  @brief  Display start-up prompts to user
 *
 *  @param *message     Prompt to be displayed to user
 *  @param messageSize  Size of the prompt
 *
 *  @return userEntry   Character entered by user
 *
 */
char userInput(char *message, int messageSize)
{
    char userEntry;
    char newLine[] = "\r\n";
    // Display prompt
    uartWriteSimple(uartHandle, message, messageSize);
    // Read user input and check for an error
    int uartStatus = UART_read(uartHandle, &userEntry, sizeof(userEntry));
    uartReadSimple(uartStatus);

    // Display user entry back to user
    uartWriteSimple(uartHandle, &userEntry, sizeof(userEntry));
    uartWriteSimple(uartHandle, newLine, sizeof(newLine));

    return userEntry;
}

/**
 *  @brief  Process the data entered by the user
 *
 *  @return none
 *
 */
void userEntryCompile()
{
    // Declare chars for user input
    char numberOfCommands;
    char sleepInput;
    char femtoAddressInput;

    /* Declare uint8_ts that will be sent in the packet.
     * ASCII conversion will be used to convert char types into numbers.
     */
    uint8_t commandNumber;
    uint8_t sleepTime;
    uint8_t femtoAddress;

    // Set up user prompts
    char userCommandMessage[] = "Enter number of commands you wish to send:\r\n";
    char userSleepMessage[]  = "Enter the sleep time between commands(s):\r\n";
    char userFemtoAddress[] = "Enter the desired femtosat address:\r\n";
    char femtoOptions[] = "\r\na: 0xDE\r\nb: 0xBB\r\nc: 0xFE\r\nd: 0xBC\r\ne: 0xCD\r\n";

    // Display prompts to the user and read their input
    numberOfCommands = userInput(userCommandMessage, sizeof(userCommandMessage));
    sleepInput = userInput(userSleepMessage, sizeof(userSleepMessage));
    uartWriteSimple(uartHandle, userFemtoAddress, sizeof(userFemtoAddress));
    femtoAddressInput = userInput(femtoOptions, sizeof(femtoOptions));

    // Perform ASCII conversion
    commandNumber = (int)numberOfCommands - 48;
    sleepTime = (int)sleepInput - 48;

    // Assign femtosat addresses
    switch(femtoAddressInput)
    {
    case 'a':
        femtoAddress = 0xDE;
        break;
    case 'b':
        femtoAddress = 0xBB;
        break;
    case 'c':
        femtoAddress = 0xFE;
        break;
    case 'd':
        femtoAddress = 0xBC;
        break;
    case 'e':
        femtoAddress = 0xCD;
        break;
    default:
        break;
    }

    /* Close UART driver before the generic Display driver is opened.
     * Display driver will not work otherwise.
     */
    UART_close(uartHandle);

    // Now that UART driver is closed, setup display driver
    displaySetup();

    /*
     * Set global femtosat address variable so that display in
     * dataRx function is correct
     */
    femtoAddr = femtoAddress;

    // Flags raised in the event of a user input error
    bool errorFlagCommands, errorFlagSleep, errorFlagAddress;

    // Test the user inputs to make sure they are valid
    errorFlagCommands = inputErrors(numberOfCommands, NUMBER_OF_COMMANDS);
    errorFlagSleep = inputErrors(sleepInput, COMMAND_SLEEP_TIME);
    errorFlagAddress = inputErrors(femtoAddressInput, FEMTOSAT_ADDRESS);

    // Only continue in the event of all three entries being valid
    if((!errorFlagCommands) && (!errorFlagSleep) && (!errorFlagAddress))
    {
        // Proceed to setup RF packet now that user entries have been checked
        rfPacketSetup(commandNumber, sleepTime, femtoAddress);
    }
}

/**
 *  @brief  Setup packet to be sent to CubeSat
 *
 *  @return none
 *
 */
void rfPacketSetup(uint8_t commandNumber, uint8_t sleepTime, uint8_t femtoAddress)
{
    // Initialise variables for RF operations
    EasyLink_Params_init(&easyLinkParams);

    // Set up desired quaternion to be sent to femtosat
    float quatFloat[4] = {1.0, 0.0, 0.0, 0.0};

    // Initialise variables to convert float into 8-bit integer array

    /* Create integer array first, then split this up
     * into 8-bit unsigned integers that are sent to CubeSat
     */
    int i, quatInt[4];
    uint8_t quatTx[16];

    // Convert float array into int array
    for(i = 0; i < 4; i++)
    {
        quatInt[i] = quatFloat[i] * 1000;
    }

    /* Split 4-byte integer into 4 individual bytes.
     * These will get transferred one byte at a time
     * as unsigned 8-bit integers.
     * On the receiver end, these will be combined back into
     * a 32-bit integer, then a float for the original
     * quaternion format.
     */
    quatTx[0] = (quatInt[0] & 0xFF000000) >> 24;
    quatTx[1] = (quatInt[0] & 0x00FF0000) >> 16;
    quatTx[2] = (quatInt[0] & 0x0000FF00) >> 8;
    quatTx[3] = (quatInt[0] & 0x000000FF);

    quatTx[4] = (quatInt[1] & 0xFF000000) >> 24;
    quatTx[5] = (quatInt[1] & 0x00FF0000) >> 16;
    quatTx[6] = (quatInt[1] & 0x0000FF00) >> 8;
    quatTx[7] = (quatInt[1] & 0x000000FF);

    quatTx[8] = (quatInt[2] & 0xFF000000) >> 24;
    quatTx[9] = (quatInt[2] & 0x00FF0000) >> 16;
    quatTx[10] = (quatInt[2] & 0x0000FF00) >> 8;
    quatTx[11] = (quatInt[2] & 0x000000FF);

    quatTx[12] = (quatInt[3] & 0xFF000000) >> 24;
    quatTx[13] = (quatInt[3] & 0x00FF0000) >> 16;
    quatTx[14] = (quatInt[3] & 0x0000FF00) >> 8;
    quatTx[15] = (quatInt[3] & 0x000000FF);

    // Open LED pins
    pinHandle = PIN_open(&pinState, pinTable);
    if (pinHandle == NULL)
    {
        while(1);
    }

    // Clear LEDs
    PIN_setOutputValue(pinHandle, Board_PIN_LED1, 0);
    PIN_setOutputValue(pinHandle, Board_PIN_LED2, 0);

    /*
     * Initialise EasyLink with the settings found in easylink_config.h
     * Modify EASYLINK_PARAM_CONFIG in easylink_config.h to change the default
     * PHY
     */
    if (EasyLink_init(&easyLinkParams) != EasyLink_Status_Success)
    {
        while(1);
    }

    /*
     * If you wish to use a frequency other than the default (e.g. 433MHz), use
     * the following API:
     * EasyLink_setFrequency(433000);
     */

    // Create the packet being sent to the CubeSat

    // The 0th byte must contain the femtosat address
    txPacket.payload[0] = femtoAddress;

    // The next byte tells the femtosat how many commands it must perform
    txPacket.payload[1] = commandNumber;
    // Fill the next byte with the quaternion command
    txPacket.payload[2] = COMMAND_QUAT;
    /* How long for the CubeSat to be in standby mode until
     * the next instruction
     */
    txPacket.payload[3] = sleepTime;
    // Fill TX packet with quaternion data
    for (i = 4; i < 20; i++)
    {
        txPacket.payload[i] = quatTx[i-4];
    }
    /* If more commands are to be completed after the quaternion command,
     * they are initialised here
     */
    for(i = 20; i < PAYLOAD_LENGTH; i++)
    {
        // Even number indices are commands
        if(!(i % 2))
        {
            txPacket.payload[i] = COMMAND_GLED;
        }
        // Odd number indices dictate sleep time
        else
        {
            txPacket.payload[i] = sleepTime;
        }
    }
    txPacket.len = PAYLOAD_LENGTH;

    txPacket.dstAddr[0] = CUBESAT_ADDRESS;
}

/**
 *  @brief  Check ack packet from CubeSat is correct
 *
 *  @param *rxp     received packet
 *  @param *txp     transmitted packet
 *
 *  @return status
 *
 */
bool isPacketCorrect(EasyLink_RxPacket *rxp, EasyLink_TxPacket *txp)
{
    uint16_t i;
    bool status = true;

    for(i = 0; i < rxp->len; i++)
    {
        if(rxp->payload[i] != txp->payload[i])
        {
            status = false;
            break;
        }
    }
    return(status);
}

/**
 *  @brief  Display the status of the femtosat to the user via UART display
 *
 *  @param femtoRssi        RSSI value for CubeSay-femtosat link
 *  @param statusByte       Byte containing status of femtosat reception
 *  @param femtoAddr        Address of femtosat
 *
 *  @return none
 *
 */
void femtosatStatusDisplay(uint8_t femtoRssi, uint8_t statusByte)
{
    Display_printf(display, 0, 0, "0x%x femtosat reception status: ", femtoAddr);
    switch(statusByte)
    {
    case 0:
        Display_printf(display, 0, 0, "successful.\n");
        // Remember to cast RSSI value from unsigned integer to signed integer.
        Display_printf(display, 0, 0, "Data received from CubeSat: %x -> %ddBm.\n", femtoAddr,
                       (int8_t)femtoRssi);
        receptionFlag = true;
        break;
    case 1:
        Display_printf(display, 0, 0, "timeout error.\n");
        receptionFlag = false;
        break;
    case 2:
        Display_printf(display, 0, 0, "error.\n");
        receptionFlag = false;
        break;
    default:
        break;
    }
}

/**
 *  @brief  Transmit command(s) to CubeSat
 *
 *  @return none
 *
 */
void cubeSatTx()
{

    // Set Tx absolute time to current time
    if(EasyLink_getAbsTime(&absTime) != EasyLink_Status_Success)
    {
        // Problem getting absolute time
        while(1);
    }

    txPacket.absTime = absTime;
    result = EasyLink_transmit(&txPacket);

    // TX to CubeSat was successful
    if (result == EasyLink_Status_Success)
    {
        // Toggle LED1 to indicate TX
        PIN_setOutputValue(pinHandle, Board_PIN_LED1,!PIN_getOutputValue(Board_PIN_LED1));
        // Turn LED2 off, in case there was a prior error
        PIN_setOutputValue(pinHandle, Board_PIN_LED2, 0);
        Display_printf(display, 0, 0, "Ground station TX successful.\n");
    }
    // TX to CubeSat failed
    else
    {
        // Set both LED1 and LED2 to indicate error
        PIN_setOutputValue(pinHandle, Board_PIN_LED1, 1);
        PIN_setOutputValue(pinHandle, Board_PIN_LED2, 1);
        Display_printf(display, 0, 0, "Ground station TX error.\n");
    }
}

/**
 *  @brief  Switch to receive mode for ack from CubeSat
 *
 *  @return ackFlag
 *
 */
bool cubeSatAckRx()
{

    /* Flag indicating that ack has been received from CubeSat.
     * When raised, data can be received.
     * When held low, ground station must retransmit data to CubeSat.
     */
    bool ackFlag;

    // Switch to RX mode immediately
    rxPacket.absTime = 0;
    // Set a timeout interval of 500ms
    rxPacket.rxTimeout = EasyLink_ms_To_RadioTime(RX_TIMEOUT);
    result = EasyLink_receive(&rxPacket);

    /* Check Received packet against what was sent, it should be identical
     * to the transmitted packet
     */
    if(result == EasyLink_Status_Success &&
            isPacketCorrect(&rxPacket, &txPacket))
    {
        // Toggle LED1, clear LED2 to indicate Echo RX
        PIN_setOutputValue(pinHandle, Board_PIN_LED1,!PIN_getOutputValue(Board_PIN_LED1));
        PIN_setOutputValue(pinHandle, Board_PIN_LED2, 0);
        Display_printf(display, 0, 0, "Ack received from CubeSat.\n");
        ackFlag = true;
    }
    else if(result == EasyLink_Status_Rx_Timeout)
    {
        // Set LED2 and clear LED1 to indicate Rx Timeout
        PIN_setOutputValue(pinHandle, Board_PIN_LED2, 1);
        PIN_setOutputValue(pinHandle, Board_PIN_LED1, 0);
        Display_printf(display, 0, 0,
                       "Device timed out before ack received from CubeSat.\n");
        ackFlag = false;
    }
    else
    {
        // Set both LED1 and LED2 to indicate error
        PIN_setOutputValue(pinHandle, Board_PIN_LED1, 1);
        PIN_setOutputValue(pinHandle, Board_PIN_LED2, 1);
        Display_printf(display, 0, 0, "RX error.\n");
        ackFlag = false;
    }

    return ackFlag;
}

/**
 *  @brief  Switch to receive mode for data from CubeSat
 *
 *  @return none
 *
 */
void dataRx(bool ackFlag)
{
    /* Stay in RX mode until data is received from CubeSat.
     * Exit loop after a five attempts to restart RF link w/ CubeSat.
     * If this is not performed, there is a chance that the ground station and
     * Cubesat get stuck waiting for each other to send data.
     */
    uint8_t count = 0;
    while(ackFlag && (count < 5))
    {
        // Restart RX mode and wait for data from CubeSat
        rxPacket.absTime = 0;
        // Set timeout interval to 1000ms (1s)
        rxPacket.rxTimeout = EasyLink_ms_To_RadioTime(RX_TIMEOUT * 2);

        result = EasyLink_receive(&rxPacket);

        // No need to check the packet since this is a data transmission
        if (result == EasyLink_Status_Success)
        {
            // Toggle LED1, clear LED2 to indicate Echo RX
            PIN_setOutputValue(pinHandle, Board_PIN_LED1,!PIN_getOutputValue(Board_PIN_LED1));
            PIN_setOutputValue(pinHandle, Board_PIN_LED2, 0);
            // Display correct message about femtosat operation
            femtosatStatusDisplay(rxPacket.payload[0], rxPacket.payload[1]);

            // Exit loop now that data has been received from CubeSat.
            ackFlag = false;
        }
        else if (result == EasyLink_Status_Rx_Timeout)
        {
            // Set LED2 and clear LED1 to indicate RX Timeout
            PIN_setOutputValue(pinHandle, Board_PIN_LED2, 1);
            PIN_setOutputValue(pinHandle, Board_PIN_LED1, 0);
            Display_printf(display, 0, 0,
                           "Device timed out before data received from CubeSat.\n");
            // Stay in loop until data is received from CubeSat
            ackFlag = true;
        }
        else
        {
            // Set both LED1 and LED2 to indicate error
            PIN_setOutputValue(pinHandle, Board_PIN_LED1, 1);
            PIN_setOutputValue(pinHandle, Board_PIN_LED2, 1);
            Display_printf(display, 0, 0,
                           "RX error.\n");
            // Stay in loop until data has been received from CubeSat
            ackFlag = true;
        }
        // Keep track of the number of times RX has been attempted
        count++;
    }
}

void *mainThread(void *arg0)
{
    uartDisplaySetup();

#if TIMING_TEST
    // Setup the GP timer
    timerSetup();
#endif

    userEntryCompile();
//    Display_printf(display, 0, 0, "Starting ground station...\n");

    /* Enter infinite loop which performs all of the
     * necessary RF commands.
     * If the user wishes to only send a command(s) to the femtosat once,
     * this while(1) can be removed.
     * In this case, the user must be aware of the fact that if this TX misses
     * the CubeSat, they can simply press the reset button on the board to
     * re-send the command(s).
     */

    while(!receptionFlag)
    {
#if TIMING_TEST
        // Start the GP timer
        timerStart();
#endif

        /*************************************************************************
         *                                                                       *
         *                               TX MODE-------------------------------->*
         *                   Send command(s) to CubeSat                          *
         *                                                                       *
         *************************************************************************/
        cubeSatTx();

        /*************************************************************************
         *                                                                       *
         *                               RX MODE<--------------------------------*
         *                      Receive ack from CubeSat                         *
         *                                                                       *
         *************************************************************************/
        bool ackFlag = cubeSatAckRx();

        /*************************************************************************
         *                                                                       *
         *                               RX MODE<--------------------------------*
         *                      Receive data from CubeSat                        *
         *                                                                       *
         *************************************************************************/
        dataRx(ackFlag);

#if TIMING_TEST
        // Stop the timer, display the value and close the driver
        timerEnd();
#endif
    }
    return(NULL);
}
