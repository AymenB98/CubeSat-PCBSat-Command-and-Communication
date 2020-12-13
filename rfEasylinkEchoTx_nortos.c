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


/*
 * This version of the code was created by Aymen Benylles by adapting TI examples.
 * This code is for the ground station, and sends commands to the CubeSat.
 *
 */


/*
 *  ======== rfEasyLinkEchoTx_nortos.c ========
 */
 // Standard C Libraries
#include <stdlib.h>

/* TI-RTOS Header files */
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/timer/GPTimerCC26XX.h>
#include <ti/display/Display.h>
#include <ti/devices/DeviceFamily.h>

// Driverlib APIs
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)

// Board Header files
#include "Board.h"

// Application Header files
#include "smartrf_settings/smartrf_settings.h"

// EasyLink API Header files
#include "easylink/EasyLink.h"

// Define macros for constants used throughout code

// Number of data bytes being transmitted/received
#define PAYLOAD_LENGTH   30
//Time (ms) until RX operations timeout
#define RX_TIMEOUT   500

// CubeSat address used for address filtering
#define CUBESAT_ADDRESS     0xCC
/* Femtosat address that CubeSat will use
 * to send data to femtosat
 */
#define FEMTO_ADDRESS      0xBB

// The number of commands to be performed by femtosat
#define NUMBER_OF_COMMANDS      3
// Define the command ID for each command
#define COMMAND_ONE     0x1
#define COMMAND_TWO     0x2
#define COMMAND_THREE   0x3
/* Time (s) the femtosat will be placed in
 * standby mode before it moves onto the next command
 */
#define SLEEP_TIME      0

// Initialise the EasyLink parameters to their default values
EasyLink_Params easyLink_params;

// Variable the RF core uses to time commands
uint32_t absTime;

EasyLink_TxPacket txPacket = {{0}, 0, 0, {0}};
EasyLink_RxPacket rxPacket = {{0}, 0, 0, 0, 0, {0}};

EasyLink_Status result;

// Function prototypes
static void displaySetup();
static void rfPacketSetup();
static void femtosatStatusDisplay(uint8_t femtoRssi, uint8_t statusByte, uint8_t femtoAddr);
static void cubeSatTx();
static bool cubeSatAckRx();
static void dataRx();

// Pin driver handle
static PIN_Handle pinHandle;
static PIN_State pinState;
static Display_Handle display;

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
static void displaySetup()
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
 *  @brief  Setup packet to be sent to CubeSat
 *
 *  @return none
 *
 */
static void rfPacketSetup()
{
    // Initialise variables for RF operations
    EasyLink_Params_init(&easyLink_params);

    // Set up desired quaternion to be sent to femtosat
    float quatFloat[4] = {-1000.0, 2.0, 3.0, -4.0};

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
    if (EasyLink_init(&easyLink_params) != EasyLink_Status_Success)
    {
        while(1);
    }

    /*
     * If you wish to use a frequency other than the default (e.g. 433MHz), use
     * the following API:
     * EasyLink_setFrequency(433000000);
     */

    // Create the packet being sent to the CubeSat

    // The 0th byte must contain the femtosat address
    txPacket.payload[0] = FEMTO_ADDRESS;
    // The next byte tells the femtosat how many commands it must perform
    txPacket.payload[1] = NUMBER_OF_COMMANDS;
    // Fill the next byte with the quaternion command
    txPacket.payload[2] = COMMAND_ONE;
    /* How long for the CubeSat to be in standby mode until
     * the next instruction
     */
    txPacket.payload[3] = SLEEP_TIME;
    // Fill TX packet with quaternion data
    for (i = 4; i < 20; i++)
    {
        txPacket.payload[i] = quatTx[i-4];
    }
    /* If more commands are to be completed after the quaternion command,
     * they are initialised here
     */
    txPacket.payload[20] = COMMAND_TWO;
    txPacket.payload[21] = SLEEP_TIME;
    txPacket.payload[22] = COMMAND_THREE;
    txPacket.payload[23] = SLEEP_TIME;
    // Fill the rest of the packet with 0s
    for(i = 24; i < PAYLOAD_LENGTH; i++)
    {
        txPacket.payload[i] = 0;
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
static void femtosatStatusDisplay(uint8_t femtoRssi, uint8_t statusByte, uint8_t femtoAddr)
{
    Display_printf(display, 0, 0, "0x%x femtosat reception status: ", femtoAddr);
    switch(statusByte)
    {
    case 0:
        Display_printf(display, 0, 0, "successful.\n");
        //Remember to cast RSSI value from unsigned integer to signed integer.
        Display_printf(display, 0, 0, "Data received from CubeSat: %x -> %ddBm.\n", femtoAddr,
                       (int8_t)femtoRssi);
        break;
    case 1:
        Display_printf(display, 0, 0, "timeout error.\n");
        break;
    case 2:
        Display_printf(display, 0, 0, "error.\n");
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
static void cubeSatTx()
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
    //TX to CubeSat failed
    else
    {
        /* Set both LED1 and LED2 to indicate error */
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
static bool cubeSatAckRx()
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
        /* Toggle LED1, clear LED2 to indicate Echo RX */
        PIN_setOutputValue(pinHandle, Board_PIN_LED1,!PIN_getOutputValue(Board_PIN_LED1));
        PIN_setOutputValue(pinHandle, Board_PIN_LED2, 0);
        Display_printf(display, 0, 0, "Ack received from CubeSat.\n");
        ackFlag = true;
    }
    else if(result == EasyLink_Status_Rx_Timeout)
    {
        /* Set LED2 and clear LED1 to indicate Rx Timeout */
        PIN_setOutputValue(pinHandle, Board_PIN_LED2, 1);
        PIN_setOutputValue(pinHandle, Board_PIN_LED1, 0);
        Display_printf(display, 0, 0,
                       "Device timed out before ack received from CubeSat.\n");
        ackFlag = false;
    }
    else
    {
        /* Set both LED1 and LED2 to indicate error */
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
static void dataRx()
{
    /* Stay in RX mode until data is received from CubeSat.
     * Exit loop after a five attempts to restart RF link w/ CubeSat.
     * If this is not performed, there is a chance that the ground station and
     * Cubesat get stuck waiting for each other to send data.
     */
    uint8_t count = 0;
    bool ackFlag = true;
    while(ackFlag && (count < 5))
    {
        // Restart RX mode and wait for data from CubeSat
        rxPacket.absTime = 0;
        // Set timeout interval to 1000ms (1s)
        rxPacket.rxTimeout = EasyLink_ms_To_RadioTime(RX_TIMEOUT * 2);

        result = EasyLink_receive(&rxPacket);
        uint8_t femtoAddr = FEMTO_ADDRESS;

        // No need to check the packet since this is a data transmission
        if (result == EasyLink_Status_Success)
        {
            // Toggle LED1, clear LED2 to indicate Echo RX
            PIN_setOutputValue(pinHandle, Board_PIN_LED1,!PIN_getOutputValue(Board_PIN_LED1));
            PIN_setOutputValue(pinHandle, Board_PIN_LED2, 0);
            // Display correct message about femtosat operation
            femtosatStatusDisplay(rxPacket.payload[0], rxPacket.payload[1], femtoAddr);

            //Exit loop now that data has been received from CubeSat.
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
    // Setup display driver
    displaySetup();
    Display_printf(display, 0, 0, "Starting ground station...\n");

    /* Enter infinite loop which performs all of the
     * necessary RF commands.
     * If the user wishes to only send a command(s) to the femtosat once,
     * this while(1) can be removed.
     * In this case, the user must be aware of the fact that if this TX misses
     * the CubeSat, they can simply press the reset button on the board to
     * resend the command(s).
     */
    rfPacketSetup();

    while(1)
    {

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
        dataRx();

    }
}
