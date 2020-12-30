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

/** ======================================================
 *  @file       rfEasyLinkRx_nortos.c
 *
 *  @brief      Source file for CubeSat
 *
 *  @author     Aymen Benylles
 *  @date       19/12/2020
 *
 *  ======================================================
 */

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

#include "rfEasyLinkRx_nortos.h"

#define PAYLOAD_LENGTH   30 /*!< Length of payload in bytes */

#define GROUND_ADDRESS    0xFF /*!< Ground station address for address filtering */

// Set RX to timeout after 0.5s(500ms)
#define RX_TIMEOUT  500 /*!< RX command times out after 500ms */

// Buffer size used for the file copy process
#define BUFFSIZE 1024 /*!< Buffer size for microSD operations */

// Starting sector to write/read to
#define STARTINGSECTOR 0 /*!< Starting sector to write to/read from */

#define BYTESPERKILOBYTE 1024 /*!< Bytes in a kilobyte */

/* WARNING: Running this example with WRITEENABLE set to 1 will cause
 * any file system present on the SD card to be corrupted!
 */
#define WRITEENABLE 1 /*!< Enable/disable writing to microSD without file system */

// Driver handles
static PIN_Handle pinHandle; /*!< Handle for pin driver */
static PIN_State pinState; /*!< Used for GPIO commands */
static Display_Handle display; /*!< Handle for UART display driver */
SD_Handle sdHandle; /*!< Handle for SD driver */

// SD variables
unsigned char sdPacket[BUFFSIZE]; /*!< Packet to be stored in microSD card */
unsigned char cpyBuff[BUFFSIZE]; /*!< Buffer used to check the success of microSD operation */

EasyLink_Params easyLinkParams; /*!< Use this to initialise EasyLink parameters to their default values */

// RF variables
EasyLink_RxPacket rxPacket = {{0}, 0, 0, 0, 0, {0}};
EasyLink_TxPacket txPacket = {{0}, 0, 0, {0}};
uint32_t absTime; /*!< Used by RF core to time RF commands */
EasyLink_Status result; /*!< Status of RF command */
static bool bBlockTransmit = false; /*!< Flag raised when TX command is to be blocked */

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
 *  @brief  Setup LED and display drivers.
 *
 *  @return none
 *
 */
void driverSetup()
{
    /* Open LED pins */
    pinHandle = PIN_open(&pinState, pinTable);
    if (pinHandle == NULL)
    {
        while(1);
    }

    /* Clear LED pins */
    PIN_setOutputValue(pinHandle, Board_PIN_LED1, 0);
    PIN_setOutputValue(pinHandle, Board_PIN_LED2, 0);

    Display_init();

    /* Open the display for output */
    display = Display_open(Display_Type_UART, NULL);
    if (display == NULL)
    {
        /* Failed to open display driver */
        while (1);
    }

    Display_printf(display, 0, 0, "Starting CubeSat...\n");
}

/**
 *  @brief  Function to initialise the (micro)SD card driver.
 *
 *  @param  rssi        RSSI value of RF link.
 *  @param  errorCode   Error code of operation.
 *
 *  @return sdOpFlag    Flag raised when microSD op fails.
 *
 */
bool sdSetup(int8_t rssi, uint8_t errorCode)
{
    int_fast8_t result;
    //Flag raised in the even of an SD operation failure.
    bool sdOpFlag = 0;
    SD_init();

    Display_printf(display, 0, 0, "Starting the SD setup...\n");

    /* Initialise the array to write to the SD card */
    int i;
    sdPacket[0] = rssi;
    sdPacket[1] = errorCode;
    //Fill the rest of the array with the ground address.
    for (i = 2; i < BUFFSIZE; i++)
    {
        sdPacket[i] = GROUND_ADDRESS;
    }

    /* Mount and register the SD Card */
    sdHandle = SD_open(Board_SD0, NULL);
    if (sdHandle == NULL)
    {
        Display_printf(display, 0, 0, "Error starting the SD card.\n");
        //Raise flag when error occurs.
        sdOpFlag = 1;
    }

    result = SD_initialize(sdHandle);
    if (result != SD_STATUS_SUCCESS)
    {
        Display_printf(display, 0, 0, "Error initialising the SD card.\n");
        //Raise flag when error occurs.
        sdOpFlag = 1;
    }
    //Perform write operation and track it's success.
    sdOpFlag = sdWrite(sdHandle, result, sdOpFlag);
    return sdOpFlag;
}


/**
 *  @brief  Write to (micro)SD card and check operation.
 *
 *  @param sdHandle     SD driver handle
 *  @param result       Result of SD driver initialisation
 *
 *  @return sdFlag      Flag raised in the event of microSD failure.
 *
 */
bool sdWrite(SD_Handle sdHandle, int_fast8_t result, bool sdFailure)
{
    uint_fast32_t sectorSize;
    uint_fast32_t sectors;

    sectorSize = SD_getSectorSize(sdHandle);

    /* Calculate number of sectors taken up by the array by rounding up */
    sectors = (sizeof(sdPacket) + sectorSize - 1) / sectorSize;

#if (WRITEENABLE)
    Display_printf(display, 0, 0, "Writing the array...\n");

    result = SD_write(sdHandle, sdPacket, STARTINGSECTOR, sectors);
    if (result != SD_STATUS_SUCCESS)
    {
        Display_printf(display, 0, 0, "Error writing to the SD card\n");
        sdFailure = 1;
    }
#endif

    Display_printf(display, 0, 0, "Reading the array...\n");
    result = SD_read(sdHandle, cpyBuff, STARTINGSECTOR, sectors);
    if (result != SD_STATUS_SUCCESS)
    {
        Display_printf(display, 0, 0, "Error reading from the SD card\n");
        sdFailure = 1;
    }

    // Compare data read from the SD card with expected values
    int i;
    for (i = 0; i < BUFFSIZE; i++)
    {
        if (cpyBuff[i] != sdPacket[i])
        {
            sdFailure = 1;
            Display_printf(display, 0, 0,
                    "Data read from SD card differed from expected value\n");
            Display_printf(display, 0, 0,
                    "    Expected value for index %d: %d, got %d\n", i,
                    sdPacket[i], cpyBuff[i]);
            Display_printf(display, 0, 0, "Run the example with WRITEENABLE "
                    "= 1 to write expected values to the SD card\n");
            break;
        }
    }

    if (i == BUFFSIZE)
    {
        Display_printf(display, 0, 0,
                "Data read from SD card matched expected values\n");
        //Make sure to convert from unsigned char to signed integer to display RSS correctly.
        Display_printf(display, 0, 0, "Data from SD card: %d\n", (int8_t)sdPacket[0]);
    }

    SD_close(sdHandle);
    return sdFailure;
}

/**
 *  @brief  Enter RX mode to get commands from ground station
 *
 *  @return none
 *
 */
void commandRx()
{
    rxPacket.absTime = 0;
    rxPacket.rxTimeout = EasyLink_ms_To_RadioTime(RX_TIMEOUT * 3);
    EasyLink_Status result = EasyLink_receive(&rxPacket);

    if (result == EasyLink_Status_Success)
    {
        // Toggle LED2 to indicate RX, clear LED1
        PIN_setOutputValue(pinHandle, Board_PIN_LED2,!PIN_getOutputValue(Board_PIN_LED2));
        PIN_setOutputValue(pinHandle, Board_PIN_LED1, 0);
        // Copy contents of RX packet to TX packet
        memcpy(&txPacket.payload, &rxPacket.payload, rxPacket.len);
        Display_printf(display, 0, 0, "Command received from ground station.\n");
        // Permit echo transmission
        bBlockTransmit = false;
    }
    else if(result == EasyLink_Status_Rx_Timeout)
    {
        // Set LED1 and clear LED2 to indicate timeout
        PIN_setOutputValue(pinHandle, Board_PIN_LED1, 1);
        PIN_setOutputValue(pinHandle, Board_PIN_LED2, 0);
        //Notify user that CubeSat has not received command yet.
        Display_printf(display, 0, 0, "Timeout error: waiting for command from ground station...\n");
        // Block echo transmission
        bBlockTransmit = true;
    }
    else
    {
        // Set LED1 and clear LED2 to indicate timeout
        PIN_setOutputValue(pinHandle, Board_PIN_LED1, 1);
        PIN_setOutputValue(pinHandle, Board_PIN_LED2, 0);
        //Notify user that CubeSat has not received command yet.
        Display_printf(display, 0, 0, "Error receiving command from ground station.\n");
        // Block echo transmission
        bBlockTransmit = true;
    }
}

/**
 *  @brief  Echo the packet from the ground station (ack)
 *
 *  @return none
 *
 */
void groundStationAckTx()
{
    txPacket.len = PAYLOAD_LENGTH;

    // Send packet back to ground station for ack
    txPacket.dstAddr[0] = GROUND_ADDRESS;

    // Set Tx absolute time to current time + 100ms
    if(EasyLink_getAbsTime(&absTime) != EasyLink_Status_Success)
    {
        // Problem getting absolute time
    }
    txPacket.absTime = absTime;

    EasyLink_Status result = EasyLink_transmit(&txPacket);

    if (result == EasyLink_Status_Success)
    {
        // Toggle LED2 to indicate Echo TX, clear LED1
        PIN_setOutputValue(pinHandle, Board_PIN_LED2,!PIN_getOutputValue(Board_PIN_LED2));
        PIN_setOutputValue(pinHandle, Board_PIN_LED1, 0);
        Display_printf(display, 0, 0, "Ack sent to ground station.\n");
    }
    else
    {
        // Set LED1 and clear LED2 to indicate error
        PIN_setOutputValue(pinHandle, Board_PIN_LED1, 1);
        PIN_setOutputValue(pinHandle, Board_PIN_LED2, 0);
        Display_printf(display, 0, 0, "CubeSat TX error.\n");
    }
}

/**
 *  @brief  Perform TX command to send commands to femtosat
 *
 *  @return none
 *
 */
void commandTx()
{
    // Now that ack has been sent, relay data to femtosat
    txPacket.dstAddr[0] = txPacket.payload[0];

    result = EasyLink_transmit(&txPacket);
    if (result == EasyLink_Status_Success)
    {
        PIN_setOutputValue(pinHandle, Board_PIN_LED2,!PIN_getOutputValue(Board_PIN_LED2));
        PIN_setOutputValue(pinHandle, Board_PIN_LED1, 0);
        Display_printf(display, 0, 0, "CubeSat TX to femtosat successful.\n");
    }
    else
    {
        // Set LED1 and clear LED2 to indicate error
        PIN_setOutputValue(pinHandle, Board_PIN_LED1, 1);
        PIN_setOutputValue(pinHandle, Board_PIN_LED2, 0);
        Display_printf(display, 0, 0, "CubeSat TX error.\n");
    }
}

/**
 *  @brief  Enter RX mode to receive ack from femtosat
 *
 *  @return none
 *
 */
void femtosatAckRx()
{
    // Switch to Receiver, set a timeout interval of 500ms */
    rxPacket.absTime = 0;
    rxPacket.rxTimeout = EasyLink_ms_To_RadioTime(RX_TIMEOUT);
    uint8_t error;
    bool sdFlag;

    result = EasyLink_receive(&rxPacket);

    /* Check Received packet against what was sent, it should be identical
     * to the transmitted packet
     */
    if (result == EasyLink_Status_Success &&
            isPacketCorrect(&rxPacket, &txPacket))
    {
        // Successful RX.
        error = 0;
        // Toggle LED1, clear LED2 to indicate Echo RX
        PIN_setOutputValue(pinHandle, Board_PIN_LED1,!PIN_getOutputValue(Board_PIN_LED1));
        PIN_setOutputValue(pinHandle, Board_PIN_LED2, 0);

        Display_printf(display, 0, 0, "Ack received from femtosat.\n");
        Display_printf(display, 0, 0, "%x RSSI: %ddBm\n", txPacket.dstAddr[0], rxPacket.rssi);
        // Log successful exchange in microSD card
        sdFlag = sdSetup(rxPacket.rssi, error);
    }
    else if (result == EasyLink_Status_Rx_Timeout)
    {
        // Set LED2 and clear LED1 to indicate Rx Timeout
        PIN_setOutputValue(pinHandle, Board_PIN_LED2, 1);
        PIN_setOutputValue(pinHandle, Board_PIN_LED1, 0);
        Display_printf(display, 0, 0, "Device timed out before ack received from femtosat.\n");
        // Log timeout error in microSD card.
        error = 1;
        // There will be no RSSI since RX not successful, pass default value to function
        sdFlag = sdSetup(0, error);
    }
    else
    {
        // Set both LED1 and LED2 to indicate error
        PIN_setOutputValue(pinHandle, Board_PIN_LED1, 1);
        PIN_setOutputValue(pinHandle, Board_PIN_LED2, 1);
        Display_printf(display, 0, 0, "Error.\n");
        // Log error in microSD card
        error = 2;
        sdFlag = sdSetup(0, error);
    }

    // Set up packet to be transmitted to ground station
    txPacket.dstAddr[0] = GROUND_ADDRESS;
    // If sdFlag is not raised, use value read from microSD card
    if(!sdFlag)
    {
        // Send RSSI data stored in first byte of sdPacket
        txPacket.payload[0] = (uint8_t)cpyBuff[0];
        // Send status stored in second byte of sdPacket
        txPacket.payload[1] = cpyBuff[1];
    }
    // If flag is raised, use locally stored values
    else
    {
        //RSSI value
        txPacket.payload[0] = (uint8_t)rxPacket.rssi;
        //Status of RSSI operation
        txPacket.payload[1] = error;
    }
}

/**
 *  @brief  Enter TX mode and send data to ground station
 *
 *  @return none
 *
 */
void dataTx()
{
    result = EasyLink_transmit(&txPacket);
    if (result == EasyLink_Status_Success)
    {
        // Toggle LED2 and clear LED 1 to indicate success
        PIN_setOutputValue(pinHandle, Board_PIN_LED2,!PIN_getOutputValue(Board_PIN_LED2));
        PIN_setOutputValue(pinHandle, Board_PIN_LED1, 0);
        Display_printf(display, 0, 0, "CubeSat TX to ground station successful.\n");
    }
    else
    {
        // Set LED1 and clear LED2 to indicate error
        PIN_setOutputValue(pinHandle, Board_PIN_LED1, 1);
        PIN_setOutputValue(pinHandle, Board_PIN_LED2, 0);
        Display_printf(display, 0, 0, "CubeSat TX error.\n");
    }
}


/**
 *  @brief  Check that received packet is the same as transmitted packet
 *
 *  @param *rxp     Pointer to RX packet.
 *  @param *txp     Pointer to TX packet.
 *
 *  @return status  Flag pulled low when RX and TX packets are not the same
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

void *mainThread(void *arg0)
{
    driverSetup();
    EasyLink_Params_init(&easyLinkParams);

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
     * If you wish to use a frequency other than the default, use
     * the following API:
     * EasyLink_setFrequency(868000000);
     *
     * When the final PC/104 board has been made for this project,
     * this must be set to 433MHz (i.e. 433000000).
     * This version of the code uses 868MHz since the dev kit for
     * the CC1310 has an 868MHz antenna, not a 433MHz antenna.
     */

    while(1)
    {
        /* RX mode:
         * Get commands from ground station
         */
        commandRx();

        if(bBlockTransmit == false)
        {
            /* TX mode:
             * Send ack to ground station
             */
            groundStationAckTx();

            /* TX mode:
             * Send command(s) to femtosat
             */
            commandTx();

            /* Rx mode:
             * Get ack from femtosat
             */
            femtosatAckRx();

            /* TX mode:
             * Send data to ground station
             */
            dataTx();
        }
    }
}
