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
 *  ======== rfEasyLinkEchoTx_nortos.c ========
 */
 /* Standard C Libraries */
#include <stdlib.h>

/* TI-RTOS Header files */
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/timer/GPTimerCC26XX.h>
#include <ti/display/Display.h>
#include <ti/devices/DeviceFamily.h>

/* Driverlib APIs */
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)

/* Board Header files */
#include "Board.h"

/* Application Header files */
#include "smartrf_settings/smartrf_settings.h"

/* EasyLink API Header files */
#include "easylink/EasyLink.h"

#define RUN_STOP    0
#define RFEASYLINKECHO_PAYLOAD_LENGTH   30
#define RX_TIMEOUT   500

#define CUBESAT_ADDRESS     0xCC
#define FEMTO_ADDRESS      0xBB

#define NUMBER_OF_COMMANDS      1
#define COMMAND_ONE     0x3
#define COMMAND_TWO     0x2
#define SLEEP_TIME      0

//Function prototypes
static void displaySetup();

/* Pin driver handle */
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
 *  @brief  Setup display driver.
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
    uint32_t absTime;
    static volatile bool bEchoDoneFlag;
    EasyLink_TxPacket txPacket = {{0}, 0, 0, {0}};
    EasyLink_RxPacket rxPacket = {{0}, 0, 0, 0, 0, {0}};

    // Set up desired quaternion to be sent to femtosat
    float quatFloat[4] = {1.0, 0.0, 0.0, 0.0};
    int i, quatInt[4];
    uint8_t quatTx[16];

    for(i = 0; i < 4; i++)
    {
        quatInt[i] = quatFloat[i] * 1000;
    }

    /* Split 4-bye integer into 4 individual bytes.
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

    /* Open LED pins */
    pinHandle = PIN_open(&pinState, pinTable);
    if (pinHandle == NULL)
    {
        while(1);
    }

    /* Clear LED pins */
    PIN_setOutputValue(pinHandle, Board_PIN_LED1, 0);
    PIN_setOutputValue(pinHandle, Board_PIN_LED2, 0);

    displaySetup();
    Display_printf(display, 0, 0, "Starting ground station...\n");

    // Initialize the EasyLink parameters to their default values
    EasyLink_Params easyLink_params;
    EasyLink_Params_init(&easyLink_params);

    /*
     * Initialize EasyLink with the settings found in easylink_config.h
     * Modify EASYLINK_PARAM_CONFIG in easylink_config.h to change the default
     * PHY
     */
    if (EasyLink_init(&easyLink_params) != EasyLink_Status_Success)
    {
        while(1);
    }

    /*
     * If you wish to use a frequency other than the default, use
     * the following API:
     * EasyLink_setFrequency(868000000);
     */

    // Packet Originator
    while(1)
    {
        /* Create packet with incrementing sequence number and random payload */
        txPacket.payload[0] = FEMTO_ADDRESS;
        txPacket.payload[1] = NUMBER_OF_COMMANDS;
        txPacket.payload[2] = COMMAND_ONE;
        txPacket.payload[3] = SLEEP_TIME;
        // Fill TX packet with quaternion data
        for (i = 4; i < 23; i++)
        {
            txPacket.payload[i] = quatTx[i-4];
        }

        txPacket.len = RFEASYLINKECHO_PAYLOAD_LENGTH;

        /*
         * Address filtering is enabled by default on the Rx device with the
         * an address of 0xAA. This device must set the dstAddr accordingly.
         */
        txPacket.dstAddr[0] = CUBESAT_ADDRESS;

        /* Set Tx absolute time to current time + 1000ms */
        if(EasyLink_getAbsTime(&absTime) != EasyLink_Status_Success)
        {
            // Problem getting absolute time
        }
        txPacket.absTime = absTime + EasyLink_ms_To_RadioTime(100);
        EasyLink_Status result = EasyLink_transmit(&txPacket);

        if (result == EasyLink_Status_Success)
        {
            /* Toggle LED1 to indicate TX */
            PIN_setOutputValue(pinHandle, Board_PIN_LED1,!PIN_getOutputValue(Board_PIN_LED1));
            /* Turn LED2 off, in case there was a prior error */
            PIN_setOutputValue(pinHandle, Board_PIN_LED2, 0);
            Display_printf(display, 0, 0, "Ground station TX successful.\n");
        }
        else
        {
            /* Set both LED1 and LED2 to indicate error */
            PIN_setOutputValue(pinHandle, Board_PIN_LED1, 1);
            PIN_setOutputValue(pinHandle, Board_PIN_LED2, 1);
            Display_printf(display, 0, 0, "Ground station TX error.\n");
        }

        /* Switch to Receiver, set a timeout interval of 500ms */
        rxPacket.absTime = 0;
        rxPacket.rxTimeout = EasyLink_ms_To_RadioTime(RX_TIMEOUT);
        result = EasyLink_receive(&rxPacket);
        //Flag indicating that ack has been received from CubeSat.
        //When raised, data can be received.
        //When held low, ground station must retransmit data to CubeSat.
        bool ackFlag;

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
            Display_printf(display, 0, 0,
                           "RX error.\n");
            ackFlag = false;
        }

        //Stay in RX mode until data is received from CubeSat.
        /* Exit loop after a five attempts to restart RF link w/ CubeSat.
        If this is not performed, there is a chance that the ground station and
        Cubesat get stuck waiting for each other to send data.*/
        uint8_t count = 0;
        while(ackFlag && (count < 5))
        {
            //Restart RX mode and wait for data from CubeSat.
            rxPacket.absTime = 0;
            rxPacket.rxTimeout = EasyLink_ms_To_RadioTime(RX_TIMEOUT * 2);
            result = EasyLink_receive(&rxPacket);
            uint8_t femtoAddr = FEMTO_ADDRESS;

            //No need to check the packet since this is a data transmission.
            if (result == EasyLink_Status_Success)
            {
                /* Toggle LED1, clear LED2 to indicate Echo RX */
                PIN_setOutputValue(pinHandle, Board_PIN_LED1,!PIN_getOutputValue(Board_PIN_LED1));
                PIN_setOutputValue(pinHandle, Board_PIN_LED2, 0);
                Display_printf(display, 0, 0, "0x%x femtosat reception status: ", femtoAddr);
                //Display correct message about femtosat operation.
                switch(rxPacket.payload[1])
                {
                case 0:
                    Display_printf(display, 0, 0, "successful.\n");
                    //Remember to cast RSSI value from unsigned integer to signed integer.
                    Display_printf(display, 0, 0, "Data received from CubeSat: %x -> %ddBm.\n", femtoAddr,
                                   (int8_t)rxPacket.payload[0]);
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

                //Exit loop now that data has been received from CubeSat.
                ackFlag = false;
            }
            else if (result == EasyLink_Status_Rx_Timeout)
            {
                /* Set LED2 and clear LED1 to indicate RX Timeout */
                PIN_setOutputValue(pinHandle, Board_PIN_LED2, 1);
                PIN_setOutputValue(pinHandle, Board_PIN_LED1, 0);
                Display_printf(display, 0, 0,
                               "Device timed out before data received from CubeSat.\n");
                //Stay in loop until data is received from CubeSat.
                ackFlag = true;
            }
            else
            {
                /* Set both LED1 and LED2 to indicate error */
                PIN_setOutputValue(pinHandle, Board_PIN_LED1, 1);
                PIN_setOutputValue(pinHandle, Board_PIN_LED2, 1);
                Display_printf(display, 0, 0,
                               "RX error.\n");
                //Stay in loop until data has been received from CubeSat.
                ackFlag = true;
            }
            count++;
        }
    }
}
