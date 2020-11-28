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
 *  ======== rfEasyLinkEchoRx_nortos.c ========
 */
/* Standard C Libraries */
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

/* Undefine to not use async mode */
//#define RFEASYLINKECHO_ASYNC

#define RFEASYLINKECHO_PAYLOAD_LENGTH     30

#define CUBESAT_ADDRESS  0xCC

#define BUFF_SIZE   1024

static void displaySetup();
static void dummyCommand(uint8_t commandID, uint8_t rxPacket[30], uint8_t sleepTime);
static void displayQuat(uint8_t rxPacket[30]);

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

/**
 *  @brief  Perform dummyCommand
 *
 *  @param commandID  Specific command to be performed.
 *  @return none
 *
 */
static void dummyCommand(uint8_t commandID, uint8_t rxPacket[30], uint8_t sleepTime)
{
    switch(commandID)
    {
    case 0x1:
        //RSSI command -> do nothing.
        break;
    case 0x2:
        //Set green LED high for 2 seconds.
        PIN_setOutputValue(pinHandle, Board_PIN_LED1, 1);
        sleep(2);
        PIN_setOutputValue(pinHandle, Board_PIN_LED1, 0);
        break;
    case 0x3:
        displayQuat(rxPacket);
        break;
    case 0x4:
        // Place any new commands here
        break;
    default:
        break;
    }
    sleep(sleepTime);
}

/**
 *  @brief  Recombine RX packet elements to obtain desired quaternion.
 *
 *  @param quatRx  Quaternion bytes
 *  @return none
 *
 */
static void displayQuat(uint8_t rxPacket[30])
{
    uint8_t i, quatRx[16];
    int quatInt[4];
    float quatFloat[4];

    // Get quaternion data from RX packet
    for(i = 0; i < 16; i++)
    {
        quatRx[i] = rxPacket[i+4];
    }

    // Combine all 4 bytes from RX packet to get correct quaternion format
    for(i = 0; i < 4; i++)
    {
        quatInt[i] = (quatRx[i*4] << 24) | (quatRx[(i*4)+1] << 16) | (quatRx[(i*4)+2] << 8) | (quatRx[(i*4)+3]);
        quatFloat[i] = quatInt[i] / 1000;
    }

    //Display the final, recovered quaternion to the user
    Display_printf(display, 0, 0, "Quaternion: {%f, %f, %f, %f}\n",
                   quatFloat[0], quatFloat[1], quatFloat[2], quatFloat[3]);

}

void *mainThread(void *arg0)
{
    uint32_t absTime;
    static volatile bool bEchoDoneFlag;
    static bool bBlockTransmit = false;
    EasyLink_RxPacket rxPacket = {{0}, 0, 0, 0, 0, {0}};
    EasyLink_TxPacket txPacket = {{0}, 0, 0, {0}};

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
    Display_printf(display, 0, 0, "Starting femtosat...\n");

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

    while(1)
    {
        rxPacket.absTime = 0;
        EasyLink_Status result = EasyLink_receive(&rxPacket);

        if (result == EasyLink_Status_Success)
        {
            /* Toggle LED2 to indicate RX, clear LED1 */
            PIN_setOutputValue(pinHandle, Board_PIN_LED2,!PIN_getOutputValue(Board_PIN_LED2));
            PIN_setOutputValue(pinHandle, Board_PIN_LED1, 0);
            /* Copy contents of RX packet to TX packet */
            memcpy(&txPacket.payload, &rxPacket.payload, rxPacket.len);
            /* Permit echo transmission */
            bBlockTransmit = false;
            Display_printf(display, 0, 0, "Packet received from CubeSat.\n");
        }
        else
        {
            /* Set LED1 and clear LED2 to indicate error */
            PIN_setOutputValue(pinHandle, Board_PIN_LED1, 1);
            PIN_setOutputValue(pinHandle, Board_PIN_LED2, 0);
            Display_printf(display, 0, 0, "Packet not received from CubeSat.\n");
            /* Block echo transmission */
            bBlockTransmit = true;

        }

        if(bBlockTransmit == false)
        {
            /* Switch to Transmitter and echo the packet if transmission
             * is not blocked
             */
            txPacket.len = RFEASYLINKECHO_PAYLOAD_LENGTH;

            /*
             * Address filtering is enabled by default on the Rx device with the
             * an address of 0xAA. This device must set the dstAddr accordingly.
             */
            txPacket.dstAddr[0] = CUBESAT_ADDRESS;

            /* Set Tx absolute time to current time + 100ms*/
            if(EasyLink_getAbsTime(&absTime) != EasyLink_Status_Success)
            {
                // Problem getting absolute time
            }
            txPacket.absTime = absTime + EasyLink_ms_To_RadioTime(100);

            EasyLink_Status result = EasyLink_transmit(&txPacket);

            if (result == EasyLink_Status_Success)
            {
                /* Toggle LED2 to indicate Echo TX, clear LED1 */
                PIN_setOutputValue(pinHandle, Board_PIN_LED2,!PIN_getOutputValue(Board_PIN_LED2));
                PIN_setOutputValue(pinHandle, Board_PIN_LED1, 0);
                Display_printf(display, 0, 0, "Ack sent to CubeSat.\n");

                //Perform command(s) sent by CubeSat.
                uint8_t i;
                uint8_t commands = rxPacket.payload[1];
                /* Set up for-loop so that it iterates through the array
                 * the correct number of times.
                 */
                uint8_t loopSize = (commands * 2) + 2;
                for(i = 2; i < loopSize; i++)
                {
                    //Command IDs are only found on every other element (starting from element 2).
                    if(!(i % 2))
                    {
                        Display_printf(display, 0, 0, "Performing command: %x...\n", rxPacket.payload[i]);
                        dummyCommand(rxPacket.payload[i], rxPacket.payload, rxPacket.payload[i+1]);
                    }
                }
            }
            else
            {
                /* Set LED1 and clear LED2 to indicate error */
                PIN_setOutputValue(pinHandle, Board_PIN_LED1, 1);
                PIN_setOutputValue(pinHandle, Board_PIN_LED2, 0);
                Display_printf(display, 0, 0, "Femtosat error.\n");
            }
        }
    }
}
