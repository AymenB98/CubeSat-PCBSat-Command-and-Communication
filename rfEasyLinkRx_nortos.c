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

#define GROUND_ADDRESS    0xFF;

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

static volatile bool bEchoDoneFlag;

static bool bBlockTransmit = false;

EasyLink_TxPacket txPacket = {{0}, 0, 0, {0}};

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

#ifdef RFEASYLINKECHO_ASYNC
void echoTxDoneCb(EasyLink_Status status)
{
    if (status == EasyLink_Status_Success)
    {
        /* Toggle LED2 to indicate Echo TX, clear LED1 */
        PIN_setOutputValue(pinHandle, Board_PIN_LED2,!PIN_getOutputValue(Board_PIN_LED2));
        PIN_setOutputValue(pinHandle, Board_PIN_LED1, 0);
    }
    else
    {
        /* Set LED1 and clear LED2 to indicate error */
        PIN_setOutputValue(pinHandle, Board_PIN_LED1, 1);
        PIN_setOutputValue(pinHandle, Board_PIN_LED2, 0);
    }

    bEchoDoneFlag = true;
}

void echoRxDoneCb(EasyLink_RxPacket * rxPacket, EasyLink_Status status)
{
    if (status == EasyLink_Status_Success)
    {
        /* Toggle LED2 to indicate RX, clear LED1 */
        PIN_setOutputValue(pinHandle, Board_PIN_LED2,!PIN_getOutputValue(Board_PIN_LED2));
        PIN_setOutputValue(pinHandle, Board_PIN_LED1, 0);
        /* Copy contents of RX packet to TX packet */
        memcpy(&txPacket.payload, rxPacket->payload, rxPacket->len);
        /* Permit echo transmission */
        bBlockTransmit = false;
    }
    else
    {
        /* Set LED1 and clear LED2 to indicate error */
        PIN_setOutputValue(pinHandle, Board_PIN_LED1, 1);
        PIN_setOutputValue(pinHandle, Board_PIN_LED2, 0);
        /* Block echo transmission */
        bBlockTransmit = true;
    }

    bEchoDoneFlag = true;
}
#endif //RFEASYLINKECHO_ASYNC

void *mainThread(void *arg0)
{
    uint32_t absTime;
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
    Display_printf(display, 0, 0, "Starting CubeSat...\n");

#ifndef RFEASYLINKECHO_ASYNC
    EasyLink_RxPacket rxPacket = {{0}, 0, 0, 0, 0, {0}};
#endif //RFEASYLINKECHO_ASYNC

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

    while(1) {
#ifdef RFEASYLINKECHO_ASYNC
        // Set the echo done flag to false, callback will
        // set it to true
        bEchoDoneFlag = false;

        // Wait to receive a packet
        EasyLink_receiveAsync(echoRxDoneCb, 0);

        /* Wait indefinitely for Rx */
        while(bEchoDoneFlag == false){
            bool previousHwiState = IntMasterDisable();
            /*
             * Tricky IntMasterDisable():
             * true  : Interrupts were already disabled when the function was
             *         called.
             * false : Interrupts were enabled and are now disabled.
             */
            IntMasterEnable();
            Power_idleFunc();
            IntMasterDisable();

            if(!previousHwiState)
            {
                IntMasterEnable();
            }
        };
#else
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
        }
        else
        {
            /* Set LED1 and clear LED2 to indicate error */
            PIN_setOutputValue(pinHandle, Board_PIN_LED1, 1);
            PIN_setOutputValue(pinHandle, Board_PIN_LED2, 0);
            /* Block echo transmission */
            bBlockTransmit = true;
        }
#endif // RFEASYLINKECHO_ASYNC

        if(bBlockTransmit == false)
        {
            /* Switch to Transmitter and echo the packet if transmission
             * is not blocked
             */
            txPacket.len = RFEASYLINKECHO_PAYLOAD_LENGTH;

            //Send packet back to ground station for ack.
            txPacket.dstAddr[0] = GROUND_ADDRESS;

            /* Set Tx absolute time to current time + 100ms*/
            if(EasyLink_getAbsTime(&absTime) != EasyLink_Status_Success)
            {
                // Problem getting absolute time
            }
            txPacket.absTime = absTime + EasyLink_ms_To_RadioTime(100);

#ifdef RFEASYLINKECHO_ASYNC
            // Set the echo done flag to false, callback will
            // set it to true
            bEchoDoneFlag = false;
            EasyLink_transmitAsync(&txPacket, echoTxDoneCb);

            /* Wait for Tx to complete. A Successful TX will cause the echoTxDoneCb
             * to be called and the bEchoDoneFlag to be set
             */
            while(bEchoDoneFlag == false){
                bool previousHwiState = IntMasterDisable();
                /*
                 * Tricky IntMasterDisable():
                 * true  : Interrupts were already disabled when the function was
                 *         called.
                 * false : Interrupts were enabled and are now disabled.
                 */
                IntMasterEnable();
                Power_idleFunc();
                IntMasterDisable();

                if(!previousHwiState)
                {
                    IntMasterEnable();
                }
            };


#else
            EasyLink_Status result = EasyLink_transmit(&txPacket);

            if (result == EasyLink_Status_Success)
            {
                /* Toggle LED2 to indicate Echo TX, clear LED1 */
                PIN_setOutputValue(pinHandle, Board_PIN_LED2,!PIN_getOutputValue(Board_PIN_LED2));
                PIN_setOutputValue(pinHandle, Board_PIN_LED1, 0);
                Display_printf(display, 0, 0, "Ack sent to ground station.\n");
            }
            else
            {
                /* Set LED1 and clear LED2 to indicate error */
                PIN_setOutputValue(pinHandle, Board_PIN_LED1, 1);
                PIN_setOutputValue(pinHandle, Board_PIN_LED2, 0);
                Display_printf(display, 0, 0, "CubeSat TX error.\n");
            }
            //Now that ack has been sent, relay data to femtosat
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
                /* Set LED1 and clear LED2 to indicate error */
                PIN_setOutputValue(pinHandle, Board_PIN_LED1, 1);
                PIN_setOutputValue(pinHandle, Board_PIN_LED2, 0);
                Display_printf(display, 0, 0, "CubeSat TX error.\n");
            }

            /* Switch to Receiver, set a timeout interval of 500ms */
            rxPacket.absTime = 0;
            rxPacket.rxTimeout = EasyLink_ms_To_RadioTime(500);
            result = EasyLink_receive(&rxPacket);

            /* Check Received packet against what was sent, it should be identical
             * to the transmitted packet
             */
            if (result == EasyLink_Status_Success &&
                    isPacketCorrect(&rxPacket, &txPacket))
            {
                /* Toggle LED1, clear LED2 to indicate Echo RX */
                PIN_setOutputValue(pinHandle, Board_PIN_LED1,!PIN_getOutputValue(Board_PIN_LED1));
                PIN_setOutputValue(pinHandle, Board_PIN_LED2, 0);
                Display_printf(display, 0, 0, "Ack received from femtosat.\n");
            }
            else if (result == EasyLink_Status_Rx_Timeout)
            {
                /* Set LED2 and clear LED1 to indicate Rx Timeout */
                PIN_setOutputValue(pinHandle, Board_PIN_LED2, 1);
                PIN_setOutputValue(pinHandle, Board_PIN_LED1, 0);
                Display_printf(display, 0, 0, "Device timed out before ack received from femtosat.\n");
            }
            else
            {
                /* Set both LED1 and LED2 to indicate error */
                PIN_setOutputValue(pinHandle, Board_PIN_LED1, 1);
                PIN_setOutputValue(pinHandle, Board_PIN_LED2, 1);
                Display_printf(display, 0, 0, "Error.\n");
            }

#endif //RFEASYLINKECHO_ASYNC
        }
    }
}
