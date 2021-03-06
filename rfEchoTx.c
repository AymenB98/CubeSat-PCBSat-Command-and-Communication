/*
 * Copyright (c) 2019, Texas Instruments Incorporated
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

/***** Includes *****/
/* Standard C Libraries */
#include <stdlib.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

/* TI Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/SD.h>
#include <ti/display/Display.h>
#include <ti/drivers/Power.h>

/* Driverlib Header files */
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)

/* Board Header files */
#include "Board.h"

/* Application Header files */
#include "RFQueue.h"
#include "smartrf_settings/smartrf_settings.h"

/***** Defines *****/
/* Packet TX/RX Configuration */
#define PAYLOAD_LENGTH      48
/* Set packet interval to 1000ms */
#define PACKET_INTERVAL     (uint32_t)(4000000*1.0f)
/* Set Receive timeout to 500ms */
#define RX_TIMEOUT          (uint32_t)(4000000*0.5f)
/* NOTE: Only two data entries supported at the moment */
#define NUM_DATA_ENTRIES    2
/* The Data Entries data field will contain:
 * 1 Header byte (RF_cmdPropRx.rxConf.bIncludeHdr = 0x1)
 * Max 30 payload bytes
 * 1 status byte (RF_cmdPropRx.rxConf.bAppendStatus = 0x1) */
#define NUM_APPENDED_BYTES  2

/* Buffer size used for the file copy process */
#define BUFFSIZE 1024

 /* Starting sector to write/read to */
#define STARTINGSECTOR 0

#define BYTESPERKILOBYTE 1024

#define DOWN_TIME 5

#define MAX_INSTRUCTIONS    14
#define INSTRUCTION_COUNT   1

#define QUAT_TEST 0
/*
 * Set this constant to 1 in order to write to the SD card.
 * WARNING: Running this example with WRITEENABLE set to 1 will cause
 * any filesystem present on the SD card to be corrupted!
 */
#define WRITEENABLE 1

static Display_Handle display;

unsigned char sdPacket[BUFFSIZE];

unsigned char cpy_buff[BUFFSIZE];

/* Log radio events in the callback */
//#define LOG_RADIO_EVENTS

/***** Prototypes *****/
static void echoCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);
static void sdSetup();
static void sdWrite(SD_Handle sdHandle, int_fast8_t result);
static void rfSleep(uint8_t delayTime);
static void timeoutError();
static void genError();
static void ackError();
static void txSuccess();
static void dataSuccess();
static void displaySetup();
static void ledSetup();
static void rfSetup();
static void dummyCommand(uint8_t command, uint8_t commandNumber, uint8_t totalCommands);
static void greenBlinky();
static void redBlinky();
static void getRssi();
static void customStandby(uint8_t sleepTimeMins);
static void setQuat(uint8_t packet[45]);
static void commandDone();

/***** Variable declarations *****/
static RF_Object rfObject;
static RF_Handle rfHandle;
SD_Handle     sdHandle;

//Variables for SD operations
int_fast8_t   result;
uint_fast32_t cardCapacity;

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

/* Buffer which contains all Data Entries for receiving data.
 * Pragmas are needed to make sure this buffer is aligned to a 4 byte boundary
 * (requirement from the RF core)
 */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(rxDataEntryBuffer, 4)
static uint8_t
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  PAYLOAD_LENGTH,
                                                  NUM_APPENDED_BYTES)];
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment = 4
static uint8_t
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  PAYLOAD_LENGTH,
                                                  NUM_APPENDED_BYTES)];
#elif defined(__GNUC__)
static uint8_t
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  PAYLOAD_LENGTH,
                                                  NUM_APPENDED_BYTES)]
                                                  __attribute__((aligned(4)));
#else
#error This compiler is not supported
#endif //defined(__TI_COMPILER_VERSION__)

/* Receive Statistics */
static rfc_propRxOutput_t rxStatistics;

/* Receive dataQueue for RF Core to fill in data */
static dataQueue_t dataQueue;
static rfc_dataEntryGeneral_t* currentDataEntry;
static uint8_t packetLength;
static uint8_t* packetDataPointer;
static int* quatPacketDataPointer;

static uint8_t txPacket[PAYLOAD_LENGTH];
static int txQuatPacket[PAYLOAD_LENGTH];
static uint8_t rxPacket[PAYLOAD_LENGTH + NUM_APPENDED_BYTES - 1];

static volatile bool bRxSuccess = false;

#ifdef LOG_RADIO_EVENTS
static volatile RF_EventMask eventLog[32];
static volatile uint8_t evIndex = 0;
#endif // LOG_RADIO_EVENTS

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config pinTable[] =
{
#if defined(Board_CC1350_LAUNCHXL)
 Board_DIO30_SWPWR | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#endif
 Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
 Board_PIN_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
 PIN_TERMINATE
};

//Enum type "cubeState" moves CubeSat into different states of operation.
enum cubeState
{
    ACK_PENDING,
    ACK_RECEIVED,
    DATA_PENDING,
    DATA_RECEIVED,
};

/***** Function definitions *****/

/**
 *  @brief  Function to initialise the (micro)SD card driver.
 *
 *  @return none
 *  @remark This function also setups up the display driver.
 *
 */
static void sdSetup()
{
    int_fast8_t   result;
    SD_init();

    Display_printf(display, 0, 0, "Starting the SD setup...\n");

    /* Initialise the array to write to the SD card */
    int i;
    for (i = 0; i < BUFFSIZE; i++)
    {
        sdPacket[i] = 0xA;
    }

    /* Mount and register the SD Card */
    sdHandle = SD_open(Board_SD0, NULL);
    if (sdHandle == NULL)
    {
        Display_printf(display, 0, 0, "Error starting the SD card.\n");
        while (1);
    }

    result = SD_initialize(sdHandle);
    if (result != SD_STATUS_SUCCESS)
    {
        Display_printf(display, 0, 0, "Error initialising the SD card.\n");
        while (1);
    }
    sdWrite(sdHandle, result);
}

/**
 *  @brief  Write to (micro)SD card and check operation.
 *
 *  @param sdHnalde     SD driver handle
 *  @param result       Result of SD driver initialisation
 *
 *  @return none
 *
 */
static void sdWrite(SD_Handle sdHandle, int_fast8_t result)
{
    uint_fast32_t totalSectors;
    uint_fast32_t sectorSize;
    uint_fast32_t sectors;

    totalSectors = SD_getNumSectors(sdHandle);
    sectorSize = SD_getSectorSize(sdHandle);
    cardCapacity = (totalSectors / BYTESPERKILOBYTE) * sectorSize;

    /* Calculate number of sectors taken up by the array by rounding up */
    sectors = (sizeof(sdPacket) + sectorSize - 1) / sectorSize;

    #if (WRITEENABLE)
    Display_printf(display, 0, 0, "Writing the array...\n");

    result = SD_write(sdHandle, sdPacket, STARTINGSECTOR, sectors);
    if (result != SD_STATUS_SUCCESS)
    {
        Display_printf(display, 0, 0, "Error writing to the SD card\n");
        while (1);
    }
    #endif

    Display_printf(display, 0, 0, "Reading the array...\n");
    result = SD_read(sdHandle, cpy_buff, STARTINGSECTOR, sectors);
    if (result != SD_STATUS_SUCCESS)
    {
        Display_printf(display, 0, 0, "Error reading from the SD card\n");
        while (1);
    }

    /* Compare data read from the SD card with expected values */
    int i;
    for (i = 0; i < BUFFSIZE; i++)
    {
        if (cpy_buff[i] != sdPacket[i])
        {
            Display_printf(display, 0, 0,
                    "Data read from SD card differed from expected value\n");
            Display_printf(display, 0, 0,
                    "    Expected value for index %d: %d, got %d\n", i,
                    sdPacket[i], cpy_buff[i]);
            Display_printf(display, 0, 0, "Run the example with WRITEENABLE "
                    "= 1 to write expected values to the SD card\n");
            break;
        }
    }

    if (i == BUFFSIZE)
    {
        Display_printf(display, 0, 0,
                "Data read from SD card matched expected values\n");
        Display_printf(display, 0, 0, "Data from SD card: %x\n", sdPacket[0]);
    }

    SD_close(sdHandle);
}

/**
 *  @brief  Callback function attached to RF command.
 *
 *  @param h    RF driver handle
 *  @param ch   Command handle
 *  @param e    Event mask
 *
 *  @return none
 *
 */
static void echoCallback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
#ifdef LOG_RADIO_EVENTS
    eventLog[evIndex++ & 0x1F] = e;
#endif// LOG_RADIO_EVENTS

    static uint8_t ackPacket[PAYLOAD_LENGTH];
    static uint8_t dataPacket[PAYLOAD_LENGTH + NUM_APPENDED_BYTES - 1];
    int i;

    for(i = 0; i < PAYLOAD_LENGTH; i++)
    {
        ackPacket[i] = 0xA;
    }
    dataPacket[0] = 0xA;

    //Set number of commands and handle user input error.
    if(INSTRUCTION_COUNT > MAX_INSTRUCTIONS)
    {
        dataPacket[1] = MAX_INSTRUCTIONS;
        Display_printf(display, 0, 0, "#Error: Invalid number of instructions.\n");
    }
    else if(INSTRUCTION_COUNT < 1)
    {
        dataPacket[1] = 1;
        Display_printf(display, 0, 0, "#Error: Invalid number of instructions.\n");
    }
    else
    {
        dataPacket[1] = INSTRUCTION_COUNT;
    }


    for(i = 2; i < (PAYLOAD_LENGTH + NUM_APPENDED_BYTES - 1); i++)
    {
        dataPacket[i] = i;
    }
    dataPacket[2] = 0x1;
    dataPacket[4] = 0x12;
    dataPacket[6] = 0x3;

    if((e & RF_EventCmdDone) && !(e & RF_EventLastCmdDone))
    {
        /* Successful TX */
        txSuccess();
    }
    else if(e & RF_EventRxEntryDone)
    {
        // Successful RX
        bRxSuccess = true;
        /* Get current unhandled data entry */
        currentDataEntry = RFQueue_getDataEntry();

        /* Handle the packet data, located at &(currentDataEntry->data):
         * - Length is the first byte with the current configuration
         * - Data starts from the second byte
         */
        packetLength      = *(uint8_t *)(&(currentDataEntry->data));
        packetDataPointer = (uint8_t *)(&(currentDataEntry->data) + 1);

        /* Copy the payload + status byte to the rxPacket variable */
        memcpy(rxPacket, packetDataPointer, (packetLength + 1));

        /* Check the packet against what was transmitted */
        int16_t statusAck = memcmp(ackPacket, rxPacket, packetLength);
        int16_t statusData = memcmp(dataPacket, rxPacket, packetLength);
        typedef enum cubeState state_t;
        state_t state;
        state = DATA_RECEIVED;

//        //Use first RX byte to determine entry state.
//        if(statusData == 0)
//        {
//            state = DATA_RECEIVED;
//        }
//        else
//        {
//            /* Error Condition: clear both LEDs */
//            /* If ACK packet not sent first by femtosat
//               exchange will fail. */
//            ackError();
//            state = ACK_PENDING;
//        }

        switch(state)
        {
        case ACK_PENDING:
            //Put CC1310 into sleep mode.
            break;

        case DATA_PENDING:
            //Put CC1310 into sleep mode
            break;

        case DATA_RECEIVED:
            //Log data from femtosat in microSD
            dataSuccess();
            //Power down radio.
            break;

        }
    }

    else if((e & RF_EventLastCmdDone) && !(e & RF_EventRxEntryDone))
    {
        if(bRxSuccess == true)
        {
            /* Received packet successfully but RX command didn't complete at
             * the same time RX_ENTRY_DONE event was raised. Reset the flag
             */
            bRxSuccess = false;
        }
        else
        {
            /* RX timed out */
            /* Set LED2, clear LED1 to indicate TX */
            timeoutError();
        }
    }
    else
    {
        /* Error Condition. */
        genError();
    }
    RFQueue_nextEntry();
}

/**
 *  @brief  Put CC1310 in sleep mode.
 *
 *  @param delayTime    Sleep time for CC1310.
 *
 *  @return none
 *
 */
static void rfSleep(uint8_t delayTime)
{
    Display_printf(display, 0, 0, "Entering sleep mode...\n");
    RF_yield(rfHandle);
    sleep(delayTime + 30);
}

/**
 *  @brief  Set red LED high when timeout error occurs.
 *
 *  @return none
 *
 */
static void timeoutError()
{
    PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 0);
    PIN_setOutputValue(ledPinHandle, Board_PIN_LED2, 1);
    Display_printf(display, 0, 0, "#Error: Timeout error...\n");
}

/**
 *  @brief  Set red LED high when error occurs.
 *
 *  @return none
 *
 */
static void genError()
{
    PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 0);
    PIN_setOutputValue(ledPinHandle, Board_PIN_LED2, 1);
    Display_printf(display, 0, 0, "#Error: eneric RF error...\n");
}

/**
 *  @brief  Set red LED high when acknowledgement error occurs.
 *
 *  @return none
 *
 */
static void ackError()
{
    PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 0);
    PIN_setOutputValue(ledPinHandle, Board_PIN_LED2, 1);
    Display_printf(display, 0, 0, "#Error: Ack error...\n");
}

/**
 *  @brief  Clear LEDs on successful TX.
 *
 *  @return none
 *
 */
static void txSuccess()
{
    PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 1);
    PIN_setOutputValue(ledPinHandle, Board_PIN_LED2, 1);
}

/**
 *  @brief  Toggle LEDs upon successful transmission.
 *
 *  @return none
 *
 */
static void dataSuccess()
{
    PIN_setOutputValue(ledPinHandle, Board_PIN_LED1,
                       !PIN_getOutputValue(Board_PIN_LED1));
    PIN_setOutputValue(ledPinHandle, Board_PIN_LED2,
                       !PIN_getOutputValue(Board_PIN_LED2));
}

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
 *  @brief  Setup LED driver.
 *
 *  @return none
 *
 */
static void ledSetup()
{
    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, pinTable);
    if (ledPinHandle == NULL)
    {
        while(1);
    }
}

/**
 *  @brief  Setup RF params and commands.
 *
 *  @return none
 *
 */
static void rfSetup()
{
    uint32_t curtime;
    RF_Params rfParams;
    RF_Params_init(&rfParams);
    bool commandFlag = 0;
    uint8_t quatPacket[6] = {10, 1, 1, 5, 0, 6};

    if(RFQueue_defineQueue(&dataQueue,
                           rxDataEntryBuffer,
                           sizeof(rxDataEntryBuffer),
                           NUM_DATA_ENTRIES,
                           PAYLOAD_LENGTH + NUM_APPENDED_BYTES))
    {
        /* Failed to allocate space for all data entries */
        PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 1);
        PIN_setOutputValue(ledPinHandle, Board_PIN_LED2, 1);
        while(1);
    }

    /* Modify CMD_PROP_TX and CMD_PROP_RX commands for application needs */
    RF_cmdPropTx.pktLen = PAYLOAD_LENGTH;

#if QUAT_TEST
    RF_cmdPropTx.pPkt = txQuatPacket;
#endif

    RF_cmdPropTx.pPkt = txPacket;
    RF_cmdPropTx.startTrigger.triggerType = TRIG_NOW;
    RF_cmdPropTx.startTrigger.pastTrig = 1;
    RF_cmdPropTx.startTime = 0;
    RF_cmdPropTx.pNextOp = (rfc_radioOp_t *)&RF_cmdPropRx;
    /* Only run the RX command if TX is successful */
    RF_cmdPropTx.condition.rule = COND_STOP_ON_FALSE;

    /* Set the Data Entity queue for received data */
    RF_cmdPropRx.pQueue = &dataQueue;
    /* Discard ignored packets from Rx queue */
    RF_cmdPropRx.rxConf.bAutoFlushIgnored = 1;
    /* Discard packets with CRC error from Rx queue */
    RF_cmdPropRx.rxConf.bAutoFlushCrcErr = 1;
    /* Implement packet length filtering to avoid PROP_ERROR_RXBUF */
    RF_cmdPropRx.maxPktLen = PAYLOAD_LENGTH;
    RF_cmdPropRx.pktConf.bRepeatOk = 0;
    RF_cmdPropRx.pktConf.bRepeatNok = 0;
    RF_cmdPropRx.pOutput = (uint8_t *)&rxStatistics;
    /* Receive operation will end RX_TIMEOUT ms after command starts */
    RF_cmdPropRx.endTrigger.triggerType = TRIG_REL_PREVEND;
    RF_cmdPropRx.endTime = RX_TIMEOUT;

    /* Request access to the radio */
#if defined(DeviceFamily_CC26X0R2)
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioSetup, &rfParams);
#else
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);
#endif// DeviceFamily_CC26X0R2

    /* Set the frequency */
    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);

    /* Get current time */
    curtime = RF_getCurrentTime();

    while(!commandFlag)
    {
        uint8_t i;
        for (i = 0; i < PAYLOAD_LENGTH; i++)
        {
                txPacket[i] = sdPacket[i];
        }
#if QUAT_TEST
        memcpy(txQuatPacket, quatPacket, PAYLOAD_LENGTH);
#endif
        /* Set absolute TX time to utilise automatic power management */
        curtime += PACKET_INTERVAL;
        RF_cmdPropTx.startTime = curtime;

        /* Transmit a packet and wait for its echo.
         * - When the first of the two chained commands (TX) completes, the
         * RF_EventCmdDone event is raised but not RF_EventLastCmdDone
         * - The RF_EventLastCmdDone in addition to the RF_EventCmdDone events
         * are raised when the second, and therefore last, command (RX) in the
         * chain completes
         * -- If the RF core successfully receives the echo it will also raise
         * the RF_EventRxEntryDone event
         * -- If the RF core times out while waiting for the echo it does not
         * raise the RF_EventRxEntryDone event
         */
        RF_EventMask terminationReason =
                RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx, RF_PriorityNormal,
                          echoCallback, (RF_EventCmdDone | RF_EventRxEntryDone |
                          RF_EventLastCmdDone));

        switch(terminationReason)
        {
            case RF_EventLastCmdDone:
                // A stand-alone radio operation command or the last radio
                // operation command in a chain finished.
                break;
            case RF_EventCmdCancelled:
                // Command cancelled before it was started; it can be caused
            // by RF_cancelCmd() or RF_flushCmd().
                break;
            case RF_EventCmdAborted:
                // Abrupt command termination caused by RF_cancelCmd() or
                // RF_flushCmd().
                break;
            case RF_EventCmdStopped:
                // Graceful command termination caused by RF_cancelCmd() or
                // RF_flushCmd().
                break;
            default:
                // Uncaught error event
                while(1);
        }

        uint32_t cmdStatus = ((volatile RF_Op*)&RF_cmdPropTx)->status;
        uint8_t numberCommands = rxPacket[1];
        int j;

        if(rxPacket[0] == 0xA)
        {
            //Loop through commands.
            for(j = 2; j < ((numberCommands*2) + 2); j++)
            {
                if((j % 2) == 0)
                {
                    //Perform command.
                    dummyCommand(rxPacket[j], (j / 2), numberCommands);
                    //Do nothing for specified period.
                    rfSleep(rxPacket[j+1] / 2);
                }
            }
            commandDone();
        }

        switch(cmdStatus)
        {
            case PROP_DONE_OK:
                break;
            case PROP_DONE_STOPPED:
                // received CMD_STOP while transmitting packet and finished
                // transmitting packet
                break;
            case PROP_DONE_ABORT:
                // Received CMD_ABORT while transmitting packet
                break;
            case PROP_ERROR_PAR:
                // Observed illegal parameter
                break;
            case PROP_ERROR_NO_SETUP:
                // Command sent without setting up the radio in a supported
                // mode using CMD_PROP_RADIO_SETUP or CMD_RADIO_SETUP
                break;
            case PROP_ERROR_NO_FS:
                // Command sent without the synthesiser being programmed
                break;
            case PROP_ERROR_TXUNF:
                // TX underflow observed during operation
                break;
            default:
                // Uncaught error event - these could come from the
                // pool of states defined in rf_mailbox.h
                while(1);
        }
    }
}

/**
 *  @brief  Perform commands.
 *
 *  @param command          Command byte from ground station.
 *  @param numberCommands   Number of commands to perform.
 *
 *  @return none
 *
 */
static void dummyCommand(uint8_t command, uint8_t commandNumber, uint8_t totalCommands)
{
    uint8_t commandMask = command & 0xF;
    uint8_t i, sleepTime;
    uint8_t quatMask[45];

    for(i = 0; i < 45; i++)
    {
        quatMask[i] = rxPacket[i+3];
    }

    switch(commandMask)
    {
    case 0x1:
        //Use RSSI
        Display_printf(display, 0, 0, "Performing command %d out of %d...\n",
                       (commandNumber), totalCommands);
        getRssi();
        break;

    case 0x2:
        //Standby mode
        Display_printf(display, 0, 0, "Performing command %d out of %d...\n",
                       (commandNumber), totalCommands);
        sleepTime = command >> 4;
        customStandby(sleepTime);
        break;

    case 0x3:
        //Blink LED1
        Display_printf(display, 0, 0, "Performing command %d out of %d...\n",
                       (commandNumber), totalCommands);
        setQuat(quatMask);
        break;

    case 0x4:
        //Blink LED2
        Display_printf(display, 0, 0, "Performing command %d out of %d...\n",
                       (commandNumber), totalCommands);
        redBlinky();
        break;

    case 0x5:
        //Blink LED1
        Display_printf(display, 0, 0, "Performing command %d out of %d...\n",
                       (commandNumber), totalCommands);
        greenBlinky();
        break;

    case 0x6:
        //Blink LED2
        Display_printf(display, 0, 0, "Performing command %d out of %d...\n",
                       (commandNumber), totalCommands);
        redBlinky();
        break;
    case 0x7:
        //Blink LED1
        Display_printf(display, 0, 0, "Performing command %d out of %d...\n",
                       (commandNumber), totalCommands);
        greenBlinky();
        break;

    case 0x8:
        //Blink LED1
        Display_printf(display, 0, 0, "Performing command %d out of %d...\n",
                       (commandNumber), totalCommands);
        greenBlinky();
        break;

    case 0x9:
        //Blink LED2
        Display_printf(display, 0, 0, "Performing command %d out of %d...\n",
                       (commandNumber), totalCommands);
        redBlinky();
        break;

    case 0xA:
        //Blink LED1
        Display_printf(display, 0, 0, "Performing command %d out of %d...\n",
                       (commandNumber), totalCommands);
        greenBlinky();
        break;

    case 0xB:
        //Blink LED2
        Display_printf(display, 0, 0, "Performing command %d out of %d...\n",
                       (commandNumber), totalCommands);
        redBlinky();
        break;

    case 0xC:
        //Blink LED1
        Display_printf(display, 0, 0, "Performing command %d out of %d...\n",
                       (commandNumber), totalCommands);
        greenBlinky();
        break;

    case 0xD:
        //Blink LED2
        Display_printf(display, 0, 0, "Performing command %d out of %d...\n",
                       (commandNumber), totalCommands);
        redBlinky();
        break;

    case 0xE:
        //Blink LED1
        Display_printf(display, 0, 0, "Performing command %d out of %d...\n",
                       (commandNumber), totalCommands);
        greenBlinky();
        break;

    default:
        Display_printf(display, 0, 0, "Command Mask: %d\n", commandMask);
        break;
    }
}

/**
 *  @brief  Dummy command for testing. Blink green LED.
 *
 *  @return none
 *
 */
static void greenBlinky()
{
    int i = 0;
    while(i < 5)
    {
        PIN_setOutputValue(ledPinHandle, Board_PIN_LED1,
                           !PIN_getOutputValue(Board_PIN_LED1));
        sleep(1);
        i++;
    }
    PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, 0);
}

/**
 *  @brief  Dummy command for testing. Blink red LED.
 *
 *  @return none
 *
 */
static void redBlinky()
{
    int i = 0;
    while(i < 5)
    {
        PIN_setOutputValue(ledPinHandle, Board_PIN_LED2,
                           !PIN_getOutputValue(Board_PIN_LED2));
        sleep(1);
        i++;
    }
    PIN_setOutputValue(ledPinHandle, Board_PIN_LED2, 0);
}

/**
 *  @brief  Find received signal strength of RF link.
 *
 *  @return none
 *
 */
static void getRssi()
{
    int8_t rssiValue = rxStatistics.lastRssi;
    Display_printf(display, 0, 0, "RSSI: %ddBm\n", rssiValue);
}

/**
 *  @brief  Place device in standby mode for 5 minutes.
 *
 *  @param  sleepTime
 *
 *  @return none
 *
 */
static void customStandby(uint8_t sleepTimeMins)
{
    uint16_t sleepTimeSeconds = sleepTimeMins * 60;
    Display_printf(display, 0, 0, "Entering sleep mode for %d minutes...\n", sleepTimeMins);
    sleep(sleepTimeSeconds);
}

/**
 *  @brief  Interperate quaternion data.
 *
 *  @param  packet  Pointer to packer array.
 *  @return none
 *
 */
static void setQuat(uint8_t packet[45])
{
    uint8_t i, sign;
    uint8_t fillCount = 0;
    uint8_t checkArray[45];
    memcpy(checkArray, packet, PAYLOAD_LENGTH);
    int quat[9];
    for(i = 0; i < 45; i++)
    {
        if(!(i % 3))
        {
            sign = packet[i];
            switch(sign)
            {
            case 0:
                quat[fillCount] = packet[i+1] + packet[i+2];
                fillCount++;
                break;
            case 1:
                quat[fillCount] = -(packet[i+1] + packet[i+2]);
                fillCount++;
                break;
            default:
                break;
            }
        }
    }
    for(i = 0; i < 9; i++)
    {
        Display_printf(display, 0, 0, "quat[%d]: %d\n", i, quat[i]);
    }
}

/**
 *  @brief  Notify user that commands have been completed.
 *
 *  @return none
 *
 */
static void commandDone()
{
    Display_printf(display, 0, 0, "Commands finished.\n");
}

void *mainThread(void *arg0)
{
    displaySetup();
    ledSetup();
    sdSetup();
    rfSetup();

    return(NULL);
}
