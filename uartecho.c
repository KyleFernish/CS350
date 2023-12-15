/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
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
 *  ======== uartecho.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/*
 *  ======== mainThread ========
 */
enum states {initialize, oRecognized, nRecognized, fRecognized, secondFRecognized} state = initialize;
char input = 0;

void stateMachine() {
    //state transitions
    switch(state) {
    case initialize://initialize starts with the LEDs on to show that the board is functioning
        if (input == 'O' || input == 'o') {
            state = oRecognized;
        }
        else{ //only transition if 'O' or 'o' input
            state = initialize;
        }
        break;

    case oRecognized:
        if (input == 'N' || input == 'n'){
            state = nRecognized;
        }
        else if(input == 'F' || input == 'f') {
            state = fRecognized;
        }
        else {//if input !N && !F or !n && !f return to oRecognized
            state = oRecognized;
        }
        break;

    case nRecognized:
        state = initialize;//returns to initialized state for further input after action taken
        break;

    case fRecognized:
        if (input == 'F' || input == 'f') {
            state = secondFRecognized;
        }
        else {//if input !F or !f return to fRecognized
            state = fRecognized;
        }
        break;

    case secondFRecognized:
        state = initialize; //returns to initialize for further input
        break;
    }

    //state actions
    switch(state) {
        //actions do nothing
        case initialize:
        case oRecognized:
        case fRecognized:
        break;

        case nRecognized://LED Turns on
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            break;
        case secondFRecognized://LED turns off
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            break;
        }
}

void *mainThread(void *arg0)
{
    const char  echoPrompt[] = "Echoing characters:\r\n";
    UART_Handle uart;
    UART_Params uartParams;

    /* Call driver init functions */
    GPIO_init();
    UART_init();

    /* Configure the LED pin */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    uart = UART_open(CONFIG_UART_0, &uartParams);

    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }

    /* Turn on user LED to indicate successful initialization */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);

    UART_write(uart, echoPrompt, sizeof(echoPrompt));


    /* Loop forever echoing */
    while (1) {
        UART_read(uart, &input, 1);//reads input
        stateMachine();//calls stateMachine to execute different cases and actions
        UART_write(uart, &input, 1);//writes input
    }
}
