/** @file HL_sys_main.c 
*   @brief Application main file
*   @date 07-July-2017
*   @version 04.07.00
*
*   This file contains an empty main function,
*   which can be used for the application.
*/

/* 
* Copyright (C) 2009-2016 Texas Instruments Incorporated - www.ti.com  
* 
* 
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*
*    Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


/* USER CODE BEGIN (0) */
/* USER CODE END */

/* Include Files */

#include "HL_sys_common.h"

/* USER CODE BEGIN (1) */
#include "HL_sys_core.h"
#include "HL_system.h"
#include "HL_gio.h"
#include "HL_sci.h"
#include "HL_ecap.h"
#include "HL_etpwm.h"

#include <stdio.h>
#include <string.h>
/* USER CODE END */

/** @fn void main(void)
*   @brief Application main function
*   @note This function is empty by default.
*
*   This function is called after startup.
*   The user can use this function to implement the application.
*/

/* USER CODE BEGIN (2) */
#define UART sciREG1

char buf[128];
unsigned int buf_len;
int aileron1, aileron2;

void sci_display(sciBASE_t * sci, uint8 * text, uint32 len);
void wait(uint32 delay);
/* USER CODE END */

int main(void)
{
/* USER CODE BEGIN (3) */
    int i;

    sciInit();
    wait(10000);

    sprintf(buf, "SCI Init Success!!!\n\r\0");
    buf_len = strlen(buf);
    sci_display(UART, (uint8 *)buf, buf_len);

    gioInit();
    gioSetDirection(gioPORTA, 0xffffffff);
    wait(10000);

    sprintf(buf, "GIO Init Success!!!\n\r\0");
    buf_len = strlen(buf);
    sci_display(UART, (uint8 *)buf, buf_len);

    etpwmInit();
    wait(10000);
/*
    etpwmREG1->CMPB = 1250;
    wait(10000);
    etpwmREG1->CMPB = 2500;
    wait(10000);
*/
    sprintf(buf, "PWM Init Success!!!\n\r\0");
    buf_len = strlen(buf);
    sci_display(UART, (uint8 *)buf, buf_len);

    ecapInit();
    wait(10000);

    sprintf(buf, "ECAP Init Success!!!\n\r\0");
    buf_len = strlen(buf);
    sci_display(UART, (uint8 *)buf, buf_len);

    _enable_interrupt_();
    ecapStartCounter(ecapREG3);
    ecapStartCounter(ecapREG2);
    ecapEnableCapture(ecapREG3);
    ecapEnableCapture(ecapREG2);

    for(;;)
    {

    }

/* USER CODE END */

    return 0;
}


/* USER CODE BEGIN (4) */
void ecapNotification(ecapBASE_t * ecap, uint16 flags)
{
    int i;
    uint32 cap[8];

    cap[0] = ecapGetCAP1(ecapREG3);
    cap[1] = ecapGetCAP2(ecapREG3);
    cap[2] = ecapGetCAP1(ecapREG2);
    cap[3] = ecapGetCAP2(ecapREG2);

    aileron1 = (cap[1] - cap[0]) * 1000/VCLK3_FREQ/1000;
    aileron2 = (cap[3] - cap[2]) * 1000/VCLK3_FREQ/1000;

    aileron1 *= 10;

    sprintf(buf, "ch1 = %d, ", aileron1);
    buf_len = strlen(buf);
    sci_display(UART, (uint8 *)buf, buf_len);

    if(aileron2 < 1500)
    {
        gioSetBit(gioPORTA, 3, 1);
    }
    else
    {
        gioSetBit(gioPORTA, 3, 0);
    }

    sprintf(buf, "ch2 = %d \n\r\0", aileron2);
    buf_len = strlen(buf);
    sci_display(UART, (uint8 *)buf, buf_len);

 //   etpwmREG1->CMPB = aileron;

    for(i=0; i<100; i++)
    {
        sprintf(buf, "ch1 = %d ms\n\r\0 ", aileron1);
        buf_len = strlen(buf);
        sci_display(UART, (uint8 *)buf, buf_len);

        gioSetBit(gioPORTA, 0, 1);
        wait(aileron1);
        gioSetBit(gioPORTA, 0, 0);
        wait(aileron1);
    }
}

void sci_display(sciBASE_t * sci, uint8 * text, uint32 len)
{
    while(len--)
    {
        while((UART->FLR & 0x04)==0x04)
            ;
        sciSendByte(UART, *text++);
    }
}

void wait(uint32 delay)
{
    int i;
    for(i=0; i<delay; i++)
        ;
}
/* USER CODE END */
