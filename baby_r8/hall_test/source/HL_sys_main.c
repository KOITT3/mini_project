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
#include "HL_rti.h"
#include "HL_sci.h"
#include "HL_etpwm.h"

#include <string.h>
#include <stdio.h>
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
#define UART1 sciREG2

int pwm[5] = {1500, 1530, 1540, 1550, 1560};

char buf[128];
unsigned int buf_len;
uint8 receive_data = 0;
uint8 receive_before = 0;
uint32 value = 0;

void wait(uint32 delay);
void sci_display(sciBASE_t * sci, uint8 * text, uint32 len);
void catch_command(void);

/* USER CODE END */

int main(void)
{
/* USER CODE BEGIN (3) */
    sciInit();
    wait(10000);

    sprintf(buf, "SCI Init Success!!!\n\r\0");
    buf_len = strlen(buf);
    sci_display(UART, (uint8 *)buf, buf_len);

    etpwmInit();
    wait(10000);

    sprintf(buf, "PWM Init Success!!!\n\r\0");
    buf_len = strlen(buf);
    sci_display(UART, (uint8 *)buf, buf_len);

    gioInit();
    wait(10000);

    gioSetDirection(gioPORTA, 0xfffffff0);

    sprintf(buf, "GIO Init Success!!!\n\r\0");
    buf_len = strlen(buf);
    sci_display(UART, (uint8 *)buf, buf_len);

    rtiInit();
    wait(10000);

    sprintf(buf, "RTI Init Success!!!\n\r\0");
    buf_len = strlen(buf);
    sci_display(UART, (uint8 *)buf, buf_len);

    rtiEnableNotification(rtiREG1, rtiNOTIFICATION_COMPARE0);
    _enable_IRQ_interrupt_();
    rtiStartCounter(rtiREG1, rtiCOUNTER_BLOCK0);

    sprintf(buf, "RTI Counter Start!!!\n\r\0");
    buf_len = strlen(buf);
    sci_display(UART, (uint8 *)buf, buf_len);

/* USER CODE END */

    return 0;
}


/* USER CODE BEGIN (4) */
void rtiNotification(rtiBASE_t * rtiREG, uint32 notification)
{

    catch_command();
    if(receive_before != receive_data)
    {
        sprintf(buf, "pwm is changed\n\r\0");
        buf_len = strlen(buf);
        sci_display(UART, (uint8 *)buf, buf_len);

        if((receive_data - 97 < 5) && (receive_data - 97 >= 0))
        {

            value = pwm[receive_data - 97];
            etpwmREG1->CMPA = 1.25 * value;
        }
        else
        {
            value = 0;
            etpwmREG1->CMPA = 0;
        }
    }
    sprintf(buf, "pwm = %d, value= %d\n\r\0", value, gioGetBit(gioPORTA, 0));
    buf_len = strlen(buf);
    sci_display(UART, (uint8 *)buf, buf_len);

    receive_before = receive_data;

}

void wait(uint32 delay)
{
    int i;
    for(i=0; i<delay; i++)
        ;
}

void sci_display(sciBASE_t * sci, uint8 * text, uint32 len)
{
    while(len--)
    {
        while((UART->FLR & 0x04)==4)
            ;
        sciSendByte(UART, *text++);
    }
}

void catch_command(void)
{
    while((UART->FLR & 0x4) == 4)
        ;

   receive_data = sciReceiveByte(UART);

}
/* USER CODE END */
