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

char buf[128];
unsigned int buf_len;
uint8 receive_data[4]={0};
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

    etpwmREG1->CMPA = 1250;
    wait(10000);

    etpwmREG1->CMPA = 2500;
    wait(10000);

    etpwmREG1->CMPA = 1500;
    wait(10000);

    sprintf(buf, "PWM Init Success!!!\n\r\0");
    buf_len = strlen(buf);
    sci_display(UART, (uint8 *)buf, buf_len);

    for(;;)
    {
        catch_command();
        value = (receive_data[0]-48)*1000
                + (receive_data[1]-48)*100
                + (receive_data[2]-48)*10
                + (receive_data[3]-48);

        if()
        else if(value > 2500)
        {
            etpwmREG1->CMPA = 1250;
        }
        else
        {
        etpwmREG1->CMPA = 1.25 * value;
        }

        sprintf(buf, "value = %d\n\r\0", value);
        buf_len = strlen(buf);
        sci_display(UART, (uint8 *)buf, buf_len);
    }

/* USER CODE END */

    return 0;
}


/* USER CODE BEGIN (4) */
void wait(uint32 delay)
{
    int i;
    for(i=0 ; i<delay; i++)
        ;
}

void sci_display(sciBASE_t * sci, uint8 * text, uint32 len)
{
    while(len--)
    {
        while((UART->FLR & 0x4) == 4)
            ;
        sciSendByte(UART, *text++);
    }
}

void catch_command(void)
{
    int i;
    while((UART->FLR & 0x4) == 4)
        ;

    for(i=0; i<4; i++)
    {
        receive_data[i] = sciReceiveByte(UART);
    }

}

/* USER CODE END */
