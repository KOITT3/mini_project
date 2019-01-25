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
#include<stdio.h>
#include<stdlib.h>
#include<string.h>

#include"HL_system.h"
#include"HL_etpwm.h"
#include"HL_sci.h"
#include"HL_i2c.h"
#include"HL_rti.h"

/* USER CODE END */

/** @fn void main(void)
*   @brief Application main function
*   @note This function is empty by default.
*
*   This function is called after startup.
*   The user can use this function to implement the application.
*/

/* USER CODE BEGIN (2) */
void sci_display(sciBASE_t *, uint8 *, uint32);
void wait(int);
void disp_set(char *);
void command(sciBASE_t *);

uint32 receive;
uint32 cnt;
/* USER CODE END */

int main(void)
{
/* USER CODE BEGIN (3) */
    char txt_buf[128];
    unsigned int buf_len, value1 = 0, value2 = 0;

    sciInit();
    disp_set("SCI configure complete!\n\r\0");

    etpwmInit();
    etpwmStartTBCLK();
    disp_set("ETPWM configure complete!\n\r\0");

    i2cInit();
    disp_set("I2C configure complete!\n\r\0");

    wait(10000);
    etpwmSetCmpA(etpwmREG3, value1);
    etpwmSetCmpA(etpwmREG6, value2);

    sprintf(txt_buf, "init 1: %d\t init 2: %d\n\r\0",value1, value2);
    buf_len = strlen(txt_buf);
    sci_display(sciREG1, (uint8 *)txt_buf, (uint32)buf_len);

    rtiInit();
    rtiEnableNotification(rtiREG1, rtiNOTIFICATION_COMPARE0);
    _enable_IRQ_interrupt_();
    rtiStartCounter(rtiREG1, rtiCOUNTER_BLOCK0);

    for(;;)
    {
        command(sciREG1);

        if(receive == 1)
            value1 = value1 + 100;
        if(receive == 2)
            value1 = value1 - 100;
        if(receive == 3)
            value2 = value2 + 100;
        if(receive == 4)
            value2 = value2 - 100;
        if(receive == 0)
            value1 = value2 = 0;
        if(receive == 9)
            value2 = 0;
        if(receive == 8)
            value1 = 0;

        if(value1 <= 2500 && value1 > 0)
            value1 = 2500;
        if(value2 <= 4000 && value2 > 0)
            value2 = 4000;

        etpwmSetCmpA(etpwmREG3, value1);
        etpwmSetCmpA(etpwmREG6, value2);

        sprintf(txt_buf, "init 1: %d\t init 2: %d\t cnt = %d\n\r\0",value1, value2, cnt);
        buf_len = strlen(txt_buf);
        sci_display(sciREG1, (uint8 *)txt_buf, (uint32)buf_len);
        cnt = 0;
    }

/* USER CODE END */

    return 0;
}


/* USER CODE BEGIN (4) */
void wait(int delay)
{
    int i;

    for(i=0; i<delay; i++)
        ;
}

void sci_display(sciBASE_t *sci,uint8 *text, uint32 len)
{
    while(len--)
    {
        while((sci->FLR & 0x04) == 4)
            ;
        sciSendByte(sci, *text++);
    }
}

void disp_set(char *text)
{
    unsigned int len;

    len = strlen(text);
    sci_display(sciREG1, (uint8 *)text, (uint32)len);
}

void command(sciBASE_t *sci)
{
    while((sci->FLR & 0x04) == 4)
        ;
    receive = sciReceiveByte(sci);
    receive = receive - 48;
    cnt = 0;
}

void rtiNotification(rtiBASE_t *rti, uint32 notification)
{
    cnt++;
}
/* USER CODE END */
