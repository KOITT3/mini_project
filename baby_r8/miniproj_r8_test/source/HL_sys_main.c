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
#include "HL_system.h"
#include "HL_sci.h"
#include "HL_etpwm.h"
#include "HL_ecap.h"
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
#define ETPWM etpwmREG1

char buf[128];
unsigned int buf_len;

int throttle;


void wait(uint32 delay);
void ecapNotification(ecapBASE_t *ecap, uint16 flags);
void sci_display(sciBASE_t *sci, uint8 *text, uint32 len);

/* USER CODE END */

int main(void)
{
/* USER CODE BEGIN (3) */
    // ecap3. etpwm1A 사용
    sciInit();

    sprintf(buf, "sci Init Success!!!\n\r\0");
    buf_len = strlen(buf);
    sci_display(UART, (uint8 *)buf, buf_len);

    etpwmInit();
    sprintf(buf, "etpwm Init Success!!!\n\r\0");
    buf_len = strlen(buf);
    sci_display(UART, (uint8 *)buf, buf_len);

    ecapInit();
    sprintf(buf, "ecapInit Success!!!\n\r\0");
    buf_len = strlen(buf);
    sci_display(UART, (uint8 *)buf, buf_len);

    _enable_interrupt_();

    ecapStartCounter(ecapREG3);
    ecapEnableCapture(ecapREG3);

    for(;;)
        ;
/* USER CODE END */

    return 0;
}


/* USER CODE BEGIN (4) */

void ecapNotification(ecapBASE_t *ecap, uint16 flags)
{
    uint32 cap[2];

    cap[0] = ecapGetCAP1(ecapREG3);
    cap[1] = ecapGetCAP2(ecapREG3);

    throttle = (cap[1]-cap[0]) * 1000 / VCLK3_FREQ / 1000;

    sprintf(buf, "ch3 = %d ms\n\r\0", throttle);
    buf_len = strlen(buf);
    sci_display(UART, (uint8 *)buf ,buf_len);

    if(throttle > 1800)                 // 최대값 제한
    {
        etpwmREG1->CMPA = 1.25 * 1800;
    }else if(throttle < 1000)           // 최소값 제한
    {
        etpwmREG1->CMPA = 1.25 * 1000;
    }else if(1480 < throttle && throttle < 1520)    // 중립값 제한
    {
        etpwmREG1->CMPA = 1.25 * 1500;
    }else                               // RF값 입력
    {
        etpwmREG1->CMPA = throttle * 1.25;
    }

    wait(2000);
}

void sci_display(sciBASE_t *sci, uint8 *text, uint32 len)
{
    while(len--)
    {
        while((UART->FLR) == 4)
            ;
        sciSendByte(UART, *text++);
    }
}

void wait(uint32 delay)
{
    int i;

    for(i = 0; i < delay; i++)
        ;
}
/* USER CODE END */
