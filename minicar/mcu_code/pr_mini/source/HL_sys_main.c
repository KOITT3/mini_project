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
#include "HL_gio.h"
#include "HL_sci.h"
#include "HL_ecap.h"
#include "HL_etpwm.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"

#include "os_queue.h"
#include "os_semphr.h"
#include "os_task.h"
#include "HL_system.h"

#include <string.h>
#include <stdio.h>

xTaskHandle motor_b;
xTaskHandle motor_f;

QueueHandle_t mutex=NULL;

/* USER CODE END */

/* Include Files */

#include "HL_sys_common.h"

/* USER CODE BEGIN (1) */
/* USER CODE END */

/** @fn void main(void)
*   @brief Application main function
*   @note This function is empty by default.
*
*   This function is called after startup.
*   The user can use this function to implement the application.
*/

/* USER CODE BEGIN (2) */
void wait(int delay);
void motor_dc(void *pbParameters);
void sci_display(sciBASE_t* sci,uint8* buf, int len);
void ecapNotification(ecapBASE_t *ecap, uint16 flag);
void motor_dc_arduino(void);
void motor_stop(void);



int throttle;
int rudder;
int lh_switch;
/* USER CODE END */

int main(void)
{
/* USER CODE BEGIN (3) */
    sciInit();
    gioInit();
    etpwmInit();
    etpwmStartTBCLK();
    ecapInit();

    _enable_interrupt_();
    etpwmREG1->CMPA=20000;
    wait(3000);
    etpwmREG1->CMPA=0;
    etpwmREG1->CMPB=20000;
    wait(3000);

    etpwmREG1->CMPB=0;
    etpwmREG2->CMPA=20000;
    wait(3000);
    etpwmREG2->CMPB=20000;
    etpwmREG2->CMPA=0;
    wait(3000);
    etpwmREG2->CMPB=0;

    ecapStartCounter(ecapREG1);
    ecapStartCounter(ecapREG2);
    ecapStartCounter(ecapREG3);

    ecapEnableCapture(ecapREG1);
    ecapEnableCapture(ecapREG2);
    ecapEnableCapture(ecapREG3);

    vSemaphoreCreateBinary(mutex);

    if(xTaskCreate(motor_dc,"motor_back",configMINIMAL_STACK_SIZE,NULL,0,&motor_b)!=pdTRUE){
        for(;;)
            ;
    }

    //for(;;)
      //  ;

    vTaskStartScheduler();


/* USER CODE END */

    return 0;
}


/* USER CODE BEGIN (4) */

void wait(int delay){
    int i;

    for(i=0;i<delay;i++)
        ;
}

void motor_dc(void *pbParameters){
    for(;;){
        if(xSemaphoreTake(mutex,(TickType_t)0x01)==pdTRUE){
#if 0
            if(throttle>2000){
               etpwmREG1->CMPA=throttle/2;
               etpwmREG1->CMPB=0;
               xSemaphoreGive(mutex);
               vTaskDelay(10);
            }else{
                etpwmREG1->CMPA=0;
                etpwmREG1->CMPB=throttle;
                xSemaphoreGive(mutex);
                vTaskDelay(10);
            }
#endif
        }
    //etpwmREG1->CMPA=throttle;
    xSemaphoreGive(mutex);
    }

}

void motor_servo(void *pbParameters){
    for(;;){
        if(xSemaphoreTake(mutex,(TickType_t)0x01)==pdTRUE){
            etpwmREG2->CMPA=rudder;
        }
    }
}


void ecapNotification(ecapBASE_t *ecap, uint16 flag){
    uint32 cap[6];
    char buf[128];
    int buf_len;

    cap[0]=ecapGetCAP1(ecapREG1);
    cap[1]=ecapGetCAP2(ecapREG1);

    cap[2]=ecapGetCAP1(ecapREG2);
    cap[3]=ecapGetCAP2(ecapREG2);

    cap[4]=ecapGetCAP1(ecapREG3);
    cap[5]=ecapGetCAP2(ecapREG3);

    throttle=(cap[1]-cap[0])/VCLK3_FREQ;
    rudder=(cap[3]-cap[2])/VCLK3_FREQ;
    lh_switch=(cap[5]-cap[4])/VCLK3_FREQ;

    if(lh_switch>=1450&&lh_switch<=1550){//송수신기가 꺼져있을때 처리
        return;
    }

    //sprintf(buf,"cmpA:%d\tcmpB:%d\t throttle:%d\t rudder:%d ch7:%d\n\r\0",etpwmREG1->CMPA,etpwmREG1->CMPB,throttle,rudder,lh_switch);
    sprintf(buf,"cmp1A:%d\t cmp1B:%d\t cmp2A:%d\t cmp2B:%d\n\r\0",etpwmREG1->CMPA,etpwmREG1->CMPB,etpwmREG2->CMPA,etpwmREG2->CMPB);
   // sprintf(buf,"switch:%d\n\r\0",lh_switch);
    buf_len=strlen(buf);
    sci_display(sciREG1,(uint8*)buf,buf_len);

    wait(30000);

    if(lh_switch>1900){//switch가 하이(스위치를 내린 경우) 모터 동작을 꺼버리고 함수를 나감
        motor_stop();
        return;
    }

    motor_dc_arduino();

    wait(30000);
}

void motor_stop(void){
    etpwmREG1->CMPA=0;
    etpwmREG1->CMPB=0;
    etpwmREG2->CMPA=0;
    etpwmREG2->CMPB=0;
}

void motor_dc_arduino(void){
    int vel=throttle; //etpwmREG1:right, etpwmREG2:left
    int dir_left=0;
    int dir_right=0;
    int tmp;

    vel-=1000;

    if(vel<0){
        vel=0;
    }

    vel*=40;//수치 보정(0~1000을 0~40000으로)

    if(rudder>1600){
        dir_right=(rudder-1500)*20;
    }else if(rudder<1400){
        dir_left=(1500-rudder)*20;
    }

    if(vel>20000){
        vel=vel-20000;
        if(vel<2500){
              vel=0;
        }

        tmp=vel-dir_left;

        if(tmp<0){
            tmp=0;
        }


        etpwmREG1->CMPA=tmp;
        etpwmREG1->CMPB=0;

        tmp=vel-dir_right;
        if(tmp<0){
          tmp=0;
        }

        etpwmREG2->CMPA=tmp;
        etpwmREG2->CMPB=0;

       }else{
           vel=20000-vel;
           if(vel<2500){
               vel=0;
           }

           tmp=vel-dir_left;
           if(tmp<0){
               tmp=0;
           }

           etpwmREG1->CMPA=0;
           etpwmREG1->CMPB=tmp;

           tmp=vel-dir_right;
           if(tmp<0){
               tmp=0;
           }

           etpwmREG2->CMPA=0;
           etpwmREG2->CMPB=tmp;
       }
}

void sci_display(sciBASE_t *sci,uint8* buf, int len){
    while(len--){
        while((sci->FLR&0x4)==4)
            ;
        sciSendByte(sci,*buf++);
    }
}


/* USER CODE END */
