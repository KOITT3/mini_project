//raw distance data
#include "HL_sys_common.h"
#include "HL_gio.h"
#include "HL_sci.h"
#include "HL_adc.h"
#include "HL_system.h"
#include <string.h>
#include <stdio.h>

adcData_t adc_data[1];

void sci_display_data(sciBASE_t *, uint8 *, uint32);
void wait(uint32);

int main(void)
{
/* USER CODE BEGIN (3) */
    char txt_buf[256]={0};
    unsigned int buf_len;

    uint32 ch_cnt = 0;
    uint32 id = 0;
    uint32 value = 0;
    int i=0,logg;
    float temp=0;
    sciInit();
    adcInit();
    gioInit();
    gioSetDirection(gioPORTA,0xffffffff);
    gioSetDirection(gioPORTB,0xffffffff);
    for(;;)
    {
        gioSetBit(gioPORTA,0,1);
        gioSetBit(gioPORTB,6,1);
        adcStartConversion(adcREG1, adcGROUP1);

       while((adcIsConversionComplete(adcREG1, adcGROUP1)) == 0)
            ;

       adcGetData(adcREG1, adcGROUP1, &adc_data[0]);

        id = adc_data[0].id;
        value = adc_data[0].value;
        printf("reverse distance = %d\n\r\0",value);

/*  RAW DATA
        sprintf(txt_buf, "temperature = %d\n\r\0", value);
        buf_len = strlen(txt_buf);
        sci_display_txt(sciREG1, (uint8 *) txt_buf, buf_len);
*/



        sprintf(txt_buf, "reverse distance = %d\n\r\0", value);
        buf_len = strlen(txt_buf);
        sci_display_txt(sciREG1, (uint8 *) txt_buf, buf_len);
        gioSetBit(gioPORTB,6,0);
        wait(0x10000);

    }

    return 0;
}

void sci_display_txt(sciBASE_t *sci, uint8 *txt, uint32 len)
{
    while(len--)
    {
        while((sciREG1->FLR & 0x4) == 4)
            ;
        sciSendByte(sciREG1, *txt++);
    }
}

void wait(uint32 delay)
{
    int i;
    for(i = 0; i < delay; i++)
        ;
}
