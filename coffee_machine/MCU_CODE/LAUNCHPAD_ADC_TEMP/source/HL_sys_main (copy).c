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


    for(;;)
    {
        adcStartConversion(adcREG1, adcGROUP1);

       while((adcIsConversionComplete(adcREG1, adcGROUP1)) == 0)
            ;

       adcGetData(adcREG1, adcGROUP1, &adc_data[0]);

        id = adc_data[0].id;
        value = adc_data[0].value;
//        printf("temperature = %lf\n\r",value);

/*  RAW DATA
        sprintf(txt_buf, "temperature = %d\n\r\0", value);
        buf_len = strlen(txt_buf);
        sci_display_txt(sciREG1, (uint8 *) txt_buf, buf_len);
*/

        temp = (float)value/16383;
        temp = -66.875 + (float)(217.75*3.3*temp);

        sprintf(txt_buf, "temperature = %lf\n\r\0", temp);
        buf_len = strlen(txt_buf);
        sci_display_txt(sciREG1, (uint8 *) txt_buf, buf_len);

        wait(0x100000);
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
