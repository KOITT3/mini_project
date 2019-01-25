#include "HL_sys_common.h"
#include "HL_gio.h"
#include "HL_sci.h"
#include <stdio.h>
#include <stdlib.h>

#include "types.h"
#define DS1820_DATAPIN 0x00
#include "ds1820.h"

void wait(uint32 time);
void sci_display(sciBASE_t *sci, uint8 *text, uint32 len);

int main(void)
{
    char txt_buf[256]={0};
    unsigned int buf_len;
    int i=0;
    float temp=0;
    float temperature_float=0;
    char temperature[8]={0};
    uint8 sensor_count=0;
    sint16 temperature_raw=0;
    gioInit();
    sciInit();
    gioSetDirection(gioPORTB,0x00000040);

    sprintf(txt_buf, "starting now\n\r\0");
    buf_len = strlen(txt_buf);
    sci_display(sciREG1, (uint8 *) txt_buf, buf_len);

    for(;;)
    {
        if(DS1820_FindNextDevice())
        {
            sprintf(txt_buf, "n1\n\r\0");
            buf_len = strlen(txt_buf);
            sci_display(sciREG1, (uint8 *) txt_buf, buf_len);

        temperature_raw = DS1820_GetTempRaw();

        sprintf(txt_buf, "n2\n\r\0");
        buf_len = strlen(txt_buf);
        sci_display(sciREG1, (uint8 *) txt_buf, buf_len);

        DS1820_GetTempString(temperature_raw,temperature);

        sprintf(txt_buf, "n3\n\r\0");
        buf_len = strlen(txt_buf);
        sci_display(sciREG1, (uint8 *) txt_buf, buf_len);

        temperature_float=DS1820_GetTempFloat();

        sprintf(txt_buf, "temperature = %f\n\r\0", temperature_float);
        buf_len = strlen(txt_buf);
        sci_display(sciREG1, (uint8 *) txt_buf, buf_len);

        delay_ms(2000);
        }
    }

    return 0;
}

void wait(uint32 time)
{
    int i;
    for(i=0;i<time;i++)
        ;
}

void sci_display(sciBASE_t *sci, uint8 *text, uint32 len)
{
    while(len--)
    {
        while((sciREG1->FLR & 0x4) == 4) // if not busy 4면   2번비트
        ;  //바쁘면 계속 루프 도는 것.
        sciSendByte(sciREG1, *text++);
    }
}
