// 전원단 47uF CAP 추가했더니 IR리모콘 오동작 사라졌다.
// ADC 거리센서(ADC1번채널 0번 : 센싱하면 온도센싱값이 흔들린다. 전원 공급부족인줄 알았는데 그게아님. 채널분리해도 소용X
#include "HL_sys_common.h"
//#include "sys_core.h"
#include "HL_gio.h"
#include "HL_sci.h"
#include "HL_rti.h"
#include "HL_adc.h"
#include "HL_system.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "HL_reg_het.h"


#define top_drop hetPORT1,4
#define bottom_drop hetPORT1,9
#define remote_receive gioPORTA,7
#define fan_relay hetPORT1,8
#define tec_relay1 hetPORT1,24
#define tec_relay2 hetPORT1,26
#define drop_relay gioPORTB,2

#define lcd_rs gio
#define lcd_enable gio

#define distance_sensor_to_water_ground 16




// lcd data 6 7 8 9 순서대로 gio portB 0 1 2 5

uint8 bitcount[122]={0};
uint8 letter[12]={0};
uint8 letter1[12]={0xd7,0xd5,0xdD,0xd5,0xd7,0xd5,0xd7,0xd5,0xdD,0xdD,0xdD,0xd7};
uint8 letter2[12]={0xd5,0xdd,0xdd,0xd5,0xd7,0xd5,0xdd,0xd5,0xdD,0xdD,0xdD,0xd7};
uint8 letter3[12]={0xd7,0xd7,0xd7,0xd5,0xd5,0xdd,0xd5,0xd5,0xdD,0xdD,0xdD,0xd7};
uint8 letter4[12]={0xd5,0xd7,0xd5,0xd5,0xdd,0xd7,0xd7,0xd5,0xdD,0xdD,0xdD,0xd7};
uint8 letter5[12]={0xd5,0xd5,0xd5,0xd7,0xd5,0xdd,0xdd,0xdd,0xdD,0xdD,0xdD,0xd7};
uint8 letter6[12]={0xd7,0xd7,0xd5,0xd5,0xd7,0xd5,0xd5,0xdd,0xdD,0xdD,0xdD,0xd7};
uint8 letter7[12]={0xd7,0xd7,0xd7,0xd5,0xd5,0xd5,0xd5,0xd7,0xd7,0xd7,0xd7,0xd7};
uint8 letter8[12]={0xd7,0xd5,0xdD,0xd7,0xd5,0xd5,0xd7,0xd5,0xdD,0xd7,0xd7,0xd7};
uint8 letter9[12]={0xd7,0xd5,0xd7,0xd5,0xd5,0xd5,0xdd,0xdd,0xd7,0xd7,0xd7,0xd7};
uint8 letter0[12]={0xd7,0xd5,0xd7,0xd7,0xd5,0xd5,0xd7,0xd7,0xd5,0xd7,0xd7,0xd7};
uint8 letterstar[12]={0xd5,0xdd,0xdD,0xd7,0xd5,0xd5,0xdd,0xd5,0xdD,0xd7,0xd7,0xd7};
uint8 lettersharp[12]={0xd7,0xd5,0xdD,0xdd,0xd5,0xd5,0xd7,0xd5,0xd7,0xd7,0xd7,0xd7};
uint8 letterok[12]={0xd5,0xd7,0xd7,0xd7,0xd5,0xd5,0xdd,0xdd,0xd5,0xd7,0xd7,0xd7};
uint8 letterleft[12]={0xd5,0xd5,0xdD,0xd5,0xd5,0xdd,0xdd,0xdd,0xd7,0xd7,0xd7,0xd7};
uint8 letterright[12]={0xd5,0xdd,0xd7,0xd7,0xd5,0xd,0xd7,0xd5,0xdD,0xd5,0xdD,0xd7};
uint8 letterup[12]={0xd5,0xd5,0xdD,0xdd,0xd5,0xd7,0xd7,0xd7,0xd5,0xd7,0xd7,0xd7};
uint8 letterdown[12]={0xd5,0xdd,0xd5,0xdd,0xd7,0xd5,0xdd,0xd7,0xd7,0xd5,0xdD,0xd7};

int distance_counter=0;
double distance_array[200]={0};

uint8 drop_before, drop_time_counter;
uint32 drop_count=0,drop_time_avg;
uint32 drop_time_array[5];
uint8 drop_sens_timing=0;
uint8 time_gap_set=300;
double time_left=0;
uint8 drop_meantime=10;

uint8 remote_now=0, remote_value=0;

void decode_ir(void);

uint32 cnt,cnt2,start;

int fal=0;
volatile int decode=0;

char buf[256]={0};
unsigned int buf_len;

adcData_t adc_data[4];
double voltage=0, distance=0, distance_avg=0, water_height=0;
float temp=0;
uint32 value[4]={0};

void sci_display(sciBASE_t *, uint8 *, uint32);
void wait(uint32);

void delay_us(uint32 time){
    int i=28*time;
    while(i--)
        ;
}

void delay_ms(uint32 time){
    int i=27778*time;
    while(i--)
        ;
}

void lcd_command(unsigned char cmmd){

    gioSetBit(gioPORTA,5,((cmmd>>7) & 0x01));
    gioSetBit(gioPORTA,2,((cmmd>>6) & 0x01));
    gioSetBit(gioPORTA,1,((cmmd>>5) & 0x01));
    gioSetBit(gioPORTA,0,((cmmd>>4) & 0x01));

    delay_us(10);
    gioSetBit(gioPORTA,6,1);
    delay_us(200);
    gioSetBit(gioPORTA,6,0);
}

void lcd_char(unsigned char cmmd)
{
    gioSetBit(gioPORTA,5,((cmmd>>7) & 0x01));
    gioSetBit(gioPORTA,2,((cmmd>>6) & 0x01));
    gioSetBit(gioPORTA,1,((cmmd>>5) & 0x01));
    gioSetBit(gioPORTA,0,((cmmd>>4) & 0x01));

    gioSetBit(gioPORTA,6,1);
    delay_us(100);
    gioSetBit(gioPORTA,6,0);

    delay_us(20);

    gioSetBit(gioPORTA,5,((cmmd>>3) & 0x01));
    gioSetBit(gioPORTA,2,((cmmd>>2) & 0x01));
    gioSetBit(gioPORTA,1,((cmmd>>1) & 0x01));
    gioSetBit(gioPORTA,0,(cmmd & 0x01));

    gioSetBit(gioPORTA,6,1);
    delay_us(100);
    gioSetBit(gioPORTA,6,0);
    delay_us(20);
}

void lcd_string(char *str)
{
    int i=0;
    gioSetBit(gioPORTB,3,1);    //rs=1

    for(i=0;str[i]!=0;i++)
    {
        lcd_char(str[i]);
    }
}

void lcd_init(void){

    int i=0;
    // RS=0, E=0

    gioSetBit(gioPORTB,3,0);
    gioSetBit(gioPORTA,6,0);
    delay_ms(35);
    //func set
    for(i=0;i<2;i++)
    {
        lcd_command(0x30);
        delay_ms(1);
    }
    //lcd_command(0x30);
    //delay_us(150);
    //lcd_command(0x30);
    lcd_command(0x20);

    lcd_char(0x28);
    lcd_char(0x0c);
    lcd_char(0x01);
    lcd_char(0x06);
    lcd_char(0x80);

    //gioSetBit(gioPORTB,6, (0x00 & (0x10 >>4))); //bit moving is working
    delay_ms(5);


    lcd_string("hello");
    gioSetBit(gioPORTB, 6, 0);
    gioSetBit(gioPORTB,3,0);
    lcd_char(0xc0); // go to 2nd line
    lcd_string("HELLO LCD");
}

void lcd_display(void){
    char data[5]={0};
    int num=0,i=0;

    ///lcd 표기 초기화

    if((remote_now%2)==0){

    gioSetBit(gioPORTB,3,0);
    lcd_char(0x80);             //  first line
    lcd_string("T:");
    lcd_char((((int)time_left/10000)%10)+48);
    lcd_char((((int)time_left/1000)%10)+48);
    lcd_char((((int)time_left/100)%10)+48);
    lcd_char((((int)time_left/10)%10)+48);
    lcd_char(((int)time_left%10)+48);

    lcd_string(", Gap:");
    lcd_char((((time_gap_set)/100)%10)+48);
    lcd_string(".");
    lcd_char((((time_gap_set)/10)%10)+48);
    lcd_string("s");
/*
    //디스플레이 표시 숫자
    num=cnt;

    data[0]=(num/10000)%10;
    data[1]=(num/1000)%10;
    data[2]=(num/100)%10;
    data[3]=(num/10)%10;
    data[4]=num%10;

    gioSetBit(gioPORTB,3,0);
    lcd_char(0x88);             //  first line. 0x8f : last word. 0x88 9번째
    gioSetBit(gioPORTB,3,1);    //rs=1distance_sensor_to_water_ground 17.5
    for(i=0;i<5;i++)
    {
        lcd_char(data[i]+48);
    }

    lcd_char(223);  //degree display
    //lcd_char(48);
    //lcd_char(49);
     *
     *
     */

    gioSetBit(gioPORTB,3,0);
    lcd_char(0xc0);             // second line
    lcd_string("on for ");
    lcd_char((((int)cnt/10000)%10)+48);
    lcd_char((((int)cnt/1000)%10)+48);
    lcd_char((((int)cnt/100)%10)+48);
    lcd_string("sec   ");

    gioSetBit(fan_relay,0);
    gioSetBit(tec_relay1,0);
    gioSetBit(tec_relay2,0);

    }



    if((remote_now%2)==1){

        gioSetBit(gioPORTB,3,0);
        lcd_char(0x80);             //  first line
        lcd_string("water : ");

        lcd_char(0x88);             //  first line. 0x8f : last word. 0x88 9번째
        gioSetBit(gioPORTB,3,1);    //rs=1distance_sensor_to_water_ground 17.5
        for(i=0;i<1;i++)
        {
            lcd_char(((int)water_height/10)+48);
            lcd_char(((int)water_height%10)+48);
            lcd_string("  cm ");
        }

        gioSetBit(gioPORTB,3,0);
        lcd_char(0xc0);             // second line
        lcd_string("temperature: ");
        lcd_char(((int)temp/10)+48);
        lcd_char(((int)temp%10)+48);
        lcd_char(223);  //degree display
        lcd_string("C");

        gioSetBit(fan_relay,1);
        gioSetBit(tec_relay1,1);
        gioSetBit(tec_relay2,1);
    }
}

void display_clear(){   // doesnt work

    if((cnt%13)==1){

    gioSetBit(gioPORTB,3,0);
    lcd_char(0x80);             //  first line
    lcd_string("                ");
    gioSetBit(gioPORTB,3,0);
    lcd_char(0xc0);             // second line
    lcd_string("                ");
    }
}

void remote_data_calc()
{
    gioSetPort(gioPORTB, gioGetPort(gioPORTB)^ 0x0000080);
    int i=0;
    while(!((bitcount[i]==48)&&(bitcount[i+1]==49)&&(bitcount[i+2]==49)&&(bitcount[i+3]==49)&&(bitcount[i+4]==48)&&(bitcount[i+5]==49)&&(bitcount[i+6]==49)&&(bitcount[i+7]==49)&&(bitcount[i+8]==48)))
    {
        i++;
        if(i==50)
            break;
    }
/*
    sprintf(buf, "num = %d!\n\r\0",i);
    buf_len = strlen(buf);
    sci_display(sciREG1, (uint8 *)buf, buf_len);
*/
    letter[0]+=bitcount[i+32]*pow(2,3);
    letter[0]+=bitcount[i+33]*pow(2,2);
    letter[0]+=bitcount[i+34]*pow(2,1);
    letter[0]+=bitcount[i+35]*pow(2,0);
    letter[1]+=bitcount[i+36]*pow(2,3);
    letter[1]+=bitcount[i+37]*pow(2,2);
    letter[1]+=bitcount[i+38]*pow(2,1);
    letter[1]+=bitcount[i+39]*pow(2,0);

    letter[2]+=bitcount[i+40]*pow(2,3);
    letter[2]+=bitcount[i+41]*pow(2,2);
    letter[2]+=bitcount[i+42]*pow(2,1);
    letter[2]+=bitcount[i+43]*pow(2,0);
    letter[3]+=bitcount[i+44]*pow(2,3);
    letter[3]+=bitcount[i+45]*pow(2,2);
    letter[3]+=bitcount[i+46]*pow(2,1);
    letter[3]+=bitcount[i+47]*pow(2,0);

    letter[4]+=bitcount[i+48]*pow(2,3);
    letter[4]+=bitcount[i+49]*pow(2,2);
    letter[4]+=bitcount[i+50]*pow(2,1);
    letter[4]+=bitcount[i+51]*pow(2,0);
    letter[5]+=bitcount[i+52]*pow(2,3);
    letter[5]+=bitcount[i+53]*pow(2,2);
    letter[5]+=bitcount[i+54]*pow(2,1);
    letter[5]+=bitcount[i+55]*pow(2,0);

    letter[6]+=bitcount[i+56]*pow(2,3);
    letter[6]+=bitcount[i+57]*pow(2,2);
    letter[6]+=bitcount[i+58]*pow(2,1);
    letter[6]+=bitcount[i+59]*pow(2,0);
    letter[7]+=bitcount[i+60]*pow(2,3);
    letter[7]+=bitcount[i+61]*pow(2,2);
    letter[7]+=bitcount[i+62]*pow(2,1);
    letter[7]+=bitcount[i+63]*pow(2,0);

    letter[8]+=bitcount[i+64]*pow(2,3);
     letter[8]+=bitcount[i+65]*pow(2,2);
     letter[8]+=bitcount[i+66]*pow(2,1);
     letter[8]+=bitcount[i+67]*pow(2,0);
     letter[9]+=bitcount[i+68]*pow(2,3);
     letter[9]+=bitcount[i+69]*pow(2,2);
     letter[9]+=bitcount[i+70]*pow(2,1);
     letter[9]+=bitcount[i+71]*pow(2,0);

     letter[10]+=bitcount[i+72]*pow(2,3);
      letter[10]+=bitcount[i+73]*pow(2,2);
      letter[10]+=bitcount[i+74]*pow(2,1);
      letter[10]+=bitcount[i+75]*pow(2,0);
      letter[11]+=bitcount[i+76]*pow(2,3);
      letter[11]+=bitcount[i+77]*pow(2,2);
      letter[11]+=bitcount[i+78]*pow(2,1);
      letter[11]+=bitcount[i+79]*pow(2,0);
/*
      sprintf(buf," result : %x %x %x %x %x %x %x %x %x %x %x %x !\n\r\0",letter[0],letter[1],letter[2],letter[3],letter[4],letter[5],letter[6],letter[7],letter[8],letter[9],letter[10],letter[11]);
      buf_len = strlen(buf);
      sci_display(sciREG1, (uint8 *)buf, buf_len);
*/
      fal=0;
      for(i=0;i<12;i++)
      {
          if(letter[i]!=letterok[i])
              fal++;
      }

      if(fal<=1){
          gioSetPort(gioPORTB, gioGetPort(gioPORTB)^ 0x0000040);
          remote_now++;
      }

      i=0;
      for(i=0;i<12;i++)
          letter[i]=0;
      fal=0;
      decode=0;
}

void distance_calculation(void){
    int i=0;
    double total=0;
    distance_array[distance_counter%200]=distance;
    distance_counter++;
    if((distance_counter%200)==199){
        for(i=0;i<200;i++)
           total+=distance_array[i];
        distance_avg=total/200;
    }

    if(distance_counter==29999)
        distance_counter=0;

    water_height=(double)distance_sensor_to_water_ground-distance_avg;
}

void adc_conversion(void){
    adcStartConversion(adcREG1, adcGROUP1);
    while((adcIsConversionComplete(adcREG1, adcGROUP1)) == 0)
                ;
    adcGetData(adcREG1, adcGROUP1, &adc_data[0]);

    value[0] = adc_data[0].value;

    voltage = (double)value[0]/4095;
    voltage*=3.3;
    distance = 13.225 / (pow(voltage,1.157));
    distance_calculation();


    value[1] = adc_data[1].value;
    temp = (float)value[1]/16383;
    temp = -66.875 + (float)(217.75*3.3*temp);
}

void drop_sens(void){
    int i=0,j=0, drop_now=0;
    double total=0;

    for(j=0;j<2;j++){

        drop_now=gioGetBit(top_drop);
        if((drop_before==1)&&(drop_now==0)){
            drop_count++;
            drop_time_array[drop_count%5]=cnt;
            if((drop_count%5)==4){
                drop_time_avg=(drop_time_array[4]-drop_time_array[0])/4;

            }
        }
        drop_before=drop_now;
    }
}

double drop_uL_at_current_height(void){
    double series1=0, series2=0, series3=0, series4=0;
    series1=(double)2*(double)0.00001*water_height*water_height*water_height*1000;
    series2=(double)(0.0042)*water_height*water_height*100;
    series3=(double)0.7086*water_height*10;
    series4=(double)8.9434;

    return  series1-series2+series3+series4;
}

uint8 time_interval_now(void)
{
    double gap=0,b=56.67,a=3;

    gap=a*drop_uL_at_current_height();
    gap=(double)gap/b;

    time_gap_set=(uint8)(gap*100);

    return time_gap_set;
}

void time_calc(void)
{
    double c=1463, d=0;
    //int result=0;
    d=c*water_height;
    time_left=d;
}

int main(void)
{
/* USER CODE BEGIN (3) */
    int i=0;

    sciInit();
    adcInit();
    gioInit();
    rtiInit();
    wait(1000);
    gioSetDirection(gioPORTA, 0b01100111);
    gioSetDirection(gioPORTB, 0b11001100);
    gioSetDirection(hetPORT1, 0b0101000000000000000100000000);

    rtiEnableNotification(rtiREG1, rtiNOTIFICATION_COMPARE0);
    gioEnableNotification(remote_receive);
    _enable_IRQ_interrupt_();
    rtiStartCounter(rtiREG1,rtiCOUNTER_BLOCK0);

    //gioSetBit(gioPORTB,6, (0x01 & (0x10 >>4))); //bit moving is working

    /*
    gioSetBit(fan_relay,1);
    delay_ms(2000);
    gioSetBit(tec_relay1,1);
    delay_ms(2000);
    gioSetBit(tec_relay2,1);

*/

    lcd_init();

    for(;;)
    {
        if(decode==1)
        {
            remote_data_calc();
        }

        lcd_display();

        adc_conversion();

        // 아래는 드랍센싱 안되어서 5번 연속 드랍센싱만 한 코드. 이랬더니 물방울 센싱 된다. 00100 이나 01111100 이런식
/*
        sprintf(buf, "%d   %d \n\r",gioGetBit(top_drop),gioGetBit(bottom_drop));
        buf_len = strlen(buf);
        sci_display(sciREG1, (uint8 *) buf, buf_len);
        delay_us(10);

        sprintf(buf, "%d   %d \n\r",gioGetBit(top_drop),gioGetBit(bottom_drop));
        buf_len = strlen(buf);
        sci_display(sciREG1, (uint8 *) buf, buf_len);
        delay_us(10);
*/
        sprintf(buf, "calc_Gap= %d H : %lf T_left = %lf ch1: %lf   ch2 : %f  drop_count = %d, drop_gap : %d    \n\r\0",time_gap_set, water_height, time_left, distance , temp,drop_count, drop_time_avg );
         buf_len = strlen(buf);
         sci_display(sciREG1, (uint8 *) buf, buf_len);


        /*
        sprintf(buf, " ch1: %lf ch2 : %f  drop_count = %d, drop_gap : %d    \n\r\0",water_height , temp,drop_count, drop_time_avg );
        buf_len = strlen(buf);
        sci_display(sciREG1, (uint8 *) buf, buf_len);
  */

        //sci_display_txt(sciREG1,&value ,1); // &안해주면 값 안나온다.
        delay_us(10);

    }
    return 0;
}

void sci_display(sciBASE_t *sci, uint8 *txt, uint32 len)
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

void decode_ir(void){
    int i=0;
    //_disable_IRQ_interrupt_();

    while(gioGetBit(gioPORTA,7)==1){

    }

    for(i=0;i<122;i++)
    {
        delay_us(670);

        if(gioGetBit(gioPORTA,7)==1)
            bitcount[i]=1+48;
        else
            bitcount[i]=0+48;
    }

    decode=1;

    sci_display(sciREG1,bitcount,122);
    delay_us(10);
}

void gioNotification(gioPORT_t *port, uint32 bit)
{
    _disable_IRQ_interrupt_();

    //gioSetBit(gioPORTB, 6, 1);
    if(decode==0){
        decode_ir();
    }

    //_enable_IRQ_interrupt_();
}


void rtiNotification(rtiBASE_t *rtiREG, uint32 notification)
{
    drop_sens();

    if(start==1)
    {

        if(cnt2==drop_meantime){

            gioSetBit(drop_relay,0);
            start=0;
            cnt2=0;
        }
        cnt2++;
    }

    if(cnt%time_gap_set==(time_gap_set-1)){
    gioSetBit(drop_relay,1);
    gioSetBit(gioPORTB,6,gioGetBit(gioPORTB,6)^0x01);

    start=1;
    }

    cnt++;

    if((cnt%1000)==999){
        time_interval_now();
        if(cnt>2000)
        time_calc();

    }

    if(gioGetBit(gioPORTB,4)==0)
        rtiStopCounter(rtiREG1,rtiCOUNTER_BLOCK0);

}

