/** @file HL_sys_main.c
*   @brief Application main file
*   @date 07-July-2017
*   @version 04.07.00
*
*   This file contains an empty main function,
*   which can be used for the application.
*/

/* USER CODE BEGIN (0) */
/* USER CODE END */

/* Include Files */

#include "HL_sys_common.h"

/* USER CODE BEGIN (1) */
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <locale.h>
#include <wchar.h>
/* USER CODE END */

/** @fn void main(void)
*   @brief Application main function
*   @note This function is empty by default.
*
*   This function is called after startup.
*   The user can use this function to implement the application.
*/

/* USER CODE BEGIN (2) */
#include "HL_system.h"
#include "HL_sci.h"
#include "HL_gio.h"

#define UART    sciREG1

void sci_display(sciBASE_t *sci, uint32 *text, uint32 len);
uint32 sci_receive(sciBASE_t *sci);
void sci_scanf(sciBASE_t *sci, uint32 *rev_data);
uint32 UTF8toUnicode(uint32 *s);
bool chkhan(const char *ch);
bool chkHanUnicode(const uint32 *ch);
void BreakHan(uint32 *str, uint32 *buffer, uint8 nSize);
void check_msg(uint32 *data);
//void wait(int delay);

uint32 han_ch [999] = {0};
uint32 han_ch2 [999] = {0};
uint32 buffer [] = {0};
uint8 data_buffer [] = {0};



static uint32 wcF[] = {
    'ㄱ', 'ㄲ', 'ㄴ', 'ㄷ',
    'ㄸ', 'ㄹ', 'ㅁ', 'ㅂ',
    'ㅃ', 'ㅅ', 'ㅆ', 'ㅇ',
    'ㅈ', 'ㅉ', 'ㅊ', 'ㅋ',
    'ㅌ', 'ㅍ', 'ㅎ'};

//중성
static uint32 wcM[] = {
    'ㅏ', 'ㅐ', 'ㅑ', 'ㅒ',
    'ㅓ', 'ㅔ', 'ㅕ', 'ㅖ',
    'ㅗ', 'ㅘ', 'ㅙ', 'ㅚ',
    'ㅛ', 'ㅜ', 'ㅝ', 'ㅞ',
    'ㅟ', 'ㅠ', 'ㅡ', 'ㅢ', 'ㅣ'};

//종성
static uint32 wcL[] = {
    ' ', 'ㄱ', 'ㄲ', 'ㄳ',
    'ㄴ', 'ㄵ', 'ㄶ', 'ㄷ',
    'ㄹ', 'ㄺ', 'ㄻ', 'ㄼ',
    'ㄽ', 'ㄾ', 'ㄿ', 'ㅀ',
    'ㅁ', 'ㅂ', 'ㅄ', 'ㅅ',
    'ㅆ', 'ㅇ', 'ㅈ', 'ㅊ',
    'ㅋ', 'ㅌ', 'ㅍ', 'ㅎ'};
/* USER CODE END */


int main(void)
{
/* USER CODE BEGIN (3) */

    int i;
    sciInit();
    gioInit();

    while(1)
    {
       sci_scanf(UART, han_ch2); // 데이터 입력

       for(i = 0; han_ch2[i]; i++)
       {
           if(chkhan(&han_ch2[i]))
           {
               han_ch[0] = UTF8toUnicode(&han_ch2[i]); //한글 UTF8 - Unicode 변환]


              BreakHan(&han_ch[0], buffer, sizeof(buffer));
               /*printf("F = %#X\n", buffer[0]);
               printf("M = %#X\n", buffer[1]);
               printf("L = %#X\n", buffer[2]);

               printf("Han_ch0 = %#X\n", han_ch[0]);*/

               //F, M, L 각 3바이트 한 바이트씩 나누기
               //DivideHan(i);

               /*
               for(i = 0; buffer[i]; i++) // 한글 바이트 카운트
                           ;
               sci_display(UART, buffer, i);// 출력
               */

               for(i = 0; han_ch2[i]; i++) // 한글 바이트 카운트
                    ;
               sci_display(UART, han_ch2, i);// 출력
           }
           else
           {
               sci_display(UART, &han_ch2[i], 1);
               buffer[i] = han_ch2[i];
           }

       }

       check_msg(buffer);
       memset(han_ch2, 0, sizeof(han_ch2)); // 버퍼 초기화
       memset(buffer, 0, sizeof(buffer));
       memset(han_ch, 0, sizeof(han_ch));


    }
/* USER CODE END */

    return 0;
}

/* USER CODE BEGIN (4) */
void sci_display(sciBASE_t *sci, uint32 *text, uint32 len)
{
    while(len--)
    {
        while((UART->FLR & 0x4) == 4)
            ;

        while ((sci->FLR & (uint32)SCI_TX_INT) == 0U)
            ; /* Wait */
        sci->TD = *text++;
    }
}

uint32 sci_receive(sciBASE_t *sci)
{
    while ((sci->FLR & (uint32)SCI_RX_INT) == 0U)
            ; /* WAIT */

    return (sci->RD & (uint32)0x00FFFFFFU); // return (uint32)RD[0~7]
}

#if 1
void sci_scanf(sciBASE_t *sci, uint32 *rev_data)
{
    int idx = 0;

    while(1)
    {
        rev_data[idx] = sci_receive(sci);
        if(rev_data[idx] == '\n' || rev_data[idx] == '\r')
            break;
        idx++;
    }
    idx = 0;


}
#endif

uint32 UTF8toUnicode(uint32 *s)
{
    char c = s[0];
    uint32 ws = 0, wc = 0;

    /*Debug*/
    printf("\n");
    printf("s[0] = %d %#X\n", s[0], s[0]);
    printf("s[1] = %d %#X\n", s[1], s[1]);
    printf("s[2] = %d %#X\n", s[2], s[2]);
    /*End*/

    if((c & 0xE0) == 0xE0)
    {
        wc = (s[0] & 0x0F) << 12;
        printf("s[0] = %d, %#X\n", wc, wc);

        wc |= (s[1] & 0x3F) << 6;
        printf("s[1] = %d, %#X\n", wc, wc);

        wc |= (s[2] & 0x3F);
        printf("s[2] = %d, %#X\n", wc, wc);


    }
    ws += wc;

    printf("ws = %d, %#X\n", ws , ws);

    return ws;

}

#if 1
bool chkhan(const char *ch)              // for double byte code
{
    if((*ch & 0x80) == 0x80)
        return  true; // 최상위 비트 1 인지 검사
    else return false;
}

bool chkHanUnicode(const uint32 *ch) // for unicode
{
    return !(ch < 44032 || ch > 55199); // 0xAC00(가) ~ 0xD7A3(힣)
}
#endif

// 초성 중성 종성 분리
void BreakHan(uint32 *str, uint32 *buffer, uint8 nSize)
{

    uint8 pos = 0;
/*
    while(*str != '\0' || *str != '\r' || *str != '\n')
    {

        if(*str < 256)
        {
            if(pos+2 >= nSize-1)
                break;

            buffer[pos] = *str;
            ++pos;
        }
        else
        {
            if(pos+4 >= nSize-1)
                break;
         */
            printf("str = %#X\n", *str);

            buffer[pos] = wcF[(*str - 0xAC00) / (21*28)];
            printf("a : %d\n", (*str - 0xAC00) / (21*28));
            printf("buffer0 = %#X\n", buffer[0]);

            buffer[pos+1] = wcM[(*str - 0xAC00) % (21 * 28) / 28];
            printf("b : %d\n", (*str - 0xAC00) % (21 * 28) / 28);
            printf("buffer1 = %#X\n", buffer[1]);

            buffer[pos+2] = wcL[(*str - 0xAC00) % 28];
            printf("c : %d\n", (*str - 0xAC00) % 28);
            printf("buffer2 = %#X\n", buffer[2]);

            pos+=3;
       // }

        //++str;
    //}

    buffer[pos] = '\0';
   // return pos;
}

void check_msg(uint32 *data)
{
    if( (*data) == '가' )
    {
        gioSetBit(gioPORTA, 1, 1);
    }

    else if( (*data) == 'ㄱ')
    {
        gioSetBit(gioPORTA, 2, 1);
    }

    else if( (*data) == 'a')
    {
        gioSetBit(gioPORTA, 3, 1);
    }
    else
    {
        gioSetBit(gioPORTA, 1, 0);
        gioSetBit(gioPORTA, 2, 0);
        gioSetBit(gioPORTA, 3, 0);
    }
}

#if 0
void wait(int delay)
{
    int i;
    for(i = 0; i < delay; i++)
        ;
}
#endif
/* USER CODE END */
