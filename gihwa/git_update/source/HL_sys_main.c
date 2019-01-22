#include "HL_sys_common.h"
#include "HL_gio.h"

int main(void)
{
/* USER CODE BEGIN (3) */
/* USER CODE END */



    gioInit();
    gioSetBit(gioPORTA,4,1);
    return 0;
}


/* USER CODE BEGIN (4) */
/* USER CODE END */
