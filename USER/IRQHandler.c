#include <stdio.h>
#include "Nano100Series.h"
void GPABC_IRQHandler(void)
{   
    PA->ISRC = PA->ISRC;
    PB->ISRC = PB->ISRC;
    PC->ISRC = PC->ISRC;
    printf("reset\n");
    CLK_SysTickDelay(1000);
    NVIC_SystemReset();
}