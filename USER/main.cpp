#include <stdio.h>
#include "Nano100Series.h"
#include "UC8581_driver.h"
#include "si446x_api.h"
//#include <sys.h>

#include "RX_device.h"
#include "TX_device.h"

#define RX_DEV
//#define TX_DEV

void SYS_Init(void){
    SYS_UnlockReg();

	CLK_EnableXtalRC( CLK_PWRCTL_LXT_EN_Msk | CLK_PWRCTL_HIRC_EN_Msk);
		
	
	CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC,CLK_HCLK_CLK_DIVIDER(1));
	
		
    CLK_SetCoreClock(42000000);
		
	SystemCoreClockUpdate();


    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HIRC, CLK_UART_CLK_DIVIDER(1));
    CLK_SetModuleClock(SPI2_MODULE , CLK_CLKSEL2_SPI0_S_HCLK, 0);


    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(SPI2_MODULE);
    CLK_EnableModuleClock(SPI0_MODULE);//si446x
    
    
    
    SystemCoreClockUpdate();


    //set UART0 pin 
    SYS->PA_H_MFP &= ~( SYS_PA_H_MFP_PA15_MFP_Msk | SYS_PA_H_MFP_PA14_MFP_Msk);
    SYS->PA_H_MFP |= (SYS_PA_H_MFP_PA15_MFP_UART0_TX|SYS_PA_H_MFP_PA14_MFP_UART0_RX);
    
    //set spi2 pin
    SYS->PA_H_MFP &= ~(SYS_PA_H_MFP_PA11_MFP_Msk        | SYS_PA_H_MFP_PA9_MFP_Msk       | SYS_PA_H_MFP_PA9_MFP_Msk);
    SYS->PA_H_MFP |=   SYS_PA_H_MFP_PA11_MFP_SPI2_MOSI0 | SYS_PA_H_MFP_PA9_MFP_SPI2_SCLK | SYS_PA_H_MFP_PA8_MFP_SPI2_SS0;
    
    //set spi0 pin //for si4460
    SYS->PC_L_MFP &= ~(SYS_PC_L_MFP_PC0_MFP_SPI0_SS0 | SYS_PC_L_MFP_PC1_MFP_SPI0_SCLK | SYS_PC_L_MFP_PC2_MFP_SPI0_MISO0 | SYS_PC_L_MFP_PC3_MFP_SPI0_MOSI0);
    SYS->PC_L_MFP |=   SYS_PC_L_MFP_PC0_MFP_SPI0_SS0 | SYS_PC_L_MFP_PC1_MFP_SPI0_SCLK | SYS_PC_L_MFP_PC2_MFP_SPI0_MISO0 | SYS_PC_L_MFP_PC3_MFP_SPI0_MOSI0 ;
    
    SYS_LockReg();
}

void delay_30_second(){
    for(int i=0 ; i<3000 ; i++){
         CLK_SysTickDelay(10000);
    }
}
void delay_10_second(){
    for(int i=0 ; i<1000 ; i++){
         CLK_SysTickDelay(10000);
    }
}




int main(){
		
    SYS_Init();
    UART_Open(UART0, 115200);
	
    printf("start si446x init\n");
    si446x_init();
    get_int_status(NULL);
    printf("init finish !! \n");

    #ifdef TX_DEV 
        tx_device_loop();
    #endif

    #ifdef RX_DEV
        rx_device_loop();
    #endif
}

    
    
    
    /*
    UC8151_io_init();
    
    while(1){
        printf("start send frame 1\n");
        UC8151_init();
        UC8151_send_frame(img , NULL);
        wait_busy();
        POWER_EN = POWER_OFF;
        printf("finish \n\n");

        delay_30_second();

        printf("start send frame 2\n");
        UC8151_init();
        UC8151_send_frame(NULL , img);
        wait_busy();
        POWER_EN = POWER_OFF;
        printf("finish \n\n");

        delay_30_second();
    }
    */

