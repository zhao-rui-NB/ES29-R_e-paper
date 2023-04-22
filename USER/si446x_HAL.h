#ifndef si446x_HAL_h
    #define si446x_HAL_h


#include "Nano100Series.h"

// pin define 
#define nSEL    PC0  //spi
#define SCLK    PC1  //spi
#define SDO     PC2  //spi
#define SDI     PC3  //spi 
#define SDN     PB2  
#define nIRQ    PB15  


void si446x_HAL_io_init(){
    GPIO_SetMode(PB , BIT2   , GPIO_PMD_OUTPUT);  // SDN  PB2
    GPIO_SetMode(PB , BIT15  , GPIO_PMD_INPUT);   // nIRQ PB15


    SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0, 8 , 500000);
    //SPI_EnableAutoSS(SPI0, SPI_SS0, SPI_SS0_ACTIVE_LOW);
    //SPI_DisableAutoSS(SPI0);
    //SPI_DisableFIFO(SPI0);

}
void si446x_HAL_write(uint8_t data){
    SPI_WRITE_TX0(SPI0 , data);
    SPI_TRIGGER(SPI0);
    while(SPI_IS_BUSY(SPI0)){}
}
void si446x_HAL_write_buf(uint8_t size , uint8_t* buf){
    for(int i=0 ; i<size ; i++){
        SPI_WRITE_TX0(SPI0 , *(buf++) );
        SPI_TRIGGER(SPI0);
        while(SPI_IS_BUSY(SPI0)){}
    }
}

uint8_t si446x_HAL_read_byte(){
    SPI_WRITE_TX0(SPI0 , 0x00);//send 0x00
    SPI_ClearRxFIFO(SPI0);
    SPI_TRIGGER(SPI0);
    while(SPI_IS_BUSY(SPI0)){}
    uint8_t data = SPI_READ_RX0(SPI0);
    return data;
}
void si446x_HAL_read_buf(uint8_t size , uint8_t* buf){
    for(int i=0 ; i<size ; i++){
        *(buf) = si446x_HAL_read_byte();
    }
}

void si446x_HAL_write_nSEL(uint8_t s){
    if(s)
        SPI_SET_SS0_HIGH(SPI0);
    else
        SPI_SET_SS0_LOW(SPI0);
}
void si446x_HAL_write_SDN(uint8_t s){SDN = s;CLK_SysTickDelay(1);}

uint8_t si446x_HAL_IS_IRQ(){
    return nIRQ == 0;
}

















#endif