#ifndef UC8151_driver_h
    #define UC8151_driver_h

#include <stdio.h>
//#include "Nano100Series.h"
#include "img.h"

//pin define
#define BUSYn   PA5
#define RESn    PA4
#define DCn     PB12
#define CSn     PA8
#define SCL     PA9
#define SDA     PA11
#define POWER_EN PB4

#define UC8151_cmd  0
#define UC8151_data 1

#define POWER_ON   0
#define POWER_OFF  1

// Display resolution
#define EPD_WIDTH       128
#define EPD_HEIGHT      296

//reg set----
//color define
//RW       frame reg
//00 :black  
//01 :white
//10 :red
//11 :red

//when bwr mode and DDX[0]==1
#define BORDER_BLACK    0x0   //0b00
#define BORDER_WHITE    0x1   //0b01
#define BORDER_RED      0x2   //0b10
#define VBD BORDER_BLACK//border data sel//(border color)
#define DDX 0x1//data polality//

 


void UC8151_io_init(){
    GPIO_SetMode(PA , BIT5  , GPIO_PMD_INPUT);  // BUSY
    GPIO_SetMode(PA , BIT4  , GPIO_PMD_OUTPUT);  // RESn
    GPIO_SetMode(PB , BIT12 , GPIO_PMD_OUTPUT);  // DCn 
    //GPIO_SetMode(PA , BIT8  , GPIO_PMD_OUTPUT);  // CSn 
    //GPIO_SetMode(PA , BIT9  , GPIO_PMD_OUTPUT);  // SCL 
    //GPIO_SetMode(PA , BIT11 , GPIO_PMD_OUTPUT);  // SDA 
    GPIO_SetMode(PB , BIT4  , GPIO_PMD_OUTPUT);  //POWER_EN 

    SPI_Open(SPI2, SPI_MASTER, SPI_MODE_0, 8 , 5000000);
    SPI_EnableAutoSS(SPI2, SPI_SS0, SPI_SS0_ACTIVE_LOW);
}


void wait_busy(){
    while (!BUSYn){}
}
void UC8151_write(uint8_t type , uint8_t payload){
    DCn = type;
    //CLK_SysTickDelay(1);
    SPI_WRITE_TX0(SPI2 , payload);
    SPI_TRIGGER(SPI2);
    while(SPI_IS_BUSY(SPI2)){}
    //CLK_SysTickDelay(1);
}


void UC8151_init(){
    
    POWER_EN = 0;
    CLK_SysTickDelay(1000);

    RESn = 1;
    CLK_SysTickDelay(1000);
    RESn = 0;
    CLK_SysTickDelay(1000);
    RESn = 1;
    CLK_SysTickDelay(1000);

    UC8151_write(UC8151_cmd ,0x04);//power on
    CLK_SysTickDelay(100);
    wait_busy();

    UC8151_write(UC8151_cmd  , 0x00);//panel setting
    UC8151_write(UC8151_data , 0x0f);//default data
    UC8151_write(UC8151_data , 0x89);//128x296,Temperature sensor, boost and other related timing settings


    UC8151_write(UC8151_cmd  , 0x61);//Display resolution setting // must set
    UC8151_write(UC8151_data , 0x80);
    UC8151_write(UC8151_data , 0x01);
    UC8151_write(UC8151_data , 0x28);

    UC8151_write(UC8151_cmd  , 0X50);//VCOM AND DATA INTERVAL SETTING       // color define
    UC8151_write(UC8151_data , (VBD<<6)|(DDX<<4)|0x07 );   //WBmode:VBDF 17|D7 VBDW 97 VBDB 57   
                                        //WBRmode:VBDF F7 VBDW 77 VBDB 37  VBDR B7
                                        //[7:6] 11:F 01:W 00:B 10:R
                                        //[5:4]    
    //test 
    //UC8151_write(UC8151_cmd  , 0x30);//pll
    //UC8151_write(UC8151_data , 0x3C);
    //UC8151_write(UC8151_data , 0x0F);
}

void UC8151_send_frame(const uint8_t* black_img ,const uint8_t* red_img){
    UC8151_write(UC8151_cmd,0x10);//start send black/white
    for(int y=0 ; y<EPD_HEIGHT ; y++){
        for(int x=0 ; x<EPD_WIDTH ; x++){
            if(black_img)
                UC8151_write(UC8151_data , black_img[ y*EPD_WIDTH + x]);
            else
                UC8151_write(UC8151_data , 0xFF);
        }
    }

    UC8151_write(UC8151_cmd , 0x13);//start send red
    for(int y=0 ; y<EPD_HEIGHT ; y++){
        for(int x=0 ; x<EPD_WIDTH ; x++){
            if(red_img)
                UC8151_write(UC8151_data , red_img[ y*EPD_WIDTH + x] );
            else
                UC8151_write(UC8151_data , 0x00);
        }
    }
    UC8151_write(UC8151_cmd , 0x12);
    //wait_busy();  
}
    









#endif //#ifndef UC8151_driver_h
