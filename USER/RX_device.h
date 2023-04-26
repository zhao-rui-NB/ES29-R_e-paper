#ifndef RX_device_h
    #define RX_device_h
    #include "ES29R_cmd_def.h"
    #include "UC8581_driver.h"




uint8_t frame_buffer_black[4740];
uint8_t frame_buffer_red  [4740];

void rx_device_loop(){
//get uid[0]
    SYS_UnlockReg();
    FMC_Open();
    uint32_t UID = FMC_ReadUID(0);
    printf("****UID = 0x%x  ****\n\n" , UID);
    SYS_LockReg();

    UC8151_io_init();

    UC8151_init();
    UC8151_send_frame(NULL , (uint8_t*)img);
    UC8151_wait_busy();
    POWER_EN = POWER_OFF;
    printf("while  1");//while(1);


    uint8_t wait_for_power_off_epd = 0;


    start_rx(0,23);
    while(1){
        if(si446x_HAL_IS_IRQ()){
            uint8_t read_buf[23];
            get_int_status(NULL);
            read_rx_fifo(23,read_buf);
            //printf("received !! : ");
            //for(int i=0 ; i<23 ; i++){
            //    printf("0x%x , " , read_buf[i]);
            //}
            //printf("\n");
            //
            
            uint32_t packet_ID = 0;
            for(int i=0 ; i<4 ; i++){
                packet_ID = packet_ID<<8 | read_buf[i];  
            }
            //printf("packed id : 0x%x\n" , packet_ID);
            
            if(packet_ID == UID || packet_ID==0xFFFFFFFF){//
                switch(read_buf[4]){
                    case ES29R_CMD_write_reg:
                        uint16_t reg_addr = read_buf[5]<<8 | read_buf[6];
                        printf("w addr=0x%x\n",reg_addr);
                        for(int i=0 ; i<16 ; i++){
                            if( (reg_addr&0x7FFF)+i < 4736){
                                if(reg_addr & 0x8000){
                                    //printf("red reg\n");
                                    frame_buffer_red[(reg_addr&0x7FFF)+i] = read_buf[7+i];
                                }
                                else{
                                    //printf("black reg\n");
                                    frame_buffer_black[(reg_addr&0x7FFF)+i] = read_buf[7+i];
                                }
                            }
                        }
                        break;
                    case ES29R_CMD_frame_update:
                        printf("frame update !!\n");
                        wait_for_power_off_epd = 1;
                        UC8151_init();
                        UC8151_send_frame( frame_buffer_black , frame_buffer_red );
                        break;
                }
            }
            
            
            

        }
        if(wait_for_power_off_epd && !UC8151_IS_busy()){
            POWER_EN = POWER_OFF;
            wait_for_power_off_epd = 0;
            printf("epd power off");
        }
    }
}



#endif