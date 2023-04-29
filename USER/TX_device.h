#ifndef TX_device_h
    #define TX_device_h
    #include "ES29R_cmd_def.h"

char ascii_cmd_buffer[128];
char ch_cnt = 0;
uint8_t ascii_cmd_received = 0;

void ascii_cmd_decoder(){
    
}

void tx_device_loop(){
    while(1){
        if(!UART_GET_RX_EMPTY(UART0)){
            char ch = UART_READ(UART0);
            //UART_WRITE(UART0 , ch);

            if(ch=='\r' || ch=='\n'){
                ascii_cmd_buffer[ch_cnt]   = 0;
                ascii_cmd_buffer[ch_cnt+1] = 0;
                if(ch_cnt!=0){
                    ascii_cmd_received = 1;
                    ch_cnt = 0;
                }

            }
            else if(ch>='a' && ch<='f'){
                ascii_cmd_buffer[ch_cnt++] = 'A' + (ch-'a');
            }
            else if(ch>='A' && ch<='F'){
                ascii_cmd_buffer[ch_cnt++] = ch;
            }
            else if(ch>='0' && ch<='9'){
                ascii_cmd_buffer[ch_cnt++] = ch;
            }


            if(ascii_cmd_received){
                ascii_cmd_received = 0;
                //printf("st\n");
                //printf("\nstart send data");
                //for(int i=0 ; ascii_cmd_buffer[i]!=NULL ; i++){
                //    printf("%c" , ascii_cmd_buffer[i]);
                //}
                //printf("\n");
                //for(char* ch = ascii_cmd_buffer ; ch!=NULL ; ch++){printf("0x");}

                uint8_t packet[23];
                uint8_t byte_cnt=0;
                uint8_t half_byte=0;
                for(int i=0 ; ascii_cmd_buffer[i]!=0 ; i++){
                    uint8_t num = 0;
                    switch(ascii_cmd_buffer[i]){
                        case '0': num=0;  break;
                        case '1': num=1;  break;
                        case '2': num=2;  break;
                        case '3': num=3;  break;
                        case '4': num=4;  break;
                        case '5': num=5;  break;
                        case '6': num=6;  break;
                        case '7': num=7;  break;
                        case '8': num=8;  break;
                        case '9': num=9;  break;
                        case 'A': num=10; break;
                        case 'B': num=11; break;
                        case 'C': num=12; break;
                        case 'D': num=13; break;
                        case 'E': num=14; break;
                        case 'F': num=15; break;
                    }
                    if(half_byte==0){
                        packet[byte_cnt] = num;
                        half_byte = 1;
                    }
                    else{
                        packet[byte_cnt] = (packet[byte_cnt]<<4) | num;
                        half_byte = 0;
                        byte_cnt++;
                    }
                }

                //print packet 
                //printf("packet :");for(int i=0  ;i<23 ;i++)printf(" 0x%x," , packet[i]); printf("\n");

                get_int_status(NULL);
                write_tx_fifo(23,packet);
                start_tx(0,NULL,23);
                //printf("wait TX :");
                while(!si446x_HAL_IS_IRQ());
                printf("ok\n");
                
            }
        }
    }
}






#endif