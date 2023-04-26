#ifndef si446x_api_h
    #define si446x_api_h


#include "si446x_cmd_def.h"

#include "si446x_HAL_ctrl.h"

//#define DEBUG

void get_part_info(si446x_reply_PART_INFO_map* reply){
    int reply_len = 8;
    uint8_t buf[8];
    si446x_reply_PART_INFO_map r;
    printf("sizeof : %d\n" , reply_len);
    if(reply==NULL){
        reply = &r;
    }
    si446x_HAL_ctrl_send_cmd(CMD_PART_INFO,0,NULL);
    si446x_HAL_ctrl_get_reply(reply_len,buf);
    reply->CHIPREV  = buf[0] ;
    reply->PART     = buf[1]<<8 | buf[2];
    reply->PBUILD   = buf[3] ;
    reply->ID       = buf[4]<<8 | buf[5];
    reply->CUSTOMER = buf[6] ;
    reply->ROMID    = buf[7] ;
    #ifdef DEBUG
        printf("reply.CHIPREV : 0x%x\n" ,reply->CHIPREV ); 
        printf("reply.PART    : 0x%x\n" ,reply->PART    ); 
        printf("reply.PBUILD  : 0x%x\n" ,reply->PBUILD  ); 
        printf("reply.ID      : 0x%x\n" ,reply->ID      ); 
        printf("reply.CUSTOMER: 0x%x\n" ,reply->CUSTOMER); 
        printf("reply.ROMID   : 0x%x\n" ,reply->ROMID   ); 
    #endif
}
void get_request_device_state(si446x_reply_REQUEST_DEVICE_STATE_map* reply){
    int reply_len = 2;
    uint8_t buf[2];
    si446x_reply_REQUEST_DEVICE_STATE_map r;
    if(reply==NULL){
        reply = &r;
    }
    si446x_HAL_ctrl_send_cmd(CMD_REQUEST_DEVICE_STATE,0,NULL);
    si446x_HAL_ctrl_get_reply(reply_len,buf);
    reply->CURR_STATE      = buf[0];
    reply->CURRENT_CHANNEL = buf[1];
    #ifdef DEBUG
        printf("reply->CURR_STATE       : 0x%x\n" ,reply->CURR_STATE      ); 
        printf("reply->CURRENT_CHANNEL  : 0x%x\n" ,reply->CURRENT_CHANNEL ); 
    #endif
}
void get_int_status(si446x_reply_GET_INT_STATUS_map* reply){
    int reply_len = 8;
    uint8_t buf[8];
    si446x_reply_GET_INT_STATUS_map r;
    if(reply==NULL){
        reply = &r;
    }
    uint8_t arg[] = {0,0,0};
    si446x_HAL_ctrl_send_cmd(CMD_GET_INT_STATUS,3,arg);
    si446x_HAL_ctrl_get_reply(reply_len,buf);
    reply->INT_PEND       = buf[0];
    reply->INT_STATUS     = buf[1];
    reply->PH_PEND        = buf[2];
    reply->PH_STATUS      = buf[3];
    reply->MODEM_PEND     = buf[4];
    reply->MODEM_STATUS   = buf[5];
    reply->CHIP_PEND      = buf[6];
    reply->CHIP_STATUS    = buf[7];
    #ifdef DEBUG
        printf("reply->INT_PEND      : 0x%x\n" ,reply->INT_PEND     ); 
        printf("reply->INT_STATUS    : 0x%x\n" ,reply->INT_STATUS   ); 
        printf("reply->PH_PEND       : 0x%x\n" ,reply->PH_PEND      ); 
        printf("reply->PH_STATUS     : 0x%x\n" ,reply->PH_STATUS    ); 
        printf("reply->MODEM_PEND    : 0x%x\n" ,reply->MODEM_PEND   ); 
        printf("reply->MODEM_STATUS  : 0x%x\n" ,reply->MODEM_STATUS ); 
        printf("reply->CHIP_PEND     : 0x%x\n" ,reply->CHIP_PEND    ); 
        printf("reply->CHIP_STATUS   : 0x%x\n" ,reply->CHIP_STATUS  ); 
    #endif


}

void get_packet_info(si446x_reply_PACKET_INFO_map* reply){
    si446x_reply_PACKET_INFO_map r;
    if(reply==NULL){
        reply = &r;
    }
    uint8_t buf[2];
    uint8_t arg[] = {0,0,0,0,0};
    si446x_HAL_ctrl_send_cmd(CMD_GET_PACKET_INFO,5,arg);
    si446x_HAL_ctrl_get_reply(2,buf);
    reply->LENGTH = buf[0]<<8 | buf[1];
    printf("\npacket len : %d \n" , reply->LENGTH);
}


void write_tx_fifo(uint8_t size , uint8_t* ptr){
    //si444x_HAL_ctrl_wait_CTS();
    si446x_HAL_ctrl_send_cmd(CMD_WRITE_TX_FIFO,size,ptr);
}
void start_tx(uint8_t ch, uint8_t complete_state , uint8_t tx_len){
    uint8_t arg[6];
    arg[0] = ch;
    arg[1] = (STATE_READY<<4);
    arg[2] = tx_len>>8;
    arg[3] = tx_len&0xff;
    arg[4] = 0;
    arg[5] = 0;
    si446x_HAL_ctrl_send_cmd(CMD_START_TX,6,arg);
}
void start_rx(uint8_t ch , uint16_t len){
    uint8_t arg[7];
    arg[0] = ch;
    arg[1] = 0;//go into rx now
    arg[2] = len>>8;
    arg[3] = len&0xff;
    arg[4] = 0;
    arg[5] = 8;
    arg[6] = 8;
    si446x_HAL_ctrl_send_cmd(CMD_START_RX,7,arg);
}

void read_rx_fifo(uint16_t size , uint8_t* buf){
    si446x_HAL_ctrl_send_cmd_keep_sel(CMD_READ_RX_FIFO , 0 , NULL);
    si446x_HAL_ctrl_get_reply_no_check_CTS(size , buf);
}




#endif