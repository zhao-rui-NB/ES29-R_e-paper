//radio_comm_GetResp
//radio_comm_SendCmd
//radio_comm_ReadData
//radio_comm_WriteData
//radio_comm_PollCTS
//radio_comm_ClearCTS
//radio_comm_GetResp
//radio_comm_GetResp
//radio_comm_SendCmdGetResp
//
//

//void si446x_HAL_ctrl_

#ifndef si446x_HAL_ctrl_h
    #define si446x_HAL_ctrl_h

    
#include "si446x_cmd_def.h"
#include "si446x_HAL.h"
#include "radio_config.h"

uint8_t config_array[] = RADIO_CONFIGURATION_DATA_ARRAY;

void si444x_HAL_ctrl_wait_CTS(){
    while(1){
        CLK_SysTickDelay(100);

        uint8_t CTS = 0;
        si446x_HAL_write_nSEL(0);
        uint8_t cmd = CMD_READ_CMD_BUF;
        si446x_HAL_write_buf(1 , &cmd);
        CTS = si446x_HAL_read_byte();
        si446x_HAL_write_nSEL(1);

        if(CTS==0xFF){
            return;
        }
    }
}
void si444x_HAL_ctrl_wait_CTS_keep_sel(){
    while(1){
        CLK_SysTickDelay(100);

        uint8_t CTS = 0;
        si446x_HAL_write_nSEL(0);
        uint8_t cmd = CMD_READ_CMD_BUF;
        si446x_HAL_write_buf(1 , &cmd);
        CTS = si446x_HAL_read_byte();

        if(CTS==0xFF){
            return;
        }
        si446x_HAL_write_nSEL(1);
    }
}
void si446x_HAL_ctrl_send_cmd(uint8_t cmd , uint8_t arg_size , uint8_t* arg){
    si444x_HAL_ctrl_wait_CTS();
    si446x_HAL_write_nSEL(0);
    si446x_HAL_write(cmd);
    si446x_HAL_write_buf(arg_size,arg);
    si446x_HAL_write_nSEL(1);
}
void si446x_HAL_ctrl_send_cmd_keep_sel(uint8_t cmd , uint8_t arg_size , uint8_t* arg){
    si444x_HAL_ctrl_wait_CTS();
    si446x_HAL_write_nSEL(0);
    si446x_HAL_write(cmd);
    si446x_HAL_write_buf(arg_size,arg);
    //si446x_HAL_write_nSEL(1);
}

void si446x_HAL_ctrl_get_reply(uint8_t replay_size , uint8_t* replay){
    si444x_HAL_ctrl_wait_CTS_keep_sel();
    for(int i=0 ; i<replay_size  ; i++){
        *(replay+i) = si446x_HAL_read_byte();
    }
    si446x_HAL_write_nSEL(1);
}
void si446x_HAL_ctrl_get_reply_no_check_CTS(uint8_t replay_size , uint8_t* replay){
    si446x_HAL_write_nSEL(0);
    for(int i=0 ; i<replay_size  ; i++){
        uint8_t data = si446x_HAL_read_byte();
        replay[i] = data;
    }
    si446x_HAL_write_nSEL(1);
}
void si446x_HAL_ctrl_get_reply_no_check_CTS_no_sel(uint8_t replay_size , uint8_t* replay){
    //si446x_HAL_write_nSEL(0);
    for(int i=0 ; i<replay_size  ; i++){
        uint8_t data = si446x_HAL_read_byte();
        replay[i] = data;
    }
    //si446x_HAL_write_nSEL(1);
}



//
void si446x_init(){
    si446x_HAL_io_init();
    si446x_HAL_write_nSEL(1);
    
    si446x_HAL_write_SDN(0);
    CLK_SysTickDelay(100);//min 10us
    si446x_HAL_write_SDN(1);
    CLK_SysTickDelay(100);//min 10us
    si446x_HAL_write_SDN(0);
    CLK_SysTickDelay(100);//min 10us

    CLK_SysTickDelay(14000);//14ms to reset

    //start send config array 
    uint8_t* cmd_ptr = config_array;
    while(cmd_ptr){
        uint8_t size = *cmd_ptr;
        if(size==0){
            break;
        }
        cmd_ptr++;
        si444x_HAL_ctrl_wait_CTS();
        si446x_HAL_write_nSEL(0);
        si446x_HAL_write_buf(size,cmd_ptr);
        si446x_HAL_write_nSEL(1);
        cmd_ptr+=size;
        if(si446x_HAL_IS_IRQ()){
            uint8_t arg[] = {0,0,0};
            si446x_HAL_ctrl_send_cmd(CMD_GET_INT_STATUS,3,arg);
        }
    }
}
void si446x_HAL_ctrl_(){}












#endif