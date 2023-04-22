#ifndef si446x_cmd_def_h
#define si446x_cmd_def_h

#define CMD_NOP                                             0x00
#define CMD_PART_INFO                                       0x01
#define CMD_POWER_UP                                        0x02
#define CMD_SET_PROPERTY                                    0x11
#define CMD_GET_PROPERTY                                    0x12
#define CMD_GPIO_PIN_CFG                                    0x13
#define CMD_FIFO_INFO                                       0x15
#define CMD_GET_INT_STATUS                                  0x20
#define CMD_GET_PH_STATUS                                   0x21
#define CMD_GET_MODEM_STATUS                                0x22
#define CMD_START_TX                                        0x31
#define CMD_START_RX                                        0x32
#define CMD_REQUEST_DEVICE_STATE                            0x33
#define CMD_CHANGE_STATE                                    0x34
#define CMD_READ_CMD_BUF                                    0x44
#define CMD_WRITE_TX_FIFO                                   0x66
#define CMD_READ_RX_FIFO                                    0x77

#define BOOT_OPT_NORMAL                                     0x01

#define XTAL_OPT_XTAL                                       0
#define XTAL_OPT_TCXO                                       1

#define GPIO_LOW                                            0x02
#define GPIO_HIGH                                           0x03

#define TX_COMPLETE_STATE_RDY                               0x30
#define TX_START_IMMEDIATE                                  0x00
#define TX_START_WAKEUP                                     0x01
#define TX_RETRANSMIT_NO                                    0x00
#define TX_RETRANSMIT_YES                                   0x02

#define STATE_NO_CHANGE                                     0x00
#define STATE_SLEEP                                         0x01
#define STATE_SPI_ACTIVE                                    0x02
#define STATE_READY                                         0x03
//#define STATE_READY_2                                     0x04
#define STATE_TUNE_TX                                       0x05
#define STATE_TUNE_RX                                       0x06
#define STATE_TX                                            0x07
#define STATE_RX                                            0x08

#define CLR_RX_FIFO                                         0x02
#define CLR_TX_FIFO                                         0x01


#define PROP_INT_CTL_GROUP                                  0x01
#define PROP_INT_CTL_ENABLE                                 0x00
#define PROP_INT_CTL_PH_ENABLE                              0x01
#define PROP_INT_CTL_MODEM_ENABLE                           0x02
#define PROP_INT_CTL_CHIP_ENABLE                            0x03

#define CHIP_INT_STATUS_EN                                  0x04
#define MODEM_INT_STATUS_EN                                 0x02
#define PH_INT_STATUS_EN                                    0x01

#define FILTER_MATCH                                        0x80
#define FILTER_MISS                                         0x40
#define PACKET_SENT                                         0x20
#define PACKET_RX                                           0x10
#define CRC_ERROR                                           0x08
#define TX_FIFO_ALMOST_EMPTY                                0x02
#define RX_FIFO_ALMOST_FULL                                 0x01

#define PROP_MODEM_GROUP                                    0x20
#define PROP_MODEM_MOD_TYPE                                 0x00
#define PROP_MODEM_FREQ_DEV_2                               0x0A
#define PROP_MODEM_FREQ_DEV_1                               0x0B
#define PROP_MODEM_FREQ_DEV_0                               0x0C
#define PROP_MODEM_FREQ_OFFSET_1                            0x0D
#define PROP_MODEM_FREQ_OFFSET_0                            0x0E

#define MODEM_RSSI_COMP                                     0x4E

#define PROP_MODEM_CLKGEN_BAND                              0x51

#define PROP_PA_GROUP                                       0x22
#define PROP_PA_PWR_LVL                                     0x01

#define PROP_FREQ_CTRL_GROUP                                0x40
#define PROP_FREQ_CTRL_INTE                                 0x00
#define PROP_FREQ_CTRL_FRAC_2                               0x01
#define PROP_FREQ_CTRL_FRAC_1                               0x02
#define PROP_FREQ_CTRL_FRAC_0                               0x03
#define PROP_FREQ_CTRL_CHANNEL_STEP_SIZE_1                  0x04
#define PROP_FREQ_CTRL_CHANNEL_STEP_SIZE_0                  0x05
#define PROP_FREQ_CTRL_W_SIZE                               0x06
#define PROP_FREQ_CTRL_VCOCNT_RX_ADJ                        0x07

#define PROP_PKT_GROUP                                      0x12
#define PROP_PKT_CRC_CONFIG                                 0x00
#define PROP_PKT_TX_THRESHOLD                               0x0B
#define PROP_PKT_RX_THRESHOLD                               0x0C
#define PROP_PKT_FIELD_1_CONFIG                             0x0F
#define PROP_PKT_FIELD_2_LENGTH_12_8                        0x11
#define PROP_PKT_FIELD_2_LENGTH_7_0                         0x12
#define PROP_PKT_FIELD_2_CONFIG                             0x13
#define PROP_PKT_FIELD_2_CRC_CONFIG                         0x14


struct si446x_reply_GENERIC_map {
    uint8_t REPLY[16];
};

struct si446x_reply_PART_INFO_map {
    uint8_t     CHIPREV;
    uint16_t    PART;
    uint8_t     PBUILD;
    uint16_t    ID;
    uint8_t     CUSTOMER;
    uint8_t     ROMID;
};

struct si446x_reply_FUNC_INFO_map {
    uint8_t REVEXT;
    uint8_t REVBRANCH;
    uint8_t REVINT;
    uint8_t FUNC;
};

struct si446x_reply_GET_PROPERTY_map {
    uint8_t DATA[16];
};

struct si446x_reply_GPIO_PIN_CFG_map {
    uint8_t gpio[4];
    uint8_t NIRQ;
    uint8_t SDO;
    uint8_t GEN_CONFIG;
};

struct si446x_reply_FIFO_INFO_map {
    uint8_t RX_FIFO_COUNT;
    uint8_t TX_FIFO_SPACE;
};

struct si446x_reply_GET_INT_STATUS_map {
    uint8_t INT_PEND;
    uint8_t INT_STATUS;
    uint8_t PH_PEND;
    uint8_t PH_STATUS;
    uint8_t MODEM_PEND;
    uint8_t MODEM_STATUS;
    uint8_t CHIP_PEND;
    uint8_t CHIP_STATUS;
};

struct si446x_reply_REQUEST_DEVICE_STATE_map {
    uint8_t CURR_STATE;
    uint8_t CURRENT_CHANNEL;
};

struct si446x_reply_READ_CMD_BUFF_map {
    uint8_t BYTE[16];
};

struct si446x_reply_FRR_A_READ_map {
    uint8_t FRR_A_VALUE;
    uint8_t FRR_B_VALUE;
    uint8_t FRR_C_VALUE;
    uint8_t FRR_D_VALUE;
};

struct si446x_reply_FRR_B_READ_map {
    uint8_t FRR_B_VALUE;
    uint8_t FRR_C_VALUE;
    uint8_t FRR_D_VALUE;
    uint8_t FRR_A_VALUE;
};

struct si446x_reply_FRR_C_READ_map {
    uint8_t FRR_C_VALUE;
    uint8_t FRR_D_VALUE;
    uint8_t FRR_A_VALUE;
    uint8_t FRR_B_VALUE;
};

struct si446x_reply_FRR_D_READ_map {
    uint8_t FRR_D_VALUE;
    uint8_t FRR_A_VALUE;
    uint8_t FRR_B_VALUE;
    uint8_t FRR_C_VALUE;
};

struct si446x_reply_IRCAL_MANUAL_map {
    uint8_t IRCAL_AMP_REPLY;
    uint8_t IRCAL_PH_REPLY;
};

struct si446x_reply_PACKET_INFO_map {
        uint16_t LENGTH;
};

struct si446x_reply_GET_MODEM_STATUS_map {
    uint8_t MODEM_PEND;
    uint8_t MODEM_STATUS;
    uint8_t CURR_RSSI;
    uint8_t LATCH_RSSI;
    uint8_t ANT1_RSSI;
    uint8_t ANT2_RSSI;
    uint16_t AFC_FREQ_OFFSET;
};

struct si446x_reply_READ_RX_FIFO_map {
    uint8_t DATA[2];
};

struct si446x_reply_GET_ADC_READING_map {
    uint16_t GPIO_ADC;
    uint16_t BATTERY_ADC;
    uint16_t TEMP_ADC;
};

struct si446x_reply_GET_PH_STATUS_map {
    uint8_t PH_PEND;
    uint8_t PH_STATUS;
};

struct si446x_reply_GET_CHIP_STATUS_map {
    uint8_t CHIP_PEND;
    uint8_t CHIP_STATUS;
    uint8_t CMD_ERR_STATUS;
    uint8_t CMD_ERR_CMD_ID;
};





#endif