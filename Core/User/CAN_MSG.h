#ifndef __CAN_MSG_H
#define __CAN_MSG_H

/*
CAN_TX : PA12
CAN_RX : PA11
*/


/*Êä•ÊñáËÅîÂêàÁªìÊûÑ‰Ω?*/
typedef struct{
		uint8_t id_high;
	  uint8_t id_low;
		uint8_t  func;
		uint8_t  channel;
		uint8_t data_3;     // dataÊúÄÈ´òÂ≠óËä?
    uint8_t data_2;
    uint8_t data_1;
    uint8_t data_0;
	}MsgInner_t;

typedef union{
	uint8_t buf[sizeof(MsgInner_t)];
  MsgInner_t msg;
}SSPC_CAN_Msg_t;

//==================== ∞¸Õ∑ ========================
#define SSPC_ID                   0x0012
//==================== ≤Ÿ◊˜¬Î ========================
#define SSPC_FUNC_CHN_OPEN        0x01U
#define SSPC_FUNC_CHN_CLOSE       0x02U
#define SSPC_FUNC_UNLOCK          0xEEU
#define SSPC_FUNC_SAVE_FLASH      0x99U
#define SSPC_FUNC_CFG_CURR        0x10U
#define SSPC_FUNC_CFG_OVP         0x11U
#define SSPC_FUNC_CFG_UVP         0x12U
#define SSPC_FUNC_CFG_OTP         0x13U
#define SSPC_FUNC_CFG_I2T_K       0x14U
#define SSPC_FUNC_CFG_SHORT_RETRY 0x15U
#define SSPC_FUNC_CFG_I2T_RETRY   0x16U
#define SSPC_FUNC_CFG_SHORT_INTER 0x17U
#define SSPC_FUNC_CFG_ID          0x20U
#define SSPC_FUNC_CFG_REPORT_CYC  0x30U
#define SSPC_FUNC_CFG_CAN_BAUD    0x40U
#define SSPC_FUNC_CFG_RTC_DATE    0x50U
#define SSPC_FUNC_CFG_RTC_TIME    0x51U

//==================== ◊¥Ã¨¬Î =========================
#define SSPC_STAT_NORMAL_ERR      0x00U
#define SSPC_STAT_LOCK_ERR        0x88U
#define SSPC_STAT_CMD_ACK         0x22U
#define SSPC_STAT_REPORT_VIN_TEMP 0x33U
#define SSPC_STAT_REPORT_VOUT_I   0x44U

//===================== Õ®µ¿¬Î =========================
#define SSPC_CHN_1     0x01U
#define SSPC_CHN_2     0x02U
#define SSPC_CHN_3     0x03U
#define SSPC_CHN_4     0x04U
#define SSPC_CHN_5     0x05U
#define SSPC_CHN_6     0x06U
#define SSPC_CHN_7     0x07U
#define SSPC_CHN_8     0x08U
#define SSPC_CHN_1_4   0xDFU
#define SSPC_CHN_5_8   0xEFU
#define SSPC_CHN_ALL   0xFFU

//==================== ÂºÇÂ∏∏Áä∂ÊÄ? ====================
#define SSPC_ERR_SHORT  0xFEU
#define SSPC_ERR_I2T    0xFDU
#define SSPC_ERR_OVP    0xFCU
#define SSPC_ERR_UVP    0xFBU
#define SSPC_ERR_OTP    0xF2U

// ====================== Â∫îÁ≠îÁ†? ======================
#define SSPC_ACK_OK     0x4F4BU 
#define SSPC_ACK_ERR    0x6572U 


#endif
