#include "main.h"
#include <string.h>
#include "CAN_middle.h"
#include "CAN_MSG.h"
#include "SSPC.h"

extern CAN_HandleTypeDef hcan;
extern IWDG_HandleTypeDef hiwdg;
/*鍙橀噺*/
volatile uint8_t CAN_RxFinish;
uint8_t CAN_SendBuff[8] = {0};
uint32_t CAN_ID = 0;

extern volatile uint8_t queue_head;
extern volatile uint8_t queue_tail;
extern volatile uint8_t queue_count;
extern CAN_Frame_t can_queue[];
extern volatile uint8_t sspc_ack_received;
//璁剧疆Filter杩囨护锛屼娇鑳紽IFO0锛屽苟涓嶈繃婊や换浣曚俊鎭?
extern volatile uint16_t latest_28V_value;
extern volatile uint8_t  voltage_valid;

uint8_t bsp_can1_filter_config(void)
{
	CAN_FilterTypeDef filter = {0};
	filter.FilterActivation     = ENABLE;
	filter.FilterMode           = CAN_FILTERMODE_IDMASK;
	filter.FilterScale          = CAN_FILTERSCALE_32BIT;
	filter.FilterBank           = 0;
	filter.FilterFIFOAssignment = CAN_FilterFIFO0;
	filter.FilterIdLow          = 0;
	filter.FilterIdHigh         = 0;
	filter.FilterMaskIdLow      = 0;
	filter.FilterMaskIdHigh     = 0;
	if (HAL_CAN_ConfigFilter (&hcan,&filter) != HAL_OK){return 1;} // 配置过滤，发送成功
  return 0;   // 发送失败
}

//CAN数据发送函数
uint8_t CAN_SendMsg(uint16_t msgID,uint8_t *Data)
{

	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.StdId  = 0x0000;
	TxHeader.RTR    = CAN_RTR_DATA;
	TxHeader.IDE    = CAN_ID_STD;
	TxHeader.DLC    = 8;
	TxHeader.TransmitGlobalTime = DISABLE;
	uint8_t TxData[8];
	TxData[0] = *(Data+0);
	TxData[1] = *(Data+1);
	TxData[2] = *(Data+2);
	TxData[3] = *(Data+3);
	TxData[4] = *(Data+4);
	TxData[5] = *(Data+5);
	TxData[6] = *(Data+6);
	TxData[7] = *(Data+7);

    uint32_t can_tick = HAL_GetTick();
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
	{
		if(HAL_GetTick() - can_tick  >= can_wait_tick){return 1;}
		HAL_IWDG_Refresh(&hiwdg);
	}//检查发送邮箱
	uint32_t TxMailbox;//接收返回的邮箱编号
	if(HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox) != HAL_OK){return 1;}
	return 0;
}

/**
* @brief SSPC发送指令
* @param id  : ID
* @param func: 功能，打开或关闭或解锁
* @param chn 通道1-8
* @param val 具体的操作参数
*/
void SSPC_SendCmd(uint16_t id,uint8_t func,uint8_t chn,uint32_t val)
{
	
	SSPC_CAN_Msg_t tx;
  tx.msg.id_high = (id >> 8) & 0xFF;
  tx.msg.id_low  = id & 0xFF;
  tx.msg.func    = func;
  tx.msg.channel = chn;
  tx.msg.data_3  = (val >> 24) & 0xFF;
  tx.msg.data_2  = (val >> 16) & 0xFF;
  tx.msg.data_1  = (val >> 8)  & 0xFF;
  tx.msg.data_0  = val & 0xFF;
  CAN_SendMsg (id, tx.buf);

}


/*CAN中断回调函数，有新报文*/
static CAN_RxHeaderTypeDef RxMessage;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t data[8];
	//指向FIFO的硬件寄存器组
	volatile uint32_t *pFIFO = (uint32_t *)&(hcan->Instance->sFIFOMailBox[CAN_RX_FIFO0]);

	// 定义
	uint32_t dlc_reg = pFIFO[1];   // FIFO寄存器1：DLC、IDE帧格式、RTR远程帧标记
    uint32_t low    = pFIFO[2];   // FIFO寄存器2：报文数据低4字节 data0~data3
    uint32_t high   = pFIFO[3];   // FIFO寄存器3：报文数据高4字节 data4~data7

	// 直接赋值给data
	data[0] = (uint8_t)(low >>0);
	data[1] = (uint8_t)(low >>8);
	data[2] = (uint8_t)(low >>16);
	data[3] = (uint8_t)(low >>24);
	data[4] = (uint8_t)(high >>0);
	data[5] = (uint8_t)(high >>8);
	data[6] = (uint8_t)(high >>16);
	data[7] = (uint8_t)(high >>24);

	//释放FIFO0
	SET_BIT(hcan->Instance->RF0R, CAN_RF0R_RFOM0);
		
	CAN_Enqueue(data, RxMessage.StdId);   // 入队操作
		/*CAN_RxFinish = 1;
		CAN_ID = RxMessage.StdId;
		memcpy(CAN_SendBuff,data,8);*/
		// 妫�娴? ACK
	if (data[6] == 0x4F && data[7] == 0x4B) 
	{
		sspc_ack_received = 1;
	}
	if (data[2] == 0x33 && data[3] == 0xEF)
	{
		latest_28V_value = ((uint16_t)data[4] << 8) | data[5];
		voltage_valid = 1;
	}
	
}
