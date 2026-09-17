#ifndef __Pro_H
#define __Pro_H
#include "Serial.h"
#define SKY_GND_FLAG (HAL_GPIO_ReadPin(GROUND_SWITCH_GPIO_Port,GROUND_SWITCH_PIN))
#define ENG_START_DELAY   6000U
#define ENG_STOP_DELAY    6000U
#define SSPC_UNLOCK_WAIT  1000U
#define SSPC_START_WAIT   2000U
#define SSPC_SendDelay    10U
#define SSPC_UNLOCK_ENS   1000U
#define SSPC_UNLOCK_BREAK 5000U
#define Pro_LED           200U
#define Wait_SSPC_LED     1000U
#define Wait_SSPC_28V     5000U

#define CAN_QUEUE_SIZE  16 //缓冲队列大小

typedef struct {
    uint8_t data[8];
    uint32_t id;
} CAN_Frame_t;


typedef struct{
	
	GPIO_PinState curr_read;     //锟斤拷锟斤拷锟斤拷前状态
	GPIO_PinState last_stable;   //锟斤拷锟斤拷锟饺讹拷状态
	uint32_t      filter_cnt;    //锟斤拷锟斤拷锟斤拷时
	
}KEY;

typedef struct{
	uint8_t chn1;
	uint8_t chn2;
	uint8_t chn3;
	uint8_t chn4;
	uint8_t chn5;
	uint8_t chn6;
	uint8_t chn7;
	uint8_t chn8;
	
}CAN_CHN_Status;


void Process(void);
void Fly_Control(void);
void FC_IO_CMD(uint8_t cmd,uint8_t eng);
void SSPC_Cmd(uint32_t now,uint8_t *Data);
void ENG_START_6S(uint32_t now);
void Waiting_SSPC(void);
void OLED_ShowCANWord(void);
void SSPC_CHN_Unlock(uint32_t now,uint8_t *data);
void SSPC_CHN_Read(uint32_t now,FC_SendData* readdata,uint8_t data[8]);
void Log_UnlockOp(uint16_t id);
void SSPC_Recover(void);
void SSPC_Set(void);
void SSPC_Init(uint8_t flag);
void Vcheck_28Vbus(uint32_t now,uint8_t VH,uint8_t VL);
/** 
 * @brief  CAN甯у叆闃?
 * @param  data: 甯ф暟鎹?
 * @param  id: 帧ID
 * @retval  
 * */
void CAN_Enqueue(uint8_t *data, uint32_t id);

/**
  * @brief 从队列中取出所有待处理帧并依次执行
  * @param now 褰撳墠绯荤粺鏃堕棿锛圚AL_GetTick()锛?
  */
void Process_CAN_Queue(uint32_t now);
#endif

