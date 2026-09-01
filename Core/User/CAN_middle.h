#ifndef __CAN_MIDDLE__H
#define __CAN_MIDDLE__H
#include "main.h"
#include "Pro.h"
#define CAN_QUEUE_SIZE  16
#define can_wait_tick 500U



uint8_t bsp_can1_filter_config(void);
uint8_t CAN_SendMsg(uint16_t msgID,uint8_t *Data);
void SSPC_SendCmd(uint16_t id,uint8_t func,uint8_t chn,uint32_t val);

#endif

