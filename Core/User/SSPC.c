#include "main.h"
#include "SSPC.h"
#include "Serial.h"
#include "OLED.h"
#include "LED.h"

extern IWDG_HandleTypeDef hiwdg;
extern UART_HandleTypeDef huart1;
extern uint8_t USART1_TxBusy;
static OpChn_t ChOpLog[CHN_NUM];
uint8_t tx_buf[13];

/**
*@brief 环形储存区初始化
*@param ch:输入通道数
*@retval
*/
static void OpChn_Memory_Init(uint8_t ch)
{
	if(ch>=CHN_NUM){return;}
	OpChn_t *p = &ChOpLog[ch];
	p -> wr_idx = 0;
	p -> rd_idx = 0;
	p -> cnt    = 0;
}

/**
*@brief 环形储存区系统初始化
*@retval
*/
void OpChn_Sys_Init(void)
{
	for(uint8_t i = 0;i < CHN_NUM ;i++)
	{
		OpChn_Memory_Init(i);
	}
}

/**
*@brief 环形储存区写满
*@retval 1:写满  0：未满
*/
static uint8_t OpChn_IsFull(uint8_t ch)
{
	return ChOpLog[ch].cnt >= OP_BUF_DEPTH;
}

/**
*@brief 环形储存区记录函数
*@param ch:输入通道数
*@param op_code:操作指令
*@param param  :操作参数
*@retval
*/
void LogChannelOp(uint8_t ch,uint16_t op_code,uint32_t op_param)
{
	if(ch>CHN_NUM)return;
	OpChn_t *p = &ChOpLog[ch];
	/*环形储存满，丢弃最旧的一条*/
	if(OpChn_IsFull(ch))
	{
		p->rd_idx = (p->rd_idx+1)%OP_BUF_DEPTH;
		p->cnt--;
	}
	
	OpCmd_t rec = {.tick = HAL_GetTick(),.op_code = op_code,.param = op_param};
	p->buf[p->wr_idx] = rec;
	p->wr_idx = (p->wr_idx + 1)%OP_BUF_DEPTH;
	p->cnt++;  
}

/**
*@brief 获取通道有效操作数
*@param ch:输入通道号
*@ret
*/
uint16_t GetChnOpCount(uint8_t ch)
{
	if(ch >= CHN_NUM)return 0;
	return ChOpLog[ch].cnt;
}

/**
*@brief 获取单一通道操作
*@param ch:输入通道号
*@ret
*/
uint8_t PrintALLChOp(uint8_t ch)
{
	if(ch >= CHN_NUM || ChOpLog[ch].cnt == 0) ChOp_Packet(0xFE,0xFF,0xFF,0xFF);
	OpChn_t *p = &ChOpLog[ch];
	uint16_t tmp_rd = p->rd_idx;
	uint16_t num = p->cnt;
	for(uint16_t i = 0;i<num;i++)
	{
		OpCmd_t record = p->buf[tmp_rd];
		//通过串口一发送
		ChOp_Packet(ch,record.tick,record.op_code,record.param);
		tmp_rd = (tmp_rd+1)%OP_BUF_DEPTH;
	}
	return 1 ;
}

/**
*@brief 打包通道有效操作数
*@param ch:输入通道号
*@ret
*/
void ChOp_Packet(uint8_t ch,uint32_t tick,uint16_t code,uint32_t param)
{
	 
   uint8_t idx = 0;
   tx_buf[idx++] = OP_PACKET_HEAD;
   tx_buf[idx++] = ch;
  
   tx_buf[idx++] = (tick >> 24) & 0xFF;
   tx_buf[idx++] = (tick >> 16) & 0xFF;
   tx_buf[idx++] = (tick >> 8)  & 0xFF;
   tx_buf[idx++] = tick & 0xFF;

   tx_buf[idx++] = (code >> 8) & 0xFF;
   tx_buf[idx++] = code & 0xFF;

   tx_buf[idx++] = (param >> 24) & 0xFF;
   tx_buf[idx++] = (param >> 16) & 0xFF;
   tx_buf[idx++] = (param >> 8)  & 0xFF;
   tx_buf[idx++] =  param & 0xFF;

   tx_buf[idx++] = OP_PACKET_TAIL;
	 
   while(UART_SendData(&huart1,tx_buf, 13))HAL_IWDG_Refresh(&hiwdg);
	
}

/**
*@brief 读取上一条操作函数
*@param chn:输入通道号
*@param out_cmd :输出操作结构体
*@ret
*/
uint8_t GetChnLastOpCmd(uint8_t chn, OpCmd_t *out_cmd)
{
    OpChn_t *pCh = &ChOpLog[chn];
    if(pCh->cnt == 0)
    {
        return 0;
    }
    uint16_t last_idx = (pCh->wr_idx - 1 + OP_BUF_DEPTH) % OP_BUF_DEPTH;
    *out_cmd = pCh->buf[last_idx];
    return 1;
}

/**
*@brief 读取所有通道操作数
*@param out_list[]:所有通道历史指令结构体（上一条）
*@param valid_flag:对应通道历史操作确认
*@ret
*/
void GetAllChnLastOp(OpCmd_t out_list[8], uint8_t valid_flag[8])
{
    OpCmd_t temp;
    for(uint8_t ch = 0; ch < CHN_NUM; ch++)
    {
        if(GetChnLastOpCmd(ch, &temp))
        {
            out_list[ch] = temp;
            valid_flag[ch] = 1; 
        }
        else
        {
            valid_flag[ch] = 0; 
            out_list[ch].tick = 0;
            out_list[ch].op_code = 0;
            out_list[ch].param = 0;
        }
    }
}
