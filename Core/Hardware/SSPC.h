#ifndef __SSPC_H
#define __SSPC_H

#define OP_PACKET_HEAD   0xFF
#define OP_PACKET_TAIL   0xFE

#define CHN_NUM          8U
#define OP_BUF_DEPTH     128U

/*存储通道*/
#define CHN_1     0
#define CHN_2     1
#define CHN_3     2
#define CHN_4     3
#define CHN_5     4
#define CHN_6     5
#define CHN_7     6
#define CHN_8     7


/*操作指令结构体*/
typedef struct{
	uint32_t tick;    //操作时系统时间
	uint16_t op_code; //操作类型
	uint32_t param;   //操作携带参数
}OpCmd_t;

/*操作通道结构体*/
typedef struct{
	OpCmd_t  buf[OP_BUF_DEPTH];  //储存数组深度
	uint16_t wr_idx;             //写指针
	uint16_t rd_idx;             //读指针：标记最旧的一条指令
	uint16_t cnt;                //当前记录条数
	
}OpChn_t;

void OpChn_Sys_Init(void);
static void OpChn_Memory_Init(uint8_t ch);
static uint8_t OpChn_IsFull(uint8_t ch);
void LogChannelOp(uint8_t ch,uint16_t op_code,uint32_t op_param);
uint8_t PrintALLChOp(uint8_t ch);
void ChOp_Packet(uint8_t ch,uint32_t tick,uint16_t code,uint32_t param);
uint8_t GetChnLastOpCmd(uint8_t chn, OpCmd_t *out_cmd);
void GetAllChnLastOp(OpCmd_t out_list[8], uint8_t valid_flag[8]);
#endif
