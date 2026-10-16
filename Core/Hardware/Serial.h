#ifndef __Serial_H
#define __Serial_H

typedef struct{
	uint16_t vBtry28H;
	uint16_t vBtry28L;
	uint16_t vBtry12H;
	uint16_t vBtry12L;
	uint16_t vBus28H;
	uint16_t vBus28L;
	uint16_t vBus12H;
	uint16_t vBus12L;
	uint16_t iBtry28H;
	uint16_t iBtry28L;
	uint16_t iBtry12H;
	uint16_t iBtry12L;
	uint16_t iGerator1H;
	uint16_t iGerator1L;
	uint16_t iGerator2H;
	uint16_t iGerator2L;
}FC_SendData;

typedef struct{
	
	uint16_t volt_acc;                   //绱鎬诲帇
	uint16_t volt_collect;               //閲囬泦鎬诲帇
	int16_t  raw_current;                //鐢垫祦鍘熷鍊?
	int16_t  current;                    //瀹為檯鐢垫祦 = 鐢垫祦鍘熷鍊? - 30000
	uint16_t soc_raw;                    //褰撳墠鐢甸噺鍘熷鍊?
	float    soc;                        //褰撳墠鐢甸噺鍊?
	float    total_volt_acc_real;        //绱鎬诲帇瀹為檯鍊?
	float    total_volt_collect_real;    //閲囬泦鎬诲帇瀹為檯鍊?
	
}EP_DataTypeDef;
extern EP_DataTypeDef ep_data;
#define FC_BAT_TIMEOUT   500U    //电池(RS485)数据超时(ms)：轮询周期 80ms，取 6 倍余量
#define FC_BUS_TIMEOUT   5000U   //母线(SSPC)数据超时(ms)：SSPC 上报周期约 2s，取 2.5 倍余量

uint8_t UART_SendData(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t len);
uint16_t Byte2_TO_U16(uint8_t high,uint8_t low);

void EP_DataCombine(uint8_t *buf,uint16_t*p_accVolt,uint16_t*p_collectVolt,
	                  int16_t*p_rawCurr,uint16_t*p_Soc);

void EP_0x90_Convert(uint8_t *buf,uint16_t*raw_accVolt,uint16_t*raw_collectVolt,
	                  int16_t*raw_Curr,uint16_t*raw_soc,float *out_accVolt,
										float *out_collectVolt,int16_t *out_curr,float *out_soc);
										
uint8_t USART3_EP_RE(FC_SendData *data);										
uint8_t Calc_CheckSum(uint8_t *buf, uint8_t len);
void FC_Packet_Make(FC_SendData *data);
uint8_t EP_Packet_Make(uint8_t cmd);
void USART1_LoopCall(void);										
void USART3_LoopCall(void);
uint8_t UART_TimeOut(UART_HandleTypeDef *huart);
/*
*@brief  飞控数据重置
*@param  d:飞控数据结构体指针
*/
void FC_Data_Reset(FC_SendData *d);

#endif
