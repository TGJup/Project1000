#ifndef __Serial_H
#define __Serial_H

typedef struct{
	uint16_t Vbus12H;
	uint16_t Vchn8H;
	uint16_t Ichn8H;
	uint16_t Ichn7H;
	uint16_t Ichn6H;
	uint16_t Ichn5H;
	
	uint16_t Vbus12L;
	uint16_t Vchn8L;
	uint16_t Ichn8L;
	uint16_t Ichn7L;
	uint16_t Ichn6L;
	uint16_t Ichn5L;
	uint16_t Ep_collect_V_H;
	uint16_t Ep_collect_V_L;
	uint16_t Ep_Current_H;
	uint16_t Ep_Current_L;
}FC_SendData;

typedef struct{
	
	uint16_t volt_acc;                   //累计总压
	uint16_t volt_collect;               //采集电压
	int16_t  raw_current;                //电流原始值
	int16_t  current;                    //真实电流 = 电流原始值 - 30000
	uint16_t soc_raw;                    //当前电量原始值
	float    soc;                        //当前电量值
	float    total_volt_acc_real;        //
	float    total_volt_collect_real;    //
	
}EP_DataTypeDef;
extern EP_DataTypeDef ep_data;

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

#endif
