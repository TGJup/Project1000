#include "main.h"
#include <string.h>
#include "Serial.h"
#include "Serial_MSG.h"
#include "OLED.h"
#include "LED.h"
/*USART1
PA9：TX
PA10：RX*/

/*USART3
PB10：TX
PB11：RX*/


extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern FC_SendData Read_data;   
extern FC_SendData Zero_data;

EP_DataTypeDef ep_data;

static  uint8_t MCU_PowerOn_Flag = 0;
static  uint8_t Packet_Ready_flag;
uint8_t USART1_TxBusy = 0;    // 串口1发送忙标志，1:发送中，0:空闲
uint8_t USART3_TxBusy = 0;    // 串口3发送忙标志，1:发送中，0:空闲

uint8_t USART1_RXBuffer[1];          // 串口1接收缓冲区
uint8_t USART1_TXBuffer[32];         // 串口1发送缓冲区
uint8_t USART1_RxFrame[32];          // 串口1数据接收区
uint8_t USART1_RxIdx    = 0;         // 串口1接收索引
volatile uint8_t USART1_RxFinish = 0;// 串口1接收完成标志位
uint32_t U1_last_tick = 0;           // 串口1最新时间


uint8_t USART3_RXBuffer[1];          // 串口3接收缓冲区
uint8_t USART3_TXBuffer[32];         // 串口3发送缓冲区
uint8_t USART3_RxFrame[32];          // 串口3数据接收区
uint8_t USART3_RxIdx    = 0;         // 串口3接收索引
volatile uint8_t USART3_RxFinish = 0;// 串口3接收完成标志位
static uint8_t U3_wait_ack = 0;      // 串口3等待确认标志
uint32_t U3_last_tick = 0;           // 串口3最新时间
//static uint8_t  U3_Data_Type;        // 涓插彛3鍙戦�佹暟鎹?绫诲瀷


/**
 * @brief  串口发送函数
 * @param  huart: 串口句柄
 * @param  pData: 数据帧地址
 * @param  len:   数据帧有效字节数
 * @retval 1发送失败，0发送成功
 */
uint8_t UART_SendData(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t len)
{
	  if(huart->Instance == USART1)
	  {
	  	 if(USART1_TxBusy) return 1;
       USART1_TxBusy = 1;
	  }
	  else if(huart->Instance == USART3)
	  {
	  	 if(USART3_TxBusy) return 1;
       USART3_TxBusy = 1;
	  }
    HAL_UART_Transmit_IT(huart, pData, len);
		return 0 ;
}

/**
* @brief 16位数据合成
* @retval
*/
uint16_t Byte2_TO_U16(uint8_t high,uint8_t low)
{
	return ((uint16_t)high << 8) | low;
}

/**
* @brief 飞控数据打包
* @retval
*/
void FC_Packet_Make(FC_SendData *data)
{
    USART1_TXBuffer[0] = FC_Packet_Head1;
    USART1_TXBuffer[1] = FC_Packet_Head2;

    USART1_TXBuffer[2] = data->Vbus12H;
    USART1_TXBuffer[3] = data->Vbus12L;
                         
    USART1_TXBuffer[4] = data->Vchn8H;  
    USART1_TXBuffer[5] = data->Vchn8L; 
                         
    USART1_TXBuffer[6] = data->Ichn8H; 
    USART1_TXBuffer[7] = data->Ichn8L; 
                          
    USART1_TXBuffer[8] = data->Ichn7H; 
    USART1_TXBuffer[9] = data->Ichn7L; 
                          
    USART1_TXBuffer[10] = data->Ichn6H; 
    USART1_TXBuffer[11] = data->Ichn6L; 
	                        
    USART1_TXBuffer[12] = data->Ichn5H; 
    USART1_TXBuffer[13] = data->Ichn5L; 
                          
    USART1_TXBuffer[14] = data->Ep_collect_V_H; 
    USART1_TXBuffer[15] = data->Ep_collect_V_L; 
		
    USART1_TXBuffer[16] = data->Ep_Current_H; 
    USART1_TXBuffer[17] = data->Ep_Current_L; 
    USART1_TXBuffer[18] = Calc_CheckSum(USART1_TXBuffer, 18);
//  	OLED_ShowHexNum(100,1,USART1_TXBuffer[18],2,OLED_8X16);
//  	OLED_Update();
    UART_SendData(&huart1, USART1_TXBuffer, 19);
}



/**
* @brief 数据拼接及搬运（应急电源）
* @param *buf: 读取应急电源接收数据区
* @param *p_accVolt: 输出累计总压原始值
* @param *p_collectVolt: 输出采集总压原始值
* @param *p_rawCurr: 输出电流原始值
* @param *p_Soc: 输出SOC原始值
* @retval 无
*/
void EP_DataCombine(uint8_t *buf,uint16_t*p_accVolt,uint16_t*p_collectVolt,
	                  int16_t*p_rawCurr,uint16_t*p_Soc)
{
	*p_accVolt     = Byte2_TO_U16(buf[4],buf[5]);
	*p_collectVolt = Byte2_TO_U16(buf[6],buf[7]);
	*p_rawCurr     = Byte2_TO_U16(buf[8],buf[9]);
	*p_Soc         = Byte2_TO_U16(buf[10],buf[11]);
}

/**
* @brief 数据换算及搬运（应急电源）
* @param *buf            : 读取应急电源接收数据区
* @param *raw_accVolt    : 输出累计总压原始值
* @param *raw_collectVolt: 输出采集总压原始值
* @param *raw_Curr       : 输出电流原始值
* @param *raw_Soc        : 输出SOC原始值
* @param *out_accVolt    : 输出累计电压实际值
* @param *out_collectVolt: 输出采集电压实际值
* @param *out_curr       : 输出电流实际值
* @param *out_soc        : 输出电量实际值
* @retval
*/

void EP_0x90_Convert(uint8_t *buf,uint16_t*raw_accVolt,
	                   uint16_t*raw_collectVolt,int16_t*raw_Curr,uint16_t*raw_soc,
                     float *out_accVolt,float *out_collectVolt,
										 int16_t *out_curr,float *out_soc)
{
	EP_DataCombine(buf,raw_accVolt,raw_collectVolt,raw_Curr,raw_soc);
  
	*out_accVolt     = (float)*raw_accVolt*0.1f;
  *out_collectVolt = (float)*raw_collectVolt*0.1f;
	*out_curr        = (float)*raw_Curr -30000;
	*out_soc         = (float)*raw_soc*0.1f;
}
/**
* @brief 串口三应急电源状态读取
* @param type: 0：电压状态  1：电流状态
* @param data: 传入数组
* @retval type0：0x80：电压过压  0x88：电压欠压
* @retval type1：0x00：电流正常  0x80：电流过流
*/
uint8_t EP_Status_Read(uint8_t type,uint8_t *data)
{
	if(type == 0)
	{
		if(data[4]& MASK_OP){return 0x80;}
		else if(data[4]& MASK_UP){return 0x88;}
	}
	if(type == 1)
	{
		if(data[6]& MASK_ALL_OVERCUR){return 0x80;}
	}
	return 0;
}

/**
* @brief 串口三数据打包
* @param *data: 传入数组
* @retval
* 0: 五十发素材
* 1: 没有接收到串口3的数据
* 2: 数据不对，不是电池发送的数据
* 4: 数据在传输的过程中出错，校验和未通过
* 5: 打包完成，可以发送
*/
uint8_t USART3_EP_RE(FC_SendData *data)
{
	static uint8_t temp = 0;
	if(temp == 6){temp =0;}
	
	if(USART3_RxFinish == 0) return 1;
	uint8_t *buf = USART3_RxFrame;
	if(buf[0]!= EP_Packet_Head1){USART3_RxFinish = 0;return 2;}
	if(Calc_CheckSum(buf,12)!=buf[12]){USART3_RxFinish = 0;return 4;}
	
//	EP_0x90_Convert(USART3_RxFrame,&ep_data.volt_acc,&ep_data.volt_collect,&ep_data.raw_current,
//	                 &ep_data.soc_raw,&ep_data.total_volt_acc_real,&ep_data.total_volt_collect_real,
//	                 &ep_data.current,&ep_data.soc);
	if(buf[2]==EP_CMD_0x90_ASK)
	{
        if(buf[1] == EP_28V_Address)
        {
            data->Ep_collect_V_H = buf[6];
		    data->Ep_collect_V_L = buf[7];
	        data->Ep_Current_H   = buf[8];
		    data->Ep_Current_L   = buf[9];
		    return temp =5;
        }
		
	}
//	else if(buf[2]==EP_CMD_0x98_ASK && temp == 5)
//	{
//		data->sta_volt = EP_Status_Read(1,buf);
//		data->sta_curr = EP_Status_Read(0,buf);
//		return temp = 6;
//	}
	return 0;
}

/**
* @brief 计算校验和
* @retval
*/
uint8_t Calc_CheckSum(uint8_t *buf, uint8_t len)
{
    uint16_t sum = 0;
    for(uint8_t i = 0; i < len; i++)
    {
        sum += buf[i];
    }
    return (uint8_t)(sum % 256);
}


/**
* @brief 打包操作命令给串口三
* @param cmd 操作码
* @retval 1:发送失败 0发送成功
*/
uint8_t EP_Packet_Make(uint8_t cmd)
{
	if(USART3_TxBusy)return 1;
	USART3_TXBuffer[0] = EP_Packet_Head1;
    USART3_TXBuffer[1] = EP_Packet_Head2;
	
	USART3_TXBuffer[2] = cmd;
	USART3_TXBuffer[3] = 0x08;
	USART3_TXBuffer[4] = 0x00;
	USART3_TXBuffer[5] = 0x00;
	USART3_TXBuffer[6] = 0x00;
    USART3_TXBuffer[7] = 0x00;
    USART3_TXBuffer[8] = 0x00;
    USART3_TXBuffer[9] = 0x00;
    USART3_TXBuffer[10] = 0x00;
    USART3_TXBuffer[11] = 0x00;
	
	USART3_TXBuffer[12] = Calc_CheckSum(USART3_TXBuffer, 7);
	UART_SendData(&huart3,USART3_TXBuffer,EP_Packet_Len);
  return 0;
}


/**
 * @brief 串口发送忙超时等待
 * @param huart 串口句柄
 * @retval 0：等待成功空闲  1：超时失败
 */
uint8_t UART_TimeOut(UART_HandleTypeDef *huart)
{
    uint32_t tick = 0;
    if(huart->Instance == USART1)
    {
        while(USART1_TxBusy && tick++ < 10000);
    }
    else if(huart->Instance == USART3)
    {
        while(USART3_TxBusy && tick++ < 10000);
    }
    if((huart->Instance == USART1 && USART1_TxBusy) ||
       (huart->Instance == USART3 && USART3_TxBusy))
    {
        return 1;
    }
    return 0;
}

/**
 * @brief 串口1轮询函数
 * @retval 
 */
void USART1_LoopCall(void)
{
    uint32_t tick_now = HAL_GetTick();
    uint32_t tick_diff = tick_now - U1_last_tick;
    // 刚上电发送3秒00给飞控
    if(MCU_PowerOn_Flag == 0)
    {
        if(tick_diff > FC_Zero_Time)
        {
            MCU_PowerOn_Flag = 1;    // 3秒过后上电标志位置1
                U1_last_tick = tick_now; 
            }
        else
        {
            if(tick_diff >= FC_Send_Cycle)
            {
                FC_Packet_Make(&Zero_data); 
            }
        }
    }
    // 进入正常通讯，判断是否可以发送数据给飞控
    else
    {
        
        if(tick_diff >= FC_Send_Cycle && Packet_Ready_flag == 1)
        {
            Packet_Ready_flag = 0;
            U1_last_tick = tick_now;
            FC_Packet_Make(&Read_data); 
        }
    }
}

/**
 * @brief 串口三轮询函数
 * @retval 
 */
void USART3_LoopCall(void)
{
	if(USART3_RxFinish == 1)
		 {
			  if(USART3_EP_RE(&Read_data)==5)
            {
                Packet_Ready_flag = 1;
            }
			  U3_wait_ack = 0;
			  USART3_RxFinish = 0;
		 }
	if(U3_wait_ack == 0)
		{
				
			//U3_wait_ack =1;
			if(HAL_GetTick() - U3_last_tick >EP_Send_Cycle && EP_Packet_Make(EP_CMD_0x90_ASK) == 0)
        {
			U3_last_tick = HAL_GetTick();
			U3_wait_ack = 1;
		}
//			if(U3_Data_Type == EP_GET_UI)
//			{
//				if(EP_Packet_Make(EP_CMD_0x90_ASK) == 0){U3_wait_ack = 1;}
//				U3_Data_Type = EP_GET_STATUS;
//			}
//			else if(U3_Data_Type == EP_GET_STATUS)
//			{
//				if(EP_Packet_Make(EP_CMD_0x98_ASK) == 0){U3_wait_ack = 1;}
//				U3_Data_Type = EP_GET_UI;
//			}
	 }
}


/**
 * @brief 串口接收回调函数
 * @retval 
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
        // 把接收到的数据存入接收缓冲区
		USART1_RxFrame[USART1_RxIdx++] = USART1_RXBuffer[0];
        // 判断数据来源是否是飞控，若不是则清空接收缓冲区
        if(USART1_RxIdx == 1 && USART1_RxFrame[0] != FC_Packet_Head1)
        {
            USART1_RxIdx = 0;
        }
        else if(USART1_RxIdx >= 2)
        {   
            if(USART1_RxFrame[0]== FC_Packet_Head1 && USART1_RxFrame[1]== FC_Address)
            {
                if(USART1_RxIdx >= FC_Rsp_Len)
                {
                    uint16_t sum = 0;
                    for(uint8_t i = 0; i < FC_Rsp_Len-1; i++)
                    {
                        sum += USART1_RxFrame[i];
                    }
                    uint8_t check = sum % 256;
										/*OLED_ShowHexNum(100,1,check,2,OLED_8X16);
										OLED_Update();*/
                    if(check == USART1_RxFrame[5])
                    {
                        USART1_RxFinish = 1;
                    }
                    USART1_RxIdx = 0;
                }
            }
            else
            {
                USART1_RxIdx = 0;
            }
        }
		HAL_UART_Receive_IT(&huart1, USART1_RXBuffer, 1);
	}
 
	else if(huart->Instance == USART3)
	{
		USART3_RxFrame[USART3_RxIdx++] = USART3_RXBuffer[0];
    // 判断是否是0xA5
        if(USART3_RxIdx == 1 && USART3_RxFrame[0] != EP_Packet_Head1)
        {
            USART3_RxIdx = 0;
        }
        else if(USART3_RxIdx >=2)
        {
            if(USART3_RxFrame[0]==EP_Packet_Head1 && (USART3_RxFrame[1]==EP_28V_Address))
            {
                if(USART3_RxIdx >= EP_Rsp_Len)
                {
                    uint16_t sum = 0;
                    for(uint8_t i = 0; i < EP_Rsp_Len-1; i++)
                    {
                        sum += USART3_RxFrame[i];
                    }
                    uint8_t check = sum % 256;
    //						OLED_ShowHexNum(100,1,check,2,OLED_8X16);
    //						OLED_Update();
                    if(check == USART3_RxFrame[12])
                    {
                        USART3_RxFinish = 1;
                    }
                    USART3_RxIdx = 0;
                }
            }
            else
            {
                USART3_RxIdx = 0;
            }
        }
        HAL_UART_Receive_IT(&huart3, USART3_RXBuffer, 1);
    }
}

/**
 * @brief 串口发送回调函数
 * @retval 
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        USART1_TxBusy = 0;
    }
    else if(huart->Instance == USART3)
    {
        USART3_TxBusy = 0;
    }
}

/**
 * @brief  串口错误回调（重点处理 ORE 溢出错误）
 * @note   HAL 检测到 ORE 时会调用 UART_EndRxTransfer()：关掉 RXNEIE、
 *         把 RxState 置回 READY 并中止本次接收。
 *         如果这里不重新挂上中断接收，串口会变成“能发不能收”的永久失聪状态。
 * @param  huart: 串口句柄
 * @retval 
 */
/*
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);          // 清 ORE 标志：F1 上必须先读 SR 再读 DR
        if(HAL_UART_Receive_IT(&huart1, USART1_RXBuffer, 1) == HAL_OK)
        {
            USART1_RxIdx = 0;                     // 接收确已被中止，帧计数清零，等下一个帧头
        }
    }
    else if(huart->Instance == USART3)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        if(HAL_UART_Receive_IT(&huart3, USART3_RXBuffer, 1) == HAL_OK)
        {
            USART3_RxIdx = 0;
        }
    }
    huart->ErrorCode = HAL_UART_ERROR_NONE;       // 清错误码，避免 ORE 残留
}
*/
