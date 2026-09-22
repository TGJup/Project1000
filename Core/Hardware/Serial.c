#include "main.h"
#include <string.h>
#include "Serial.h"
#include "Serial_MSG.h"
#include "OLED.h"
#include "LED.h"
/*USART1
PA9ï¼? TX
PA10ï¼šRX*/

/*USART3
PB10ï¼? TX
PB11ï¼? RX*/


extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern FC_SendData Read_data;   
extern FC_SendData Zero_data;

EP_DataTypeDef ep_data;

static  uint8_t MCU_PowerOn_Flag = 0;
static  uint8_t Packet_Ready_flag;
uint8_t USART1_TxBusy = 0;    // ´®¿Ú1·¢ËÍÃ¦±êÖ¾£¬1:·¢ËÍÖÐ£¬0:¿ÕÏÐ
uint8_t USART3_TxBusy = 0;    // ´®¿Ú3·¢ËÍÃ¦±êÖ¾£¬1:·¢ËÍÖÐ£¬0:¿ÕÏÐ

uint8_t USART1_RXBuffer[1];          // ´®¿Ú1½ÓÊÕ»º³åÇø
uint8_t USART1_TXBuffer[32];         // ´®¿Ú1·¢ËÍ»º³åÇø
uint8_t USART1_RxFrame[32];          // ´®¿Ú1Êý¾Ý½ÓÊÕÇø
uint8_t USART1_RxIdx    = 0;         // ´®¿Ú1½ÓÊÕË÷Òý
volatile uint8_t USART1_RxFinish = 0;// ´®¿Ú1½ÓÊÕÍê³É±êÖ¾Î»
uint32_t U1_last_tick = 0;           // ´®¿Ú1×îÐÂÊ±¼ä


uint8_t USART3_RXBuffer[1];          // ´®¿Ú3½ÓÊÕ»º³åÇø
uint8_t USART3_TXBuffer[32];         // ´®¿Ú3·¢ËÍ»º³åÇø
uint8_t USART3_RxFrame[32];          // ´®¿Ú3Êý¾Ý½ÓÊÕÇø
uint8_t USART3_RxIdx    = 0;         // ´®¿Ú3½ÓÊÕË÷Òý
volatile uint8_t USART3_RxFinish = 0;// ´®¿Ú3½ÓÊÕÍê³É±êÖ¾Î»
static uint8_t U3_wait_ack = 0;      // ´®¿Ú3µÈ´ýÈ·ÈÏ±êÖ¾
uint32_t U3_last_tick = 0;           // ´®¿Ú3×îÐÂÊ±¼ä
//static uint8_t  U3_Data_Type;        // ä¸²å£3å‘é€æ•°æ?ç±»åž‹


/**
 * @brief  ´®¿Ú·¢ËÍº¯Êý
 * @param  huart: ´®¿Ú¾ä±ú
 * @param  pData: Êý¾ÝÖ¡µØÖ·
 * @param  len:   Êý¾ÝÖ¡ÓÐÐ§×Ö½ÚÊý
 * @retval 1·¢ËÍÊ§°Ü£¬0·¢ËÍ³É¹¦
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
* @brief 16Î»Êý¾ÝºÏ³É
* @retval
*/
uint16_t Byte2_TO_U16(uint8_t high,uint8_t low)
{
	return ((uint16_t)high << 8) | low;
}

/**
* @brief ·É¿ØÊý¾Ý´ò°ü
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
* @brief æ•°æ®æ‹¼æŽ¥åŠæ¬è¿ï¼ˆåº”æ€¥ç”µæºï¼‰
* @param *buf:è¯»å–åº”æ€¥ç”µæºæŽ¥æ”¶æ•°æ?åŒ?
* @param *p_accVolt: è¾“å‡ºç´?è®¡æ€»åŽ‹åŽŸå?‹å€?
* @param *p_collectVolt:è¾“å‡ºé‡‡é›†æ€»åŽ‹åŽŸå?‹å€?
* @param *p_rawCurr:è¾“å‡ºç”µæµåŽŸå?‹å€?
* @param *p_Socï¼šè¾“å‡ºSOCåŽŸå?‹å€?
* @retval
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
* @brief æ•°æ®æ¢ç®—åŠæ¬è¿ï¼ˆåº”æ€¥ç”µæºï¼‰
* @param *buf            :è¯»å–åº”æ€¥ç”µæºæŽ¥æ”¶æ•°æ?åŒ?
* @param *raw_accVolt    : è¾“å‡ºç´?è®¡æ€»åŽ‹åŽŸå?‹å€?
* @param *raw_collectVolt:è¾“å‡ºé‡‡é›†æ€»åŽ‹åŽŸå?‹å€?
* @param *raw_Curr       :è¾“å‡ºç”µæµåŽŸå?‹å€?
* @param *raw_Soc        :è¾“å‡ºSOCåŽŸå?‹å€?
* @param *out_accVolt    :è¾“å‡ºç´?è®¡ç”µåŽ‹å®žé™…å€?
* @param *out_collectVolt:è¾“å‡ºé‡‡é›†ç”µåŽ‹å®žé™…å€?
* @param *out_curr       :è¾“å‡ºç”µæµå®žé™…å€?
* @param *out_soc        :è¾“å‡ºç”µé‡å®žé™…å€?
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
* @brief ä¸²å£ä¸‰åº”æ€¥ç”µæºçŠ¶æ€è?»å–
* @param type: 0:ç”µåŽ‹çŠ¶æ€? 1ï¼šç”µæµçŠ¶æ€?
* @param data: ä¼ å…¥æ•°ç»„
* @retval type0ï¼?0x80:ç”µåŽ‹è¿‡åŽ‹ 0x88ï¼šç”µåŽ‹æ¬ åŽ?
* @retval type1: 0x00:ç”µæµæ­£å¸¸ 0x80ï¼šç”µæµè¿‡æµ?
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
* @brief ´®¿ÚÈýÊý¾Ý´ò°ü
* @param *data: ´«ÈëÊý×é
* @retval
* 0: ÎåÊ®·¢ËØ²Ä
* 1: Ã»ÓÐ½ÓÊÕµ½´®¿Ú3µÄÊý¾Ý
* 2: Êý¾Ý²»¶Ô£¬²»ÊÇµç³Ø·¢ËÍµÄÊý¾Ý
* 4: Êý¾ÝÔÚ´«ÊäµÄ¹ý³ÌÖÐ³ö´í£¬Ð£ÑéºÍÎ´Í¨¹ý
* 5: ´ò°üÍê³É£¬¿ÉÒÔ·¢ËÍ
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
* @brief ¼ÆËãÐ£ÑéºÍ
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
* @brief ´ò°ü²Ù×÷ÃüÁî¸ø´®¿ÚÈý
* @param cmd ²Ù×÷Âë
* @retval 1:·¢ËÍÊ§°Ü 0·¢ËÍ³É¹¦
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
 * @brief ä¸²å£å‘é€å¿™è¶…æ—¶ç­‰å¾…
 * @param huart ä¸²å£å¥æŸ„
 * @retval 0ç­‰å¾…æˆåŠŸç©ºé—² 1è¶…æ—¶å¤±è´¥
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
 * @brief ´®¿Ú1ÂÖÑ¯º¯Êý
 * @retval 
 */
void USART1_LoopCall(void)
{
    uint32_t tick_now = HAL_GetTick();
    uint32_t tick_diff = tick_now - U1_last_tick;
    // ¸ÕÉÏµç·¢ËÍ3Ãë00¸ø·É¿Ø
    if(MCU_PowerOn_Flag == 0)
    {
        if(tick_diff > FC_Zero_Time)
        {
            MCU_PowerOn_Flag = 1;    // 3Ãë¹ýºóÉÏµç±êÖ¾Î»ÖÃ1
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
    // ½øÈëÕý³£Í¨Ñ¶£¬ÅÐ¶ÏÊÇ·ñ¿ÉÒÔ·¢ËÍÊý¾Ý¸ø·É¿Ø
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
 * @brief ´®¿ÚÈýÂÖÑ¯º¯Êý
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
 * @brief ´®¿Ú½ÓÊÕ»Øµ÷º¯Êý
 * @retval 
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
        // °Ñ½ÓÊÕµ½µÄÊý¾Ý´æÈë½ÓÊÕ»º³åÇø
		USART1_RxFrame[USART1_RxIdx++] = USART1_RXBuffer[0];
        // ÅÐ¶ÏÊý¾ÝÀ´Ô´ÊÇ·ñÊÇ·É¿Ø£¬Èô²»ÊÇÔòÇå¿Õ½ÓÊÕ»º³åÇø
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
    // ÅÐ¶ÏÊÇ·ñÊÇ0xA5
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
 * @brief ´®¿Ú·¢ËÍ»Øµ÷º¯Êý
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
 * @brief  ´®¿Ú´íÎó»Øµ÷£¨ÖØµã´¦Àí ORE Òç³ö´íÎó£©
 * @note   HAL ¼ì²âµ½ ORE Ê±»áµ÷ÓÃ UART_EndRxTransfer()£º¹Øµô RXNEIE¡¢
 *         °Ñ RxState ÖÃ»Ø READY ²¢ÖÐÖ¹±¾´Î½ÓÊÕ¡£
 *         Èç¹ûÕâÀï²»ÖØÐÂ¹ÒÉÏÖÐ¶Ï½ÓÊÕ£¬´®¿Ú»á±ä³É¡°ÄÜ·¢²»ÄÜÊÕ¡±µÄÓÀ¾ÃÊ§´Ï×´Ì¬¡£
 * @param  huart: ´®¿Ú¾ä±ú
 * @retval 
 */
/*
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);          // Çå ORE ±êÖ¾£ºF1 ÉÏ±ØÐëÏÈ¶Á SR ÔÙ¶Á DR
        if(HAL_UART_Receive_IT(&huart1, USART1_RXBuffer, 1) == HAL_OK)
        {
            USART1_RxIdx = 0;                     // ½ÓÊÕÈ·ÒÑ±»ÖÐÖ¹£¬Ö¡¼ÆÊýÇåÁã£¬µÈÏÂÒ»¸öÖ¡Í·
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
    huart->ErrorCode = HAL_UART_ERROR_NONE;       // Çå´íÎóÂë£¬±ÜÃâ ORE ²ÐÁô
}
*/
