#include "main.h"
#include <string.h>
#include "Serial.h"
#include "OLED.h"
#include "CAN_middle.h"
#include "CAN_MSG.h"
#include "LED.h"
#include "SSPC.h"
#include "Pro.h"
#include "Serial_MSG.h"

extern IWDG_HandleTypeDef hiwdg;
KEY sky_gnd_key;
/*锟斤拷锟斤拷模锟斤拷*/
extern UART_HandleTypeDef huart1;
extern uint8_t USART1_RxFrame[32];
extern uint8_t USART1_RxFinish;

/*SSPC*/
extern FC_SendData Read_data; 
extern uint8_t CAN_SendBuff[8];
extern uint8_t CAN_RxFinish;
static uint8_t SSPC_Open_Key;	// 空中模式标志锁，避免重复初始化
static uint8_t SSPC_Close_Key;	// 地面模式标志锁，避免重复初始化
static uint8_t Vbus28_Full_Flag;
static uint32_t tick_V28check;
static uint32_t Unlock_ENS_tick;
static uint8_t  SSPC_ACK_flag;
static uint8_t P1_UNLOCK_V = 0;
static uint8_t P2_UNLOCK_V = 0;
uint8_t SSPC_Data[8];

/*端口初始状态*/
CAN_CHN_Status can = {.chn1 = 0,.chn2 = 1,.chn3 = 1,.chn4 = 0,.chn5 = 0,.chn6 = 0,.chn7 = 1,.chn8 = 1};
OpCmd_t last_time[8];                //用于上一次记录
uint8_t lastOp_valid_flag[8];        //用于确认上一次操作
uint16_t SSPC_Lock_flag;             //锁定标志位


volatile uint32_t ENG_StartTick; 	// 接收到0x80命令在FC_IO_CMD里开始计时
volatile uint32_t ENG_StopTick;
uint8_t ENG_Start_Lock1;			//接收到打火命令就置1,避免重复进入打火流程，0：可以进入点火流程，可以打开通道4
uint8_t ENG_Start_Lock2;			//发动机2启动控制
uint8_t ENG_Stop_Lock;
uint8_t Eng_Num_Flag;  				// 1：发动机1 0：发动机2
uint8_t lock_channel = 0;   // 保存锁定时的通道号

#define MAX_FRAME_PER_CALL  60   	// 每次最多处理20帧，可根据实际情况调整
volatile uint8_t sspc_ack_received = 0;

CAN_Frame_t can_queue[CAN_QUEUE_SIZE];
volatile uint8_t queue_head = 0;
volatile uint8_t queue_tail = 0;
volatile uint8_t queue_count = 0;

volatile uint16_t latest_28V_value = 0;
volatile uint8_t  voltage_valid = 0;

// 全局互斥标志：任何一台发动机正在启动计时中
uint8_t ENG_Start_In_Progress;


/**
*@brief 主进程
*@retval
*/
void Process(void)
{
	  uint32_t now = HAL_GetTick();
	  ENG_START_6S(now);
		if(sky_gnd_key.last_stable == GPIO_PIN_SET)/*statu:SKY*/
		{
			SSPC_Close_Key = 0;
			if(SSPC_Open_Key == 0){
				 SSPC_Open_Key = 1;
				 SSPC_Init(sky_gnd_key.last_stable);
				 LED_ON();
			}
		}
		else                            /*statu:GROUND*/
		{
			SSPC_Open_Key = 0;
			LED_TURN(now,Pro_LED);
			if(SSPC_Close_Key == 0)
			{
				SSPC_Close_Key = 1;
				Vbus28_Full_Flag = 0;
				SSPC_Init(sky_gnd_key.last_stable);
			}
		}
/*接收飞控指令*/
		if(USART1_RxFinish == 1)
			{
				USART1_RxFinish = 0;
				Fly_Control();
			}
		  
/*接收SSPC数据*/
		/*if(CAN_RxFinish == 1)
		{
	        memcpy(SSPC_Data,CAN_SendBuff,8);
			SSPC_Cmd(now,SSPC_Data);
			CAN_RxFinish = 0;
//			OLED_ShowCANWord();
		if(SSPC_Lock_flag)
		{
			SSPC_CHN_Unlock(now,SSPC_Data);
		}
        }*/
	   Process_CAN_Queue(now);
}

/**
*@brief  分析飞控命令，判断命令来自发动机1或是发动机2
*@retval  电脑发送备用指令码：0xEB 0x92 0xFF 0x00 0x00 0x7C
*/
void Fly_Control(void)
{
	uint8_t IO_Cmd1 = 0;
	uint8_t IO_Cmd2 = 0;
	if(USART1_RxFrame[0] == FC_Packet_Head1 && USART1_RxFrame[1] == FC_Address && USART1_RxFrame[2] == 0xFF){SSPC_Set();SSPC_Open_Key = 0;}
	if(USART1_RxFrame[3] == 0xFF && USART1_RxFrame[4] ==0x00)
	{
		Eng_Num_Flag = 1;
		IO_Cmd1 = USART1_RxFrame[2];
		FC_IO_CMD(IO_Cmd1,1);
	}
	if(USART1_RxFrame[3] == 0x00 && USART1_RxFrame[4] ==0xFF)
	{
		Eng_Num_Flag = 0;
		IO_Cmd2 = USART1_RxFrame[2];
		FC_IO_CMD(IO_Cmd2,0);
	}
		
}

/**
*@brief  处理飞控命令
*@param  cmd:具体指令
*@param  eng:1：发动机1 0：发动机2
*@retval
*/
void FC_IO_CMD(uint8_t cmd,uint8_t eng)
{
  switch(eng)
	{
		case 1:       /*发动机1*/
		    if(cmd&0x80)
			{
				if(ENG_Start_Lock1 != 1 && ENG_Start_In_Progress == 0)
				{
					ENG_Start_In_Progress = 1;
					ENG_Start_Lock1 = 1;				//发动机1点火打开通道4
				    ENG_StartTick  = HAL_GetTick(); 
                    SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_4,0);
				    LogChannelOp(CHN_4,SSPC_FUNC_CHN_OPEN,0);
				}
			}
			if(cmd&0x40)
			{
				if(ENG_Stop_Lock != 1)
				{
					ENG_Stop_Lock = 1;
				    ENG_StopTick  = HAL_GetTick(); 
                    SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_2,0);
				    LogChannelOp(CHN_2,SSPC_FUNC_CHN_CLOSE,0);
				}

			}
			if(cmd&0x08)
			{
                if(can.chn5 == 0)
                {
					can.chn5 = 1;
					SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_5,0);
					//SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_6,0);
					//LogChannelOp(CHN_6,SSPC_FUNC_CHN_OPEN,0);
					LogChannelOp(CHN_5,SSPC_FUNC_CHN_OPEN,0);
				}
			}

			if(!(cmd&0x08)&&can.chn5 == 1)
			{
				can.chn5 = 0;
				SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_5,0);
				//SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_6,0);
				LogChannelOp(CHN_5,SSPC_FUNC_CHN_CLOSE,0);
				//LogChannelOp(CHN_6,SSPC_FUNC_CHN_CLOSE,0);
			}
		break;
			
		case 0:       /*发动机2*/
			 if(cmd&0x80)
			{
				if(ENG_Start_Lock2 != 1 && ENG_Start_In_Progress == 0)
				{
					ENG_Start_In_Progress = 1;
					ENG_Start_Lock2 = 1;
				    ENG_StartTick  = HAL_GetTick(); 
                    SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_8,0);
				    LogChannelOp(CHN_8,SSPC_FUNC_CHN_OPEN,0);
				}
			}
			if(cmd&0x40)
			{
				if(ENG_Stop_Lock != 1)
				{
					ENG_Stop_Lock = 1;
				    ENG_StopTick  = HAL_GetTick(); 
                    SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_2,0);
				    LogChannelOp(CHN_2,SSPC_FUNC_CHN_CLOSE,0);
				}
			}
			if(cmd&0x08)
			{
                if(can.chn6 == 0)
                {
					can.chn6 = 1;
					//SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_5,0);
					SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_6,0);
					LogChannelOp(CHN_6,SSPC_FUNC_CHN_OPEN,0);
					//LogChannelOp(CHN_5,SSPC_FUNC_CHN_OPEN,0);
				}
			}
			if(!(cmd&0x08)&&can.chn6 == 1)
			{
				can.chn6 = 0;
				SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_6,0);
				//SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_6,0);
				LogChannelOp(CHN_6,SSPC_FUNC_CHN_CLOSE,0);
				//LogChannelOp(CHN_6,SSPC_FUNC_CHN_CLOSE,0);
			}
			break;
			
		default:
			switch(cmd)
			{
				case 0x04: can.chn7^=1;                                             
                   if(can.chn7 == 1)
                   {
						SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_7,0);
						LogChannelOp(CHN_7,SSPC_FUNC_CHN_OPEN,0);
					}
				    else 
					{
						SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_7,0);
						LogChannelOp(CHN_7,SSPC_FUNC_CHN_CLOSE,0);
					}
					break;
//		  case 0x02:                                                        break;
//	  	  case 0x01:                                                        break;
			}break;
	}
}

/**
 * @brief 收到SSPC指令解析
 * @retval 
 */
void SSPC_Cmd(uint32_t now,uint8_t *Data)
{
    switch(Data[2])
	{
		case SSPC_STAT_LOCK_ERR       : SSPC_Lock_flag ++;lock_channel = Data[3];break;
		case SSPC_STAT_REPORT_VIN_TEMP :
		case SSPC_STAT_REPORT_VOUT_I  : SSPC_CHN_Read(now,&Read_data,Data);break;
		case SSPC_STAT_CMD_ACK        :	if(SSPC_Lock_flag!=0&&Data[6] == 0x4F &&Data[7] == 0x4B)
		                                 {
											SSPC_ACK_flag = 1;
										 };break;
	    default: break;
			}

}



/**
 * @brief 点火等待6秒
 * @retval 
 */
void ENG_START_6S(uint32_t now)
{
	static uint8_t ENG_Start_Finsh_Lock1 = 1;		
	static uint8_t ENG_Stop_Finsh_Lock = 1;		
	static uint8_t ENG_Start_Finsh_Lock2 = 1;	
	if(ENG_Start_Lock1)							//是否接收到打火命令
	{
		if(now-ENG_StartTick <= ENG_START_DELAY)  // 少于6秒
		{
			ENG_Start_Lock1 = 1;					//点火时间少于6秒，	ENG_Start_Lock 保持为1		
			ENG_Start_Finsh_Lock1 = 0;			//点火时间少于6秒，ENG_Start_Finsh_Lock置0
		}
		else 	//超过六秒就会进入这个流程
		{
			if(ENG_Start_Finsh_Lock1 == 0)		//点火大于6秒
			{
				ENG_Start_Finsh_Lock1 = 1;		// 进入点火流程且大于6秒后，重新给ENG_Start_Finsh_Lock置1
				SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_4,0);	// 点火完毕，关闭通道4
				LogChannelOp(CHN_4,SSPC_FUNC_CHN_CLOSE,0);
			}
			ENG_Start_Lock1 = 0;					//可以重新进入点火流程
			ENG_Start_In_Progress = 0;  			//释放互斥，允许另一台发动机启动
		}
	}
	//发动机2点火六秒流程
	if(ENG_Start_Lock2)							//是否接收到打火命令
	{
		if(now-ENG_StartTick <= ENG_START_DELAY)  // 少于6秒
		{
			ENG_Start_Lock2 = 1;					//点火时间少于6秒，	ENG_Start_Lock 保持为1		
			ENG_Start_Finsh_Lock2 = 0;			//点火时间少于6秒，ENG_Start_Finsh_Lock置0
		}
		else 	//超过六秒就会进入这个流程
		{
			if(ENG_Start_Finsh_Lock2 == 0)		//点火大于6秒
			{
				ENG_Start_Finsh_Lock2 = 1;		// 进入点火流程且大于6秒后，重新给ENG_Start_Finsh_Lock置1
				SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_8,0);	// 点火完毕，关闭通道8
				LogChannelOp(CHN_8,SSPC_FUNC_CHN_CLOSE,0);
			}
			ENG_Start_Lock2 = 0;					//可以重新进入点火流程
			ENG_Start_In_Progress = 0;  			//释放互斥，允许另一台发动机启动
		}
	}	
	if(ENG_Stop_Lock && (now-ENG_StopTick <= ENG_STOP_DELAY))
	{
		ENG_Stop_Lock = 1;
		ENG_Stop_Finsh_Lock = 0;
	}
	else 
	{
		if(ENG_Stop_Finsh_Lock == 0)
		{
			ENG_Stop_Finsh_Lock = 1;
			SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_2,0);
			LogChannelOp(CHN_2,SSPC_FUNC_CHN_OPEN,0);
		}
	ENG_Stop_Lock = 0;
	}
}

/**
 * @brief  上电等待SSPC回复
 * @retval 
 */
void Waiting_SSPC(void)
{
    uint32_t tick = HAL_GetTick();
    sspc_ack_received = 0;   // 清空标志，开始等待

    while (1)   // 一直等到 ACK 为止（按你的要求，不设超时）
    {
        uint32_t now = HAL_GetTick();
        LED_TURN(now, Wait_SSPC_LED);

        // 每 2 秒发一次关闭所有通道的命令（原有逻辑）
        if (now - tick >= SSPC_START_WAIT) {
            tick = now;   
            SSPC_SendCmd(SSPC_ID, SSPC_FUNC_CHN_CLOSE, SSPC_CHN_ALL, 0);
        }

        HAL_IWDG_Refresh(&hiwdg);

        if (sspc_ack_received) {
            break;   // 收到 ACK，退出等待
        }
    }
}
/**
 * @brief  OLED锟斤拷示SSPC锟斤拷锟斤拷
 * @retval 
 */
void OLED_ShowCANWord(void)
{
	OLED_ShowString(1,1,"DevID:",OLED_8X16);
	OLED_ShowHexNum(64,1,CAN_SendBuff[0],2,OLED_8X16);
	OLED_ShowHexNum(80,1,CAN_SendBuff[1],2,OLED_8X16);
	OLED_ShowString(1,16,"FUN  :",OLED_8X16);
	OLED_ShowHexNum(64,16,CAN_SendBuff[2],2,OLED_8X16);
	OLED_ShowString(1,32,"CHN  :",OLED_8X16);
	OLED_ShowHexNum(64,32,CAN_SendBuff[3],2,OLED_8X16);
	OLED_ShowString(1,48,"STATUS:",OLED_8X16);
	OLED_ShowHexNum(64,48,CAN_SendBuff[4],2,OLED_8X16);
	OLED_ShowHexNum(80,48,CAN_SendBuff[5],2,OLED_8X16);
	OLED_ShowHexNum(96,48,CAN_SendBuff[6],2,OLED_8X16);
	OLED_ShowHexNum(112,48,CAN_SendBuff[7],2,OLED_8X16);
	OLED_Update();
}

/**
* @brief SSPC锟斤拷锟斤拷
* @param 
*/
void SSPC_Set(void)
{
	SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_ALL,0);      //锟截闭癸拷锟斤拷通锟斤拷
	SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CFG_REPORT_CYC,0,0x1E8480);     //锟较憋拷锟斤拷锟斤拷50ms
	SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CFG_UVP,SSPC_CHN_ALL,0x2710);   //欠压10V
	SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CFG_OVP,SSPC_CHN_ALL,0x7530);   //锟斤拷压50V
	SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CFG_CURR,SSPC_CHN_5_8,0xC350);  //通锟斤拷5-8锟筋定锟斤拷锟斤拷50A
	SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CFG_CURR,SSPC_CHN_1,0x7530);    //通锟斤拷1  锟筋定锟斤拷锟斤拷30A
	SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CFG_CURR,SSPC_CHN_2,0x2710);    //通锟斤拷234锟筋定锟斤拷锟斤拷10A
	SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CFG_CURR,SSPC_CHN_3,0x2710);
	SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CFG_CURR,SSPC_CHN_4,0x2710);
	//SSPC_SendCmd(SSPC_ID,SSPC_FUNC_SAVE_FLASH,0,0);              //锟斤拷锟斤拷
	
}

/** 
 * @brief  CAN帧入队
 * @param  data: 帧数据
 * @param  id: 帧ID
 * @retval  
 * */
void CAN_Enqueue(uint8_t *data, uint32_t id)
{
// 如果队列已满，覆盖最旧帧（保留最新数据）
    if (queue_count >= CAN_QUEUE_SIZE) {
        // 读指针前进，丢弃最旧帧
        queue_head = (queue_head + 1) % CAN_QUEUE_SIZE;
        queue_count--;
    }
    // 拷贝数据到队列尾部
    memcpy(can_queue[queue_tail].data, data, 8);
    can_queue[queue_tail].id = id;
    // 更新队尾索引
    queue_tail = (queue_tail + 1) % CAN_QUEUE_SIZE;
    queue_count++;
	
}

/**
  * @brief 从队列中取出所有待处理帧并依次执行
  * @param now 当前系统时间（HAL_GetTick()）
  */
void Process_CAN_Queue(uint32_t now)
{
    // 循环处理，直到队列为空
    while (queue_count > 0) {
        uint8_t temp_data[8];
       // uint32_t temp_id;

        // 临界区保护（防止中断干扰）
        __disable_irq();
        if (queue_count == 0) {
            __enable_irq();
            break;
        }
        // 取出队首帧
        memcpy(temp_data, can_queue[queue_head].data, 8);
       // temp_id = can_queue[queue_head].id;
        // 移动队首指针
        queue_head = (queue_head + 1) % CAN_QUEUE_SIZE;
        queue_count--;
        __enable_irq();

        // ------ 处理该帧（原 `if(CAN_RxFinish == 1)` 中的逻辑） ------
        SSPC_Cmd(now, temp_data);   // 解析命令并执行
        if (SSPC_Lock_flag) {
            SSPC_CHN_Unlock(now, temp_data);
        }
    }
}

/**
* @brief SSPC初始化
* @param flag:判断地空模式标志位
* @param 
*/
void SSPC_Init(uint8_t flag)
{
	if(flag == GPIO_PIN_SET)
	{
		SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_1,0);HAL_Delay(SSPC_SendDelay);	//空中模式关闭通道1
		SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_2,0);HAL_Delay(SSPC_SendDelay);   /*打开熄火通道*/
		//  SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_3,0);HAL_Delay(SSPC_SendDelay);   
		// SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_8,0);HAL_Delay(SSPC_SendDelay);   
		// SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_7,0); HAL_Delay(SSPC_SendDelay);

		LogChannelOp(CHN_1,SSPC_FUNC_CHN_CLOSE,0); 
		LogChannelOp(CHN_2,SSPC_FUNC_CHN_OPEN,0);
		// LogChannelOp(CHN_3,SSPC_FUNC_CHN_OPEN,0);
		// LogChannelOp(CHN_8,SSPC_FUNC_CHN_OPEN,0);
		// LogChannelOp(CHN_7,SSPC_FUNC_CHN_OPEN,0);
	}
	
	else if(flag == GPIO_PIN_RESET)
	{
		//SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_5_8,0);HAL_Delay(SSPC_SendDelay);
		SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_5,0);HAL_Delay(SSPC_SendDelay);
		SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_6,0);HAL_Delay(SSPC_SendDelay);
		SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_1,0);HAL_Delay(SSPC_SendDelay);   
		SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_2,0);HAL_Delay(SSPC_SendDelay);   
		//SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_3,0);HAL_Delay(SSPC_SendDelay);  
		LogChannelOp(CHN_1,SSPC_FUNC_CHN_OPEN,0);
		LogChannelOp(CHN_2,SSPC_FUNC_CHN_OPEN,0);
		//LogChannelOp(CHN_3,SSPC_FUNC_CHN_OPEN,0);
		LogChannelOp(CHN_5,SSPC_FUNC_CHN_CLOSE,0);
		LogChannelOp(CHN_6,SSPC_FUNC_CHN_CLOSE,0);
		//LogChannelOp(CHN_7,SSPC_FUNC_CHN_CLOSE,0);
		//LogChannelOp(CHN_8,SSPC_FUNC_CHN_CLOSE,0);
	}
}

/**
 * @brief  SSPC解锁
 * @retval 
 */
void SSPC_CHN_Unlock(uint32_t now,uint8_t *data)
{
    static uint16_t time = 0; 
	static uint16_t count = 0;
	if(count != SSPC_Lock_flag)	//	判断是否接收到自锁指令
	{
       count = SSPC_Lock_flag;
	   time  = 1;
	   SSPC_ACK_flag = 0;
	}
	
	if(time == 1)
	{
	    if(lock_channel <= SSPC_CHN_4 && P1_UNLOCK_V)
	    {
		    SSPC_SendCmd(SSPC_ID,SSPC_FUNC_UNLOCK,SSPC_CHN_1_4,0);/*锟斤拷锟酵?锟斤拷锟斤拷锟斤拷*/
	    }
	    else if (lock_channel>= SSPC_CHN_5 && lock_channel <= SSPC_CHN_8 && P2_UNLOCK_V)
	    {
		    SSPC_SendCmd(SSPC_ID,SSPC_FUNC_UNLOCK,SSPC_CHN_5_8,0);
	    }
		Unlock_ENS_tick = HAL_GetTick();
		time = 2;
	}
	if(time == 2)
	{
		if(HAL_GetTick()-Unlock_ENS_tick >SSPC_UNLOCK_ENS)
		{
			time = 3;
		}
	}
	if(time == 3)
	{
		
	    if(SSPC_ACK_flag ==1)
		{
			SSPC_ACK_flag = 0;
			SSPC_Recover();
			SSPC_Lock_flag = 0;
		    time = 0;
		}

		
	}
}

/**
 * @brief  SSPC读取操作
 * @retval 
 */
void SSPC_CHN_Read(uint32_t now,FC_SendData* readdata,uint8_t data[8])
{
	switch(data[3])
	{
		case SSPC_CHN_1:   readdata->Vbus12H = data[4];readdata->Vbus12L = data[5];break;
		case SSPC_CHN_5:   readdata->Ichn5H = data[6];readdata->Ichn5L = data[7];  break;
		case SSPC_CHN_6:   readdata->Ichn6H = data[6];readdata->Ichn6L = data[7];  break;
	//case SSPC_CHN_7:   readdata->Ichn7H = data[6];readdata->Ichn7L = data[7];  break;
		case SSPC_CHN_8:   readdata->Ichn8H = data[6];readdata->Ichn8L = data[7]; break;
		case SSPC_CHN_5_8: readdata->Vchn8H = data[4];readdata->Vchn8L = data[5];
							//Vcheck_28Vbus(now,data[4],data[5]);
							if (Byte2_TO_U16(data[4],data[5]) > 0x2710)
							{P2_UNLOCK_V = 1;}
							else{P2_UNLOCK_V = 0;}
							break;
		case SSPC_CHN_1_4 : if(Byte2_TO_U16(data[4],data[5]) > 0x2710){P1_UNLOCK_V = 1;}
							else{P1_UNLOCK_V = 0;}
							break;
		default :break;
	}
}

/**
 * @brief  解锁操作记录
 * @retval 
 */
void Log_UnlockOp(uint16_t id)
{
	if(id == SSPC_CHN_1_4 )
	{
		LogChannelOp(CHN_1,SSPC_FUNC_UNLOCK,0);
	  LogChannelOp(CHN_2,SSPC_FUNC_UNLOCK,0);
	  LogChannelOp(CHN_3,SSPC_FUNC_UNLOCK,0);
	  LogChannelOp(CHN_4,SSPC_FUNC_UNLOCK,0);
	}
	else
 {
	 LogChannelOp(CHN_5,SSPC_FUNC_UNLOCK,0);
	 LogChannelOp(CHN_6,SSPC_FUNC_UNLOCK,0);
	 LogChannelOp(CHN_7,SSPC_FUNC_UNLOCK,0);
	 LogChannelOp(CHN_8,SSPC_FUNC_UNLOCK,0);
 } 
}

/**
 * @brief  通道恢复函数
 * @retval 
 */
void SSPC_Recover(void)
{
	GetAllChnLastOp(last_time,lastOp_valid_flag);
	for(uint8_t ch=0;ch<8;ch++)
	{
		if(lastOp_valid_flag[ch] == 1 )
		{
			switch(last_time[ch].op_code)
			{
				case SSPC_FUNC_CHN_OPEN :
					switch(ch){
						case 0:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_1,0); break;
						case 1:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_2,0); break;
						case 2:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_3,0); break;
						case 3: break;
						case 4:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_5,0);  break;
						case 5:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_6,0); break;
						case 6:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_7,0); break;
						case 7:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_8,0); break;
						default :break;
					}   break;
				case SSPC_FUNC_CHN_CLOSE: 
					 switch(ch){
						case 0:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_1,0); break;
						case 1:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_2,0); break;
						case 2:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_3,0); break;
						case 3:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_4,0); break;
						case 4:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_5,0); break;
						case 5:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_6,0); break;
						case 6:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_7,0); break;
						case 7:SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_8,0); break;
						default :break;
					}   break;
				default :break;
			}
		}
		HAL_Delay(SSPC_SendDelay);
	}
}

/**
 * @brief  28V电压检测
 * @param  now:当前时间
 * @param  VH:电压高位
 * @param  VL:电压低位
 * @retval 
 */
void Vcheck_28Vbus(uint32_t now,uint8_t VH,uint8_t VL)
{
	static uint8_t counter = 0;
	//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
    if (!voltage_valid) return;
	
    uint16_t vol = latest_28V_value;

    if (vol > 0x6B6C && SKY_GND_FLAG == GPIO_PIN_SET && counter == 0 && Vbus28_Full_Flag == 0) 
	{
        SSPC_SendCmd(SSPC_ID, SSPC_FUNC_CHN_OPEN, SSPC_CHN_1, 0);
        LogChannelOp(CHN_1, SSPC_FUNC_CHN_OPEN, 0);
    }
	if(now - tick_V28check > Wait_SSPC_28V )
	{
		tick_V28check = now;
		if(vol > 0x6B6C && SKY_GND_FLAG == GPIO_PIN_SET && Vbus28_Full_Flag == 0)
	   {
			
			 counter++; 
			 if(counter == 3)
		  	{
				counter = 0;
				Vbus28_Full_Flag = 1;
				//SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_OPEN,SSPC_CHN_1,0);
				//LogChannelOp(CHN_1,SSPC_FUNC_CHN_OPEN,0);
				SSPC_SendCmd(SSPC_ID,SSPC_FUNC_CHN_CLOSE,SSPC_CHN_3,0);
				LogChannelOp(CHN_3,SSPC_FUNC_CHN_CLOSE,0);
				return;				
	    	}
		}
		 else{ 
			counter = 0;
			//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
			return;
		}
	}
	else {return;}
	
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM4)
  {
    GPIO_PinState now_io = SKY_GND_FLAG;
    if(now_io == sky_gnd_key.curr_read)
    {
        sky_gnd_key.filter_cnt++;
        if(sky_gnd_key.filter_cnt >= 20)
        {
            sky_gnd_key.last_stable = now_io;
            sky_gnd_key.filter_cnt = 20;
        }
    }
    else
    {
        sky_gnd_key.curr_read = now_io;
        sky_gnd_key.filter_cnt = 0;
    }
  }
}

