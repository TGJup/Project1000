#include "main.h"
#include "LED.h"

/**
 * @brief   LED亮操作函数
 * @retval 
 */
void LED_ON(void)
{
	HAL_GPIO_WritePin(LED_GPIO_Port,LED_PIN,GPIO_PIN_RESET);
}
/**
 * @brief   LED灭操作函数
 * @retval 
 */
void LED_OFF(void)
{
	HAL_GPIO_WritePin(LED_GPIO_Port,LED_PIN,GPIO_PIN_SET);
}
/**
 * @brief   LED转换操作函数
 * @retval 
 */
uint32_t led_tick = 0;
void LED_TURN(uint32_t now,uint32_t LED_WAIT_TIME)
{
	if(now - led_tick >= LED_WAIT_TIME)
			 {
			   led_tick = now;   
			   HAL_GPIO_TogglePin(LED_GPIO_Port, LED_PIN);		 
			 }    
    
}
