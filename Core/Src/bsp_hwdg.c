#include "bsp_hwdg.h"

/*
*brief:看门狗使能函数
*Para:0:禁止看门狗;1:使能看门狗                
*Ret: 0:成功;1:失败
*/

uint8_t API_WatchDog_Enable(uint8_t Statue_u8)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(Statue_u8 == 1)
    {
		GPIO_InitStruct.Pin = WDI_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(WDI_GPIO, &GPIO_InitStruct);
    }
	else
	{
		GPIO_InitStruct.Pin = WDI_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(WDI_GPIO, &GPIO_InitStruct);
	}
    return 0;	
}

/*
*brief:喂狗函数，喂狗时间应小于1.6s
*Para :None               
*Ret  : 0:成功,1:失败                
*/
uint8_t API_WatchDog_FeedDog(void)
{
	HAL_GPIO_TogglePin(WDI_GPIO, WDI_Pin);
    return 0;	
}






