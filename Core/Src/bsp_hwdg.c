#include "bsp_hwdg.h"

/*
*brief:���Ź�ʹ�ܺ���
*Para:0:��ֹ���Ź�;1:ʹ�ܿ��Ź�                
*Ret: 0:�ɹ�;1:ʧ��
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
*brief:ι��������ι��ʱ��ӦС��1.6s
*Para :None               
*Ret  : 0:�ɹ�,1:ʧ��                
*/
uint8_t API_WatchDog_FeedDog(void)
{
	HAL_GPIO_TogglePin(WDI_GPIO, WDI_Pin);
    return 0;	
}






