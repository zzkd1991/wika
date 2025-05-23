#include "button.h"

#define KEY_DELAY_TIME 		1000
static uint8_t key_scan_flag = 0;

uint8_t Key_Scan(void)
{
	static uint32_t curr_tick;
	if(HAL_GPIO_ReadPin(KEY_GPIO_PORT, KEY_PIN) == 0)
	{
		if(key_scan_flag == 0)
		{
			curr_tick = HAL_GetTick();
			key_scan_flag = 1;
		}

		if(HAL_GetTick() - curr_tick > 1000)
		{
			if(HAL_GPIO_ReadPin(KEY_GPIO_PORT, KEY_PIN) == 0)
			{
				key_scan_flag = 0;
				return 1;
			}
		}
	}
	return 0;
}


