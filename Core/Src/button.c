#include "button.h"

#define KEY_DELAY_TIME 		3000
key_scan_state key_state = {.key_scan_flag = 0xff};
uint8_t Key_Scan(void)
{
	if(HAL_GPIO_ReadPin(KEY_GPIO_PORT, KEY_PIN) == 0)
	{
		if(key_state.key_scan_tick_flag == 0)
		{
			key_state.key_curr_tick = HAL_GetTick();
			key_state.key_scan_tick_flag = 1;
		}
		
		if(HAL_GetTick() - key_state.key_curr_tick > 50)
		{
			if(HAL_GPIO_ReadPin(KEY_GPIO_PORT, KEY_PIN) == 1)
			{
					key_state.key_scan_flag = 0xff;
					return 0;
			}
		}
		
		if(HAL_GetTick() - key_state.key_curr_tick > KEY_DELAY_TIME)
		{
			if(HAL_GPIO_ReadPin(KEY_GPIO_PORT, KEY_PIN) == 0)
			{
				key_state.key_scan_tick_flag = 0;
				key_state.key_curr_tick = 0;
				return 1;
			}
		}
	}
	return 0;
}
