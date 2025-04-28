#include "button.h"

#define KEY_DELAY_TIME 		1000

uint8_t Key_Scan(void)
{
	if(HAL_GPIO_ReadPin(KEY_GPIO_PORT, KEY_PIN) == 0)
	{
		HAL_Delay(KEY_DELAY_TIME);

		if(HAL_GPIO_ReadPin(KEY_GPIO_PORT, KEY_PIN) == 0)
		{
			return 1;
		}
	}

	return 0;
}


