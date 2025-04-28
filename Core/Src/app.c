#include "app.h"


void API_EnteryStandby(void)
{
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	HAL_Delay(200);
	HAL_PWR_EnterSTANDBYMode();
}
