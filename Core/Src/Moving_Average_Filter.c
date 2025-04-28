#include "main.h"
#include "Moving_Average_Filter.h"

int8_t INIT_MOVING_AVERAGE_FILTER(MOVING_AVERAGE_FILTERtyp* filter, uint16_t* INPUT_BUFFER, uint16_t PointNum)
{
    filter->INIT = 1;
    filter->FILTER_POINT_NUM = PointNum;
    filter->INPUT_ARR = INPUT_BUFFER;
    return 0;
}
/**********
滑动滤波
***********/
void MOVING_AVERAGE_FILTER(MOVING_AVERAGE_FILTERtyp* filter)
{
	uint16_t  i;
	int8_t  INIT;
	int8_t  RESET;
	uint16_t INPUT;
	uint32_t SUM;
	uint16_t  FILTER_POINT_NUM;
	uint16_t  COUNT;
	uint16_t OUTPUT;

	INIT = filter->INIT;
	RESET = filter->RESET;
	INPUT = filter->INPUT;
	SUM = filter->SUM;
	FILTER_POINT_NUM = filter->FILTER_POINT_NUM;
	COUNT = filter->COUNT;
    
    if(filter->INPUT_ARR == NULL) return;
	if(INIT || RESET)
	{
		INIT = 0;
		RESET = 0;
		for(i = 0; i < FILTER_POINT_NUM; i++) filter->INPUT_ARR[i] = INPUT;
		SUM = INPUT * FILTER_POINT_NUM;
		COUNT = 0;
		OUTPUT = INPUT;
		filter->INIT = INIT;
		filter->RESET = RESET;
	}
	else
	{
		SUM -= filter->INPUT_ARR[COUNT];
		SUM += INPUT;
		filter->INPUT_ARR[COUNT] = INPUT;
		COUNT++;
		COUNT %= FILTER_POINT_NUM;
		OUTPUT = SUM / FILTER_POINT_NUM; 
	}
	filter->SUM = SUM;
	filter->COUNT = COUNT;
	filter->OUTPUT = OUTPUT;
}
/**********/
