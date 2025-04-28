#ifndef MOVING_AVERAGE_FILTER_H
#define MOVING_AVERAGE_FILTER_H

/**********
滑动滤波(16位无符号整型，AD值滤波)
使用前需要置INIT为1
使用时需要给出INPUT和FILTE_POINT_NUM
滤波结果OUTPUT
***********/
typedef struct
{		
	int8_t    INIT;			    //初始化标志
	int8_t    RESET;		    //重置标志
	uint16_t    INPUT;		    //输入值
	uint32_t    SUM;			    //累加和
	uint16_t  FILTER_POINT_NUM;	//滤波点数
	uint16_t*   INPUT_ARR;	//保存输入值数组
	uint16_t  COUNT;			//计数器
	uint16_t    OUTPUT;			//输出
}
MOVING_AVERAGE_FILTERtyp;

int8_t INIT_MOVING_AVERAGE_FILTER(MOVING_AVERAGE_FILTERtyp* filter, uint16_t* INPUT_BUFFER, uint16_t PointNum);
void MOVING_AVERAGE_FILTER(MOVING_AVERAGE_FILTERtyp* filter);

#endif
