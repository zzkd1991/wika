#ifndef CIRCULARQUEUE_H
#define CIRCULARQUEUE_H

#include "main.h"


typedef struct
{
    void*  CircularQueue_Data;
    int32_t  Write_Point;//д���
    int32_t  Read_Point;//��ȡ��
    uint32_t  Circular_Count;//ѭ������
    uint32_t Data_size;//���ݴ�С
    uint32_t Single_Data_size;//���ݴ�С
}CircularQueue_Str;

#define  GET_REMAINDATA_SIZE(QUEUE)    ((((QUEUE)->Write_Point - (QUEUE)->Read_Point) >= 0)?((QUEUE)->Write_Point - (QUEUE)->Read_Point):(((QUEUE)->Data_size - (QUEUE)->Read_Point) + (QUEUE)->Write_Point))

int8_t   Init_CircularQueue(CircularQueue_Str* queue, void* arr_data, uint32_t data_size, uint32_t single_data_size);
int8_t   PushData_CircularQueue(CircularQueue_Str* queue, void* single_data);
int32_t  GetData_CircularQueue(CircularQueue_Str* queue, void* arr_data);
int32_t GetSingleData_CircularQueue(CircularQueue_Str* queue, void* data_p);


#endif
