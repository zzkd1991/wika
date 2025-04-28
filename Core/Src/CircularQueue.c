#include <string.h>
#include "CircularQueue.h"

int8_t   Init_CircularQueue(CircularQueue_Str* queue, void* arr_data, uint32_t data_size, uint32_t single_data_size)
{
    queue->CircularQueue_Data = arr_data;
    queue->Circular_Count = 0;
    queue->Write_Point = 0;
    queue->Read_Point = 0;
    queue->Data_size = data_size;
    queue->Single_Data_size = single_data_size;
    return 0;
}

int8_t   PushData_CircularQueue(CircularQueue_Str* queue, void* single_data)
{    
    int32_t write_point_cpy;
    int32_t write_point;
    int32_t read_point;
    uint32_t cir_count;
	
    write_point = queue->Write_Point;
    read_point = queue->Read_Point;
    cir_count = queue->Circular_Count;
	
    memcpy((void*)((uint32_t)queue->CircularQueue_Data + (write_point * queue->Single_Data_size)),
           (const void *)single_data,
           queue->Single_Data_size);
    write_point_cpy = write_point;
    write_point++;
    if(write_point >= queue->Data_size) 
    {
        write_point = 0;
    }
    if(write_point_cpy < read_point && write_point >= read_point)
    {
        cir_count++;
    }
    queue->Write_Point=write_point;
    queue->Read_Point=read_point;
    queue->Circular_Count=cir_count;
    return 0;
}

int32_t GetSingleData_CircularQueue(CircularQueue_Str* queue, void* data_p)
{
    int32_t write_point;
    int32_t read_point;
    uint32_t cir_count;
    //int32_t read_len = 0;

    write_point = queue->Write_Point;
    read_point = queue->Read_Point;
    cir_count = queue->Circular_Count;

    if(write_point == read_point && cir_count == 0) return -1;
    if(cir_count > 0)
    {
        memcpy(data_p, 
               (void*)((uint32_t)queue->CircularQueue_Data + (write_point * queue->Single_Data_size)),
               queue->Single_Data_size);
        queue->Read_Point = write_point + 1;
        queue->Read_Point %= queue->Data_size;
        queue->Circular_Count = 0;
        //read_len = 1;
    }
    else
    {
        memcpy(data_p, 
               (void*)((uint32_t)queue->CircularQueue_Data + (read_point * queue->Single_Data_size)),
               queue->Single_Data_size);
        read_point++;
        read_point %= queue->Data_size;
        if(read_point == 0) queue->Circular_Count = 0;
        //read_len = 1;
    }
    queue->Read_Point = read_point;
    return 0;
}

int32_t GetData_CircularQueue(CircularQueue_Str* queue, void* arr_data)
{
    int32_t write_point;
    int32_t read_point;
    uint32_t cir_count;
    int32_t read_len = 0;
    
    write_point = queue->Write_Point;
    read_point = queue->Read_Point;
    cir_count = queue->Circular_Count;
    if(cir_count > 0)
    {
        if(write_point >= read_point)//Write point beyond read point ©¸©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤RP©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤WP©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¼
        {
            memcpy(arr_data, 
                   (void*)((uint32_t)queue->CircularQueue_Data + (write_point * queue->Single_Data_size)),
                   (queue->Data_size - write_point) * queue->Single_Data_size);
            read_len = queue->Data_size - write_point;
            memcpy((void*)((uint32_t)arr_data + read_len * queue->Single_Data_size),
                   queue->CircularQueue_Data,
                   write_point * queue->Single_Data_size);
            read_len = queue->Data_size;
        }
        else//Read point beyond write point                          ©¸©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤WP©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤RP©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¼
        {
            memcpy(arr_data, 
                   (void*)((uint32_t)queue->CircularQueue_Data + (read_point * queue->Single_Data_size)),
                   (queue->Data_size - read_point) * queue->Single_Data_size);
            read_len = queue->Data_size - read_point;
            memcpy((void*)((uint32_t)arr_data + read_len * queue->Single_Data_size),
                   queue->CircularQueue_Data,
                   write_point * queue->Single_Data_size);
            read_len += write_point;
        }
    }
    else
    {
        if(write_point >= read_point)//Write point beyond read point ©¸©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤RP©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤WP©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¼
        {
            memcpy(arr_data,
                   (void*)((uint32_t)queue->CircularQueue_Data + (read_point * queue->Single_Data_size)),
                   (write_point - read_point) * queue->Single_Data_size);
            read_len = write_point - read_point;
        }
        else//Read point beyond write point                          ©¸©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤WP©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤RP©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¼
        {
            queue->Read_Point = write_point;
            queue->Circular_Count = 0;
            return -1;
        }
    }
    queue->Read_Point = write_point;
    queue->Circular_Count = 0;
    return read_len;
}





