#include "median_filter.h"

int median_filter_init(struct median_filter_s *filter,int order)
{
    memset(filter->buffer,0x00,sizeof(filter->buffer));
    filter->head = 0;
    filter->order = order;  

    if(filter->order > MEDIAN_FILTER_MAX_ORDER){
        filter->order = MEDIAN_FILTER_MAX_ORDER;
    }
    return filter->order;
}

float median_filter_order(struct median_filter_s *filter)
{
    int i;
    int j;
    float temp_buffer[MEDIAN_FILTER_MAX_ORDER];

    for(i = 0; i < filter->order; i++){
        temp_buffer[i] = filter->buffer[(filter->head - filter->order + i) % filter->order];
    }
    
    for(i = 0; i < filter->order; i++)
    {
        for(j = i; j < filter->order; j++)
        {
            if(temp_buffer[i] > temp_buffer[j])
            {
                int temp = temp_buffer[i];
                temp_buffer[i] = temp_buffer[j];
                temp_buffer[j] = temp;
            }
        }
    }
    
    if(filter->order % 2 == 0){
        return (temp_buffer[filter->order / 2] + temp_buffer[filter->order / 2 -1]) * 0.5f;
    }else{
        return temp_buffer[(filter->order - 1) / 2];
    }
}

float median_filter_update(struct median_filter_s *filter,float input)
{
    if(filter->head > 0xfffffff0){
        filter->head -= 0xfffffff0;
    }

    filter->buffer[filter->head % filter->order] = input;
    filter->head++;

    if(filter->head >= filter->order){
        return median_filter_order(filter);
    }else{
        return 0.0f;
    }   
}

