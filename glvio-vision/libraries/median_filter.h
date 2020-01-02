#ifndef _MEDIAN_FILTER_H_
#define _MEDIAN_FILTER_H_

#include <string.h>

#define MEDIAN_FILTER_MAX_ORDER 10

struct median_filter_s{
    float buffer[MEDIAN_FILTER_MAX_ORDER];
    unsigned int head;
    unsigned int order;
};

int median_filter_init(struct median_filter_s *filter,int order);
float median_filter_update(struct median_filter_s *filter,float input);

#endif

