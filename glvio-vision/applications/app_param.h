#ifndef _APP_PARAM_H_
#define _APP_PARAM_H_

#include <stdbool.h>

//#include "config.h"

#define PARAM_NOTICE_NUM_MAX  32

struct param_notice_node{
    bool used;
    void (*notify_callback)(void);
};


void param_init(void);
void param_update(float dt);

void param_load(void);
void param_save(void);
void param_list(void);
bool param_get(char* name,float * val);
bool param_set(char* name,float val);
bool param_notice_add(void *cb);

#endif

