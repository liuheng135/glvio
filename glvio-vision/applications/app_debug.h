#ifndef _APP_DEBUG_H_
#define _APP_DEBUG_H_

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

#define MSG_RECODER_NANE_MAX 16
#define MSG_RECODER_NUM_MAX  48 

struct msg_recoder_s {
    char   *name;
    bool    debug;
    bool    used;
    float   interval;
    float   timer;
};

void msg_recoder_init(void);
int   msg_recoder_create(char *name,float interval);
void  msg_recoder_destory(int id);
void  msg_recoder_log(int id,char *fmt,...);
void  msg_recoder_debug(int id,char *fmt,...);
void  msg_recoder_debug_circlar(int id,float dt,char *fmt,...);
void  msg_recoder_enable_debug(char *name);

#endif
