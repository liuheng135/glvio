#include <string.h>
#include <stdarg.h>
#include "app_debug.h"

struct msg_recoder_s msg_recoder_list[MSG_RECODER_NUM_MAX];
char msg_recoder_debug_module[MSG_RECODER_NANE_MAX];

void msg_recoder_init(void)
{
    memset(msg_recoder_list,0x00,sizeof(msg_recoder_list));
    msg_recoder_list[0].used = true;
    msg_recoder_list[0].name = "COMMON";
    msg_recoder_list[0].interval = 0.1f;
    msg_recoder_list[0].debug = false;
}

int msg_recoder_create(char *name,float interval)
{
    int i;
    int new_recoder = -1;
    
    for(i = 0; i < MSG_RECODER_NUM_MAX;i++){
        if(msg_recoder_list[i].used == false){
            new_recoder = i;
            break;
        }
    }

    if( new_recoder > -1){
        msg_recoder_list[new_recoder].name  = name;
        msg_recoder_list[new_recoder].timer = 0;
        msg_recoder_list[new_recoder].interval = interval;
        msg_recoder_list[new_recoder].used = true;

        if(strcmp(msg_recoder_debug_module,name) == 0){
            msg_recoder_list[new_recoder].debug = true;
        }
    }

    return new_recoder;
}

void  msg_recoder_destory(int id)
{
    if(id < 0 || id > MSG_RECODER_NUM_MAX){
        return;
    }
    msg_recoder_list[id].used = false;
}



void  msg_recoder_log(int id,char *fmt,...)
{
    va_list args;
    char buff1[256];
    char buff2[256];

    if(id < 0 || id > MSG_RECODER_NUM_MAX){
        return;
    }

    va_start(args, fmt);
    vsprintf(buff1, fmt, args);
    sprintf(buff2,"[%s] %s",msg_recoder_list[id].name,buff1);
    printf("%s",buff2);
    va_end(args);
}

void  msg_recoder_debug(int id,char *fmt,...){
    va_list args;
    char buff1[256];
    char buff2[256];

    if(id < 0 || id > MSG_RECODER_NUM_MAX){
        return;
    }
    
    if(msg_recoder_list[id].debug == false){
        return;
    }
    
    va_start(args, fmt);
    vsprintf(buff1, fmt, args);
    sprintf(buff2,"[%s] %s",msg_recoder_list[id].name,buff1);
    printf("%s",buff2);
    va_end(args);
}

void  msg_recoder_debug_circlar(int id,float dt,char *fmt,...)
{
    va_list args;
    char buff[256];

    if(id < 0 || id > MSG_RECODER_NUM_MAX){
        return;
    }

    if(msg_recoder_list[id].debug == false){
        return;
    }
    
    msg_recoder_list[id].timer += dt;
    if(msg_recoder_list[id].timer > msg_recoder_list[id].interval){
        msg_recoder_list[id].timer -= msg_recoder_list[id].interval;
        va_start(args, fmt);
        vsprintf(buff, fmt, args);
        printf("[%s] %s",msg_recoder_list[id].name,buff);
        va_end(args);
    }
}

void msg_recoder_enable_debug(char *name)
{
    strcpy(msg_recoder_debug_module,name);
}


