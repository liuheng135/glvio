#include "events_manager.h"
#include "string.h"

void event_init(struct events_manager_s *man)
{
    int i;

    for(i = 0;i < EVENTS_MAX_NUM;i++){
        man->events_list[i].name = NULL;
    }
}

int event_send(struct events_manager_s *man,char *name,enum event_type_e type,float ctn_time,int arg)
{
    int i;
    int ret = -1;
    for(i = 0;i < EVENTS_MAX_NUM;i++){
        if(man->events_list[i].name == NULL){
            man->events_list[i].name = name;
            man->events_list[i].type = type;
            man->events_list[i].ctn_time = ctn_time;
            man->events_list[i].timer  = 0.0f;
            man->events_list[i].arg    = arg;
            ret = 0;
            break;
        }
    }
    return ret;

}

int event_delete(struct events_manager_s *man,char *name)
{
    int i;
    int ret = -1;
    int del_name_len,eve_name_len;

    del_name_len = strlen(name);
    for(i = 0;i < EVENTS_MAX_NUM;i++){
        if(man->events_list[i].name != NULL){
            eve_name_len = strlen(man->events_list[i].name);
            if(eve_name_len == del_name_len){
                if(strncmp(name,man->events_list[i].name,eve_name_len) == 0){
                    man->events_list[i].name = NULL;
                    break;
                }
            }
        }
    }
    return ret;        
}

int event_recv(struct events_manager_s *man,char *name,int *arg)
{
    int i;
    int ret = -1;
    int name_len,eve_name_len;

    name_len = strlen(name);
    for(i = 0;i < EVENTS_MAX_NUM;i++){
        if(man->events_list[i].name != NULL){
            eve_name_len = strlen(man->events_list[i].name);
            if(eve_name_len == name_len){
                if(strncmp(name,man->events_list[i].name,eve_name_len) == 0){
                    if(arg != NULL){
                        *arg = man->events_list[i].arg;
                    }
                    ret = 1;    
                    if(man->events_list[i].type == EVENT_TYPE_ONCE){
                        man->events_list[i].name = NULL;
                    }
                    break;
                }
            }
        }
    }
    return ret;
}

void events_manager_update(struct events_manager_s *man,float dt)
{
    int i;
    
    for(i = 0;i < EVENTS_MAX_NUM;i++){
        if(man->events_list[i].name != NULL){
            if(man->events_list[i].type == EVENT_TYPE_ENDU){
                if(man->events_list[i].timer < man->events_list[i].ctn_time){
                    man->events_list[i].timer+=dt;
                }else{
                    man->events_list[i].name = NULL;
                }
            }
        }
    }
}


