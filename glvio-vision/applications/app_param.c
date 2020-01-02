#include <malloc.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <sys/stat.h>
#include <stdlib.h>
#include "mathlib.h"
#include "config.h"
#include "app_param.h"
#include "param_manager.h"

#include "app_debug.h"

float param_update_timer;

bool param_changed;

struct param_manager_s pmanager;
struct param_notice_node param_notice_list[PARAM_NOTICE_NUM_MAX];

bool param_notice_add(void *cb)
{
    bool added = false;
    int i;

    for(i = 0;i < PARAM_NOTICE_NUM_MAX;i++){
        if(param_notice_list[i].used == false){
            param_notice_list[i].notify_callback = cb;
            param_notice_list[i].used = true;
            added = true;
            break;
        }
    }
    return added;
}

bool param_notice_remove(void *cb)
{
    bool removed = false;
    int i;

    for(i = 0;i < PARAM_NOTICE_NUM_MAX;i++){
        if(param_notice_list[i].used == true){
            if(param_notice_list[i].notify_callback == cb){
                param_notice_list[i].notify_callback = NULL;
                param_notice_list[i].used = false;
                removed = true;
                break;
            }
        }
    }
    return removed;
}
void param_notice_init(void)
{
    int i;

    for(i = 0;i < PARAM_NOTICE_NUM_MAX;i++){
        param_notice_list[i].used = false;
    }
}

void param_load(void)
{
    float param_num_saved;
    int   param_num_loaded;

    param_num_loaded = param_manager_load(&pmanager);
    param_manager_get(&pmanager,"param_count",&param_num_saved);
    
    
    if((param_num_loaded > 0) && (param_num_loaded == (int)param_num_saved)){
        printf("[PARAM]param loaded\r\n");
    }else{
        printf("[PARAM] l:%d s:%f\r\n",param_num_loaded,param_num_saved);
        if(param_manager_save(&pmanager) == 0){
            printf("[PARAM]can not find param file,create a new file!\r\n");
        }
    }
    param_changed = true;
}

void param_save(void)
{
    param_manager_save(&pmanager);
}

bool param_get(char* name,float *val)
{
    if(param_manager_get(&pmanager,name,val) == 0){
        return true;
    }else{
        return false;
    }
}

bool param_set(char* name,float val)
{
    if(param_manager_set(&pmanager,name,val) == 0){
        param_changed = true;
        return true;
    }else{
        return false;
    }
    
    return true;
}

void param_init(void)
{
    param_manager_init(&pmanager,PARAM_FILE_PATH);
    param_notice_init();

    param_update_timer = 0.0f;
    param_changed = false;
}

void param_update(float dt)
{
    int i;
    param_update_timer+=dt;

    if(param_update_timer > 1.0f){
        param_update_timer = 0.0f;

        if(param_changed == true){
            for(i = 0;i < PARAM_NOTICE_NUM_MAX;i++){
                if(param_notice_list[i].used == true){
                    param_notice_list[i].notify_callback();
                }
            } 
            param_changed = false;
        }
    }
}


