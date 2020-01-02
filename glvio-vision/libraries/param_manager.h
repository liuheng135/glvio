#ifndef _PARAM_MANAGER_H_
#define _PARAM_MANAGER_H_

#define PARAM_NAME_LENGTH_MAX 32

struct param_info_s{ 
    char name[PARAM_NAME_LENGTH_MAX];
    float val; 
    struct param_info_s *next;
};


struct param_manager_s{
    char *param_file;
    struct param_info_s param_list;
};


void param_manager_init(struct param_manager_s *manager,char* file);
int  param_manager_save(struct param_manager_s *manager);
int  param_manager_load(struct param_manager_s *manager);
int  param_manager_set(struct param_manager_s *manager,char *name,float val);
int  param_manager_get(struct param_manager_s *manager,char *name,float *val);

#endif
