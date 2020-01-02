#include "param_manager.h"
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdbool.h>
#include <malloc.h>
#include <sys/stat.h>


#define PARAM_SEPARATOR  '='

int param_check_line(char *line)
{
    int ret = -1;
    int i;
    int length = strlen(line);
    
    for(i = 0; i < length;i++){
        if(line[i] >= 'a' && line[i] <= 'z'){
            ret = 0;
        }else if(line[i] >= 'A' && line[i] <= 'Z'){
            ret = 0;
        }else if(line[i] >= '0' && line[i] <= '9'){
            ret = 0;
        }else if(line[i] == ' '){
            ret = 0;
        }else if(line[i] == '='){
            ret = 0;
        }else if(line[i] == '.'){
            ret = 0;
        }else if(line[i] == '_'){
            ret = 0;
        }else if(line[i] == '\r'){
            ret = 0;
        }else if(line[i] == '\n'){
            ret = 0;
        }else if(line[i] == '-'){
            ret = 0;
        }else{
            ret = -1;
            break;
        }
    }
    return ret;
}

void param_split(char *line,char sign)
{
    char *ptr = NULL;
    int i;
    int length = strlen(line);

    ptr = line;
    for(i = 0; i < length;i++){
        if(line[i] == sign){
            continue;
        }else{
           ptr = &line[i];
           length -= i;
           break;
        }
    }

    for(i = 0;i < length;i++){
        if(ptr[i] == sign){
            line[i] = '\0';
            break;
            
        }else{
            line[i] = ptr[i];
        }
    }
}


float param_atof(char *str)  
{  
    float s=0.0;  
  
    float d=10.0;  
    int jishu=0;  
  
    bool falg=false;  
  
    while(*str==' ')  
    {  
        str++;  
    }  
  
    if(*str=='-')//记录数字正负  
    {  
        falg=true;  
        str++;  
    }  
  
    if(!(*str>='0'&&*str<='9'))//如果一开始非数字则退出，返回0.0  
        return s;  
  
    while(*str>='0'&&*str<='9'&&*str!='.')//计算小数点前整数部分  
    {  
        s=s*10.0f+*str-'0';  
        str++;  
    }  
  
    if(*str=='.')//以后为小树部分  
        str++;  
  
    while(*str>='0'&&*str<='9')//计算小数部分  
    {  
        s=s+(*str-'0')/d;  
        d*=10.0f;  
        str++;  
    }  
  
    if(*str=='e'||*str=='E')//考虑科学计数法  
    {  
        str++;  
        if(*str=='+')  
        {  
            str++;  
            while(*str>='0'&&*str<='9')  
            {  
                jishu=jishu*10.0f+*str-'0';  
                str++;  
            }  
            while(jishu>0)  
            {  
                s*=10.0f;  
                jishu--;  
            }  
        }  
        if(*str=='-')  
        {  
            str++;  
            while(*str>='0'&&*str<='9')  
            {  
                jishu=jishu*10.0f+*str-'0';  
                str++;  
            }  
            while(jishu>0)  
            {  
                s/=10.0f;  
                jishu--;  
            }  
        }  
    }  
  
    return s*(falg?-1.0f:1.0f);  
}  


int param_parse_line(struct param_info_s *param,char *line)
{
    int i;
    char name[PARAM_NAME_LENGTH_MAX];
    char val[PARAM_NAME_LENGTH_MAX];
    int name_len = 0;
    int val_len = 0;
    int line_len = 0; 
    char *val_ptr = NULL;

    if(param == NULL){
        return -1;
    }

    if(param_check_line(line) < 0){
        return -2;
    }
    line_len = strlen(line);
    
    for(i = 0; i < line_len; i++){
        if(line[i] == PARAM_SEPARATOR){
            name_len = i;
            val_len = line_len - i - 1;
            val_ptr = &line[i+1];
            break;
        }
    }
    memset(name,0x00,PARAM_NAME_LENGTH_MAX);
    memset(val,0x00,PARAM_NAME_LENGTH_MAX);
    memcpy(name,line,name_len);
    memcpy(val,val_ptr,val_len);
    
    param_split(name,' ');
    param_split(val,' ');
    param_split(val,'\r');
    param_split(val,'\n');
  
    strncpy(param->name,name,name_len);
    param->val = param_atof(val);

    return 0;
}


void param_manager_init(struct param_manager_s *manager,char* file)
{
    char param_list_name[] = "param_count";
    manager->param_file = file;
    memset(manager->param_list.name,0x00,PARAM_NAME_LENGTH_MAX);
    memcpy(manager->param_list.name,param_list_name,sizeof(param_list_name));
    manager->param_list.val = 1;
}

int param_manager_add(struct param_manager_s *manager,char *name,float val)
{
    
    struct param_info_s *ptr;
    struct param_info_s *new_param;
    int name_len;
    ptr = &manager->param_list;

    while(ptr->next != NULL){
        ptr = ptr->next;
    }

    new_param = malloc(sizeof(struct param_info_s));

    if(new_param == NULL){
        return -1;
    }
    name_len = strlen(name);
    name_len = name_len > PARAM_NAME_LENGTH_MAX ? PARAM_NAME_LENGTH_MAX : name_len;
    memset(new_param->name,0x00,PARAM_NAME_LENGTH_MAX);
    memcpy(new_param->name,name,name_len);
    new_param->val = val;
    new_param->next = NULL;
    manager->param_list.val += 1.0f;

    ptr->next = new_param;
    return 0;
}

struct param_info_s * param_manager_find(struct param_manager_s *manager,char *name)
{ 
    struct param_info_s *ptr;
    struct param_info_s *ret = NULL;
    int name_len = strlen(name);
    name_len = name_len > PARAM_NAME_LENGTH_MAX ? PARAM_NAME_LENGTH_MAX : name_len;
    ptr = &manager->param_list;
    
    while(ptr != NULL){
        if(strncmp(ptr->name,name,name_len) == 0){
            ret = ptr;
            break;
        }
        ptr = ptr->next;
    }
 
    return ret;
}

void param_manager_delete_all(struct param_manager_s *manager)
{ 
    struct param_info_s *ptr;
    struct param_info_s *next;
    
    ptr = manager->param_list.next;

    while(ptr != NULL){
        next = ptr->next;
        free(ptr);
        ptr = next;
    }
    manager->param_list.next = NULL;
    manager->param_list.val = 1.0f;
}

int param_manager_save(struct param_manager_s *manager)
{
    int fd;
    int len;
    char buff[128];
    struct param_info_s *ptr;

    fd = open(manager->param_file,O_RDWR | O_CREAT | O_TRUNC,0777);

    if(fd < 0){
        return -1;
    }

    ptr = &manager->param_list;
    while(ptr != NULL){
        memset(buff,0x00,128);
        len = sprintf(buff,"%s %c %f\n",ptr->name,PARAM_SEPARATOR,ptr->val);
        len = write(fd,buff,len);
        ptr = ptr->next;
    }
    fsync(fd);
    close(fd);
    return 0;
}

int param_manager_load(struct param_manager_s *manager)
{
    int i;
    int fd;
    char buff;
    char line[128];
    int param_num = 0;
    struct param_info_s param;
   
    fd = open(manager->param_file,O_RDONLY);
    if(fd < 0){
        return -1;
    }

    i = 0;
    memset(line,0x00,128);
    while(read(fd,&buff,1) == 1){
        line[i++] = buff;
        if(buff == '\n'){
            if(param_parse_line(&param,line) == 0){
                param_manager_set(manager,param.name,param.val);
                param_num++;
            }
            i = 0;
            memset(line,0x00,128);
        }
        if(i > 127){
            i = 0;
            memset(line,0x00,128);
            continue;
        }
    }
	
	close(fd);
    return param_num;
}

int param_manager_set(struct param_manager_s *manager,char *name,float val)
{
    struct param_info_s *param;

    param = param_manager_find(manager,name);

    if(param == NULL){
        return param_manager_add(manager,name,val);
    }else{
        param->val = val;
        return 0;
    }
}

int param_manager_get(struct param_manager_s *manager,char *name,float *val)
{
    struct param_info_s *param;

    param = param_manager_find(manager,name);

    if(param == NULL){
        return -1;
    }else{
        *val = param->val;
        return 0;
    }
}

