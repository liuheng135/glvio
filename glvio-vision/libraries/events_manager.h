#ifndef _EVENTS_MANAGER_H_
#define _EVENTS_MANAGER_H_


#define EVENTS_MAX_NUM  64

enum event_type_e{
    EVENT_TYPE_ONCE = 0,
    EVENT_TYPE_ENDU,
    EVENT_TYPE_FOREVER,
};

struct event_s{
    char *name;
    enum event_type_e type;
    float ctn_time;
    float timer;
    int arg;
};

struct events_manager_s{
    struct event_s events_list[EVENTS_MAX_NUM];
};

void event_init(struct events_manager_s *man);
int  event_send(struct events_manager_s *man,char *name,enum event_type_e type,float ctn_time,int arg);
int  event_recv(struct events_manager_s *man,char *name,int *arg);
int  event_delete(struct events_manager_s *man,char *name);
void events_manager_update(struct events_manager_s *man,float dt);

#endif
