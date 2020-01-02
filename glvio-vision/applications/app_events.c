#include "app_events.h"
#include "events_manager.h"

static struct events_manager_s app_events_manager;

void app_events_init(void)
{
    event_init(&app_events_manager);
}

int app_events_send(char *name,enum event_type_e type,float time,int arg)
{
    return event_send(&app_events_manager,name,type,time,arg); 
}
int  app_events_recv(char *name,int *arg)
{
    return event_recv(&app_events_manager,name,arg);
}
int  app_events_delete(char *name)
{
    return event_delete(&app_events_manager,name);
}
void  app_events_update(float dt)
{
    events_manager_update(&app_events_manager,dt);
}


