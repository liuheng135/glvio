#ifndef _APP_EVENTS_H_
#define _APP_EVENTS_H_


#include "events_manager.h"


void app_events_init(void);
int app_events_send(char *name,enum event_type_e type,float time,int arg);
int  app_events_recv(char *name,int *arg);
int  app_events_delete(char *name);
void app_events_update(float dt);


#endif 

