#ifndef _APP_GLLINK_H_
#define _APP_GLLINK_H_

#include "lwlink.h"

#define LWLINK_FMODE_STABILIZE  0
#define LWLINK_FMODE_ALTHOLD    1
#define LWLINK_FMODE_POSHOLD    2
#define LWLINK_FMODE_TAKEOFF    3
#define LWLINK_FMODE_LAND       4
#define LWLINK_FMODE_FOLLOWME   5
#define LWLINK_FMODE_AUTO       6
#define LWLINK_FMODE_CIRCLE     7
#define LWLINK_FMODE_FLIP       8
#define LWLINK_FMODE_RTL        9
#define LWLINK_FMODE_STOP       10
#define LWLINK_FMODE_LOCAL_360  11
#define LWLINK_FMODE_FINDME     12
#define LWLINK_FMODE_VELHOLD    13

void  lwlink_init(void);
void  lwlink_update(float dt);
void  lwlink_info(float time,char *fmt,...);


#endif

