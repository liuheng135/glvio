#ifndef _APP_MAVLINK_H_
#define _APP_MAVLINK_H_

#define MAVLINK_FMODE_STABILIZE  0
#define MAVLINK_FMODE_ALTHOLD    1
#define MAVLINK_FMODE_POSHOLD    2
#define MAVLINK_FMODE_TAKEOFF    3
#define MAVLINK_FMODE_LAND       4
#define MAVLINK_FMODE_FOLLOWME   5
#define MAVLINK_FMODE_AUTO       6
#define MAVLINK_FMODE_CIRCLE     7
#define MAVLINK_FMODE_FLIP       8
#define MAVLINK_FMODE_RTL        9
#define MAVLINK_FMODE_STOP       10
#define MAVLINK_FMODE_LOCAL_360  11
#define MAVLINK_FMODE_FINDME     12
#define MAVLINK_FMODE_VELHOLD    13


void  mavlink_init(void);
void  mavlink_update(float dt);
void  mavlink_info(float time,char *fmt,...);


#endif

