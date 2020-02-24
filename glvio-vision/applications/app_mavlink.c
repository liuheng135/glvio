#include "app_mavlink.h"
#include "app_calibrator.h"
#include "app_flow.h"
#include "hal_device.h"
#include "app_debug.h"
#include "mathlib.h"
#include "app_events.h"

#include "mavlink.h"

int mavlink_channel_fd;
float  mavlink_send_timer;
int mavlink_if_connected;

uint8_t mavlink_global_buffer[1024];

void  mavlink_init(void)
{
    mavlink_channel_fd = hal_dev_open("net1",HAL_O_RDWR);
	if(mavlink_channel_fd > 0){
        msg_recoder_log(0,"mavlink start\r\n");
	}else{
        msg_recoder_log(0,"open mavlink channel failed\r\n");
	}
}

void mavlink_send_data(int fd,float dt)
{
    mavlink_message_t msg;
    
    int msg_length;
    
    mavlink_send_timer+=dt;
    
    if(mavlink_send_timer > 0.2f){
        mavlink_send_timer -= 0.2f;
         
        mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_MANUAL_DISARMED, 0, MAV_STATE_STANDBY);
        msg_length = mavlink_msg_to_send_buffer(mavlink_global_buffer, &msg);
        hal_dev_write(fd,mavlink_global_buffer,msg_length,0);
        /* Send Status */
        mavlink_msg_sys_status_pack(1, 200, &msg, 0, 0, 0, 500, 11000, -1, -1, 0, 0, 0, 0, 0, 0);
        msg_length = mavlink_msg_to_send_buffer(mavlink_global_buffer, &msg);
        hal_dev_write(fd,mavlink_global_buffer,msg_length,0);
        /* Send attitude */
		mavlink_msg_attitude_pack(1, 200, &msg,100000000,0.0f,0.1f,0.1f,0.0f,0.0f,0.0f);
		msg_length = mavlink_msg_to_send_buffer(mavlink_global_buffer, &msg);
		hal_dev_write(fd,mavlink_global_buffer,msg_length,0);
		printf("mavlink send\r\n");
    }
}

void  mavlink_update(float dt)
{
    int len;
    unsigned char buf[128];
    
    if(mavlink_channel_fd >= 0){
        mavlink_send_data(mavlink_channel_fd,dt);
        
        len = hal_dev_read(mavlink_channel_fd,buf,100,0);
        if(len > 0){
            msg_recoder_log(0,"mavlink received %d \r\n",len);
        }
    }
}
float mavlink_get_heartbeat_rate(void)
{
    return 1.0f;
}

void  mavlink_info(float time,char *fmt,...)
{
    
}

