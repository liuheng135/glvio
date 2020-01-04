#include "app_gllink.h"
#include "app_flow.h"
#include "hal_device.h"
#include "app_debug.h"
#include "app_vio.h"
#include "mathlib.h"
#include "app_events.h"

int gllink_channel_fd;
struct lwlink_data_handler_s lwlink_handler;

float  lwlink_send_timer;

int lwlink_if_connected;

void  lwlink_init(void)
{
    gllink_channel_fd = hal_dev_open("net1",HAL_O_RDWR);
	if(gllink_channel_fd > 0){
        msg_recoder_log(0,"open net1 successfully\r\n");
	}else{
        msg_recoder_log(0,"open net1 failed\r\n");
	}
    
	lwlink_data_handler_init(&lwlink_handler,2);
}

void lwlink_send_data(int fd,float dt)
{
    struct lwlink_attitude_s att;
    struct vio_data_s vio_dat;
    struct eulur_s vio_att_e;
    
    int msg_length;
    
    lwlink_send_timer+=dt;
    
    if(lwlink_send_timer > 0.2f){
        lwlink_send_timer -= 0.2f;

        vio_get_data(&vio_dat);
        quater_to_eulur(&vio_att_e,&vio_dat.rotation);

        att.component_id = 0;
        att.roll  = vio_att_e.roll;
        att.pitch = vio_att_e.pitch;
        att.yaw   = vio_att_e.yaw;
        
        msg_length = lwlink_msg_pack(&lwlink_handler,MSG_TYPE_ATTITUDE,(uint8_t *)&att,sizeof(att));
        hal_dev_write(fd,lwlink_handler.txbuf,msg_length,0);
    }
}

void  lwlink_update(float dt)
{
    int i,len;
    unsigned char buf[128];
    
    if(gllink_channel_fd >= 0){
        lwlink_send_data(gllink_channel_fd,dt);
        
        len = hal_dev_read(gllink_channel_fd,buf,100,0);
        if(len > 0){
            for(i = 0;i < len;i++){
                if(lwlink_data_handler_parse(&lwlink_handler,buf[i]) == 1){
                    msg_recoder_log(0,"gllink msg recved:%d\r\n",lwlink_data_handler_get_type(&lwlink_handler));
                }
            }
        }
    }
}

void  lwlink_info(float time,char *fmt,...)
{
    
}

