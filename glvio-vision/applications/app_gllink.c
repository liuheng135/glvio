#include "app_gllink.h"
#include "app_flow.h"
#include "hal_device.h"
#include "app_debug.h"
#include "app_vio.h"
#include "mathlib.h"
#include "app_events.h"


unsigned char gllink_image_buffer[FLOW_IMAGE_WIDTH*FLOW_IMAGE_HEIGHT];
float gllink_image_timestamp;

int gllink_channel_fd;
struct lwlink_data_handler_s lwlink_handler;

float  lwlink_send_timer_20hz;
float  lwlink_send_timer_10hz;

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
    lwlink_send_timer_10hz = 0.0f;
    lwlink_send_timer_20hz = 0.0f;
}

void lwlink_send_data(int fd,float dt)
{
    int i;

    struct lwlink_feature3D_s fp;
    struct lwlink_attitude_s att;
    struct lwlink_local_position_s lpos;
    struct vio_data_s vio_dat;
    struct eulur_s vio_att_e;

    float vr[3];
    
    int msg_length;
    
    lwlink_send_timer_20hz+=dt;
    
    if(lwlink_send_timer_20hz > 0.1f){
        lwlink_send_timer_20hz -= 0.1f;

        vio_get_data(&vio_dat);
        quater_to_eulur(&vio_att_e,&vio_dat.rotation);

        att.component_id = 0;
        att.roll  = vio_att_e.roll;
        att.pitch = vio_att_e.pitch;
        att.yaw   = vio_att_e.yaw;
        
        msg_length = lwlink_msg_pack(&lwlink_handler,MSG_TYPE_ATTITUDE,(uint8_t *)&att,sizeof(att));
        hal_dev_write(fd,lwlink_handler.txbuf,msg_length,0);

        for(i = 0;i < 4;i++){
            fp.image_id = 0;
            fp.feature_id = i;
            fp.pos_x = vio_dat.point_start[i].x;
            fp.pos_y = vio_dat.point_start[i].y;
            fp.pos_z = vio_dat.point_start[i].z;

            fp.vel_x = vio_dat.point_end[i].x - vio_dat.point_start[i].x;
            fp.vel_y = vio_dat.point_end[i].y - vio_dat.point_start[i].y;
            fp.vel_z = vio_dat.point_end[i].z - vio_dat.point_start[i].z;
            msg_length = lwlink_msg_pack(&lwlink_handler,MSG_TYPE_FEATURE3D,(uint8_t *)&fp,sizeof(fp));
            hal_dev_write(fd,lwlink_handler.txbuf,msg_length,0);
        }

        for(i = 4;i < 8;i++){
            fp.image_id = 0;
            fp.feature_id = i;

            vr[0] = vio_dat.point_start[i-4].x;
            vr[1] = vio_dat.point_start[i-4].y;
            vr[2] = vio_dat.point_start[i-4].z;

            quater_rotate(vr,vr,&vio_dat.rotation);

            fp.pos_x = vr[0];
            fp.pos_y = vr[1];
            fp.pos_z = vr[2];

            fp.vel_x = 0;
            fp.vel_y = 0;
            fp.vel_z = 0;
            msg_length = lwlink_msg_pack(&lwlink_handler,MSG_TYPE_FEATURE3D,(uint8_t *)&fp,sizeof(fp));
            hal_dev_write(fd,lwlink_handler.txbuf,msg_length,0);
        }

        lpos.vel_x = vio_dat.translation.x;
        lpos.vel_y = vio_dat.translation.y;
        lpos.vel_z = vio_dat.translation.z;

        msg_length = lwlink_msg_pack(&lwlink_handler,MSG_TYPE_LOCAL_POS,(uint8_t *)&lpos,sizeof(lpos));
        hal_dev_write(fd,lwlink_handler.txbuf,msg_length,0);
    }

    if(lwlink_send_timer_10hz > 0.1f){
        lwlink_send_timer_10hz -= 0.1f;
        /*if(flow_copy_image(gllink_image_buffer,&gllink_image_timestamp) > 0){
            msg_length = lwlink_msg_pack(&lwlink_handler,MSG_TYPE_RAW_IMAGE,gllink_image_buffer,FLOW_IMAGE_WIDTH * FLOW_IMAGE_WIDTH);
            hal_dev_write(fd,lwlink_handler.txbuf,msg_length,0);
        }*/
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

