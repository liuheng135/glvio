#include "app_anoclink.h"
#include "app_flow.h"
#include "hal_device.h"
#include "app_debug.h"
#include "app_vio.h"
#include "mathlib.h"
#include "app_events.h"


struct anoc_data_handler_s anoc_handler;

int anoclink_channel_fd;
float  anoclink_send_timer;
int anoclink_if_connected;

uint8_t anoclink_global_buffer[1024];

void  anoclink_init(void)
{
    anoclink_channel_fd = hal_dev_open("net1",HAL_O_RDWR);
	if(anoclink_channel_fd > 0){
        msg_recoder_log(0,"anoclink start\r\n");
	}else{
        msg_recoder_log(0,"open anoclink channel failed\r\n");
	}
}

void anoclink_send_data(int fd,float dt)
{   
    struct anoc_status_s      astatus;
    struct anoc_sensor_data_s asensor;
	struct vio_data_s         vio_dat;
    struct eulur_s            vio_att_e;

    int msg_length;

    anoclink_send_timer+=dt;
    
    
    if(anoclink_send_timer > 0.1f){
        anoclink_send_timer -= 0.1f;

        vio_get_data(&vio_dat);
        quater_to_eulur(&vio_att_e,&vio_dat.rotation);
        astatus.roll  = ANOC_SWAP16((int16_t)(vio_att_e.roll * 5730));
        astatus.pitch = ANOC_SWAP16((int16_t)(vio_att_e.pitch * 5730));
        astatus.yaw   = ANOC_SWAP16((int16_t)(vio_att_e.yaw * 5730));
        msg_length = anoc_msg_pack(&anoc_handler,ANOC_MSG_TYPE_STATUS,(uint8_t*)&astatus,sizeof(astatus)); 
		hal_dev_write(fd,anoc_handler.txbuf,msg_length,0);
		asensor.acc[0] = ANOC_SWAP16((int16_t)(vio_dat.translation.x * 100.f));
		asensor.acc[1] = ANOC_SWAP16((int16_t)(vio_dat.translation.y * 100.f)); 
		asensor.acc[2] = ANOC_SWAP16((int16_t)(vio_dat.translation.z * 100.f)); 
        msg_length = anoc_msg_pack(&anoc_handler,ANOC_MSG_TYPE_SENSOR,(uint8_t*)&asensor,sizeof(asensor)); 
		hal_dev_write(fd,anoc_handler.txbuf,msg_length,0);
    }
}

void  anoclink_update(float dt)
{
    int len;
    unsigned char buf[128];
    
    if(anoclink_channel_fd >= 0){
        anoclink_send_data(anoclink_channel_fd,dt);
        
        len = hal_dev_read(anoclink_channel_fd,buf,100,0);
        if(len > 0){
            msg_recoder_log(0,"anoclink received %d \r\n",len);
        }
    }
}
float anoclink_get_heartbeat_rate(void)
{
    return 1.0f;
}

void  anoclink_info(float time,char *fmt,...)
{
    
}

