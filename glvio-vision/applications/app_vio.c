#include <string.h>
#include <stdbool.h>
#include "app_vio.h"
#include "app_flow.h"
#include "app_imu.h"
#include "config.h"
#include "app_debug.h"


#define VIO_ATT_BUFFER_LEN (50)

struct vio_data_s vio_data;

int   vio_msg_id = -1;
bool  voi_att_inited = false;
int   vio_att_buffer_wptr,vio_att_buffer_rptr;

float vio_print_timer;
float flow_data_timestamp;

struct vio_imu_s vio_imu_buffer[VIO_ATT_BUFFER_LEN + 2];

void vio_get_data(struct vio_data_s *dat)
{
    memcpy(dat,&vio_data,sizeof(struct vio_data_s));
}

void vio_init(void)
{
    struct eulur_s init_att;
    vio_msg_id = msg_recoder_create("VIO",20.f);

    vio_att_buffer_wptr = 0;
    vio_print_timer = 0.f;
    flow_data_timestamp = 0.f;
    
    init_att.roll = 0;
    init_att.pitch = 0;
    init_att.yaw = 0;
    eulur_to_quater(&vio_data.att,&init_att);
}

void vio_update(float dt)
{
    int i;
    struct eulur_s ie = {0,0,0};
    struct quaternion_s rotation;
    struct eulur_s      rotation_eulur;
    struct gyro_data_s gyro;
    struct acc_data_s  acc;
    struct flow_matched_point_s mp[4];
    float flow_interval;
    float gyro_integraton;

    imu_get_gyro(&gyro);
    imu_get_acc(&acc);
    rotate3(gyro.calibed,VIO_IMU_ROTATE);
    rotate3(acc.calibed,VIO_IMU_ROTATE);

    for(i = 0;i < 3; i++){
        vio_imu_buffer[vio_att_buffer_wptr % VIO_ATT_BUFFER_LEN].acc[i]  = acc.calibed[i];
        vio_imu_buffer[vio_att_buffer_wptr % VIO_ATT_BUFFER_LEN].gyro[i] = gyro.calibed[i];
    }
    vio_imu_buffer[vio_att_buffer_wptr % VIO_ATT_BUFFER_LEN].dt = dt;
    vio_att_buffer_wptr++;

    if(vio_att_buffer_wptr - VIO_ATT_BUFFER_LEN < 0){
        return;
    }

    flow_copy_matched_points(mp);
    if(mp[0].timestamp > flow_data_timestamp){
        flow_interval = mp[0].timestamp - flow_data_timestamp;
        flow_data_timestamp = mp[0].timestamp;

        gyro_integraton = 0.f;
        eulur_to_quater(&rotation,&ie);
        vio_att_buffer_rptr = vio_att_buffer_wptr - VIO_ATT_BUFFER_LEN;

        while(gyro_integraton < flow_interval){
            gyro_integraton += vio_imu_buffer[vio_att_buffer_rptr % VIO_ATT_BUFFER_LEN].dt;
            quater_update_by_gyroscope(&rotation,vio_imu_buffer[vio_att_buffer_rptr % VIO_ATT_BUFFER_LEN].gyro,vio_imu_buffer[vio_att_buffer_rptr % VIO_ATT_BUFFER_LEN].dt);
            vio_att_buffer_rptr++;
            if(vio_att_buffer_rptr > vio_att_buffer_wptr){
                printf("wrong gyro time integraton\r\n");
                break;
            }
        }
        vio_data.rotation = rotation;
        quater_to_eulur(&rotation_eulur,&rotation);
        vio_data.mp1_vel[0] = mp[0].end_x - mp[0].start_x;
        vio_data.mp1_vel[1] = mp[0].end_y - mp[0].start_y;
        vio_data.mp2_vel[0] = vio_data.mp1_vel[0] * - rotation_eulur.roll;
        vio_data.mp2_vel[1] = vio_data.mp1_vel[1] * - rotation_eulur.pitch;
    }

    vio_print_timer += dt;
    if(vio_print_timer > 0.1f){
        vio_print_timer = 0.f;
        //printf("att = %f %f %f %f \r\n",vio_data.att.w,vio_data.att.x,vio_data.att.y,vio_data.att.z);
    }
    vio_att_buffer_wptr++;
}