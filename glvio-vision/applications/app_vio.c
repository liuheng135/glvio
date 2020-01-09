#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "app_vio.h"
#include "app_flow.h"
#include "app_imu.h"
#include "config.h"
#include "app_debug.h"
#include "geometry.h"

#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

int  vio_log_fd = -1;
char vio_log_buffer[256];
int  vio_log_len;

#define VIO_ATT_BUFFER_LEN (50)
#define VIO_FLOW_DELAY_NUM (30)

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

    vio_log_fd = open("./vio-log",O_CREAT | O_RDWR | O_TRUNC);
    vio_data.translation_scale = 0.0f;
}

void vio_update(float dt)
{
    int i;
    struct eulur_s              ie = {0,0,0};
    struct quaternion_s         rotation;
    struct point3f              translation;
    struct gyro_data_s          gyro;
    struct acc_data_s           acc;
    struct flow_matched_point_s mp[4];
    struct geo_matches_s        gmp[4];
    
    /*
    float  translation_length;
    float  accel_length;
    float  temp;
    */
   
    float  v_compansate[3];

    float  flow_interval;
    float  gyro_integraton;

    imu_get_gyro(&gyro);
    imu_get_acc(&acc);

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
        vio_att_buffer_rptr = vio_att_buffer_wptr - VIO_FLOW_DELAY_NUM;

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

        for(i = 0; i < 4; i++){
            v_compansate[0] = mp[i].start_x;
            v_compansate[1] = mp[i].start_y;
            v_compansate[2] = 1.0f;

            quater_rotate(v_compansate,v_compansate,&rotation);

            vio_data.point_start[i].x = mp[i].start_x;
            vio_data.point_start[i].y = mp[i].start_y;
            vio_data.point_start[i].z = 1.0f;

            vio_data.point_end[i].x = mp[i].end_x - v_compansate[0] + mp[i].start_x;
            vio_data.point_end[i].y = mp[i].end_y - v_compansate[1] + mp[i].start_y;
            vio_data.point_end[i].z = v_compansate[2];
            gmp[i].l.x = vio_data.point_start[i].x;
            gmp[i].l.y = vio_data.point_start[i].y;
            gmp[i].l.z = vio_data.point_start[i].z;

            gmp[i].r.x = vio_data.point_end[i].x;
            gmp[i].r.y = vio_data.point_end[i].y;
            gmp[i].r.z = vio_data.point_end[i].z;
        }

        geo_recovery_translation(&translation,gmp[0],gmp[1]);

        /*
        vio_data.velocity.x = translation.x / flow_interval;
        vio_data.velocity.y = translation.y / flow_interval;
        vio_data.velocity.z = translation.z / flow_interval;

        vio_data.accle.x   = (vio_data.velocity.x - vio_data.velocity_last.x) / flow_interval; 
        vio_data.accle.y   = (vio_data.velocity.y - vio_data.velocity_last.y) / flow_interval;
        vio_data.accle.z   = (vio_data.velocity.z - vio_data.velocity_last.z) / flow_interval;

        translation_length  = sqrt(vio_data.accle.x * vio_data.accle.x + vio_data.accle.y * vio_data.accle.y * vio_data.accle.z * vio_data.accle.z);

        vio_att_buffer_rptr = (vio_att_buffer_wptr - VIO_FLOW_DELAY_NUM) % VIO_ATT_BUFFER_LEN;
        accel_length = sqrt(vio_imu_buffer[vio_att_buffer_rptr].acc[0] * vio_imu_buffer[vio_att_buffer_rptr].acc[0] + \
                            vio_imu_buffer[vio_att_buffer_rptr].acc[1] * vio_imu_buffer[vio_att_buffer_rptr].acc[1] + \
                            vio_imu_buffer[vio_att_buffer_rptr].acc[2] * vio_imu_buffer[vio_att_buffer_rptr].acc[2]);

        temp = accel_length / translation_length;
        vio_data.translation_scale += (temp - vio_data.translation_scale) * 0.1f;
        vio_data.translation.x = translation.x * vio_data.translation_scale * 1000.f;
        vio_data.translation.y = translation.y * vio_data.translation_scale * 1000.f;
        vio_data.translation.z = translation.z * vio_data.translation_scale * 1000.f;
        */

        vio_data.translation.x = translation.x;
        vio_data.translation.y = translation.y;
        vio_data.translation.z = translation.z; 
        printf("T = %9.3f %9.3f %9.3f scale = %10.6f\r\n",vio_data.translation.x,vio_data.translation.y,\
                                                            vio_data.translation.z,vio_data.translation_scale);
    }

    vio_print_timer += dt;
    if(vio_print_timer > 0.1f){
        vio_print_timer = 0.f;
        //printf("att = %f %f %f %f \r\n",vio_data.att.w,vio_data.att.x,vio_data.att.y,vio_data.att.z);
    }
    vio_att_buffer_wptr++;
}