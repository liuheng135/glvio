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
    struct eulur_s              re = {0,0,0};
    
    struct gyro_data_s          gyro;
    struct acc_data_s           acc;
    struct flow_matched_point_s points_matched_from_flow[4];
    struct geo_matches_s        points_matched_for_geo[4];

    struct quaternion_s         rotation;
    struct point3f              translation;
    
    /*
    float  translation_length;
    float  accel_length;
    float  temp;
    */
   
    float  vector_compansate[3];
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
    flow_copy_matched_points(points_matched_from_flow);
    if(points_matched_from_flow[0].timestamp > flow_data_timestamp){
        flow_interval = points_matched_from_flow[0].timestamp - flow_data_timestamp;
        flow_data_timestamp = points_matched_from_flow[0].timestamp;

        gyro_integraton = 0.f;
        eulur_to_quater(&rotation,&ie);
        vio_att_buffer_rptr = vio_att_buffer_wptr - VIO_FLOW_DELAY_NUM;

        /* calulate rotation by integrating */
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
            vector_compansate[0] = points_matched_from_flow[i].start_x;
            vector_compansate[1] = points_matched_from_flow[i].start_y;
            vector_compansate[2] = 1.0f;

            /* calculate compansation */
            quater_rotate(vector_compansate,vector_compansate,&rotation);

            vio_data.point_start[i].x = points_matched_from_flow[i].start_x;
            vio_data.point_start[i].y = points_matched_from_flow[i].start_y;
            vio_data.point_start[i].z = 1.0f;

            /* compansate */
            vio_data.point_end[i].x = points_matched_from_flow[i].end_x - vector_compansate[0] + points_matched_from_flow[i].start_x;
            vio_data.point_end[i].y = points_matched_from_flow[i].end_y - vector_compansate[1] + points_matched_from_flow[i].start_y;
            vio_data.point_end[i].z = vector_compansate[2];
            points_matched_for_geo[i].l.x = vio_data.point_start[i].x;
            points_matched_for_geo[i].l.y = vio_data.point_start[i].y;
            points_matched_for_geo[i].l.z = vio_data.point_start[i].z;

            points_matched_for_geo[i].r.x = vio_data.point_end[i].x;
            points_matched_for_geo[i].r.y = vio_data.point_end[i].y;
            points_matched_for_geo[i].r.z = vio_data.point_end[i].z;
        }

        /* recovery pose */
        geo_recovery_translation(&translation,points_matched_for_geo[0],points_matched_for_geo[1]);

        vio_data.translation.x = translation.x;
        vio_data.translation.y = translation.y;
        vio_data.translation.z = translation.z; 

        if(geo_recovery_depth(&vio_data.point_recoveried[0],points_matched_for_geo[0],translation) > -1){
            geo_recovery_depth(&vio_data.point_recoveried[1],points_matched_for_geo[1],translation);
        }else{
            printf("can not recover depth\r\n");
        }
    }

    //vio_print_timer += dt;
    if(vio_print_timer > 0.05f){
        vio_print_timer = 0.f;
        quater_to_eulur(&re,&vio_data.rotation);

        vio_log_len = sprintf(vio_log_buffer," %f , %f, %f, %f \r\n",re.roll,re.pitch,
                        (points_matched_from_flow[0].end_x - points_matched_from_flow[0].start_x),
                        (points_matched_from_flow[0].end_y - points_matched_from_flow[0].start_y));
        if(vio_log_fd > 0){

            write(vio_log_fd,vio_log_buffer,vio_log_len);
        }
        printf("points: [%9.3f %9.3f %9.3f],[%9.3f %9.3f %9.3f]\r\n",
            vio_data.point_recoveried[0].x,vio_data.point_recoveried[0].y,vio_data.point_recoveried[0].z,
            vio_data.point_recoveried[1].x,vio_data.point_recoveried[1].y,vio_data.point_recoveried[1].z);
    }
    vio_att_buffer_wptr++;
}
