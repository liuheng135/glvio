#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "app_vio.h"
#include "app_flow.h"
#include "app_imu.h"
#include "config.h"
#include "app_debug.h"


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
    
    struct gyro_data_s          gyro;
    struct acc_data_s           acc;
    struct geo_feature_s        features_from_flow[4];
    struct quaternion_s         rotation;
   
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
    flow_copy_matched_points(features_from_flow);
    if(features_from_flow[0].observer_next.timestamp > flow_data_timestamp){
        flow_interval = features_from_flow[0].observer_next.timestamp - flow_data_timestamp;
        flow_data_timestamp = features_from_flow[0].observer_next.timestamp;

        gyro_integraton = 0.f;
        eulur_to_quater(&rotation,&ie);
        printf("r = %f %f %f %f\r\n",rotation.w,rotation.x,rotation.y,rotation.z);
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
            vector_compansate[0] = features_from_flow[i].observer_prev.point_pos_in_camera.x;
            vector_compansate[1] = features_from_flow[i].observer_prev.point_pos_in_camera.y;
            vector_compansate[2] = 1.0f;

            /* calculate compansation */
            quater_rotate(vector_compansate,vector_compansate,&rotation);
            
            vio_data.features[i] = features_from_flow[i];

            vio_data.features[i].observer_next.point_pos_in_camera.x -= vector_compansate[0];
            vio_data.features[i].observer_next.point_pos_in_camera.y -= vector_compansate[1];
            vio_data.features[i].observer_next.point_pos_in_camera.z -= vector_compansate[2];
        }

        /* recovery pose and depth*/
        geo_recovery_translation_2D2D(&vio_data.features[0],&vio_data.features[1]);
        geo_recovery_depth(&vio_data.features[0]);
        geo_recovery_depth(&vio_data.features[1]);

        vio_data.translation.x = vio_data.features[0].observer_next.camera_pos_in_world.x;
        vio_data.translation.y = vio_data.features[0].observer_next.camera_pos_in_world.y;
        vio_data.translation.z = vio_data.features[0].observer_next.camera_pos_in_world.z;
    }

    //vio_print_timer += dt;
    if(vio_print_timer > 0.05f){
        vio_print_timer = 0.f;
    }
    vio_att_buffer_wptr++;
}
