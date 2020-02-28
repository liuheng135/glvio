#ifndef _APP_VIO_H_
#define _APP_VIO_H_

#include "rotation.h"
#include "quaternion.h"
#include "image_lib.h"

#define CAMERA_FOCUS  0.003f

struct vio_imu_s {
    float gyro[3];
    float acc[3];
    float dt;
};


struct vio_data_s {
    struct quaternion_s att;
    struct quaternion_s rotation;
    struct point3f translation;
    struct point3f velocity;
    struct point3f accle;
    struct point3f velocity_last;
    float  translation_scale;

    struct point3f point_start[4];
    struct point3f point_end[4];
    struct point3f point_recoveried[4];
};

void vio_get_data(struct vio_data_s *dat);
void vio_init(void);
void vio_update(float dt);

#endif