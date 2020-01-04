#ifndef _APP_VIO_H_
#define _APP_VIO_H_

#include "rotation.h"
#include "quaternion.h"

#define VIO_IMU_ROTATE ROTATION_NONE


struct vio_imu_s {
    float gyro[3];
    float acc[3];
    float dt;
};


struct vio_data_s {
    struct quaternion_s att;
    struct quaternion_s rotation;
    float  translation[3];
    float  mp1_vel[2];
    float  mp2_vel[2];
};

void vio_get_data(struct vio_data_s *dat);
void vio_init(void);
void vio_update(float dt);

#endif