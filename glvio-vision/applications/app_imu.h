#ifndef _APP_IMU_H_
#define _APP_IMU_H_

#include <stdint.h>
#include <stdbool.h>

struct acc_data_s{
    float calibed[3];
    float raw[3];
    float temp;
    float timestamp;
};

struct gyro_data_s{
    float calibed[3];
    float raw[3];
    float temp;
    float timestamp;
};


void imu_init(void);
void imu_update(float dt);

void imu_set_gyro_cal(float offset[3]);
void imu_set_acc_cal(float offset[3],float scale[3]);
void imu_get_acc(struct acc_data_s *dat);
void imu_get_gyro(struct gyro_data_s *dat);
uint8_t imu_get_status(void);

#endif
