#ifndef _APP_CALIBRATOR_H_
#define _APP_CALIBRATOR_H_


#include <stdint.h>

enum calibrating_status_e{
    CALIB_STATUS_UNCAL = 0,
    CALIB_STATUS_DOING,
    CALIB_STATUS_START,
    CALIB_STATUS_CALIBED,
};


struct calibrate_status_s{
    enum calibrating_status_e acc;
    enum calibrating_status_e gyro;
    enum calibrating_status_e mag;
    float timestamp;
};

void calibrator_init(void);
void calibrator_update(float dt);

void calibrator_get_status(struct calibrate_status_s *status);
void acc_calib_start(void);
void mag_calib_start(void);

uint8_t acc_calib_get_progress(void);
uint8_t mag_calib_get_progress(void);

#endif
