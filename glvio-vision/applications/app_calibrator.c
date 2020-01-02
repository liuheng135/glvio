#include "string.h"
#include "config.h"
#include "app_param.h"
#include "mathlib.h"
#include "app_debug.h"
#include "app_imu.h"
#include "app_calibrator.h"
#include "app_events.h"

#define MAG_SAMPLES_NUM  3000
#define ACC_SAMPLES_NUM  800
#define GYRO_SAMPLES_NUM 400

struct calibrate_status_s cal_status;

int acc_samples_counter;
float acc_samples[3];
float param_acc_calib_status;
float accel_offset[3];
float accel_scale[3];

int   gyro_samples_counter;
float gyro_samples[3];
float gyro_offset[3];


float reset_timer;

void calibrator_get_status(struct calibrate_status_s *status)
{
    memcpy(status,&cal_status,sizeof(cal_status));
}


/******* accel calibration ********/
uint8_t acc_calib_get_progress()
{
	return (uint8_t)(acc_samples_counter * 100 / ACC_SAMPLES_NUM);
}

void acc_calib_start(void)
{
	cal_status.acc = CALIB_STATUS_START;
}

int acc_calib_apply(struct acc_data_s *acc)
{
	acc_samples[0] += acc->raw[0];
	acc_samples[1] += acc->raw[1];
	acc_samples[2] += acc->raw[2];

	if(acc_samples_counter >= ACC_SAMPLES_NUM){
		acc_samples[0] /= acc_samples_counter;
		acc_samples[1] /= acc_samples_counter;
		acc_samples[2] /= acc_samples_counter;

		accel_offset[0] = acc_samples[0];
		accel_offset[1] = acc_samples[1];
		accel_offset[2] = acc_samples[2] + CONSTANTS_ONE_G;
		
		accel_scale[0] = 1.0f;
		accel_scale[1] = 1.0f;
		accel_scale[2] = 1.0f;

		msg_recoder_log(0,"acc mean value:%3.3f,%3.3f,%3.3f\r\n",
		    acc_samples[0],acc_samples[1],acc_samples[2]);
		msg_recoder_log(0,"acc offset:%3.3f,%3.3f,%3.3f\r\n",
		    accel_offset[0],accel_offset[1],accel_offset[2]);
		
	    return 1;
	}else{
		acc_samples_counter++;
		return 0;
	}

}



void acc_calib_param_save(void)
{
    param_set(ACC_CALIB_STATUS_NAME,1.0f);
    param_set(ACC_OFFSET_X_NAME,accel_offset[0]);
    param_set(ACC_OFFSET_Y_NAME,accel_offset[1]);
    param_set(ACC_OFFSET_Z_NAME,accel_offset[2]);
    param_set(ACC_SCALE_X_NAME,accel_scale[0]);
    param_set(ACC_SCALE_Y_NAME,accel_scale[0]);
    param_set(ACC_SCALE_Z_NAME,accel_scale[0]);
    param_set(ACC_CALIB_STATUS_NAME,(float)cal_status.acc);
    param_save();
}
void acc_calib_update(void)
{
    struct acc_data_s acc;
	switch(cal_status.acc){
        case CALIB_STATUS_START:
		    acc_samples_counter = 0; 	
    	    memset(acc_samples,0x00,sizeof(acc_samples));
    	    msg_recoder_log(0,"acc calibration started\r\n");
    	    cal_status.acc = CALIB_STATUS_DOING;
    	    break;
    	case CALIB_STATUS_DOING:
            imu_get_acc(&acc);
            if(acc_calib_apply(&acc) == 1){
                cal_status.acc = CALIB_STATUS_CALIBED;
                reset_timer = 1.0f;
                acc_calib_param_save();
            }
            break;
        default:
            break;
   } 
}


/******* gyro calibration ********/

int gyro_calib_apply(struct gyro_data_s *gyro)
{
    float gyro_length = length3(gyro->raw[0],gyro->raw[1],gyro->raw[2]);
    if(gyro_length > 0.1f){
        gyro_samples_counter = 0;
        memset(gyro_samples,0x00,sizeof(gyro_samples));
        return 0;
    }
	gyro_samples[0] += gyro->raw[0];
	gyro_samples[1] += gyro->raw[1];
	gyro_samples[2] += gyro->raw[2];

	if(gyro_samples_counter >= ACC_SAMPLES_NUM){    
		gyro_samples[0] /= ACC_SAMPLES_NUM;
		gyro_samples[1] /= ACC_SAMPLES_NUM;
		gyro_samples[2] /= ACC_SAMPLES_NUM;

		gyro_offset[0] = gyro_samples[0];
		gyro_offset[1] = gyro_samples[1];
		gyro_offset[2] = gyro_samples[2];

        float gyro_off_length = length3(gyro_offset[0],gyro_offset[1],gyro_offset[2]);

        if(gyro_off_length > 0.5f){
            msg_recoder_log(0,"[WARNING]gyro offset is to large!\r\n");
            msg_recoder_log(0,"offset:%5.3f,%5.3f,%5.3f , length: %f\r\n",
		    gyro_offset[0],gyro_offset[1],gyro_offset[2],gyro_off_length);
        }else{
            msg_recoder_log(0,"gyro offset:%3.3f,%3.3f,%3.3f\r\n",
		    gyro_offset[0],gyro_offset[1],gyro_offset[2]);
        }
	
	    return 1;
	}else{
		gyro_samples_counter++;
		return 0;
	}
}

void gyro_calib_update(void)
{
    struct gyro_data_s gyro;
	switch(cal_status.gyro){
        case CALIB_STATUS_START:
		    gyro_samples_counter = 0; 	
    	    memset(gyro_samples,0x00,sizeof(gyro_samples));
    	    msg_recoder_log(0,"gyro calibration started\r\n");
    	    cal_status.gyro = CALIB_STATUS_DOING;
    	    break;
    	case CALIB_STATUS_DOING:
            imu_get_gyro(&gyro);
            if(gyro_calib_apply(&gyro) == 1){
                cal_status.gyro = CALIB_STATUS_CALIBED;
                imu_set_gyro_cal(gyro_offset);
            }
            break;
        default:
            break;
   } 
}


void calibrator_param_init(void)
{
    param_set(ACC_OFFSET_X_NAME,ACC_OFFSET_X_DEF);
	param_set(ACC_OFFSET_Y_NAME,ACC_OFFSET_Y_DEF);
	param_set(ACC_OFFSET_Z_NAME,ACC_OFFSET_Z_DEF);

	param_set(ACC_SCALE_X_NAME,ACC_SCALE_X_DEF);
	param_set(ACC_SCALE_Y_NAME,ACC_SCALE_Y_DEF);
	param_set(ACC_SCALE_Z_NAME,ACC_SCALE_Z_DEF);
	param_set(ACC_CALIB_STATUS_NAME,CALIB_STATUS_UNCAL);
}

void calibrator_param_update(void)
{
    float status;
    param_get(ACC_OFFSET_X_NAME,&accel_offset[0]);
	param_get(ACC_OFFSET_Y_NAME,&accel_offset[1]);
	param_get(ACC_OFFSET_Z_NAME,&accel_offset[2]);

	param_get(ACC_SCALE_X_NAME,&accel_scale[0]);
	param_get(ACC_SCALE_Y_NAME,&accel_scale[1]);
	param_get(ACC_SCALE_Z_NAME,&accel_scale[2]);
	param_get(ACC_CALIB_STATUS_NAME,&status);

    cal_status.acc = (unsigned char)status;

	imu_set_acc_cal(accel_offset,accel_scale);
}

void calibrator_init(void)
{
    calibrator_param_init();
    param_notice_add(calibrator_param_update);
	cal_status.acc = CALIB_STATUS_UNCAL;
	cal_status.gyro = CALIB_STATUS_START;
	memset(gyro_samples,0x00,sizeof(gyro_samples));
}

void calibrator_update(float dt)
{
	acc_calib_update();	
	gyro_calib_update();

	if(reset_timer > 0){
        reset_timer-=dt;
        if(reset_timer <= 0){
            reset_timer = 0;
            msg_recoder_log(0,"reset cal counter!\r\n");
            if(acc_samples_counter >= ACC_SAMPLES_NUM){
                acc_samples_counter = 0;
            }
        }
	}
}


