#include <string.h>
#include "hal_device.h"
#include "hal_sensor.h"
#include "rotation.h"
#include "app_imu.h"
#include "config.h"
#include "app_param.h"
#include "mathlib.h"
#include "app_debug.h"
#include "app_timer.h"

#define GYRO_VAL  0.1

enum Rotation imu_rorate;
struct acc_data_s imu_acc;
struct gyro_data_s imu_gyro;

float imu_acc_offset[3];
float imu_acc_scale[3];
float imu_gyro_offset[3];

bool imu_acc_calibing = false;
int imu_msg_id;
int imu_dev_fd;

void imu_set_gyro_cal(float offset[3])
{
	imu_gyro_offset[0] = offset[0];
	imu_gyro_offset[1] = offset[1];
	imu_gyro_offset[2] = offset[2];
}

void imu_set_acc_cal(float offset[3],float scale[3])
{
	imu_acc_offset[0] = offset[0];
	imu_acc_offset[1] = offset[1];
	imu_acc_offset[2] = offset[2];

	imu_acc_scale[0]  = scale[0];
	imu_acc_scale[1]  = scale[1];
	imu_acc_scale[2]  = scale[2];

	msg_recoder_log(imu_msg_id,"acc off:%f,%f,%f\r\n",imu_acc_offset[0],imu_acc_offset[1],imu_acc_offset[2]);
}


void imu_get_acc(struct acc_data_s *dat)
{
	memcpy(dat,&imu_acc,sizeof(imu_acc));
}

void imu_get_gyro(struct gyro_data_s *dat)
{
	memcpy(dat,&imu_gyro,sizeof(imu_gyro));
}

void imu_init(void)
{
    int i;
    char imu_device_list[][12] = {
        "mpu6050-0",
        "icm20689-0",
		"mpu6000",
		"mpu9250",
    };
		
    imu_msg_id = msg_recoder_create("IMU",0.1f);
	msg_recoder_log(imu_msg_id,"started\r\n");

    for(i = 0; i < sizeof(imu_device_list) / sizeof(imu_device_list[0]); i++){
        imu_dev_fd = hal_dev_open(imu_device_list[i],HAL_O_RDONLY);
        if(imu_dev_fd > -1){
            msg_recoder_log(imu_msg_id,"[imu][%d] find imu device: %s \r\n",imu_dev_fd,imu_device_list[i]);
            break;
        }
    }
    
    if(imu_dev_fd < 0){
        msg_recoder_log(imu_msg_id,"can not find any imu device!\r\n");
        return;
    }

	imu_rorate = IMU_ROTATION;

    for(i = 0;i < 3;i++){
        imu_acc_offset[i] = 0;
        imu_acc_scale[i] = 1.0f;
        imu_gyro_offset[i] = 0;
    }
}

void imu_update(float dt)
{
	uint8_t i;
	struct imu_report_s imu_report;
	float timestamp = timer_get_time();

	if(hal_dev_read(imu_dev_fd,&imu_report,sizeof(imu_report),0) > 0){
		rotate3(imu_report.acc.data,imu_rorate);
		rotate3(imu_report.gyro.data,imu_rorate);

		for(i = 0;i < 3;i++){
			imu_acc.raw[i]      = imu_report.acc.data[i];
		    imu_acc.calibed[i]  = imu_report.acc.data[i] * imu_acc_scale[i] - imu_acc_offset[i];
			imu_gyro.raw[i]     = imu_report.gyro.data[i];
			imu_gyro.calibed[i] = imu_report.gyro.data[i] - imu_gyro_offset[i];
		}

		imu_acc.timestamp  = timestamp;
		imu_gyro.timestamp = timestamp;
		imu_acc.temp       = imu_report.acc.temperature;
		imu_gyro.temp      = imu_report.gyro.temperature;
		
	}
	msg_recoder_debug_circlar(imu_msg_id,dt,
	    "acc:(%4.1f,%4.1f,%4.1f) gyro:(%3.3f,%3.3f,%3.3f) temp:%3.3f\r\n",
	    imu_acc.calibed[0],imu_acc.calibed[1],imu_acc.calibed[2],
	    imu_gyro.calibed[0],imu_gyro.calibed[1],imu_gyro.calibed[2],imu_acc.temp);
	
}

