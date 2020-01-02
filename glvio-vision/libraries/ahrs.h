#ifndef _AHRS_H_
#define _AHRS_H_

#include <stdbool.h>

#define PARAM_RP_KP_DEFUALT 			0.8f    					// 2 * proportional gain (Kp)
#define PARAM_RP_KI_DEFUALT 			0.002f    					// 2 * integral gain (Ki)

#define PARAM_Y_KP_DEFUALT				0.8f				        // 2 * proportional gain (Kp)
#define PARAM_Y_KI_DEFUALT				0.002f				        // 2 * integral gain (Ki)

typedef struct{
	bool inited;
	float gyro_err_int[3];
	float mag_err_int[3];
	float q[4];								// quaternion of sensor frame relative to auxiliary frame
	float dcm[3][3];
	float eulur[3];
	float gyro[3];
	float halferr[3];
	float param_rp_kp;
	float param_rp_ki;
	float param_y_kp;
	float param_y_ki;
	float acc_raw[3];
	float gyro_raw[3];
	float mag_raw[3];
	bool  acc_updated;
	bool  gyro_updated;
	bool  mag_updated;
}ahrs_estimator;


void ahrs_init(ahrs_estimator *est,float *acc,float *gyro,float *mag);
void ahrs_update(ahrs_estimator *est,float dt);
void ahrs_set_eular(ahrs_estimator *est,float roll,float pitch,float yaw);

void ahrs_apply_acc(ahrs_estimator *est,float acc[3]);
void ahrs_apply_gyro(ahrs_estimator *est,float gyro[3]);
void ahrs_apply_mag(ahrs_estimator *est,float mag[3]);



void ahrs_set_dcm(ahrs_estimator *est,float *d);
void ahrs_init_from_qua(ahrs_estimator *est,float *q);
void ahrs_init_from_dcm(ahrs_estimator *est,float d[3][3]);
void ahrs_init_from_eular(ahrs_estimator *est,float roll,float pitch,float yaw);
void ahrs_dcm2qua(float d[3][3],float *q);
void ahrs_qua2dcm(float *q,float d[3][3]);
void ahrs_qua2eulur(float *q,float *roll, float *pitch, float *yaw);
void ahrs_eulur2qua(float roll, float pitch, float yaw,float *q);
void ahrs_eulur2dcm(float roll,float pitch,float yaw,float d[3][3]);
void ahrs_calc_att_by_am(float *eulur,float *acc,float *mag);



#endif
