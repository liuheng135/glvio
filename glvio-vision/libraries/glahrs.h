#ifndef _GLAHRS_H_
#define _GLAHRS_H_

#include <stdbool.h>
#include "quaternion.h"

struct glahrs_estimator_s{
	bool inited;
	float wb[3];
	float wb_bias[3];
	float gravity[3];
	float geomagnetic[3];
	
	struct quaternion_s qatt;
};

void glahrs_init(struct glahrs_estimator_s *est,float *acc,float *gyro,float *mag);
void glahrs_update(struct glahrs_estimator_s *est,float dt);
void glahrs_set_eular(struct glahrs_estimator_s *est,float roll,float pitch,float yaw);

void glahrs_apply_acc(struct glahrs_estimator_s *est,float acc[3]);
void glahrs_apply_gyro(struct glahrs_estimator_s *est,float gyro[3]);
void glahrs_apply_mag(struct glahrs_estimator_s *est,float mag[3]);

#endif
