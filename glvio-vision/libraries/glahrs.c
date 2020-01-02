#include "glahrs.h"
#include "mathlib.h"

static float glahrs_wrap_pi(float angle)
{
	if (angle > (3.0f * M_PI_F) || angle < (-3.0f * M_PI_F)) {
        // for large numbers use modulus
        angle = fmod(angle,(2.0f * M_PI_F));
    }
    if (angle > M_PI_F) { angle -= (2.0f * M_PI_F); }
    if (angle < -M_PI_F) { angle += (2.0f * M_PI_F); }
    return angle;
}

static void glahrs_calc_att_by_am(struct eulur_s *eulur,float *acc,float *mag)
{
    float xh,yh;
    if(fabs(acc[2]) > 0.0f){
		eulur->roll = -atan2f(acc[1],-acc[2]);
		eulur->pitch =  asinf(-acc[0]/(-9.8f));
	}
	xh = mag[0]*cosf(eulur->pitch) + mag[1]*sinf(eulur->roll)*sinf(eulur->pitch) + mag[2]*cosf(eulur->roll)*sinf(eulur->pitch);
	yh = mag[2]*sinf(eulur->roll) - mag[1]*cosf(eulur->roll);

	if((xh == 0.0f) && (yh == 0.0f)){		
		eulur->yaw = 0.0f;
	}else{
		eulur->yaw = atan2f(yh,xh);
	}
	eulur->yaw = glahrs_wrap_pi(eulur->yaw);
}

void glahrs_init(struct glahrs_estimator_s *est,float *acc,float *gyro,float *mag)
{
    struct eulur_s eulur_init;
    glahrs_calc_att_by_am(&eulur_init,acc,mag);

    eulur_to_quater(&est->qatt,&eulur_init);
    est->inited = true;
}

void glahrs_update(struct glahrs_estimator_s *est,float dt)
{
    
}
void glahrs_set_eular(struct glahrs_estimator_s *est,float roll,float pitch,float yaw);

void glahrs_apply_acc(struct glahrs_estimator_s *est,float acc[3]);
void glahrs_apply_gyro(struct glahrs_estimator_s *est,float gyro[3]);
void glahrs_apply_mag(struct glahrs_estimator_s *est,float mag[3]);

