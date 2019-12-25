#include "ahrs.h"
#include "math.h"
#include "stdio.h"


static float ahrs_radians(float deg)
{
	return deg * M_PI_F / 180.f;
}

static float ahrs_degrees(float rad)
{
	return rad * 180.f / M_PI_F;
}

static float ahrs_invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

static float ahrs_wrap_pi(float angle)
{
	if (angle > (3.0f * M_PI_F) || angle < (-3.0f * M_PI_F)) {
        // for large numbers use modulus
        angle = fmod(angle,(2.0f * M_PI_F));
    }
    if (angle > M_PI_F) { angle -= (2.0f * M_PI_F); }
    if (angle < -M_PI_F) { angle += (2.0f * M_PI_F); }
    return angle;
}

void ahrs_calc_att_by_am(float *eulur,float *acc,float *mag)
{
    float xh,yh;
    if(fabs(acc[2]) > 0.0f){
		eulur[0] = -atan2f(acc[1],-acc[2]);
		eulur[1] =  asinf(-acc[0]/(-9.8f));
	}
	xh = mag[0]*cosf(eulur[1]) + mag[1]*sinf(eulur[0])*sinf(eulur[1]) + mag[2]*cosf(eulur[0])*sinf(eulur[1]);
	yh = mag[2]*sinf(eulur[0]) - mag[1]*cosf(eulur[0]);

	if((xh == 0.0f) && (yh == 0.0f)){		
		eulur[2] = 0.0f;
	}else{
		eulur[2] = atan2f(yh,xh);
	}
	eulur[2] = ahrs_wrap_pi(eulur[2]);
}

void ahrs_init(ahrs_estimator *est,float *acc,float *gyro,float *mag)
{ 
	int i;
	float eulur[3];

	est->param_rp_kp = PARAM_RP_KP_DEFUALT;
	est->param_rp_ki = PARAM_RP_KI_DEFUALT;
	est->param_y_kp  = PARAM_Y_KP_DEFUALT;
	est->param_y_ki  = PARAM_Y_KI_DEFUALT;

	for(i = 0;i < 3;i++){
		est->gyro_err_int[i] = 0.0f;
		est->mag_err_int[i] = 0.0f;
	}

	printf("[AHRS]acc initializing value: %5.2f %5.2f %5.2f\r\n",acc[0],acc[1],acc[2]);
	printf("[AHRS]mag initializing value: %5.2f %5.2f %5.2f\r\n",mag[0],mag[1],mag[2]);

	ahrs_calc_att_by_am(eulur,acc,mag);

	printf("[AHRS]Init Attitude from Acc and Mag:%+3.3f,%+3.3f,%+3.3f\r\n",
	    ahrs_degrees(eulur[0]),ahrs_degrees(eulur[1]),ahrs_degrees(eulur[2]));
	
	ahrs_init_from_eular(est,eulur[0],eulur[1],eulur[2]);
	est->inited = true;
}

void ahrs_apply_acc(ahrs_estimator *est,float acc[3])
{
	est->acc_raw[0] = acc[0];
	est->acc_raw[1] = acc[1];
	est->acc_raw[2] = acc[2];

	est->acc_updated = true;
	
}
void ahrs_apply_gyro(ahrs_estimator *est,float gyro[3])
{
	est->gyro_raw[0] = gyro[0];
	est->gyro_raw[1] = gyro[1];
	est->gyro_raw[2] = gyro[2];

	est->gyro_updated = true;
}
void ahrs_apply_mag(ahrs_estimator *est,float mag[3])
{
	est->mag_raw[0] = mag[0];
	est->mag_raw[1] = mag[1];
	est->mag_raw[2] = mag[2];

	est->mag_updated = true;
}


void ahrs_update(ahrs_estimator *est,float dt)
{
	int i;
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfXx, halfXy, halfXz;
	float halfex, halfey, halfez;
	float halfmx, halfmy, halfmz;
	float hx, hy, hz, bx, bz;
	float qa, qb, qc,qd;
	float acc[3],gyro[3],mag[3];

	halfmx = 0.0f;
	halfmy = 0.0f;
	halfmz = 0.0f;
	
	float gx_g=0.0f;
	float gy_g=0.0f;
	float gz_g=0.0f;
	float gx_m=0.0f;
	float gy_m=0.0f;
	float gz_m=0.0f;

	for(i = 0; i < 3;i++){
		acc[i]  = est->acc_raw[i];
		gyro[i] = est->gyro_raw[i];
		mag[i]  = est->mag_raw[i];
	}
	

	if(est->acc_updated == true && est->gyro_updated == true){
	    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	    if(!((acc[0] == 0.0f) && (acc[1] == 0.0f) && (acc[2] == 0.0f)))
	    {
		    // Normalise accelerometer measurement
		    recipNorm = ahrs_invSqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
		    acc[0] *= recipNorm;
		    acc[1] *= recipNorm;
		    acc[2] *= recipNorm;

		    // Estimated direction of gravity and vector perpendicular to magnetic flux
		    halfvx = est->q[1] * est->q[3] - est->q[0] * est->q[2];
		    halfvy = est->q[0] * est->q[1] + est->q[2] * est->q[3];
		    halfvz = 0.5f - est->q[1]*est->q[1] - est->q[2]*est->q[2];

		    // Error is sum of cross product between estimated and measured direction of gravity
		    halfex = -(acc[1] * halfvz - acc[2] * halfvy);
		    halfey = -(acc[2] * halfvx - acc[0] * halfvz);
		    halfez = -(acc[0] * halfvy - acc[1] * halfvx);

		    est->halferr[0] = halfex;
		    est->halferr[1] = halfey;
		    est->halferr[2] = halfez;

		    // Compute and apply integral feedback if enabled
		    if(est->param_rp_ki> 0.0f)
		    {
				est->gyro_err_int[0] += est->param_rp_ki * halfex * dt;  // integral error scaled by Ki
				est->gyro_err_int[1] += est->param_rp_ki * halfey * dt;
				est->gyro_err_int[2] += est->param_rp_ki * halfez * dt;
				gx_g = gyro[0];
				gy_g = gyro[1];
				gz_g = gyro[2];
				gx_g += est->gyro_err_int[0];  // apply integral feedback
				gy_g += est->gyro_err_int[1];
				gz_g += est->gyro_err_int[2];
		    }
		    else
		    {
				est->gyro_err_int[0] = 0.0f; // prevent integral windup
				est->gyro_err_int[1] = 0.0f;
				est->gyro_err_int[2] = 0.0f;
		    }

		    // Apply proportional feedback
		    gx_g += est->param_rp_kp * halfex;
		    gy_g += est->param_rp_kp * halfey;
		    gz_g += est->param_rp_kp * halfez;
	    }
		est->acc_updated  = false;
		est->gyro_updated = false;
	}else{
		return;
	}

	if(est->mag_updated == true){
		if(!((mag[0] == 0.0f) && (mag[1] == 0.0f) && (mag[2] == 0.0f)))
	    {
		    // Normalise accelerometer measurement
		    recipNorm = ahrs_invSqrt(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
		    mag[0] *= recipNorm;
		    mag[1] *= recipNorm;
		    mag[2] *= recipNorm;

			// compute reference direction of flux
	        hx = 2.0f * (mag[0] * (0.5f - est->q[2]*est->q[2] - est->q[3]*est->q[3]) + mag[1] * (est->q[1]*est->q[2] - est->q[0]*est->q[3]) + mag[2] * (est->q[1]*est->q[3] + est->q[0]*est->q[2]));

	        hy = 2.0f * (mag[0] * (est->q[1]*est->q[2] + est->q[0]*est->q[3]) + mag[1] * (0.5f - est->q[1]*est->q[1] - est->q[3]*est->q[3]) + mag[2] * (est->q[2]*est->q[3] - est->q[0]*est->q[1]));

	        hz = 2.0f * (mag[0] * (est->q[1]*est->q[3] - est->q[0]*est->q[2]) + mag[1] * (est->q[2]*est->q[3] + est->q[0]*est->q[1]) + mag[2] * (0.5f - est->q[1]*est->q[1] - est->q[2]*est->q[2]));

	        bx = sqrt((hx * hx) + (hy * hy));

	        bz = hz;

	        // estimated direction of flux (w)
	        halfXx = 2.0f * (bx * (0.5f - est->q[2]*est->q[2] - est->q[3]*est->q[3]) + bz * (est->q[1]*est->q[3] - est->q[0]*est->q[2]));

	        halfXy = 2.0f * (bx * (est->q[1]*est->q[2] - est->q[0]*est->q[3]) + bz * (est->q[0]*est->q[1] + est->q[2]*est->q[3]));

	        halfXz = 2.0f * (bx * (est->q[0]*est->q[2] + est->q[1]*est->q[3]) + bz * (0.5f - est->q[1]*est->q[1] - est->q[2]*est->q[2]));

		    // error
			halfmx = (mag[1] * halfXz - mag[2] * halfXy);
			halfmy = (mag[2] * halfXx - mag[0] * halfXz);
			halfmz = (mag[0] * halfXy - mag[1] * halfXx);
		  

		    // Compute and apply integral feedback if enabled
		    if(est->param_y_ki> 0.0f)
		    {
				est->mag_err_int[0] += est->param_y_ki * halfmx * dt;  // integral error scaled by Ki
				est->mag_err_int[1] += est->param_y_ki * halfmy * dt;
				est->mag_err_int[2] += est->param_y_ki * halfmz * dt;
				gx_m += est->mag_err_int[0];  // apply integral feedback
				gy_m += est->mag_err_int[1];
				gz_m += est->mag_err_int[2];
			}
		    else
		    {
				est->mag_err_int[0] = 0.0f; // prevent integral windup
				est->mag_err_int[1] = 0.0f;
				est->mag_err_int[2] = 0.0f;
		    }
			
		    // Apply proportional feedback	
		    gx_m += est->param_y_kp * halfmx;
		    gy_m += est->param_y_kp * halfmy;
		    gz_m += est->param_y_kp * halfmz;
	    }
		est->mag_updated = false;
	}else{
		gx_m = 0.0f; 
 		gy_m = 0.0f;
		gz_m = 0.0f;	
	}

	est->gyro[0] = gx_g;
	est->gyro[1] = gy_g;
	est->gyro[2] = gz_g + gz_m;

	gyro[0] = gx_g;
	gyro[1] = gy_g;
	gyro[2] = gz_g + gz_m;
	
    // Integrate rate of change of quaternion
    gyro[0] *= (0.5f * dt);   // pre-multiply common factors
    gyro[1] *= (0.5f * dt);
    gyro[2] *= (0.5f * dt);
    qa = est->q[0];
    qb = est->q[1];
    qc = est->q[2];
	qd = est->q[3];
    est->q[0] += (-qb * gyro[0] - qc * gyro[1] - qd * gyro[2]);
    est->q[1] += ( qa * gyro[0] + qc * gyro[2] - qd * gyro[1]);
    est->q[2] += ( qa * gyro[1] - qb * gyro[2] + qd * gyro[0]);
    est->q[3] += ( qa * gyro[2] + qb * gyro[1] - qc * gyro[0]);
  
    // Normalise quaternion
    recipNorm = ahrs_invSqrt(est->q[0] * est->q[0] + est->q[1] * est->q[1] + est->q[2] * est->q[2] + est->q[3] * est->q[3]);
    est->q[0] *= recipNorm;
    est->q[1] *= recipNorm;
    est->q[2] *= recipNorm;
    est->q[3] *= recipNorm;

	ahrs_qua2eulur(est->q,&(est->eulur[0]),&(est->eulur[1]),&(est->eulur[2]));
	ahrs_qua2dcm(est->q,est->dcm);
}

void ahrs_qua2eulur(float *q,float *roll, float *pitch, float *yaw)
{
	*roll  = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
	*pitch = asinf(2.0f * (q[0] * q[2] - q[3] * q[1]));
	*yaw   = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
}

void ahrs_qua2dcm(float *q,float d[3][3])
{
	float aSq = q[0] * q[0];
	float bSq = q[1] * q[1];
	float cSq = q[2] * q[2];
	float dSq = q[3] * q[3];
	d[0][0] = aSq + bSq - cSq - dSq;
	d[0][1] = 2.0f * (q[1] * q[2] - q[0] * q[3]);
	d[0][2] = 2.0f * (q[0] * q[2] + q[1] * q[3]);
	d[1][0] = 2.0f * (q[1] * q[2] + q[0] * q[3]);
	d[1][1] = aSq - bSq + cSq - dSq;
	d[1][2] = 2.0f * (q[2] * q[3] - q[0] * q[1]);
	d[2][0] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
	d[2][1] = 2.0f * (q[0] * q[1] + q[2] * q[3]);
	d[2][2] = aSq - bSq - cSq + dSq;
}


void ahrs_eulur2qua(float roll, float pitch, float yaw,float *q)
{
	double cosPhi_2 = cos((double)(roll) / 2.0);
	double sinPhi_2 = sin((double)(roll) / 2.0);
	double cosTheta_2 = cos((double)(pitch) / 2.0);
	double sinTheta_2 = sin((double)(pitch) / 2.0);
	double cosPsi_2 = cos((double)(yaw) / 2.0);
	double sinPsi_2 = sin((double)(yaw) / 2.0);

	q[0] = (float)(cosPhi_2 * cosTheta_2 * cosPsi_2 + sinPhi_2 * sinTheta_2 * sinPsi_2);
	q[1] = (float)(sinPhi_2 * cosTheta_2 * cosPsi_2 - cosPhi_2 * sinTheta_2 * sinPsi_2);
	q[2] = (float)(cosPhi_2 * sinTheta_2 * cosPsi_2 + sinPhi_2 * cosTheta_2 * sinPsi_2);
	q[3] = (float)(cosPhi_2 * cosTheta_2 * sinPsi_2 - sinPhi_2 * sinTheta_2 * cosPsi_2);
}

void ahrs_eulur2dcm(float roll,float pitch,float yaw,float d[3][3])
{
	float cp = cosf(pitch);
	float sp = sinf(pitch);
	float sr = sinf(roll);
	float cr = cosf(roll);
	float sy = sinf(yaw);
	float cy = cosf(yaw);

	d[0][0] = cp * cy;
	d[0][1] = (sr * sp * cy) - (cr * sy);
	d[0][2] = (cr * sp * cy) + (sr * sy);
	d[1][0] = cp * sy;
	d[1][1] = (sr * sp * sy) + (cr * cy);
	d[1][2] = (cr * sp * sy) - (sr * cy);
	d[2][0] = -sp;
	d[2][1] = sr * cp;
	d[2][2] = cr * cp;
}


/* Reference: Shoemake, Quaternions, http://www.cs.ucr.edu/~vbz/resources/quatut.pdf */
void ahrs_dcm2qua(float d[3][3],float *q)
{

	float tr = d[0][0] + d[1][1] + d[2][2];
	if (tr > 0.0f) {
		float s = sqrtf(tr + 1.0f);
		q[0] = s * 0.5f;
		s = 0.5f / s;
		q[1] = (d[2][1] - d[1][2]) * s;
		q[2] = (d[0][2] - d[2][0]) * s;
		q[3] = (d[1][0] - d[0][1]) * s;
	} else {
		/* Find maximum diagonal element in dcm
		* store index in dcm_i */
		int dcm_i = 0,i = 0;
		for (i = 1; i < 3; i++) {
			if (d[i][i] > d[dcm_i][dcm_i]) {
				dcm_i = i;
			}
		}
		int dcm_j = (dcm_i + 1) % 3;
		int dcm_k = (dcm_i + 2) % 3;
		float s = sqrtf((d[dcm_i][dcm_i] - d[dcm_j][dcm_j] -
		d[dcm_k][dcm_k]) + 1.0f);
		q[dcm_i + 1] = s * 0.5f;
		s = 0.5f / s;
		q[dcm_j + 1] = (d[dcm_i][dcm_j] + d[dcm_j][dcm_i]) * s;
		q[dcm_k + 1] = (d[dcm_k][dcm_i] + d[dcm_i][dcm_k]) * s;
		q[0] = (d[dcm_k][dcm_j] - d[dcm_j][dcm_k]) * s;
	}
}

void ahrs_init_from_eular(ahrs_estimator *est,float roll,float pitch,float yaw)
{
	est->eulur[0] = roll;
	est->eulur[1] = pitch;
	est->eulur[2] = yaw;
	
	ahrs_eulur2qua(roll,pitch,yaw,est->q);
	ahrs_qua2dcm(est->q,est->dcm);

}
void ahrs_init_from_dcm(ahrs_estimator *est,float d[3][3])
{
	int i,j;
	for(i = 0;i < 3;i++)
		for(j = 0;j < 3;j++)
			est->dcm[i][j] = d[i][j];

	ahrs_dcm2qua(d,est->q);
	ahrs_qua2eulur(est->q,&(est->eulur[0]),&(est->eulur[1]),&(est->eulur[2]));
}

void ahrs_init_from_qua(ahrs_estimator *est,float *q)
{
	int i;
	for(i = 0;i < 4;i++)
		est->q[i] = q[i];

	ahrs_qua2eulur(q,&(est->eulur[0]),&(est->eulur[1]),&(est->eulur[2]));
	ahrs_qua2dcm(q,est->dcm);
}



