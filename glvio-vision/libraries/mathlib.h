#ifndef _MATHLIB_H_
#define _MATHLIB_H_

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include <stdlib.h>
#include <float.h>
#include <sys/time.h>
#include <time.h> 

#ifndef M_PI
#define M_PI		(3.14159265358979323846)
#endif
#ifndef M_PI_F
#define M_PI_F		(float)(M_PI)
#endif

#define DEG_TO_RAD	(M_PI / 180.0f)
#define RAD_TO_DEG	(180.0f / M_PI)

#define CONSTANTS_ONE_G   (9.795f)

static inline float complementary_filter(float input1,float input2,float filter)
{
	return ((input1 * filter) + (input2 * (1.0f - filter)));
}

static inline float radians(float deg)
{
	return deg * DEG_TO_RAD;
}

static inline float degrees(float rad)
{
	return rad * RAD_TO_DEG;
}

static inline float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

static inline int min(int val1, int val2)
{
	return (val1 < val2) ? val1 : val2;
}

static inline int max(int val1, int val2)
{
	return (val1 > val2) ? val1 : val2;
}

static inline float sq(float v) 
{
	return v*v;
}

static inline float length2(float a, float b)
{
	return sqrtf(sq(a)+sq(b));
}

static inline float length3(float a, float b, float c) 
{
	return sqrtf(sq(a)+sq(b)+sq(c));
}


static inline float included_angle(float x1,float y1,float x2,float y2)
{
	float len1 = length2(x1,y1);
	float len2 = length2(x2,y2);
	float inner_product = x1*x2+y1*y2;

	return acosf(inner_product / (len1 * len2));
}

static inline float constrain_float(float amt, float low, float high)
{
	if (isnan(amt)) {
		return (low+high)*0.5f;
	}
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

static inline int8_t constrain_int8(int8_t amt, int8_t low, int8_t high) 
{
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

static inline int16_t constrain_int16(int16_t amt, int16_t low, int16_t high) 
{
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

static inline uint16_t constrain_uint16(uint16_t amt, uint16_t low, uint16_t high) 
{
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

static inline int32_t constrain_int32(int32_t amt, int32_t low, int32_t high)
{
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

static inline float get_diff_time(struct timespec * start , bool update)
{
	float dt;
	struct timespec now;
	
	clock_gettime(CLOCK_MONOTONIC,&now);
	dt = (float)(now.tv_sec  - start->tv_sec);
	dt += (float)(now.tv_nsec - start->tv_nsec) * 1e-9;

	if(update == true){
		start->tv_sec = now.tv_sec;
		start->tv_nsec = now.tv_nsec;
	}

	return dt;
}

static inline float wrap_180_cd_float(float angle)
{
    if (angle > 540.0f || angle < -540.0f) {
        // for large numbers use modulus
        angle = fmod(angle,360.0f);
    }
    if (angle > 180.0f) { angle -= 360.0f; }
    if (angle < -180.0f) { angle += 360.0f; }
    return angle;
}

static inline float wrap_pi(float angle)
{
    /*if (angle > M_PI_F * 3.0f || angle < -M_PI_F * 3.0f) {
        // for large numbers use modulus
        angle = fmod(angle,M_PI_F * 2.0f);
    }
    if (angle > M_PI_F) { angle -= M_PI_F * 2.0f; }
    if (angle < -M_PI_F) { angle += M_PI_F * 2.0f; }*/

	if(angle > M_PI_F){
		angle = fmod(angle,M_PI_F) - M_PI_F;
	}else if(angle < -M_PI_F){
		angle = fmod(angle,M_PI_F) + M_PI_F;
	}
    return angle;
}


static inline float wrap_360_cd_float(float angle)
{
    if (angle >= 720.0f || angle < -360.0f) {
        // for larger number use fmodulus
        angle = fmod(angle, 360.0f);
    }
    if (angle >= 360.0f) angle -= 360.0f;
    if (angle < 0.0f) angle += 360.0f;
    return angle;
}

static inline void bf_to_ef(float r[3][3],float bf[3],float ef[3])
{
	int i,j;

	for (i = 0; i < 3; i++) {
		ef[i] = 0.0f;
		for (j = 0; j < 3; j++){
			ef[i] += r[i][j] * bf[j];
		}
	}
}

static inline void ef_to_bf(float r[3][3],float bf[3],float ef[3])
{
	int i,j;
	for (i = 0; i < 3; i++) {
		bf[i] = 0.0f;
	
		for (j = 0; j < 3; j++) {
			bf[i] += r[j][i] * ef[j];
		}
	}
}


#endif

