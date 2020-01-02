#ifndef _SPHERE_FIT_H_
#define _SPHERE_FIT_H_

int sphere_fit_least_squares(const float x[], const float y[], const float z[],
			     unsigned int size, unsigned int max_iterations, float delta, float *sphere_x, float *sphere_y, float *sphere_z, float *sphere_radius);

#endif

