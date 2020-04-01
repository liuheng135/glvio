#ifndef _OPTFLOW_H_
#define _OPTFLOW_H_

#include "image_lib.h"

struct optflow_lk{
    int    max_layer;
    float  lambda;
    float  max_vel;
    struct size2i win_size;

    struct matrix_s kx10;
    struct matrix_s ky10;

    struct matrix_s kx20;
    struct matrix_s ky20;
    
    struct matrix_s kx01;
    struct matrix_s ky01;
    
    struct matrix_s kx02;
    struct matrix_s ky02;

    struct matrix_s kx11;
    struct matrix_s ky11;

    /* ch0 ~ I */
    /* ch1 ~ Ix */
    /* ch2 ~ Iy */
    /* ch3 ~ Ixx */
    /* ch4 ~ Ixy */
    /* ch5 ~ Iyy */
    struct matrix_s deriv_buffer;

#ifdef IMAGE_LIB_USING_NEON
    int kbuffer10[3][4];
    int kbuffer20[3][4];
    int kbuffer01[3][4];
    int kbuffer02[3][4];
    int kbuffer11[3][4];
#endif
};

void optflow_lk_create(struct optflow_lk *op,int max_layer,float max_vel,float lambda,struct size2i *win_size);
void optflow_lk_set_lambda(struct optflow_lk *op,float lambda);
void optflow_lk_set_win_size(struct optflow_lk *op,struct size2i *win_size);
void optflow_lk_destroy(struct optflow_lk *op);
int  optflow_lk_calc(struct optflow_lk *op,struct matrix_s *prev_img,struct matrix_s *next_img,struct point2f *prev_point,struct point2f *next_point,float *err);
#ifdef IMAGE_LIB_USING_NEON
int  optflow_lk_calc_neon_u8(struct optflow_lk *op,struct matrix_s *prev_img,struct matrix_s *next_img,struct point2f *prev_point,struct point2f *next_point,float *err);
#endif

#endif
