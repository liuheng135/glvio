#include "optflow_lk.h"
#include <stdio.h>
#include <stdlib.h>
#include <float.h>

void optflow_lk_create(struct optflow_lk *op,int max_layer,float max_vel,float lambda,struct size2i *win_size)
{
    op->lambda     = lambda;
    op->max_layer  = max_layer;
    op->win_size.x = win_size->x;
    op->win_size.y = win_size->y;
    op->max_vel    = max_vel;

    matrix_create(&op->deriv_buffer,win_size->x ,win_size->y,6,IMAGE_TYPE_32F);

    matrix_create(&op->kx01,3,1,1,IMAGE_TYPE_32S);
    matrix_create(&op->ky01,3,1,1,IMAGE_TYPE_32S);
    matrix_create(&op->kx02,3,1,1,IMAGE_TYPE_32S);
    matrix_create(&op->ky02,3,1,1,IMAGE_TYPE_32S);
    matrix_create(&op->kx10,3,1,1,IMAGE_TYPE_32S);
    matrix_create(&op->ky10,3,1,1,IMAGE_TYPE_32S);
    matrix_create(&op->kx20,3,1,1,IMAGE_TYPE_32S);
    matrix_create(&op->ky20,3,1,1,IMAGE_TYPE_32S);
    matrix_create(&op->kx11,3,1,1,IMAGE_TYPE_32S);
    matrix_create(&op->ky11,3,1,1,IMAGE_TYPE_32S);

    make_deriv_kernel(&op->kx10,&op->ky10,1,0,3);
    make_deriv_kernel(&op->kx20,&op->ky20,2,0,3);
    make_deriv_kernel(&op->kx01,&op->ky01,0,1,3);
    make_deriv_kernel(&op->kx02,&op->ky02,0,2,3);
    make_deriv_kernel(&op->kx11,&op->ky11,1,1,3);
}

void optflow_lk_set_lambda(struct optflow_lk *op,float lambda)
{
    op->lambda = lambda;
}

void optflow_lk_set_win_size(struct optflow_lk *op,struct size2i *win_size)
{
    matrix_destroy(&op->deriv_buffer);

    op->win_size.x = win_size->x;
    op->win_size.y = win_size->y;

    matrix_create(&op->deriv_buffer,win_size->x,win_size->y,6,IMAGE_TYPE_32S);
}

void optflow_lk_destroy(struct optflow_lk *op)
{
    op->lambda = 0;
    op->max_layer = 0;
    op->win_size.x = 0;
    op->win_size.y = 0;

    matrix_destroy(&op->deriv_buffer);
    matrix_destroy(&op->kx01);
    matrix_destroy(&op->ky01);
    matrix_destroy(&op->kx10);
    matrix_destroy(&op->kx10);
    matrix_destroy(&op->kx11);
    matrix_destroy(&op->kx11);
    matrix_destroy(&op->kx02);
    matrix_destroy(&op->ky02);
    matrix_destroy(&op->kx20);
    matrix_destroy(&op->kx20);
}

void optflow_lk_calc(struct optflow_lk *op,struct matrix_s *prev_img,struct matrix_s *next_img,struct point2f *prev_point,struct point2f *next_point,float *err)
{
	int i,j,k,ii,jj;
	int *src;
	double D;
    float lambda1 = 1. - op->lambda, lambda2 = op->lambda;
    
    float w00,w01,w10,w11,a,b;
    int vi00,vi01,vi10,vi11;

    float scale1 = 0.5f / 4.f;
    float scale2 = 0.25f / 4.f;

    float A11 = 0, A12 = 0, A22 = 0;
    float iA11 = 0, iA12 = 0, iA22 = 0;

    int win_size_half_x = (op->win_size.x - 1) / 2;
    int win_size_half_y = (op->win_size.y - 1) / 2;

    int ppx = (int)prev_point->x;
    int ppy = (int)prev_point->y;
	int npx,npy;
	
	float It,Ixt,Iyt;
	float b1, b2,ib1,ib2;
	float *Ibuf;
	float delta_x,delta_y;
	struct point2f new_point;

    int if_break = 0;
	
    a = prev_point->x - (float)ppx;
    b = prev_point->y - (float)ppy;

    w00 = (1.f - a)*(1.f - b); 
    w01 = a*(1.f - b);
    w10 = (1.f - a)*b; 
    w11 = a*b;
    
    Ibuf = (float *)(op->deriv_buffer.data);
    for(j = 0; j < op->win_size.y; j++){
        for(i = 0; i < op->win_size.x; i++){
            ii = ppx - win_size_half_x + i;
            jj = ppy - win_size_half_y + j;

            src = (int *)prev_img->data;  
            src+= jj * prev_img->cols + ii;
            vi00 = src[0];
            vi01 = src[1];
            vi10 = src[prev_img->cols];
            vi11 = src[prev_img->cols + 1];
            Ibuf[0] = w00 * (float)vi00 + w01 * (float)vi01 + w10 * (float)vi10 + w11 * (float)vi11;
            vi00 = matrix_calc_pixel_deriv(prev_img,ii,jj,&op->kx10,&op->ky10);
            vi01 = matrix_calc_pixel_deriv(prev_img,ii,jj+1,&op->kx10,&op->ky10);
            vi10 = matrix_calc_pixel_deriv(prev_img,ii+1,jj,&op->kx10,&op->ky10);
            vi11 = matrix_calc_pixel_deriv(prev_img,ii+1,jj+1,&op->kx10,&op->ky10);
            Ibuf[1] = w00 * (float)vi00 + w01 * (float)vi01 + w10 * (float)vi10 + w11 * (float)vi11;
            Ibuf[1] *= scale1;

            vi00 = matrix_calc_pixel_deriv(prev_img,ii,jj,&op->kx01,&op->ky01);
            vi01 = matrix_calc_pixel_deriv(prev_img,ii,jj+1,&op->kx01,&op->ky01);
            vi10 = matrix_calc_pixel_deriv(prev_img,ii+1,jj,&op->kx01,&op->ky01);
            vi11 = matrix_calc_pixel_deriv(prev_img,ii+1,jj+1,&op->kx01,&op->ky01);
            Ibuf[2]  = w00 * (float)vi00 + w01 * (float)vi01 + w10 * (float)vi10 + w11 * (float)vi11;
            Ibuf[2] *= scale1;

            vi00 = matrix_calc_pixel_deriv(prev_img,ii,jj,&op->kx20,&op->ky20);
            vi01 = matrix_calc_pixel_deriv(prev_img,ii,jj+1,&op->kx20,&op->ky20);
            vi10 = matrix_calc_pixel_deriv(prev_img,ii+1,jj,&op->kx20,&op->ky20);
            vi11 = matrix_calc_pixel_deriv(prev_img,ii+1,jj+1,&op->kx20,&op->ky20);
            Ibuf[3]  = w00 * (float)vi00 + w01 * (float)vi01 + w10 * (float)vi10 + w11 * (float)vi11;
            Ibuf[3] *= scale2;

            vi00 = matrix_calc_pixel_deriv(prev_img,ii,jj,&op->kx11,&op->ky11);
            vi01 = matrix_calc_pixel_deriv(prev_img,ii,jj+1,&op->kx11,&op->ky11);
            vi10 = matrix_calc_pixel_deriv(prev_img,ii+1,jj,&op->kx11,&op->ky11);
            vi11 = matrix_calc_pixel_deriv(prev_img,ii+1,jj+1,&op->kx11,&op->ky11);
            Ibuf[4]  = w00 * (float)vi00 + w01 * (float)vi01 + w10 * (float)vi10 + w11 * (float)vi11;
            Ibuf[4] *= scale2;

            vi00 = matrix_calc_pixel_deriv(prev_img,ii,jj,&op->kx02,&op->ky02);
            vi01 = matrix_calc_pixel_deriv(prev_img,ii,jj+1,&op->kx02,&op->ky02);
            vi10 = matrix_calc_pixel_deriv(prev_img,ii+1,jj,&op->kx02,&op->ky02);
            vi11 = matrix_calc_pixel_deriv(prev_img,ii+1,jj+1,&op->kx02,&op->ky02);
            Ibuf[5] = w00 * (float)vi00 + w01 * (float)vi01 + w10 * (float)vi10 + w11 * (float)vi11;
            Ibuf[5] *= scale2;

            iA11 += Ibuf[1] * Ibuf[1];
            iA22 += Ibuf[2] * Ibuf[2];
            iA12 += Ibuf[1] * Ibuf[2];

            A11 += Ibuf[3]*Ibuf[3] + Ibuf[4]*Ibuf[4];
            A12 += Ibuf[4]*(Ibuf[3] + Ibuf[5]);
            A22 += Ibuf[4]*Ibuf[4] + Ibuf[5]*Ibuf[5];
            Ibuf +=  op->deriv_buffer.channel;
        }
    }
    A11 = lambda1*iA11 + lambda2*A11;
    A12 = lambda1*iA12 + lambda2*A12;
    A22 = lambda1*iA22 + lambda2*A22;

    D = A11*A22 - A12*A12;

    if(err){
        *err = (A22 + A11 - sqrt((A11-A22)*(A11-A22) + 4.f*A12*A12))/(2* op->win_size.x * op->win_size.y);
    }
    if( D < DBL_EPSILON )
    {
        next_point->x = prev_point->x;
        next_point->y = prev_point->y;
        return;
    }
    D = 1./D;

    new_point.x = prev_point->x;
    new_point.y = prev_point->y;
    
    if_break = 0;

    for(k = 0;k < 10;k++){
        b1 = 0.0;
        b2 = 0.0;
        ib1 = 0.0;
        ib2 = 0.0;
        npx = (int)new_point.x;
        npy = (int)new_point.y;

        a = new_point.x - (float)npx;
        b = new_point.y - (float)npy;

        w00 = (1.f - a)*(1.f - b); 
        w01 = a*(1.f - b);
        w10 = (1.f - a)*b; 
        w11 = a*b;

        Ibuf = (float *)(op->deriv_buffer.data);

        for( j = 0; j < op->win_size.y; j++){
            for( i = 0; i < op->win_size.x; i++){
                ii = npx - win_size_half_x + i;
                jj = npy - win_size_half_y + j;

                src = (int *)next_img->data;  
                src += jj * next_img->cols + ii;

                vi00 = src[0];  
                vi01 = src[1];
                vi10 = src[next_img->cols];
                vi11 = src[next_img->cols + 1];

                It = w00 * (float)vi00 + w01 * (float)vi01 + w10 * (float)vi10 + w11 * (float)vi11 - Ibuf[0];

                vi00 = matrix_calc_pixel_deriv(next_img,ii,jj,&op->kx10,&op->ky10);
                vi01 = matrix_calc_pixel_deriv(next_img,ii+1,jj,&op->kx10,&op->ky10);
                vi10 = matrix_calc_pixel_deriv(next_img,ii,jj+1,&op->kx10,&op->ky10);
                vi11 = matrix_calc_pixel_deriv(next_img,ii+1,jj+1,&op->kx10,&op->ky10);
                Ixt = (w00 * (float)vi00 + w01 * (float)vi01 + w10 * (float)vi10 + w11 * (float)vi11) * scale1 - Ibuf[1];
                vi00 = matrix_calc_pixel_deriv(next_img,ii,jj,&op->kx01,&op->ky01);
                vi01 = matrix_calc_pixel_deriv(next_img,ii+1,jj,&op->kx01,&op->ky01);
                vi10 = matrix_calc_pixel_deriv(next_img,ii,jj+1,&op->kx01,&op->ky01);
                vi11 = matrix_calc_pixel_deriv(next_img,ii+1,jj+1,&op->kx01,&op->ky01);
                Iyt = (w00 * (float)vi00 + w01 * (float)vi01 + w10 * (float)vi10 + w11 * (float)vi11) * scale1 - Ibuf[2];

                b1 += Ixt*(float)Ibuf[3] + Iyt*(float)Ibuf[4];
                b2 += Ixt*(float)Ibuf[4] + Iyt*(float)Ibuf[5];
                ib1 += It*(float)Ibuf[1];
                ib2 += It*(float)Ibuf[2];

                Ibuf +=  op->deriv_buffer.channel;
            }
        }

        b1 = lambda1*ib1 + lambda2*b1;
        b2 = lambda1*ib2 + lambda2*b2;
        
        delta_x = (A12*b2 - A22*b1) * D;
        delta_y = (A12*b1 - A11*b2) * D;

        new_point.x += delta_x;
        new_point.y += delta_y;

        if((new_point.x - prev_point->x) > op->max_vel){
            new_point.x = prev_point->x + op->max_vel;
            if_break = 1;
        }else if((new_point.x - prev_point->x) < -op->max_vel){
            new_point.x = prev_point->x - op->max_vel;
            if_break = 1;
        }

        if((new_point.y - prev_point->y) > op->max_vel){
            new_point.y = prev_point->y + op->max_vel;
             if_break = 1;
        }else if((new_point.y - prev_point->y) < -op->max_vel){
            new_point.y = prev_point->y - op->max_vel;
             if_break = 1;
        }

        if((delta_x * delta_x + delta_y * delta_y) < 0.001f){
             if_break = 1;
        }

        if( if_break != 0){
            break;
        }
    }

    next_point->x = new_point.x;
    next_point->y = new_point.y;
}
