#include "app_flow.h"
#include <stdio.h>
#include <pthread.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include "camera.h"
#include "lwlink.h"
#include "image_lib.h"
#include "optflow_lk.h"
#include "jpegencoder.h"

struct block_sad_info_s{
    int inited;
    int id;
    int sad;
    struct point2i pos;
    struct point2i range_start;
    struct point2i range_size;
};

pthread_mutex_t mp_mutex = PTHREAD_MUTEX_INITIALIZER;

struct geo_feature_s flow_features[4];
unsigned char image_buffer[PREVIEW_IMAGE_WIDTH*PREVIEW_IMAGE_HEIGHT];
float image_timestamp;

int flow_copy_matched_points(struct geo_feature_s *ft)
{
    int i;
    if(pthread_mutex_trylock(&mp_mutex) == 0){
        for(i = 0; i < 4; i++){
            ft[i] = flow_features[i];
        }
        pthread_mutex_unlock(&mp_mutex);
        return 1;
    }   
    return 0;
}

int flow_copy_image(unsigned char *img,float *timestamp)
{
    if(image_timestamp > *timestamp){
        if(pthread_mutex_trylock(&mp_mutex) == 0){
            memcpy(img,image_buffer,PREVIEW_IMAGE_WIDTH*PREVIEW_IMAGE_HEIGHT);
            *timestamp = image_timestamp;
            pthread_mutex_unlock(&mp_mutex);
            return PREVIEW_IMAGE_WIDTH * PREVIEW_IMAGE_HEIGHT;
        }
        return 1;
    }
    return 0;
}

struct optflow_lk  optflow0;

static inline double get_time_now(void)
{
	double dt;
	struct timespec now;
	
	clock_gettime(CLOCK_MONOTONIC,&now);
	dt = (double)now.tv_sec;
	dt += 1e-9 * (double)now.tv_nsec;
	return dt;
}

unsigned char image_buffer_a[CAM0_IMAGE_WIDTH*CAM0_IMAGE_HEIGHT / 4];
unsigned char image_buffer_b[CAM0_IMAGE_WIDTH*CAM0_IMAGE_HEIGHT / 4];

unsigned char image_buffer_c[CAM0_IMAGE_WIDTH*CAM0_IMAGE_HEIGHT / 16];
unsigned char image_buffer_d[CAM0_IMAGE_WIDTH*CAM0_IMAGE_HEIGHT / 64];

int flow_find_best_block(struct matrix_s *img,struct point2i *start,struct point2i *size,struct block_sad_info_s *block)
{
    int i,j;
    int sad;
    struct point2i pos;

    block->sad = 0;

    if(start->x + size->x + 8 >= img->cols || start->y + size->y + 8 >= img->rows){
        return -2;
    }

    for(j = start->y;j < start->y + size->y;j+=8){
        for(i = start->x ;i < start->x + size->x ;i+=8){
            pos.x = i;
            pos.y = j;
            sad = matrix_block_sad_8x8_neon_u8(img,&pos);
            if(sad > block->sad ){
                block->sad = sad;
                block->pos = pos;
            }
        }
    }

    block->pos.x += 4;
    block->pos.y += 4;

    return 0;
}

void *thread_flow(void *arg)
{
    int i,j;
    int ret = 0;

    uint8_t *image = NULL;
    uint8_t *swap_ptr = NULL;

    float  dt;
    double now;
    double last;
	
	struct matrix_s raw_img;

    struct matrix_s prev_img;
    struct matrix_s next_img;

    struct matrix_s next_img_half;
    struct matrix_s next_img_quater;

    struct size2i  opt_win_size = {.x = 17,.y = 17}; 

    struct point2f prev_point[4];
    struct point2f next_point[4];
    float  flow_err[4];

    struct block_sad_info_s block_best[4];
    struct point2i block_roi_start;
    struct point2i block_roi_size;
    int block_roi_border = 14;

    ret = camera_init("/dev/video1",CAM0_IMAGE_WIDTH,CAM0_IMAGE_HEIGHT);
    if(ret < 0){
        exit(-1);
    }
    
    image = camera_get_image(); 
    matrix_init(&raw_img,CAM0_IMAGE_WIDTH,CAM0_IMAGE_HEIGHT,1,IMAGE_TYPE_8U,image);
    matrix_init(&prev_img,CAM0_IMAGE_WIDTH / 2,CAM0_IMAGE_HEIGHT / 2,1,IMAGE_TYPE_8U,image_buffer_a);
    matrix_init(&next_img,CAM0_IMAGE_WIDTH / 2,CAM0_IMAGE_HEIGHT / 2,1,IMAGE_TYPE_8U,image_buffer_b);
    matrix_init(&next_img_half,CAM0_IMAGE_WIDTH / 4,CAM0_IMAGE_HEIGHT / 4,1,IMAGE_TYPE_8U,image_buffer_c);
    matrix_init(&next_img_quater,CAM0_IMAGE_WIDTH / 8,CAM0_IMAGE_HEIGHT / 8,1,IMAGE_TYPE_8U,image_buffer_d);
    
    matrix_binning_neon_u8(&raw_img,&prev_img);
    optflow_lk_create(&optflow0,1,5.0f,0.5f,&opt_win_size);

    last = get_time_now();
    while(1) {
        image = camera_get_image();
        now = get_time_now();
        dt = (float)(now - last);
        last = now;

		matrix_init(&raw_img,CAM0_IMAGE_WIDTH,CAM0_IMAGE_HEIGHT,1,IMAGE_TYPE_8U,image);
        matrix_binning_neon_u8(&raw_img,&next_img);

        block_roi_start.x = block_roi_border;
        block_roi_start.y = block_roi_border;
        block_roi_size.x  = next_img.cols / 2;
        block_roi_size.y  = next_img.rows / 2;
        flow_find_best_block(&prev_img,&block_roi_start,&block_roi_size,&block_best[0]);
        block_roi_start.x = next_img.cols / 2;
        block_roi_start.y = block_roi_border;
        block_roi_size.x  = next_img.cols / 2 - block_roi_border;
        block_roi_size.y  = next_img.rows / 2;
        flow_find_best_block(&prev_img,&block_roi_start,&block_roi_size,&block_best[1]);
        block_roi_start.x = block_roi_border;
        block_roi_start.y = next_img.rows / 2;
        block_roi_size.x  = next_img.cols / 2;
        block_roi_size.y  = next_img.rows / 2 - block_roi_border;
        flow_find_best_block(&prev_img,&block_roi_start,&block_roi_size,&block_best[2]);
        block_roi_start.x = next_img.cols / 2;
        block_roi_start.y = next_img.rows / 2;
        block_roi_size.x  = next_img.cols / 2 - block_roi_border;
        block_roi_size.y  = next_img.rows / 2 - block_roi_border;
        flow_find_best_block(&prev_img,&block_roi_start,&block_roi_size,&block_best[3]);


        for(j = 0;j < 4;j++){
            prev_point[j].x = block_best[j].pos.x;
            prev_point[j].y = block_best[j].pos.y;
        }

        for(i = 0; i < 4; i++){
            optflow_lk_calc_neon_u8(&optflow0,&prev_img,&next_img,&prev_point[i],&next_point[i],&flow_err[i]);
        }

        swap_ptr = next_img.data;
        next_img.data = prev_img.data;
        prev_img.data = swap_ptr;

        pthread_mutex_lock(&mp_mutex);
        for(i = 0; i < 4; i++){
            flow_features[i].observer_prev.point_pos_in_camera.x  = (prev_point[i].x - next_img.cols * 0.5f) * FLOW_RAD_PER_PIXEL;
            flow_features[i].observer_prev.point_pos_in_camera.y  = (prev_point[i].y - next_img.rows * 0.5f) * FLOW_RAD_PER_PIXEL;
            flow_features[i].observer_prev.point_pos_in_camera.z  = 1.0f;
            flow_features[i].observer_next.point_pos_in_camera.x  = (next_point[i].x - next_img.cols * 0.5f) * FLOW_RAD_PER_PIXEL;
            flow_features[i].observer_next.point_pos_in_camera.y  = (next_point[i].y - next_img.rows * 0.5f) * FLOW_RAD_PER_PIXEL;
            flow_features[i].observer_prev.point_pos_in_camera.z  = 1.0f;
            flow_features[i].observer_next.quality    = flow_err[i];
            flow_features[i].observer_next.timestamp += dt;
            flow_features[i].observer_prev.point_pos_valid = 1;
            flow_features[i].observer_next.point_pos_valid = 1;
        }
        matrix_binning_neon_u8(&next_img,&next_img_half);
        matrix_binning_neon_u8(&next_img_half,&next_img_quater);
        memcpy(image_buffer,next_img_quater.data,next_img_quater.cols * next_img_quater.rows);
        image_timestamp+=dt;
        pthread_mutex_unlock(&mp_mutex);
    }
    camera_deinit();
    return NULL;
}


