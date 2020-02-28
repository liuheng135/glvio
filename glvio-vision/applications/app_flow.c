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

struct block_sad_info_s{
    int id;
    struct point2i pos;
    int sad;
};

pthread_mutex_t mp_mutex = PTHREAD_MUTEX_INITIALIZER;

struct flow_matched_point_s flow_mp[4];
unsigned char image_buffer[FLOW_IMAGE_WIDTH*FLOW_IMAGE_HEIGHT];
float image_timestamp;

int flow_copy_matched_points(struct flow_matched_point_s *mps)
{
    int i;
    if(pthread_mutex_trylock(&mp_mutex) == 0){
        for(i = 0; i < 4; i++){
            mps[i] = flow_mp[i];
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
            memcpy(img,image_buffer,FLOW_IMAGE_WIDTH * FLOW_IMAGE_HEIGHT);
            *timestamp = image_timestamp;
            pthread_mutex_unlock(&mp_mutex);
            return FLOW_IMAGE_WIDTH * FLOW_IMAGE_HEIGHT;
        }
        return 0;
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

int image_buffer_a[FLOW_IMAGE_WIDTH*FLOW_IMAGE_HEIGHT];
int image_buffer_b[FLOW_IMAGE_WIDTH*FLOW_IMAGE_HEIGHT];

unsigned char image_binning_buffer_0[FLOW_IMAGE_WIDTH*FLOW_IMAGE_HEIGHT * 16];
unsigned char image_binning_buffer_1[FLOW_IMAGE_WIDTH*FLOW_IMAGE_HEIGHT * 4];
unsigned char image_binning_buffer_2[FLOW_IMAGE_WIDTH*FLOW_IMAGE_HEIGHT];

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
    struct matrix_s roi_img;
    struct matrix_s bin1_img;
	struct matrix_s bin2_img;

    struct matrix_s prev_img;
    struct matrix_s next_img;

	struct point2i roi_start = {
            .x = (CAM0_IMAGE_WIDTH  - FLOW_IMAGE_WIDTH * 4) / 2,
            .y = (CAM0_IMAGE_HEIGHT - FLOW_IMAGE_HEIGHT * 4)  /2,};

	struct size2i  roi_size = {.x = FLOW_IMAGE_WIDTH * 4, .y = FLOW_IMAGE_HEIGHT * 4};
    struct size2i  opt_win_size = {.x = 17,.y = 17}; 

    struct point2f prev_point[4];
    struct point2f next_point[4];
    float  flow_err[4];

    struct block_sad_info_s block_sad_list[FLOW_IMAGE_WIDTH * FLOW_IMAGE_HEIGHT / 64];
    struct block_sad_info_s block_best[4];
    int block_count;

    ret = camera_init("/dev/video1",CAM0_IMAGE_WIDTH,CAM0_IMAGE_HEIGHT);
    if(ret < 0){
        exit(-1);
    }
    
    image = camera_get_image(); 
    matrix_init(&raw_img,CAM0_IMAGE_WIDTH,CAM0_IMAGE_HEIGHT,1,IMAGE_TYPE_8U,image);
    matrix_init(&roi_img,FLOW_IMAGE_WIDTH * 4,FLOW_IMAGE_HEIGHT * 4,1,IMAGE_TYPE_8U,image_binning_buffer_0);
    matrix_init(&bin1_img,FLOW_IMAGE_WIDTH * 2,FLOW_IMAGE_HEIGHT * 2,1,IMAGE_TYPE_8U,image_binning_buffer_1);
    matrix_init(&bin2_img,FLOW_IMAGE_WIDTH,FLOW_IMAGE_HEIGHT,1,IMAGE_TYPE_8U,image_binning_buffer_2);

    matrix_init(&prev_img,FLOW_IMAGE_WIDTH,FLOW_IMAGE_HEIGHT,1,IMAGE_TYPE_32S,(unsigned char *)image_buffer_a);
    matrix_init(&next_img,FLOW_IMAGE_WIDTH,FLOW_IMAGE_HEIGHT,1,IMAGE_TYPE_32S,(unsigned char *)image_buffer_b);

    printf("roi start = %d, %d   size = %d, %d  \r\n",roi_start.x,roi_start.y,roi_size.x,roi_size.y); 
    matrix_copy_aera(&raw_img,&roi_img,&roi_start,&roi_size);
    matrix_binning_neon_u8(&roi_img,&bin1_img);
    matrix_binning_neon_u8(&bin1_img,&bin2_img);
    if(matrix_convert_type(&bin2_img,&prev_img) != 0 ){
        printf("unsupport image type\r\n");
    }
    optflow_lk_create(&optflow0,1,5.0f,0.5f,&opt_win_size);

    last = get_time_now();
    while(1) {
        //从摄像头获取一帧图像
        image = camera_get_image();
        now = get_time_now();
        dt = (float)(now - last);
        last = now;

		matrix_init(&raw_img,CAM0_IMAGE_WIDTH,CAM0_IMAGE_HEIGHT,1,IMAGE_TYPE_8U,image);
        matrix_copy_aera(&raw_img,&roi_img,&roi_start,&roi_size);
        matrix_binning_neon_u8(&roi_img,&bin1_img);
        matrix_binning_neon_u8(&bin1_img,&bin2_img);
        if(matrix_convert_type(&bin2_img,&next_img) != 0 ){
            printf("unsupport image type\r\n");
            continue;
        }

        block_count = 0;
        for(j = 12;j < bin2_img.cols-12;j+=8){
            for(i = 12 ;i < bin2_img.rows-12 ;i+=8){
                block_sad_list[block_count].id = block_count;
                block_sad_list[block_count].pos.x = i-4;
                block_sad_list[block_count].pos.y = j-4;
                block_sad_list[block_count].sad = matrix_block_sad_8x8_neon_u8(&bin2_img,&block_sad_list[block_count].pos);
                block_count++;
            }
        }

        for(j = 0;j < 4;j++){
            block_best[j].sad = 0;
            for(i = 0;i < block_count;i++){
                if(block_sad_list[i].sad > block_best[j].sad){
                    block_best[j].sad = block_sad_list[i].sad;
                    block_best[j].pos = block_sad_list[i].pos;
                    block_best[j].id  = i;
                }
            }
            block_sad_list[block_best[j].id].sad = 0;
        }

        printf("best: [%2d,%2d,%4d],[%2d,%2d,%4d],[%2d,%2d,%4d],[%2d,%2d,%4d]\r\n",
                    block_best[0].pos.x,block_best[0].pos.y,block_best[0].sad,
                    block_best[1].pos.x,block_best[1].pos.y,block_best[1].sad,
                    block_best[2].pos.x,block_best[2].pos.y,block_best[2].sad,
                    block_best[3].pos.x,block_best[3].pos.y,block_best[3].sad
        );

        for(j = 0;j < 4;j++){
            prev_point[j].x = block_best[j].pos.x+4;
            prev_point[j].y = block_best[j].pos.y+4;
        }

        for(i = 0; i < 4; i++){
            optflow_lk_calc(&optflow0,&prev_img,&next_img,&prev_point[i],&next_point[i],&flow_err[i]);
        }

        swap_ptr = next_img.data;
        next_img.data = prev_img.data;
        prev_img.data = swap_ptr;

        pthread_mutex_lock(&mp_mutex);
        for(i = 0; i < 4; i++){
            flow_mp[i].start_x = (prev_point[i].x - FLOW_IMAGE_WIDTH * 0.5f) * 0.00425f;
            flow_mp[i].start_y = (prev_point[i].y - FLOW_IMAGE_HEIGHT * 0.5f) * 0.00425f;
            flow_mp[i].end_x = (next_point[i].x - FLOW_IMAGE_WIDTH * 0.5f) * 0.00425f;
            flow_mp[i].end_y = (next_point[i].y - FLOW_IMAGE_HEIGHT * 0.5f) * 0.00425f;
            flow_mp[i].quality = flow_err[i];
            flow_mp[i].timestamp += dt;
        }
        memcpy(image_buffer,bin2_img.data,FLOW_IMAGE_WIDTH * FLOW_IMAGE_HEIGHT);
        image_timestamp+=dt;
        pthread_mutex_unlock(&mp_mutex);
    }
    camera_deinit();
    return NULL;
}


