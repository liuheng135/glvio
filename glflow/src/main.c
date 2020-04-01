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
#include "network.h"
#include "image_lib.h"
#include "optflow_lk.h"
#include "msgque.h"
#include "lpfilter.h"

#define STACK_SIZE                      (512*1024)
#define ARRAY_SIZE(arr)                 ( sizeof(arr) / sizeof((arr)[0]) )
#define IS_IN_RANGE(var, min, max)      ( ((min) <= (var)) && ((var) <= (max)) )

#define CAM0_IMAGE_WIDTH    640
#define CAM0_IMAGE_HEIGHT   480


//#define FLOW_USING_LWLINK
#define FLOW_USING_MSGQUE

pthread_t thid1;

#ifdef FLOW_USING_LWLINK
struct net_data_s net0;
struct lwlink_data_handler_s link_handler;
#endif


struct optflow_lk  optflow0;
struct optflow_lk  optflow1;

struct lpfilter_s flow_filter[2];

int  flow_log_fd = -1;
char flow_log_buffer[128];
int  flow_log_length;

static inline double get_time_now(void)
{
	double dt;
	struct timespec now;
	
	clock_gettime(CLOCK_MONOTONIC,&now);
	dt = (double)now.tv_sec;
	dt += 1e-9 * (double)now.tv_nsec;
	return dt;
}

unsigned char image_binning_buffer_0[CAM0_IMAGE_WIDTH*CAM0_IMAGE_HEIGHT / 4];
unsigned char image_binning_buffer_1[CAM0_IMAGE_WIDTH*CAM0_IMAGE_HEIGHT / 16];
unsigned char image_binning_buffer_2[CAM0_IMAGE_WIDTH*CAM0_IMAGE_HEIGHT / 64];
unsigned char image_binning_buffer_3[CAM0_IMAGE_WIDTH*CAM0_IMAGE_HEIGHT / 64];
unsigned char image_binning_buffer_4[CAM0_IMAGE_WIDTH*CAM0_IMAGE_HEIGHT / 256];
unsigned char image_binning_buffer_5[CAM0_IMAGE_WIDTH*CAM0_IMAGE_HEIGHT / 256];

static void *thread_flow(void *arg)
{
    int ret = 0;
    int counter = 0;
	int msg_length = 0;
    uint8_t *image = NULL;
    uint8_t *swap_ptr = NULL;
	
	struct matrix_s raw_img;
    struct matrix_s half_img;
    struct matrix_s quater_img;
    struct matrix_s next_img;
    struct matrix_s prev_img;
    struct matrix_s next_img_half;
    struct matrix_s prev_img_half;
 
    struct size2i  opt_win_size = {.x = 19,.y = 19}; 

    struct point2f prev_point;
    struct point2f next_point;
    struct point2i temp_point;
    float          flow_err;

    float  dt;
    float  calc_time;
    double now;
    double last;

    ret = camera_init("/dev/video1",CAM0_IMAGE_WIDTH,CAM0_IMAGE_HEIGHT);
    if(ret < 0){
        exit(-1);
    }
    
    image = camera_get_image(); 
    matrix_init(&raw_img,CAM0_IMAGE_WIDTH,CAM0_IMAGE_HEIGHT,1,IMAGE_TYPE_8U,image);
    matrix_init(&half_img,CAM0_IMAGE_WIDTH / 2,CAM0_IMAGE_HEIGHT / 2,1,IMAGE_TYPE_8U,image_binning_buffer_0);
    matrix_init(&quater_img,CAM0_IMAGE_WIDTH / 4,CAM0_IMAGE_HEIGHT / 4,1,IMAGE_TYPE_8U,image_binning_buffer_1);
    matrix_init(&prev_img,CAM0_IMAGE_WIDTH / 8,CAM0_IMAGE_HEIGHT / 8,1,IMAGE_TYPE_8U,image_binning_buffer_2);
    matrix_init(&next_img,CAM0_IMAGE_WIDTH / 8,CAM0_IMAGE_HEIGHT / 8,1,IMAGE_TYPE_8U,image_binning_buffer_3);
    matrix_init(&prev_img_half,CAM0_IMAGE_WIDTH / 16,CAM0_IMAGE_HEIGHT / 16,1,IMAGE_TYPE_8U,image_binning_buffer_4);
    matrix_init(&next_img_half,CAM0_IMAGE_WIDTH / 16,CAM0_IMAGE_HEIGHT / 16,1,IMAGE_TYPE_8U,image_binning_buffer_5);
    matrix_binning_neon_u8(&raw_img,&half_img);
    matrix_binning_neon_u8(&half_img,&quater_img);
    matrix_binning_neon_u8(&quater_img,&prev_img);
    matrix_binning_neon_u8(&prev_img,&prev_img_half);

    set_cutoff_param(&flow_filter[0],50,5);
    set_cutoff_param(&flow_filter[1],50,5);

#ifdef FLOW_USING_LWLINK
    if(network_init(&net0,"192.168.0.20",3366) < 0){
		printf("Failed to init network\r\n");
	}
	lwlink_data_handler_init(&link_handler,0x02);
#endif

#ifdef FLOW_USING_MSGQUE
    msgque_init();
#endif

    flow_log_fd = open("/tmp/flow_log.txt",O_CREAT | O_RDWR | O_TRUNC);

    optflow_lk_create(&optflow0,1,5.0f,0.5f,&opt_win_size);
    optflow_lk_create(&optflow1,1,5.0f,0.5f,&opt_win_size);

    last = get_time_now();

    while(1) {
        //从摄像头获取一帧图像
        now = get_time_now();
        dt = (float)(now - last);
        last = now;
        
        image = camera_get_image();  
		matrix_init(&raw_img,CAM0_IMAGE_WIDTH,CAM0_IMAGE_HEIGHT,1,IMAGE_TYPE_8U,image);
        matrix_binning_neon_u8(&raw_img,&half_img);
        matrix_binning_neon_u8(&half_img,&quater_img);
        matrix_binning_neon_u8(&quater_img,&next_img);
        matrix_binning_neon_u8(&next_img,&next_img_half);

        prev_point.x = next_img_half.cols / 2;
        prev_point.y = next_img_half.rows / 2;
        optflow_lk_calc_neon_u8(&optflow0,&prev_img_half,&next_img_half,&prev_point,&next_point,&flow_err);
        temp_point.x = (int)(next_point.x * 2.f + 0.5f);
        temp_point.y = (int)(next_point.y * 2.f + 0.5f);
        prev_point.x = (float)temp_point.x;
        prev_point.y = (float)temp_point.y;
        optflow_lk_calc_neon_u8(&optflow1,&prev_img,&next_img,&prev_point,&next_point,&flow_err);

        swap_ptr = next_img_half.data;
        next_img_half.data = prev_img_half.data;
        prev_img_half.data = swap_ptr;

        swap_ptr = next_img.data;
        next_img.data = prev_img.data;
        prev_img.data = swap_ptr;

        counter++;
#ifdef FLOW_USING_LWLINK
        if(counter % 3 == 0){
            char *recv_buffer[128];
            struct lwlink_feature2D_s fp;

            network_read(&net0,recv_buffer,32);
            msg_length = lwlink_msg_pack(&link_handler,MSG_TYPE_RAW_IMAGE,bin3_img.data,bin3_img.cols * bin3_img.rows);		
            msg_length = network_write(&net0,link_handler.txbuf,msg_length);
            
            fp.feature_id = 0;
            fp.image_id   = 0;
            fp.pos_x      = prev_point.x;
            fp.pos_y      = prev_point.y;
            fp.vel_x      = next_point.x - prev_point.x;
            fp.vel_y      = next_point.y - prev_point.y;
            fp.quality    = flow_err;
            msg_length = lwlink_msg_pack(&link_handler,MSG_TYPE_FEATURE2D,&fp,sizeof(struct lwlink_feature2D_s));
            msg_length = network_write(&net0,link_handler.txbuf,msg_length);
            //printf("calc time = %f \r\n",calc_time * 1000.f);
            printf("flow = % 8.2f % 8.2f %  6.2f % 6.2f \r\n", fp.vel_x / dt,fp.vel_y / dt,calc_time * 1000.f,flow_err);
        }
#endif

#ifdef FLOW_USING_MSGQUE
        struct msgque_flow_info_s msg_info;

        if(flow_err < 1){
            flow_err = 0;
        }
        if(flow_err > 250.f){
            flow_err = 250.f;
        }
        msg_info.msg_type = 10;
        msg_info.vx = lowpassfilter2p(&flow_filter[0],(next_point.x - next_img.cols / 2) / dt);
        msg_info.vy = lowpassfilter2p(&flow_filter[1],(next_point.y - next_img.rows / 2) / dt);
        msg_info.version = 307;
        msg_info.time_loop = dt * 1000.f;
        msg_info.confidence = (uint8_t)flow_err;
        //msgque_send(&msg_info);
        if(counter % 5 == 0){
            printf("flow = % 8.2f % 8.2f % 6.2f \r\n", msg_info.vx,msg_info.vy,flow_err);
        }
#endif
    }
    camera_deinit();
    return NULL;
}

static int platform_create_thread(const char *name, int priority, int stack_size, void *(*entry) (void *), void* parameter, pthread_t *thid)
{
	int rv;
	pthread_t task;
	pthread_attr_t attr;
	struct sched_param param;

	rv = pthread_attr_init(&attr);
	if (rv != 0) {
		printf("%s(): failed to init thread attrs\n", __func__);
		return rv;
	}

	rv = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	if (rv != 0) {
		printf("%s(): failed to set inherit sched\n", __func__);
		return rv;
	}

	rv = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
	if (rv != 0) {
		printf("%s(): failed to set sched policy\n", __func__);
		return rv;
	}

	rv = pthread_attr_setstacksize(&attr, stack_size);
	if (rv != 0) {
		printf("%s(): failed to set stack size\n", __func__);
		return rv;
	}

	param.sched_priority = priority;
	rv = pthread_attr_setschedparam(&attr, &param);
	if (rv != 0) {
		printf("%s(): failed to set sched param\n", __func__);
		return rv;
	}

	rv = pthread_create(&task, &attr, entry, parameter);
	if (rv != 0) {
		if (rv == EPERM) {
			printf("%s(): warning: unable to start\n", __func__);
			rv = pthread_create(&task, NULL, entry, parameter);
			if (rv != 0) {
				printf("%s(): failed to create thread\n", __func__);
				return (rv < 0) ? rv : -rv;
			}
		}
		else {
			return rv;
		}
	}
    
    *thid = task;
	return 0;
}

void hal_linux_mlock(void)
{
	if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1){
		perror("mlockall failed");
		exit(-2);
	}
}

int main(int argc, char *argv[])
{
    int ret = 0;
    
	hal_linux_mlock();

	ret = platform_create_thread("glvio-vision", 60, STACK_SIZE, thread_flow, NULL, &thid1);
    if(ret != 0) {
        exit(-1);
    }
    
    pthread_join(thid1, NULL);
	return 0;
}
