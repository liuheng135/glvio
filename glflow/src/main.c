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


#define STACK_SIZE                      (64*1024)
#define ARRAY_SIZE(arr)                 ( sizeof(arr) / sizeof((arr)[0]) )
#define IS_IN_RANGE(var, min, max)      ( ((min) <= (var)) && ((var) <= (max)) )

#define CAM0_IMAGE_WIDTH    640
#define CAM0_IMAGE_HEIGHT   480

#define FLOW_IMAGE_WIDTH    32
#define FLOW_IMAGE_HEIGHT   32

//#define FLOW_USING_LWLINK
#define FLOW_USING_MSGQUE

pthread_t thid1;

#ifdef FLOW_USING_LWLINK
struct net_data_s net0;
struct lwlink_data_handler_s link_handler;
#endif

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

unsigned char image_binning_buffer_0[FLOW_IMAGE_WIDTH*FLOW_IMAGE_HEIGHT * 64];
unsigned char image_binning_buffer_1[FLOW_IMAGE_WIDTH*FLOW_IMAGE_HEIGHT * 16];
unsigned char image_binning_buffer_2[FLOW_IMAGE_WIDTH*FLOW_IMAGE_HEIGHT * 4];
unsigned char image_binning_buffer_3[FLOW_IMAGE_WIDTH*FLOW_IMAGE_HEIGHT];

static void *thread_flow(void *arg)
{
    int i;
    int ret = 0;
    int counter = 0;
	int msg_length = 0;
    uint8_t *image = NULL;
    uint8_t *swap_ptr = NULL;

    float  dt;
    float  calc_time;
    double now;
    double last;
	
	struct matrix_s raw_img;
    struct matrix_s roi_img;
    struct matrix_s bin1_img;
	struct matrix_s bin2_img;
    struct matrix_s bin3_img;
    struct matrix_s prev_img;
    struct matrix_s next_img;

	struct point2i roi_start = {
            .x = (CAM0_IMAGE_WIDTH  - FLOW_IMAGE_WIDTH * 8) / 2,
            .y = (CAM0_IMAGE_HEIGHT - FLOW_IMAGE_HEIGHT * 8)  /2,};

	struct size2i  roi_size = {.x = FLOW_IMAGE_WIDTH * 8, .y = FLOW_IMAGE_HEIGHT * 8};
    struct size2i  opt_win_size = {.x = 17,.y = 17}; 

    struct point2f prev_point;
    struct point2f next_point;
    float          flow_err;

    ret = camera_init("/dev/video1",CAM0_IMAGE_WIDTH,CAM0_IMAGE_HEIGHT);
    if(ret < 0){
        exit(-1);
    }

    
    image = camera_get_image(); 
    matrix_init(&raw_img,CAM0_IMAGE_WIDTH,CAM0_IMAGE_HEIGHT,1,IMAGE_TYPE_8U,image);
    matrix_init(&roi_img,FLOW_IMAGE_WIDTH * 8,FLOW_IMAGE_HEIGHT * 8,1,IMAGE_TYPE_8U,image_binning_buffer_0);
    matrix_init(&bin1_img,FLOW_IMAGE_WIDTH * 4,FLOW_IMAGE_HEIGHT * 4,1,IMAGE_TYPE_8U,image_binning_buffer_1);
    matrix_init(&bin2_img,FLOW_IMAGE_WIDTH * 2,FLOW_IMAGE_HEIGHT * 2,1,IMAGE_TYPE_8U,image_binning_buffer_2);
    matrix_init(&bin3_img,FLOW_IMAGE_WIDTH,FLOW_IMAGE_HEIGHT,1,IMAGE_TYPE_8U,image_binning_buffer_3);
    matrix_init(&prev_img,FLOW_IMAGE_WIDTH,FLOW_IMAGE_HEIGHT,1,IMAGE_TYPE_32S,(unsigned char *)image_buffer_a);
    matrix_init(&next_img,FLOW_IMAGE_WIDTH,FLOW_IMAGE_HEIGHT,1,IMAGE_TYPE_32S,(unsigned char *)image_buffer_b);
    

    printf("roi start = %d, %d   size = %d, %d  \r\n",roi_start.x,roi_start.y,roi_size.x,roi_size.y); 
    matrix_copy_aera(&raw_img,&roi_img,&roi_start,&roi_size);
    matrix_binning(&roi_img,&bin1_img);
    matrix_binning(&bin1_img,&bin2_img);
    matrix_binning(&bin2_img,&bin3_img);
    if(matrix_convert_type(&bin3_img,&prev_img) != 0 ){
        printf("unsupport image type\r\n");
    }

#ifdef FLOW_USING_LWLINK
    if(network_init(&net0,"192.168.0.20",3366) < 0){
		printf("Failed to init network\r\n");
	}
	lwlink_data_handler_init(&link_handler,0x02);
#endif

#ifdef FLOW_USING_MSGQUE
    msgque_init();
#endif

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
        matrix_binning(&roi_img,&bin1_img);
        matrix_binning(&bin1_img,&bin2_img);
        matrix_binning(&bin2_img,&bin3_img);
        if(matrix_convert_type(&bin3_img,&next_img) != 0 ){
            printf("unsupport image type\r\n");
            continue;
        }

        prev_point.x = FLOW_IMAGE_WIDTH  / 2;
        prev_point.y = FLOW_IMAGE_HEIGHT / 2;

        optflow_lk_calc(&optflow0,&prev_img,&next_img,&prev_point,&next_point,&flow_err);
        swap_ptr = next_img.data;
        next_img.data = prev_img.data;
        prev_img.data = swap_ptr;

        counter++;

        now = get_time_now();
        calc_time = (float)(now - last);

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
        msg_info.vx = (next_point.x - prev_point.x) / dt;
        msg_info.vy = (next_point.y - prev_point.y) / dt;
        msg_info.version = 300;
        msg_info.time_loop = dt * 1000.f;
        msg_info.confidence = (uint8_t)flow_err;
        msgque_send(&msg_info);

        //printf("flow = % 8.2f,% 8.2f % 7.2f % 8.2f\r\n",msg_info.vx,msg_info.vy,msg_info.time_loop,flow_err);
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
