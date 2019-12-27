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

#define STACK_SIZE                      (64*1024)
#define ARRAY_SIZE(arr)                 ( sizeof(arr) / sizeof((arr)[0]) )
#define IS_IN_RANGE(var, min, max)      ( ((min) <= (var)) && ((var) <= (max)) )

#define CAM0_IMAGE_WIDTH    640
#define CAM0_IMAGE_HEIGHT   480

#define FLOW_IMAGE_WIDTH    64
#define FLOW_IMAGE_HEIGHT   64

#define FLOW_USING_LWLINK


pthread_t thid1;

#ifdef FLOW_USING_LWLINK
struct net_data_s net0;
struct lwlink_data_handler_s link_handler;
#endif

struct optflow_lk  optflow0;


int image_buffer_a[FLOW_IMAGE_WIDTH*FLOW_IMAGE_HEIGHT];
int image_buffer_b[FLOW_IMAGE_WIDTH*FLOW_IMAGE_HEIGHT];
unsigned char image_buffer_c[FLOW_IMAGE_WIDTH*FLOW_IMAGE_HEIGHT];
unsigned char image_buffer_d[FLOW_IMAGE_WIDTH*FLOW_IMAGE_HEIGHT*4];

static void *thread_flow(void *arg)
{
    int ret = 0;
    int counter = 0;
	int msg_length = 0;
    uint8_t *image = NULL;
    uint8_t *swap_ptr = NULL;
	
	struct matrix_s raw_img;
    struct matrix_s bin_img;
	struct matrix_s roi_img;
    struct matrix_s prev_img;
    struct matrix_s next_img;

	struct point2i bin_start = {
            .x = CAM0_IMAGE_WIDTH /2 - FLOW_IMAGE_WIDTH,
            .y = CAM0_IMAGE_HEIGHT / 2 - FLOW_IMAGE_HEIGHT,};

	struct size2i  bin_size = {.x = FLOW_IMAGE_WIDTH * 2, .y = FLOW_IMAGE_HEIGHT * 2};
    struct size2i  opt_win_size = {.x = 17,.y = 17}; 

    struct point2f prev_point = {.x = FLOW_IMAGE_WIDTH / 2, .y = FLOW_IMAGE_HEIGHT / 2};
    struct point2f next_point = {.x = 0, .y = 0};
    float  flow_err;

    ret = camera_init("/dev/video1",CAM0_IMAGE_WIDTH,CAM0_IMAGE_HEIGHT);
    if(ret < 0){
        exit(-1);
    }

    image = camera_get_image(); 
    matrix_init(&raw_img,CAM0_IMAGE_WIDTH,CAM0_IMAGE_HEIGHT,1,IMAGE_TYPE_8U,image);
    matrix_init(&bin_img,FLOW_IMAGE_WIDTH * 2,FLOW_IMAGE_HEIGHT * 2,1,IMAGE_TYPE_8U,image_buffer_d);
    matrix_init(&prev_img,FLOW_IMAGE_WIDTH,FLOW_IMAGE_HEIGHT,1,IMAGE_TYPE_32S,(unsigned char *)image_buffer_a);
    matrix_init(&next_img,FLOW_IMAGE_WIDTH,FLOW_IMAGE_HEIGHT,1,IMAGE_TYPE_32S,(unsigned char *)image_buffer_b);
    matrix_init(&roi_img,FLOW_IMAGE_WIDTH,FLOW_IMAGE_HEIGHT,1,IMAGE_TYPE_8U,(unsigned char *)image_buffer_c);

    matrix_copy_aera(&raw_img,&bin_img,&bin_start,&bin_size);
    matrix_binning(&bin_img,&roi_img);
    if(matrix_convert_type(&roi_img,&prev_img) != 0 ){
        printf("unsupport image type\r\n");
    }

#ifdef FLOW_USING_LWLINK
    if(network_init(&net0,"192.168.0.20",3366) < 0){
		printf("Failed to init network\r\n");
	}
	lwlink_data_handler_init(&link_handler,0x02);
#endif

    optflow_lk_create(&optflow0,1,5.0f,0.5f,&opt_win_size);

    while(1) {
        //从摄像头获取一帧图像
        image = camera_get_image();  
		matrix_init(&raw_img,CAM0_IMAGE_WIDTH,CAM0_IMAGE_HEIGHT,1,IMAGE_TYPE_8U,image);
        matrix_copy_aera(&raw_img,&bin_img,&bin_start,&bin_size); 
        matrix_binning(&bin_img,&roi_img);
        if(matrix_convert_type(&roi_img,&next_img) != 0 ){
            printf("unsupport image type\r\n");
            continue;
        }
        optflow_lk_calc(&optflow0,&prev_img,&next_img,&prev_point,&next_point,&flow_err);
        swap_ptr = next_img.data;
        next_img.data = prev_img.data;
        prev_img.data = swap_ptr;

        counter++;

#ifdef FLOW_USING_LWLINK
        if(counter % 3 == 0){
            char *recv_buffer[128];
            struct lwlink_feature2D_s fp1;

            network_read(&net0,recv_buffer,32);
            msg_length = lwlink_msg_pack(&link_handler,MSG_TYPE_RAW_IMAGE,roi_img.data,roi_img.cols * roi_img.rows);		
            msg_length = network_write(&net0,link_handler.txbuf,msg_length);
            
            fp1.feature_id = 0;
            fp1.image_id   = 0;
            fp1.pos_x      = FLOW_IMAGE_WIDTH / 2;
            fp1.pos_y      = FLOW_IMAGE_HEIGHT / 2;
            fp1.vel_x      = next_point.x - prev_point.x;
            fp1.vel_y      = next_point.y - prev_point.y;
            fp1.quality    = flow_err;
            msg_length = lwlink_msg_pack(&link_handler,MSG_TYPE_FEATURE2D,&fp1,sizeof(fp1));
            msg_length = network_write(&net0,link_handler.txbuf,msg_length); 
        }
#endif

#ifdef FLOW_USING_MSGQUE


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
