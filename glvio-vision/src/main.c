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

#define STACK_SIZE                      (64*1024)
#define ARRAY_SIZE(arr)                 ( sizeof(arr) / sizeof((arr)[0]) )
#define IS_IN_RANGE(var, min, max)      ( ((min) <= (var)) && ((var) <= (max)) )

pthread_t thid1;

struct net_data_s net0;
struct lwlink_data_handler_s link_handler;

static void *thread_flow(void *arg)
{
	int i,j;
    int ret = 0;
	int msg_length = 0;
    uint8_t *image = NULL;
	uint8_t *image_ptr = NULL;
	char *recv_buffer[128];
	struct matrix_s raw_img;
	struct matrix_s s_img;

	struct point2i start = {.x = 270,.y = 190};
	struct point2i size = {.x = 100,.y = 100};

    ret = camera_init("/dev/video1", 640,480);
    if(ret < 0){
        exit(-1);
    }

	if(network_init(&net0,"192.168.0.21",3366) < 0){
		printf("Failed to init network\r\n");
	}

	lwlink_data_handler_init(&link_handler,0x02);
    matrix_create(&raw_img,640,480,1,IMAGE_TYPE_8U);
	matrix_create(&s_img,100,100,1,IMAGE_TYPE_8U);

    while(1) {
        //从摄像头获取一帧图像
        image = camera_get_image();  
		image_ptr = image;

		for(j = 0; j < raw_img.rows; j++){
			for(i = 0; i < raw_img.cols; i++){
				raw_img.data[j*raw_img.cols + i] =*image_ptr++;
			}
		}

		matrix_copy_aera(&raw_img,&s_img,&start,&size);
		msg_length = lwlink_msg_pack(&link_handler,MSG_TYPE_RAW_IMAGE,s_img.data,100*100);		
		if(network_read(&net0,recv_buffer,64) > 0){
			printf("data received\r\n");
		}

        printf("send a image:%d \r\n",network_write(&net0,link_handler.txbuf,msg_length));
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
