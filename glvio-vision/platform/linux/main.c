#include <pthread.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h> 
#include <sys/time.h>
#include <unistd.h>
#include <termios.h>
#include <sys/mman.h> 
#include "config.h"
#include "app_param.h" 
#include "app_imu.h"
#include "app_calibrator.h"
#include "app_flow.h"
#include "mathlib.h"
#include "app_debug.h"
#include "app_timer.h"
#include "app_events.h"
#include "app_vio.h"
#include "app_gllink.h"
#include "app_log.h"
#include "app_mavlink.h"
#include "app_anoclink.h"

#define MAIN_LOOP_TIME	(1.0f/MAIN_LOOP_HZ)
#define MAX_SAFE_STACK  512 * 1024  


void board_initialize(void);

int platform_create_thread(const char *name, int priority, int stack_size, void* entry,void* parameter)
{
    int rv;
    pthread_t task;
	pthread_attr_t attr;
	struct sched_param param;
	rv = pthread_attr_init(&attr);
    if (rv != 0) {
		printf("platform_create_thread: failed to init thread attrs");
		return rv;
	}
	rv = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	
	if (rv != 0) {
		printf("platform_create_thread: failed to set inherit sched");
		return rv;
	}

	rv = pthread_attr_setschedpolicy(&attr, SCHED_DEFAULT);

	if (rv != 0) {
		printf("platform_create_thread: failed to set sched policy");
		return rv;
	}
	rv = pthread_attr_setstacksize(&attr, stack_size);

	if (rv != 0) {
		printf("platform_create_thread: failed to set stack size");
		return rv;
	}
	param.sched_priority = priority;

	rv = pthread_attr_setschedparam(&attr, &param);

	if (rv != 0) {
		printf("platform_create_thread: failed to set sched priority");
		return rv;
	}
	rv = pthread_create(&task, &attr, entry,parameter);
	if (rv != 0) {
		if (rv == EPERM) {
			printf("warning: unable to start\n");
			rv = pthread_create(&task, NULL, entry, parameter);

			if (rv != 0) {
				printf("platform_create_thread: failed to create thread\n");
				return (rv < 0) ? rv : -rv;
			}
		}else{
			return rv;
		}
	}
	return 0;
}


void stack_prefault(void)
{  
	unsigned char dummy[MAX_SAFE_STACK];  

	memset(dummy, 0, MAX_SAFE_STACK);  
	return;  
} 


int thread_vio(void* paramter)
{
	struct timespec prev;
	float dt,run_time_dt;

	board_initialize();
	param_init();
    timer_init();
    app_events_init();
	imu_init();
	calibrator_init();
    lwlink_init();
    //mavlink_init();
    //anoclink_init();
	vio_init();
    //log_init();
	param_load();
	get_diff_time(&prev,true);
	usleep(2000);
	
	while(1){
		dt = get_diff_time(&prev,true);
		if(dt > (MAIN_LOOP_TIME + 0.001f)){
			printf("dt over:%f \r\n",dt);
		} 
		if(dt < 1.0f){
		    timer_update(dt);
		    app_events_update(dt);
			imu_update(dt);
			calibrator_update(dt);
            vio_update(dt);
            lwlink_update(dt);
            //mavlink_update(dt);
            //anoclink_update(dt);
			param_update(dt);
            //log_update(dt);
		}else{
			printf("over skip:%3.3f\n",dt);
		}

		run_time_dt = get_diff_time(&prev,false);
		if(run_time_dt > MAIN_LOOP_TIME){
			printf("runtime over:%3.3fms\n",run_time_dt*1000);
			run_time_dt = MAIN_LOOP_TIME - 0.0005f;
		}

		usleep((MAIN_LOOP_TIME - run_time_dt) * 1e6f);
	}
}

void arc_handle(int argc, char *argv[])
{
    if(argc > 1){
        if(strncmp(argv[1],"debug",5) == 0){
    		if(argc == 3){
    			msg_recoder_enable_debug(argv[2]);
    		}	
    	}
	}
}

#include "signal.h"

void signal_handler(int sig_no)
{
        int fd;
        int ret;
        char msg[2];

        fd = open("/mnt/signal_id.txt",O_CREAT | O_RDWR,0777);

        if(fd > -1){
                msg[0] = 0x30 + sig_no / 10;
                msg[1] = 0x30 + sig_no % 10;
                ret = write(fd,&msg,2);
                close(fd);
                ret = ret;
        }
        printf("signal %d got\r\n",sig_no);
        exit(0);
}

void signal_handler_init(void)
{
    signal(SIGINT,signal_handler);
    signal(SIGABRT,signal_handler);
    signal(SIGSEGV,signal_handler);
    signal(SIGSTOP,signal_handler);
    signal(SIGXCPU,signal_handler);
}


int main(int argc, char *argv[])
{
    pthread_t thread_vio_id;
    pthread_t thread_flow_id;

	if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {  
		perror("mlockall failed");  
		exit(-2);  
	}  
	stack_prefault();
	msg_recoder_init(); 
	arc_handle(argc,argv);
	signal_handler_init();

    platform_create_thread("vio",98,1024 * 256,thread_vio,&thread_vio_id);
    platform_create_thread("flow",5,1024 * 1024,thread_flow,&thread_flow_id);

    pthread_join(thread_flow_id,NULL);
    pthread_join(thread_vio_id,NULL);
    return 0;
}
