#include <stdint.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include "ahrs.h"
#include "app_log.h"
#include "app_imu.h"
#include "app_vio.h"
#include "app_timer.h"
#include "config.h"
#include "mathlib.h"
#include "app_debug.h"
#include "quaternion.h"

#define LOG_FD_PATH_VAL LOG_PATH_ROOT"log.bin"

int platform_create_thread(const char *name, int priority, int stack_size, void* entry,void* parameter);

float log_timer;
int log_msg_id;

int log_recoder_init(struct log_recoder_s *recoder, char *mem, int size)
{
	ring_buffer_init(&recoder->rbuffer,mem,size);	
	pthread_mutex_init (&recoder->rbuffer_mutex, NULL);
	recoder->writed_size = 0;
	recoder->file_size_limitation = 100 * 1024 *1024;
	return 0;
}

int log_recoder_write(struct log_recoder_s *recoder,char *data,int size)
{
    int ret = 0;

	if((recoder->writed_size + size) > recoder->file_size_limitation){
		return 0;
	}
	
	if(pthread_mutex_trylock(&recoder->rbuffer_mutex) == 0){
		ret = ring_buffer_put(&recoder->rbuffer,data,size);
		recoder->writed_size += ret;
		pthread_mutex_unlock(&recoder->rbuffer_mutex);
	}
	return ret;
}

void log_recoder_clean(struct log_recoder_s *recoder)
{
    if(pthread_mutex_trylock(&recoder->rbuffer_mutex) == 0){
		recoder->rbuffer.head = 0;
        recoder->rbuffer.tail = 0;
        recoder->writed_size = 0;
        pthread_mutex_unlock(&recoder->rbuffer_mutex);
	}
}

int log_recoder_read(struct log_recoder_s *recoder,char *data,int size){
	int ret = 0;
	if(pthread_mutex_trylock(&recoder->rbuffer_mutex) == 0){
		ret = ring_buffer_get(&recoder->rbuffer,data,size);
		pthread_mutex_unlock(&recoder->rbuffer_mutex);
	}
	return ret;
}

static char log_buffer_mem[LOG_BUF_SIZE + 100];
struct log_recoder_s log_recoder;

void log_imu(bool armed)
{
	struct acc_data_s acc;
	struct gyro_data_s gyro;

	struct log_IMU_s pkt = {
		LOG_PACKET_HEADER_INIT(LOG_IMU_MSG),
	};

	imu_get_acc(&acc);
	imu_get_gyro(&gyro);
	
	pkt.time = log_timer;
	pkt.acc_x = acc.raw[0];
	pkt.acc_y = acc.raw[1];
	pkt.acc_z = acc.raw[2];
	pkt.gyro_x = gyro.raw[0];
	pkt.gyro_y = gyro.raw[1];
	pkt.gyro_z = gyro.raw[2];
	pkt.gyro_xb = gyro.calibed[0];
	pkt.gyro_yb = gyro.calibed[1];
	pkt.gyro_zb = gyro.calibed[2];
	pkt.temp = acc.temp;

	if(armed){
	    log_recoder_write(&log_recoder,(char *)&pkt,sizeof(pkt));
	}
}

void log_vio(bool armed)
{
	struct log_VIO_s pkt = {
		LOG_PACKET_HEADER_INIT(LOG_VIO_MSG),
	};
	struct vio_data_s vio_data;
    struct eulur_s er;

	vio_get_data(&vio_data);
    quater_to_eulur(&er,&vio_data.rotation);
	
	pkt.mp1_vel_x      = vio_data.mp1_vel[0];
	pkt.mp1_vel_y      = vio_data.mp1_vel[1];
	pkt.mp2_vel_x      = vio_data.mp2_vel[0];
	pkt.mp2_vel_y      = vio_data.mp2_vel[1];
	pkt.rotation_roll  = er.roll;
	pkt.rotation_pitch = er.pitch;
	pkt.rotation_yaw   = er.yaw;
	pkt.translation_x  = vio_data.translation[0];
	pkt.translation_y  = vio_data.translation[1];
	pkt.translation_z  = vio_data.translation[2];

	if(armed){
		log_recoder_write(&log_recoder,(char *)&pkt,sizeof(pkt));
	}
}

void log_msg(uint8_t level,uint8_t * buf)
{
	struct log_MSG_s pkt = {
		LOG_PACKET_HEADER_INIT(LOG_MSG_MSG),
	};
	
	pkt.time	= log_timer;
	pkt.level	= level;

	snprintf((char *)pkt.msg,64,"%s",(char *)buf);

	log_recoder_write(&log_recoder,(char *)&pkt,sizeof(pkt));
}

void log_write_formats()
{
	uint8_t i;
	struct {
		LOG_PACKET_HEADER
		struct log_format_s body;
	} log_msg_format = {
		LOG_PACKET_HEADER_INIT(LOG_FORMAT_MSG),
	};

    log_recoder_clean(&log_recoder);
	/* fill message format packet for each format and write it */
	for (i = 0; i < log_formats_num; i++) {
		log_msg_format.body = log_formats[i];
		log_recoder_write(&log_recoder,(char *)&log_msg_format,sizeof(log_msg_format));
	}
}

void log_write_version()
{
	char version[10] = "2.4.3";

	struct log_VER_s pkt = {
		LOG_PACKET_HEADER_INIT(LOG_VER_MSG),
	};

	/* fill version message and write it */
	strncpy(pkt.fw_git,version,sizeof(version));
	log_recoder_write(&log_recoder,(char *)&pkt,sizeof(pkt));
}

void log_write_thread(void *param)
{
	char write_buffer[2560];
	int read_size;
	int sync_cnt;

	log_recoder.fd = -1;
    sync_cnt = 0;

	while(1){
		if(log_recoder.fd < 0){
		    log_recoder.fd = open(LOG_FD_PATH_VAL,O_CREAT | O_RDWR | O_TRUNC,0777);
        	if(log_recoder.fd > 0){
        		msg_recoder_log(log_msg_id,"init log file:%s ok,fd=%d\r\n",
        		    LOG_FD_PATH_VAL,log_recoder.fd);

        	    log_write_formats();
            	log_write_version();
        	}else{
        	    sleep(2);
        	}
		}else{
			read_size = log_recoder_read(&log_recoder,write_buffer,2500);
			if(read_size > 0){
				if(write(log_recoder.fd,write_buffer,read_size) < 0){
                    //close(log_recoder.fd);
                    //log_recoder.fd = -1;
                    //sync();
                    //msg_recoder_log(log_msg_id,"failed to write log data,reinit log file\r\n");
				}
			}
			usleep(20000);
			sync_cnt++;
            if(sync_cnt > 50){
                sync_cnt = 0;
                fsync(log_recoder.fd);
            }
			
		}
	}
}

void log_init(void)
{
    log_msg_id = msg_recoder_create("LOG",1.0f);
	msg_recoder_log(log_msg_id,"started\r\n");

	log_recoder_init(&log_recoder,log_buffer_mem,LOG_BUF_SIZE);
	platform_create_thread("pilot-log",1,1024 * 64,log_write_thread,NULL);
}

void log_update(float dt)
{
	static float time_400hz = 0;
	static float time_200hz = 0;
	static float time_50hz = 0;
	static float time_20hz = 0;
	static float time_10hz = 0;
	static float time_5hz = 0;
	static float time_1hz = 0;

	time_400hz += dt;
	time_200hz += dt;
	time_50hz += dt;
	time_20hz += dt;
	time_10hz += dt;
	time_5hz += dt;
	time_1hz += dt;

    log_timer = timer_get_time();
	if(time_400hz >= (1.0f/400)){
		time_400hz = 0;
	}
	
	if(time_200hz >= (1.0f/200)){
		time_200hz = 0;
	}
	
	if(time_50hz >= (1.0f/50)){
		time_50hz = 0;
		log_vio(true);
		log_imu(true);
	}

	if(time_20hz >= (1.0f/20)){
		time_20hz = 0;
	}
	
	if(time_10hz >= (1.0f/10)){
		time_10hz = 0;
	}

	if(time_5hz >= (1.0f/5)){
		time_5hz = 0;
	}

	if(time_1hz >= (1.0f/1)){
		time_1hz = 0;
	}
}



