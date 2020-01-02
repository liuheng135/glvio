#ifndef __CAMERA_H__
#define __CAMERA_H__

#include <stdint.h>
#include <pthread.h>
#include <linux/videodev2.h>

#define FPS 50
#define BUFFER_CNT                  4   //»º³åÇøÊýÁ¿


struct camera_parameter{
	int ae;
	int awb;
	int brightness;
	int contrast;
};

struct photograph_parameter {
	unsigned int width;
	unsigned int height;
	unsigned int fps;
	unsigned int format;
};

struct camera_data{
	int id;
	int fd;
	int running_state;
	unsigned int width;
	unsigned int height;
	unsigned int fps;
	unsigned int format;
	unsigned int mode;
	unsigned int buf_len;
	void *virtual_buf[BUFFER_CNT];
	struct v4l2_buffer phy_buf;
	struct camera_parameter sys_param;
	struct photograph_parameter photograph;
	struct VideoOpr *opr;
	pthread_mutex_t mutex;
	pthread_cond_t frm_update;
	struct camera_data *ptNext;
};

int      camera_init(const char *dev_name, int width, int height);
uint8_t *camera_get_image(void);
void     camera_deinit(void);

#endif
