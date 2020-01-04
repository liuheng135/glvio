#ifndef _LWLINK_H
#define _LWLINK_H

#ifdef __cplusplus 
extern "C" {
#endif

#include "stdint.h"

#define PACKED __attribute__((__packed__))

#define MSG_TYPE_RAW_IMAGE   0x02
#define MSG_TYPE_ATTITUDE    0x22
#define MSG_TYPE_FEATURE2D   0x25

#define MSG_HEAD1  0x01
#define MSG_HEAD2  0xFE
#define MSG_END1   0x02
#define MSG_END2   0xFD
#define MSG_LENGTH_MAX  20480


enum handler_status_e{
	HANDLER_STATUS_WHEAD1= 0,
    HANDLER_STATUS_WHEAD2,
	HANDLER_STATUS_DATA,
    HANDLER_STATUS_WEND2,
};

struct lwlink_data_handler_s{
    uint8_t id;
    uint8_t txbuf[MSG_LENGTH_MAX+10];
	uint8_t rxbuf[MSG_LENGTH_MAX+10];
    uint16_t rxbuf_ptr;
    uint16_t msg_length;
    uint8_t heartbeat_count;
	float heartbeat_rate;
	enum handler_status_e status;
};

struct lwlink_msg_s{
    uint8_t  head1;
    uint8_t  head2;
    uint8_t  id;
    uint8_t  type;
    uint16_t len;
    uint8_t  checksum;
    uint8_t  *buf;
    uint8_t  *end1;
    uint8_t  *end2;
};

struct lwlink_feature2D_s{
    uint8_t  image_id;
    uint16_t feature_id;
    float    pos_x;
    float    pos_y;
    float    vel_x;
    float    vel_y;
    float    quality;
};

struct lwlink_attitude_s{
    uint8_t  component_id;
    float    roll;
    float    pitch;
    float    yaw;
    float    roll_speed;
    float    pitch_speed;
    float    yaw_speed;
};

int      lwlink_msg_pack(struct lwlink_data_handler_s *handler,uint8_t type,uint8_t *data,uint32_t len);
int      lwlink_data_handler_init(struct lwlink_data_handler_s *handler,uint8_t id);
int      lwlink_data_handler_parse(struct lwlink_data_handler_s *handler,uint8_t ch);
uint8_t  lwlink_data_handler_get_type(struct lwlink_data_handler_s *handler);
uint8_t  lwlink_data_handler_get_id(struct lwlink_data_handler_s *handler);
uint8_t  lwlink_data_handler_get_length(struct lwlink_data_handler_s *handler);
uint8_t* lwlink_data_handler_get_data(struct lwlink_data_handler_s *handler);

#ifdef __cplusplus 
}
#endif

#endif
