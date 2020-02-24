#ifndef __ANOC_LINK_H__
#define __ANOC_LINK_H__

#include <stdint.h>

#define PACKED __attribute__((__packed__))

#define ANOC_SWAP16(dat) (((dat << 8) & 0xff00) | ((dat >> 8) & 0x00ff))
#define ANOC_SWAP32(dat) (((dat << 24) & 0xff000000) | ((dat << 8)  & 0x00ff0000) | ((dat >> 8) & 0x0000ff00) | ((dat >> 24) & 0x000000ff))

#define ANOC_HEAD             0xAA
#define ANOC_DIR_FROM_PC      0xAF
#define ANOC_DIR_FROM_PILOT   0xAA


#define ANOC_MSG_TYPE_VERSION  0x00
#define ANOC_MSG_TYPE_STATUS   0x01
#define ANOC_MSG_TYPE_SENSOR   0x02
#define ANOC_MSG_TYPE_MANUAL   0x03
#define ANOC_MSG_TYPE_GPS      0x04
#define ANOC_MSG_TYPE_POWER    0x05
#define ANOC_MSG_TYPE_MOTOR    0x06
#define ANOC_MSG_TYPE_SENSOR2  0x07

#define ANOC_MSG_TYPE_FMODE    0x0A
#define ANOC_MSG_TYPE_PID1     0x10
#define ANOC_MSG_TYPE_PID2     0x11
#define ANOC_MSG_TYPE_PID3     0x12
#define ANOC_MSG_TYPE_PID4     0x13
#define ANOC_MSG_TYPE_PID5     0x14
#define ANOC_MSG_TYPE_PID6     0x15

#define ANOC_MSG_TYPE_LPOS     0x32
#define ANOC_MSG_TYPE_VERSION  0x00
#define ANOC_MSG_TYPE_STATUS   0x01

#define ANOC_MSG_LENGTH_MAX    128

struct PACKED anoc_msg_s{
    uint8_t head;
    uint8_t direction;
    uint8_t type;
    uint8_t length;
    uint8_t checksum;
    uint8_t *data;
};

struct PACKED anoc_ver_s{
    uint8_t  hardware_type;
    uint16_t hardware_version;
    uint16_t software_version;
    uint16_t potocal_version;
    uint16_t bootloader_version;
};

struct PACKED anoc_status_s{
    int16_t roll;            /* degrees * 100 */
    int16_t pitch;           /* degrees * 100 */
    int16_t yaw;             /* degrees * 100 */
    int32_t altitude;        /* centimeter */   
    uint8_t  flight_mode;
    uint8_t  armed;
};

struct PACKED anoc_sensor_data_s{
    uint16_t acc[3];
    uint16_t gyro[3];
    uint16_t mag[3];
};

struct PACKED anoc_manual_command_s{
    uint16_t throttle;
    uint16_t yaw;
    uint16_t roll;
    uint16_t pitch;
    uint16_t aux[6];
};

struct PACKED anoc_gps_info_s{
    uint8_t  fixed_type;
    uint8_t  satelite_num;
    int32_t  longitude;      /*degrees * 10000000*/
    int32_t  latitude;       /*degrees * 10000000*/
    uint16_t direction;
};

struct PACKED anoc_local_pos_s{
    uint16_t x;
    uint16_t y;
    uint16_t z;
};


enum anoc_handler_status_e{
	ANOC_HANDLER_STATUS_WHEAD1= 0,
    ANOC_HANDLER_STATUS_WHEAD2,
	ANOC_HANDLER_STATUS_DATA,
    ANOC_HANDLER_STATUS_WEND2,
};

struct anoc_data_handler_s{
    uint8_t  txbuf[ANOC_MSG_LENGTH_MAX+10];
	uint8_t  rxbuf[ANOC_MSG_LENGTH_MAX+10];
    uint16_t rxbuf_ptr;
    uint16_t msg_length;
    uint8_t  heartbeat_count;
	float    heartbeat_rate;
	enum anoc_handler_status_e status;
};


uint16_t anoc_swap_data_u16(uint16_t dat);
int16_t  anoc_swap_data_s16(int16_t dat);
uint32_t anoc_swap_data_u32(uint32_t dat);
int32_t  anoc_swap_data_s32(int32_t dat);

int      anoc_msg_pack(struct anoc_data_handler_s *handler,uint8_t type,uint8_t *data,uint32_t len);
int      anoc_data_handler_init(struct anoc_data_handler_s *handler);
int      anoc_data_handler_parse(struct anoc_data_handler_s *handler,uint8_t ch);
uint8_t  anoc_data_handler_get_type(struct anoc_data_handler_s *handler);
uint8_t  anoc_data_handler_get_id(struct anoc_data_handler_s *handler);
uint8_t  anoc_data_handler_get_length(struct anoc_data_handler_s *handler);
uint8_t* anoc_data_handler_get_data(struct anoc_data_handler_s *handler);


#endif
