#ifndef _APP_LOG_H_
#define _APP_LOG_H_


#include "ringbuffer.h"
#include "pthread.h"

#define PACKED __attribute__((__packed__))

#define LOG_PACKET_HEADER_LEN	   3
#define LOG_PACKET_HEADER	       uint8_t head1, head2, msg_type;
#define LOG_PACKET_HEADER_INIT(id) .head1 = HEAD_BYTE1, .head2 = HEAD_BYTE2, .msg_type = id

#define HEAD_BYTE1  0xA3    // Decimal 163
#define HEAD_BYTE2  0x95    // Decimal 149

#define LOG_CTRL_MSG	1
#define LOG_ATT_MSG		2
#define LOG_RCI_MSG		3
#define LOG_RCO_MSG		4
#define LOG_IMU_MSG		5
#define LOG_PID_MSG		6
#define LOG_BARO_MSG	7
#define LOG_SYS_MSG		8
#define LOG_NAV_MSG		9
#define LOG_FM_MSG		10
#define LOG_FMD_MSG		11
#define LOG_GPS_MSG		12
#define LOG_BAT_MSG		13
#define LOG_VIO_MSG	    14
#define LOG_MSG_MSG		15

#define LOG_FORMAT_MSG	0x80
#define LOG_VER_MSG		0x82

struct log_recoder_s{
	int fd;
	int writed_size;
	int file_size_limitation;
    struct ring_buffer_s rbuffer;	
	pthread_mutex_t rbuffer_mutex;
};

struct PACKED log_GPS_s {
	LOG_PACKET_HEADER
	float time;
	double lat;
	double lon;
    double alt;
	float eph;
    float vel_d_m_s;
	float vel_n_m_s;
	float vel_e_m_s;
	float cog;
	uint8_t satellites_used;
	uint8_t fix_type;
};

struct PACKED log_SYS_s {
	LOG_PACKET_HEADER
	float time;
	uint8_t mode;
	uint8_t fs_mode;
	float heart;
	uint8_t m_v[4];
	int m_err;
	float c_val;
};

struct PACKED log_IMU_s {
	LOG_PACKET_HEADER
	float time;
	float acc_x;
	float acc_y;
	float acc_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float mag_x;
	float mag_y;
	float mag_z;
	float gyro_xb;
	float gyro_yb;
	float gyro_zb;
	float temp;
};

struct PACKED log_BARO_s {
	LOG_PACKET_HEADER
	float time;
	float temp;
	float press;
	float alt;
	float vel;
};

struct PACKED log_RCO_s {
	LOG_PACKET_HEADER
	float time;
	float roll_out;
	float pitch_out;
	float yaw_out;
	float thr_out;
	float rc1;
	float rc2;
	float rc3;
	float rc4;
};

struct PACKED log_RCI_s {
	LOG_PACKET_HEADER
	float time;
	float rc1;
	float rc2;
	float rc3;
	float rc4;
	float rc5;
	float rc_raw1;
	float rc_raw2;
	float rc_raw3;
	float rc_raw4;
};

struct PACKED log_ATT_s {
	LOG_PACKET_HEADER
	float time;
	float roll;
	float pitch;
	float yaw;
	float rollrate;
	float pitchrate;
	float yawrate;
	float roll_acc;
	float pitch_acc;
	float yaw_mag;
};

struct PACKED log_PID_s {
	LOG_PACKET_HEADER
	float time;
	float error;
	float p;
	float i;
	float d;
	float all;
};

struct PACKED log_FM_s {
	LOG_PACKET_HEADER
	float time;
	float phone_x;
	float phone_y;
	float dis2phone;
	float dis2sp;
	float spx;
	float spy;
	float vel;
	float diff_x;
	float diff_y;
};

struct PACKED log_FMD_s {
	LOG_PACKET_HEADER
	float time;
	float mode;
	float distance;
	float target_x;
	float target_y;
	float target_roll;
	float target_pitch;
	float target_yaw;
};

struct PACKED log_NAV_s {
	LOG_PACKET_HEADER
	float pos_est_x;
	float pos_est_y;
	float pos_est_z;
	float vel_est_x;
	float vel_est_y;
	float vel_est_z;
	float pos_gps_x;
	float pos_gps_y;
	float pos_gps_z;
	float acc_ned_x;
	float acc_ned_y;
	float acc_ned_z;
	float acc_bias_x;
	float acc_bias_y;
    float acc_bias_z;
};

struct PACKED log_CTRL_s {
	LOG_PACKET_HEADER
	float pdx;
	float pdy;
	float pdz;
	float vdx;
	float vdy;
	float vdz;
	float rd;
	float pd;
	float yd;
	float rrd;
	float prd;
	float yrd;
	uint8_t control_mode;
};


struct PACKED log_VER_s {
	LOG_PACKET_HEADER
	char fw_git[64];
};


struct PACKED log_BAT_s {
	LOG_PACKET_HEADER
	float time;
	int raw_adc_val;
	int raw_bat_val;
	float vol_raw;
	float vol_filterd;
	uint8_t cap;
};

struct PACKED log_VIO_s {
	LOG_PACKET_HEADER
	float time;
	float mp1_vel_x;
	float mp1_vel_y;
	float mp2_vel_x;
	float mp2_vel_y;
	float rotation_roll;
	float rotation_pitch;
	float rotation_yaw;
	float translation_x;
	float translation_y;
	float translation_z;
	float reserve1;
	float reserve2;
};

struct PACKED log_MSG_s {
	LOG_PACKET_HEADER
	float time;
	uint8_t level;
	uint8_t msg[64];
};

struct PACKED log_format_s {
	uint8_t type;
	uint8_t length;
	char name[4];
	char format[16];
	char labels[64];
};

#define LOG_FORMAT_S(_name, _struct_name, _format, _labels) { \
	.type = LOG_##_name##_MSG, \
	.length = sizeof(struct log_##_struct_name##_s), \
	.name = #_name, \
	.format = _format, \
	.labels = _labels \
}

/*
Format characters in the format string for binary log messages
  b   : int8_t
  B   : uint8_t
  h   : int16_t
  H   : uint16_t
  i   : int32_t
  I   : uint32_t
  f   : float
  d   : double
  Z   : char[64]
  M   : uint8_t flight mode
  q   : int64_t
  Q   : uint64_t
 */

static const struct log_format_s log_formats[] = {
	LOG_FORMAT_S(VER,VER,"Z","Version"),
	LOG_FORMAT_S(CTRL,CTRL,"ffffffffffffB","pdx,pdy,pdz,vdx,vdy,vdz,rd,pd,yd,rrd,prd,yrd,md"),
	LOG_FORMAT_S(ATT,ATT,"ffffffffff","T,r,p,y,rr,pr,yr,ra,pa,ym"),
	LOG_FORMAT_S(RCI,RCI,"ffffffffff","T,rc1,rc2,rc3,rc4,rc5,rw1,rw2,rw3,rw4"),
	LOG_FORMAT_S(RCO,RCO,"fffffffff","T,r,p,y,thr,rc1,rc2,rc3,rc4"),
	LOG_FORMAT_S(IMU,IMU,"ffffffffffffff","T,ax,ay,az,gx,gy,gz,mx,my,mz,gbx,gby,gbz,temp"),
	LOG_FORMAT_S(PID,PID,"ffffff","T,error,p,i,d,all"),
	LOG_FORMAT_S(BARO,BARO,"fffff","T,temp,press,alt,vel"),
	LOG_FORMAT_S(SYS,SYS,"fBBfBBBBif","T,mode,fs,heart,mv1,mv2,mv3,mv4,m_err,c_val"),
    LOG_FORMAT_S(NAV,NAV,"fffffffffffffff","pcx,pcy,pcz,vcx,vcy,vcz,pgx,pgy,pgz,anx,any,anz,abx,aby,abz"),
	LOG_FORMAT_S(FM,FM,"ffffffffff","T,phx,phy,dph,dsp,spx,spy,vel,dx,dy"),
	LOG_FORMAT_S(FMD,FMD,"ffffffff","T,mode,dis,tx,ty,tr,tp,tya"),
    LOG_FORMAT_S(GPS,GPS,"fdddfffffBB","T,lat,lon,alt,eph,vd,vn,ve,cog,num,fix"),
	LOG_FORMAT_S(BAT,BAT,"fiiffB","T,r_adc,r_bat,rvol,fvol,cap"),
	LOG_FORMAT_S(VIO,VIO,"fffffffffffff","T,v1x,v1y,v2x,v2y,rr,rp,ry,tx,ty,tz,r1,r2"),
	LOG_FORMAT_S(MSG,MSG,"fbZ","T,level,msg"),
};

static const unsigned log_formats_num = sizeof(log_formats) / sizeof(log_formats[0]);


int log_recoder_init(struct log_recoder_s *recoder,char *mem,int size);
int log_recoder_write(struct log_recoder_s *recoder,char *data,int size);
int log_recoder_read(struct log_recoder_s *recoder,char *data,int size);

void log_init(void);
void log_update(float dt);
void log_print(uint8_t file_num);
void log_msg(uint8_t level,uint8_t * buf);
int log_thread_main(void* paramter);

#endif

