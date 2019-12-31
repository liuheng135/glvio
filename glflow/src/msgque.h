#ifndef _MSGQUE_H_
#define _MSGQUE_H_

#include <stdint.h>

struct msgque_flow_info_s {
	long		msg_type; 
	uint16_t	version; 
	float		vx; 
	float		vy;    
	uint8_t		confidence;    
    uint8_t		image_brightness;  
    uint8_t		image_score;    
    int8_t		image_grade; 
    float		time_loop;
    float		time_run;
	int			frame;
} ;

int msgque_init(void);
int msgque_send(struct msgque_flow_info_s *msg);


#endif