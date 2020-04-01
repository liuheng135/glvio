
#ifndef _LOWPASSFILTER2P_H_
#define _LOWPASSFILTER2P_H_

#include <math.h>

struct lpfilter_s{
	float cutoff_freq;
	float a_lp2p[3];
	float b_lp2p[3];
	float delay_element_1;		// buffered sample -1
	float delay_element_2;		// buffered sample -2
};

void set_cutoff_param(struct lpfilter_s *filter_param, float sample_freq, float cutoff_freq);

float lowpassfilter2p(struct lpfilter_s *filter_param, float rawdata);


#endif


