#ifndef _APP_FLOW_H_
#define _APP_FLOW_H_

#include "geometry.h"

#define CAM0_IMAGE_WIDTH    640
#define CAM0_IMAGE_HEIGHT   480

#define PREVIEW_IMAGE_WIDTH    (CAM0_IMAGE_WIDTH/8)
#define PREVIEW_IMAGE_HEIGHT   (CAM0_IMAGE_HEIGHT/8)


#define FLOW_STACK_SIZE      (64*1024)

#define FLOW_RAD_PER_PIXEL   (0.00425f)


int flow_copy_matched_points(struct geo_feature_s *ft);
int flow_copy_image(unsigned char *img,float *timestamp);
void *thread_flow(void *arg);

#endif
