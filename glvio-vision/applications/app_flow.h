#ifndef _APP_FLOW_H_
#define _APP_FLOW_H_

#define CAM0_IMAGE_WIDTH    640
#define CAM0_IMAGE_HEIGHT   480

#define FLOW_IMAGE_WIDTH    60
#define FLOW_IMAGE_HEIGHT   60


#define FLOW_STACK_SIZE                      (64*1024)


struct flow_matched_point_s{
    float start_x;
    float start_y;
    float end_x;
    float end_y;
    float quality;
    float timestamp;
};

int flow_copy_matched_points(struct flow_matched_point_s *mps);
void *thread_flow(void *arg);

#endif
