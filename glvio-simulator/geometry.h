#ifndef _GEOMETRY_H_
#define _GEOMETRY_H_

#include "image_lib.h"

struct geo_point_observer_s{
    int point_pos_valid;
    int camera_pos_valid;
    struct point3f point_pos_in_camera;
    struct point3f camera_pos_in_world;
    float quality;
};

struct geo_feature_s{
    int pos_in_world_valid;
    int observe_count;
    struct point3f pos_in_world;
    struct geo_point_observer_s observer_prev;
    struct geo_point_observer_s observer_next;
};

void geo_recovery_translation_2D2D(struct geo_feature_s *p1,struct geo_feature_s *p2);
void geo_recovery_translation_2D3D(struct point3f *T,struct geo_feature_s *p1,struct geo_feature_s *p2);
void geo_recovery_depth(struct geo_feature_s *p1);


#endif