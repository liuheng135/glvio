#ifndef _GEOMETRY_H_
#define _GEOMETRY_H_

#include "image_lib.h"

struct geo_matches_s{
    struct point3f l;
    struct point3f r;
    float quality;
};

void geo_recovery_translation(struct point3f *T,struct geo_matches_s p1,struct geo_matches_s p2);
int geo_recovery_depth(struct point3f *p,struct geo_matches_s mp,struct point3f T);



#endif