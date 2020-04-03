#include "geometry.h"
#include "stdio.h"

float calculate_ka(struct geo_feature_s p)
{
    return p.observer_prev.point_pos_in_camera.z * p.observer_next.point_pos_in_camera.y - p.observer_prev.point_pos_in_camera.y * p.observer_next.point_pos_in_camera.z;
}

float calculate_kb(struct geo_feature_s p)
{
    return p.observer_prev.point_pos_in_camera.x * p.observer_next.point_pos_in_camera.y - p.observer_prev.point_pos_in_camera.y * p.observer_next.point_pos_in_camera.x;
}

float calculate_m(struct geo_feature_s p,float ka,float kb)
{
    return kb * p.observer_prev.point_pos_in_camera.z - ka * p.observer_prev.point_pos_in_camera.x;
}

float calculate_n(struct geo_feature_s p,float ka)
{
    return ka * p.observer_prev.point_pos_in_camera.y;
}

void geo_recovery_translation_2D2D(struct geo_feature_s *p1,struct geo_feature_s *p2)
{
    float ka1,ka2;
    float kb1,kb2;
    float m1,m2;
    float n1,n2;
    float denominator;
    float s;
    struct point3f T;

    if((p1->observer_prev.point_pos_valid == 0) || (p2->observer_prev.point_pos_valid == 0)){
        return;
    }

    if((p1->observer_next.point_pos_valid == 0) || (p2->observer_next.point_pos_valid == 0)){
        return;
    }

    ka1 = calculate_ka(*p1);
    ka2 = calculate_ka(*p2);
    kb1 = calculate_kb(*p1);
    kb2 = calculate_kb(*p2);
    m1  = calculate_m(*p1,ka1,kb1);
    m2  = calculate_m(*p2,ka2,kb2);
    n1  = calculate_n(*p1,ka1); 
    n2  = calculate_n(*p2,ka2);

    T.z  = 1.0f;

    denominator = m2 * ka1 * p1->observer_prev.point_pos_in_camera.y - m1 * ka2 * p2->observer_prev.point_pos_in_camera.y;
    if(denominator != 0.f){
        T.x = (m2 * kb1 * p1->observer_prev.point_pos_in_camera.y - m1 * kb2 * p2->observer_prev.point_pos_in_camera.y) / denominator;
    }else{
        T.x = 0.f;
    }

    denominator = n2 * kb1 * p1->observer_prev.point_pos_in_camera.z - n2 * ka1 * p1->observer_prev.point_pos_in_camera.x - n1 * kb2 * p2->observer_prev.point_pos_in_camera.z + n1 * ka2 * p2->observer_prev.point_pos_in_camera.x;
    if(denominator != 0.f){
        T.y = (n2 * kb1 * p1->observer_prev.point_pos_in_camera.y - n1 * kb2 * p2->observer_prev.point_pos_in_camera.y) / denominator;
    }else{
        T.y = 0.f;
    }

    denominator = p1->observer_prev.point_pos_in_camera.x * p1->observer_next.point_pos_in_camera.y - p1->observer_prev.point_pos_in_camera.y * p1->observer_next.point_pos_in_camera.x;
    if(denominator != 0.f){
        s  = (p1->observer_prev.point_pos_in_camera.y * T.x - p1->observer_prev.point_pos_in_camera.x * T.y) / denominator;
    }else{
        s = 0;
    }

    if(T.x + p1->observer_next.point_pos_in_camera.x * s < 0){
        T.x = -T.x;
        T.y = -T.y;
        T.z = -T.z;
    }

    p1->observer_prev.camera_pos_in_world.x = 0;
    p1->observer_prev.camera_pos_in_world.y = 0;
    p1->observer_prev.camera_pos_in_world.z = 0;
    p1->observer_prev.camera_pos_valid  = 1;

    p2->observer_prev.camera_pos_in_world.x = 0;
    p2->observer_prev.camera_pos_in_world.y = 0;
    p2->observer_prev.camera_pos_in_world.z = 0;
    p2->observer_prev.camera_pos_valid  = 1;

    p1->observer_next.camera_pos_in_world.x = T.x;
    p1->observer_next.camera_pos_in_world.y = T.y;
    p1->observer_next.camera_pos_in_world.z = T.z;
    p1->observer_next.camera_pos_valid  = 1;

    p2->observer_next.camera_pos_in_world.x = T.x;
    p2->observer_next.camera_pos_in_world.y = T.y;
    p2->observer_next.camera_pos_in_world.z = T.z;
    p2->observer_next.camera_pos_valid  = 1;
}

void geo_recovery_depth(struct geo_feature_s *p1)
{
    float s;
    float denominator;
    struct point3f T;

    if((p1->observer_prev.camera_pos_valid == 0) || (p1->observer_next.camera_pos_valid == 0)){
        return;
    }

    T.x = p1->observer_next.camera_pos_in_world.x - p1->observer_prev.camera_pos_in_world.x;
    T.y = p1->observer_next.camera_pos_in_world.y - p1->observer_prev.camera_pos_in_world.y;
    T.z = p1->observer_next.camera_pos_in_world.z - p1->observer_prev.camera_pos_in_world.z;

    denominator = p1->observer_prev.point_pos_in_camera.x * p1->observer_next.point_pos_in_camera.y - p1->observer_prev.point_pos_in_camera.y * p1->observer_next.point_pos_in_camera.x;
    if(denominator != 0.f){
        s  = (p1->observer_prev.point_pos_in_camera.y * T.x - p1->observer_prev.point_pos_in_camera.x * T.y) / denominator;
        p1->pos_in_world.x = T.x + p1->observer_next.point_pos_in_camera.x * s;
        p1->pos_in_world.y = T.y + p1->observer_next.point_pos_in_camera.y * s;
        p1->pos_in_world.z = T.z + p1->observer_next.point_pos_in_camera.z * s;
        p1->pos_in_world_valid = 1;
    }else{
        p1->pos_in_world.x = 0.f;
        p1->pos_in_world.y = 0.f;
        p1->pos_in_world.z = 0.f;
        p1->pos_in_world_valid = 0;
    } 
}