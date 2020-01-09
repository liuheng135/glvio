#include "geometry.h"

float calculate_ka(struct geo_matches_s p)
{
    return p.l.z * p.r.y - p.l.y * p.r.z;
}

float calculate_kb(struct geo_matches_s p)
{
    return p.l.x * p.r.y - p.l.y * p.r.x;
}

float calculate_m(struct geo_matches_s p,float ka,float kb)
{
    return kb * p.l.z - ka * p.l.x;
}

float calculate_n(struct geo_matches_s p,float ka)
{
    return ka * p.l.y;
}

void geo_recovery_translation(struct point3f *T,struct geo_matches_s p1,struct geo_matches_s p2)
{
    float ka1,ka2;
    float kb1,kb2;
    float m1,m2;
    float n1,n2;
    float denominator;

    ka1 = calculate_ka(p1);
    ka2 = calculate_ka(p2);
    kb1 = calculate_kb(p1);
    kb2 = calculate_kb(p2);
    m1  = calculate_m(p1,ka1,kb1);
    m2  = calculate_m(p2,ka2,kb2);
    n1  = calculate_n(p1,ka1); 
    n2  = calculate_n(p2,ka2);

    T->z  = 1.0f;

    denominator = m2 * ka1 * p1.l.y - m1 * ka2 * p2.l.y;
    if(denominator != 0.f){
        T->x = (m2 * kb1 * p1.l.y - m1 * kb2 * p2.l.y) / denominator;
    }else{
        T->x = 0.f;
    }

    denominator = n2 * kb1 * p1.l.z - n2 * ka1 * p1.l.x - n1 * kb2 * p2.l.z + n1 * ka2 * p2.l.x;
    if(denominator != 0.f){
        T->y = (n2 * kb1 * p1.l.y - n1 * kb2 * p2.l.y) / denominator;
    }else{
        T->y = 0.f;
    }
}

void geo_recovery_depth(struct point3f *p,struct geo_matches_s mp,struct point3f T)
{
    float s;
    float denominator;

    denominator = mp.l.x * mp.r.y - mp.l.y * mp.r.x;
    if(denominator != 0.f){
        s  = (mp.l.y * T.x - mp.l.x * T.y) / denominator;
        p->x = T.x + mp.r.x * s;
        p->y = T.y + mp.r.y * s;
        p->z = T.z + mp.r.z * s;
    }else{
        s = 0.f;
        p->x = 0.f;
        p->y = 0.f;
        p->z = 0.f;
    } 
}