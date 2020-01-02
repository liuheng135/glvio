#include "app_timer.h"

static float timer_stamp;

float timer_get_time(void)
{
    return timer_stamp;
}
void timer_init(void)
{
    timer_stamp = 0.0f;
}
void  timer_update(float dt){
    timer_stamp += dt;
}



