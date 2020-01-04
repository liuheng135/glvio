#ifndef _QUATERNION_H_
#define _QUATERNION_H_

struct axis_angle_s{
    float ax;
    float ay;
    float az;
    float angle;
};

struct eulur_s{
    float roll;
    float pitch;
    float yaw;
};

struct quaternion_s{
    float w;
    float x;
    float y;
    float z;
};

void  axis_angle_to_quater(struct quaternion_s *q,struct axis_angle_s *a);
void  quater_to_eulur(struct eulur_s *e,struct quaternion_s *q);
void  eulur_to_quater(struct quaternion_s *q,struct eulur_s *e);
float quater_length(struct quaternion_s *q);
void  quater_multiply(struct quaternion_s *qp,struct quaternion_s *a,struct quaternion_s *b);
void  quater_normlize(struct quaternion_s *qn,struct quaternion_s *q);
void  quater_conjufate(struct quaternion_s *qc,struct quaternion_s *q);
void  quater_inverse(struct quaternion_s *qi,struct quaternion_s *q);
void  quater_rotate(float *res,float *v,struct quaternion_s *q);
void  quater_update_by_gyroscope(struct quaternion_s *a,float *gyro,float dt);

#endif