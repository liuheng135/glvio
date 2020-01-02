#include "quaternion.h"
#include <math.h>

void  axis_angle_to_quater(struct quaternion_s *q,struct axis_angle_s *a)
{
    q->w = cosf(a->angle * 0.5f);
    q->x = a->ax * sinf(a->angle * 0.5f);
    q->y = a->ay * sinf(a->angle * 0.5f);
    q->z = a->az * sinf(a->angle * 0.5f);
}

void  quater_to_eulur(struct eulur_s *e,struct quaternion_s *q)
{
    e->roll  = atan2f(2.0f * (q->w * q->x + q->y * q->z), 1.0f - 2.0f * (q->x * q->x + q->y * q->y));
	e->pitch = asinf(2.0f * (q->w * q->y  - q->z * q->x));
	e->yaw   = atan2f(2.0f * (q->w * q->z + q->x * q->y ), 1.0f - 2.0f * (q->y  * q->y  + q->z * q->z));
}

void  eulur_to_quater(struct quaternion_s *q,struct eulur_s *e)
{
    double cosPhi_2 = cos((double)(e->roll) / 2.0);
	double sinPhi_2 = sin((double)(e->roll) / 2.0);
	double cosTheta_2 = cos((double)(e->pitch) / 2.0);
	double sinTheta_2 = sin((double)(e->pitch) / 2.0);
	double cosPsi_2 = cos((double)(e->yaw) / 2.0);
	double sinPsi_2 = sin((double)(e->yaw) / 2.0);

	q->w = (float)(cosPhi_2 * cosTheta_2 * cosPsi_2 + sinPhi_2 * sinTheta_2 * sinPsi_2);
	q->x = (float)(sinPhi_2 * cosTheta_2 * cosPsi_2 - cosPhi_2 * sinTheta_2 * sinPsi_2);
	q->y = (float)(cosPhi_2 * sinTheta_2 * cosPsi_2 + sinPhi_2 * cosTheta_2 * sinPsi_2);
	q->z = (float)(cosPhi_2 * cosTheta_2 * sinPsi_2 - sinPhi_2 * sinTheta_2 * cosPsi_2);
}

float quater_length(struct quaternion_s *q)
{
    return sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
}

void  quater_normlize(struct quaternion_s *qn,struct quaternion_s *q)
{
    float len;
    struct quaternion_s src;
    
    src = *q;
    len = quater_length(&src);

    qn->w = src.w / len;
    qn->x = src.x / len;
    qn->y = src.y / len;
    qn->z = src.z / len;
}

void  quater_conjufate(struct quaternion_s *qc,struct quaternion_s *q)
{
    struct quaternion_s src;
    
    src = *q;

    qc->w =  src.w;
    qc->x = -src.x;
    qc->y = -src.y;
    qc->z = -src.z;
}

void  quater_inverse(struct quaternion_s *qi,struct quaternion_s *q)
{
    float len;
    struct quaternion_s src;
    struct quaternion_s qc;

    src = *q;

    len = quater_length(&src);
    quater_conjufate(&qc,&src);

    qi->w = qc.w / len;
    qi->x = qc.x / len;
    qi->y = qc.y / len;
    qi->z = qc.z / len;
}

void  quater_multiply(struct quaternion_s *qp,struct quaternion_s *a,struct quaternion_s *b)
{
    struct quaternion_s _a,_b;

    _a = *a;
    _b = *b;

    qp->w = _a.w * _b.w - _a.x * _b.x - _a.y * _b.y - _a.z * _b.z;
    qp->x = _a.w * _b.x + _a.x * _b.w + _a.y * _b.z - _a.z * _b.y;
    qp->y = _a.w * _b.y - _a.x * _b.z + _a.y * _b.w + _a.z * _b.x;
    qp->z = _a.w * _b.z + _a.x * _b.y - _a.y * _b.x + _a.z * _b.w;
}

void  quater_rotate(float *res,float *v,struct quaternion_s *q)
{
    struct quaternion_s qv;
    struct quaternion_s qi;
    struct quaternion_s qm;

    // construct qv = [ 0,v[0],v[1],v[2] ]
    qv.w = 0.f;
    qv.x = v[0];
    qv.y = v[1];
    qv.z = v[2];

    quater_inverse(&qi,q);         // calc q.inverse
    quater_multiply(&qm,&qv,&qi);  // qm = qv * q.inverse
    quater_multiply(&qm,q,&qm);   // qm = q * qm

    res[0] = qm.x;
    res[1] = qm.y;
    res[2] = qm.z;
}