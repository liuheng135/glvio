#ifndef _ATT_EST_IMU_H_
#define _ATT_EST_IMU_H_

#include "ahrs.h"
#include <vector>
#include "csv.hpp"
#include <fstream>

class attitude_info_s{
    public:

    float eulur[3];
    float quaternion[4];

    long long timestamp;
};

class imu_att_estimator{

private:
    int imu_data_index;
    ahrs_estimator att_est;

    float dcm[3][3];
    float quaternion[4];

    std::ofstream fileout;

    std::vector<imu_data> imu;
    std::vector<pose_info> pose;

public:

    void init();
    void update(void);

    void get_attitude(attitude_info_s &att);
};

#endif