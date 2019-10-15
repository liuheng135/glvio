#include "att_est_imu.hpp"
#include "rotation.h"

void imu_att_estimator::init(void)
{
    float mag[3] = {0.0f,0.0f,0.0f};

    imu.resize(8000);
    pose.resize(8000);
    load_imu_data("../datasets/mav0/imu0/data.csv",imu,8000);
    int pos_num = load_pose("../datasets/mav0/imu0/groundtruth.csv",pose,8000);

    std::cout << "pose num is " << pos_num << std::endl;

    imu_data_index = 1;
    ahrs_init(&att_est,imu[imu_data_index].acc,imu[imu_data_index].gyro,mag);

    fileout.open("../result/imu_att.csv");
}

void imu_att_estimator::update(void)
{
    imu_data_index++;

    float dt = (imu[imu_data_index].timestamp - imu[imu_data_index - 1].timestamp) * 1e-9f;
    
    rotate3(imu[imu_data_index].acc,ROTATION_PITCH_270);
    rotate3(imu[imu_data_index].gyro,ROTATION_PITCH_270);
    ahrs_apply_acc(&att_est,imu[imu_data_index].acc);
    ahrs_apply_gyro(&att_est,imu[imu_data_index].gyro);

    ahrs_update(&att_est,dt);

    quaternion[0] = att_est.q[0];
    quaternion[1] = att_est.q[1];
    quaternion[2] = att_est.q[2];
    quaternion[3] = att_est.q[3];

    ahrs_qua2dcm(quaternion,dcm);

    
    std::cout<< "acc:" << imu[imu_data_index].acc[0] << "   " \
        << imu[imu_data_index].acc[1] << "   "   \
        << imu[imu_data_index].acc[2] << "   "   \
        << imu[imu_data_index].gyro[0] << "   "  \
        << imu[imu_data_index].gyro[1] << "   "  \
        << imu[imu_data_index].gyro[2] << "   "  \
        << std::endl;
    
    
    float quat_vicon[4];
    float eulur_vicon[3];

    quat_vicon[0] = pose[imu_data_index].Q[0];
    quat_vicon[1] = pose[imu_data_index].Q[1];
    quat_vicon[2] = pose[imu_data_index].Q[2];
    quat_vicon[3] = pose[imu_data_index].Q[3];

    ahrs_qua2eulur(quat_vicon,&eulur_vicon[0],&eulur_vicon[1],&eulur_vicon[2]);
    
    fileout << att_est.eulur[0] * 57.3f << "," \
        << att_est.eulur[1] * 57.3f << "," \
        << att_est.eulur[2] * 57.3f << "," \
        << eulur_vicon[0] * 57.3f << "," \
        << eulur_vicon[1] * 57.3f << "," \
        << eulur_vicon[2] * 57.3f << "," \
        << dt << std::endl; 

    if(imu_data_index % 200 == 0){
        fileout.flush();
    }
}

void imu_att_estimator::get_attitude(attitude_info_s &att)
{
    for(int i = 0;i < 4;i++){
        att.quaternion[i] = quaternion[i];
    }

    att.timestamp = imu[imu_data_index].timestamp;
}

