#ifndef __CSV_HPP__
#define __CSV_HPP__

#include <string>
#include <iostream>
#include <vector>

#if (defined WIN32 || defined _WIN32 || defined WINCE || defined __CYGWIN__) && defined CVAPI_EXPORTS
#  define CSV_EXPORTS __declspec(dllexport)
#elif defined __GNUC__ && __GNUC__ >= 4
#  define CSV_EXPORTS __attribute__ ((visibility ("default")))
#else
#  define CSV_EXPORTS
#endif

class imu_data{
public:
    float acc[3];
    float gyro[3];
    long long timestamp;   // ns
};

class img_info{
public:
    long long timestamp;
    std::string filename;
};

class pose_info{
public:
    long long timestamp;
    float R[3][3];
    float eulur[3];
    float Q[4];
    float pos[3];
    float vel[3];
};

class point4f {
public:
    float x;
    float y;
    float z;
    float s;
};


CSV_EXPORTS int read_csv(std::string filename,std::vector< std::vector< std::string > >& vv,int max_line);
CSV_EXPORTS int load_imu_data(std::string path,std::vector<imu_data>& imu,int max_num);
CSV_EXPORTS int load_img(std::string path,std::vector<img_info>& img,int max_num);
CSV_EXPORTS int load_pose(std::string path,std::vector<pose_info>& ap,int max_num);
CSV_EXPORTS void save_pos(std::string path,std::vector<pose_info> pos);
CSV_EXPORTS void save_cloud(std::string path,std::vector<point4f> points);
CSV_EXPORTS void save_imu_att(std::string path,std::vector<pose_info> pos);

#endif
