#ifndef _VISUAL_ODOMETRY_H_
#define _VISUAL_ODOMETRY_H_

#include "opencv2/opencv.hpp"
#include "quaternion.h"
#include <vector>
#include "csv.hpp"


class visual_odometry{
    private:
        double camera_intrinsics[9];
        double distortion_coefficients[4];
        int    focus_length;

        int image_index;
        std::vector<img_info>    images;

        cv::Ptr< cv::FastFeatureDetector >  fast_detector;
        cv::Mat essential_matrix;

        cv::Mat R,t;
        cv::Mat point_cloud;

        void pose_estimation_2d2d();
        void triangulation();
        void feature_detect();

    public:
        cv::Mat image_now;
        cv::Mat image_last;

        long long timestamp;
        std::vector<cv::Point2f> keypoints_now;
        std::vector<cv::Point2f> keypoints_last;

        struct quaternion_s rotate;
        cv::Vec3f           translation;

        void init(void);
        void update(void);
};

#endif