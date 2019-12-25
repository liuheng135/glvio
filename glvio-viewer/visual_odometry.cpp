#include "visual_odometry.hpp"

void visual_odometry::init(void)
{
    image_index = 2;
    images.resize(2000);
    load_img("../datasets/mav0/cam0/data.csv",images,2000);

    fast_detector =  cv::FastFeatureDetector::create(20);

    timestamp = images[0].timestamp;

    camera_intrinsics[0] = 458.654;
    camera_intrinsics[1] = 0;
    camera_intrinsics[2] = 367.215;
    camera_intrinsics[3] = 0;
    camera_intrinsics[4] = 457.296;
    camera_intrinsics[5] = 248.375;
    camera_intrinsics[6] = 0;
    camera_intrinsics[7] = 0;
    camera_intrinsics[8] = 1;
}

void visual_odometry::pose_estimation_2d2d(void)
{
    cv::Point2d principal_point(367.215,248.375);
    cv::Mat K(3,3,CV_64F,camera_intrinsics);
    int    focus_length = 521;

    essential_matrix   = cv::findEssentialMat(keypoints_now,keypoints_now,focus_length,principal_point,cv::RANSAC);
    cv::recoverPose(essential_matrix,keypoints_now,keypoints_now,R,t,focus_length,principal_point,cv::noArray());
}

void visual_odometry::triangulation(void)
{
    cv::Mat T1 = (cv::Mat_<double> (3,4) <<  
        1,0,0,0,
        0,1,0,0,
        0,0,1,0);

    cv::Mat T2 = (cv::Mat_<double> (3,4) << 
        R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),t.at<double>(0,0),
        R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),t.at<double>(1,0),
        R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2),t.at<double>(2,0));

    cv::Mat K(3,3,CV_64F,camera_intrinsics);

    triangulatePoints(T1,T2,keypoints_now,keypoints_last,point_cloud);
}

float getDistanceBetweenPoints(cv::Point2f point1,cv::Point2f point2)
{
    float dx = point1.x - point2.x;
    float dy = point1.y - point2.y;

    return sqrtf(dx * dx + dy * dy);
}

void visual_odometry::update(void)
{
    std::string img_now_name,img_last_name;
    cv::Mat image_now_uncalid,image_last_uncalid;

    cv::Mat camera_matrix(3,3,CV_64F,camera_intrinsics);
    cv::Mat dist_coeff(1,4,CV_64F,distortion_coefficients);

    std::vector<cv::KeyPoint> kp_detected;
    std::vector<cv::Point2f>  kp_now,kp_last;

    img_now_name = "../datasets/mav0/cam0/data/" + images[image_index].filename;
    img_last_name = "../datasets/mav0/cam0/data/" + images[image_index-1].filename;
    image_now_uncalid  = cv::imread(cv::String(img_now_name),cv::IMREAD_UNCHANGED);
    image_last_uncalid = cv::imread(cv::String(img_last_name),cv::IMREAD_UNCHANGED);

    if(!image_now_uncalid.empty() && !image_last_uncalid.empty()){
        cv::undistort(image_now_uncalid, image_now, camera_matrix,dist_coeff);
        cv::undistort(image_last_uncalid, image_last, camera_matrix,dist_coeff);

        kp_last.clear();
        kp_now.clear();

        fast_detector->detect(image_last,kp_detected,cv::noArray());

        for(int i = 0;i < kp_detected.size();i++){
            kp_last.push_back(kp_detected[i].pt);
        }

        std::vector<unsigned char> status;
        std::vector<float> error; 
        cv::calcOpticalFlowPyrLK(image_last,image_now,kp_last,kp_now,status,error);             

        keypoints_now.clear();
        keypoints_last.clear();
        for(int i = 1; i < error.size(); i++){
            if(error[i] < 10.0 && getDistanceBetweenPoints(kp_now[i],kp_last[i]) < 50.f){
                keypoints_last.push_back(kp_last[i]);
                keypoints_now.push_back(kp_now[i]);
            }
        }

        if(keypoints_last.size() < 8){
            std::cout << "dropped! total point num is " << kp_last.size() << std::endl;

            for(int i = 0;i < 3;i++){
                translation[i] = 0.0f;
            }
            rotate.x = 0;
            rotate.y = 0;
            rotate.z = 0;
            rotate.w = 0;
        }else{
            pose_estimation_2d2d();

            float dcm[3][3];
            for(int i = 0;i < 3; i++){
                for(int j = 0;j < 3;j++){
                    dcm[i][j] = R.at<double>(i,j);
                }
            }
            dcm_to_quaternion(&rotate,dcm);

            for(int i = 0;i < 3; i++){
                translation[i] = t.at<double>(i);
            }
        }
    }else{
        std::cout << "can not open image:" << img_now_name << std::endl;
    }
    timestamp = images[image_index].timestamp;
    image_index++;
}