#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include "csv.hpp"
#include "quaternion.h"
#include "att_est_imu.hpp"
#include "visual_odometry.hpp"
#include <opencv2/viz.hpp>

using namespace std;
using namespace cv;

double distortion_coefficients[4]  = {-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};

void drawMatchArrow(InputOutputArray& img,vector<Point2f> points1,vector<Point2f> points2)
{
    for(int i = 0; i < points1.size();i++){
        arrowedLine(img,points1[i],points2[i],Scalar::all(-1),1,8,0,0.1);
    }
}

int main(void)
{
    Vec3f cam_pos(0.f,0.f,0.f);
    

    imu_att_estimator att_estor;
    attitude_info_s att_info;

    visual_odometry vo;
    
    namedWindow("image",1);
    viz::Viz3d myWindow("3D sense");
    myWindow.showWidget("3D sense", viz::WCoordinateSystem());
    

    att_estor.init();
    vo.init();
    vo.update();
    while(1){
        att_estor.update();
        att_estor.get_attitude(att_info);

        if(att_info.timestamp == vo.timestamp){
            vo.update();
            cam_pos+= vo.translation * 0.1f;
                        
            struct quaternion_s q;
            q.w = att_info.quaternion[0];
            q.x = att_info.quaternion[1];
            q.y = att_info.quaternion[2];
            q.z = att_info.quaternion[3];
            
            float camera_dir_up[3] = {0.f,0.f,1.f};
            float camera_dir_front[3] = {1.f,0.f,0.f};
            quater_rotate(camera_dir_up,camera_dir_up,&q);
            quater_rotate(camera_dir_front,camera_dir_front,&q);
            Vec3f cam_df(camera_dir_front[0],camera_dir_front[1],camera_dir_front[2]);
            Vec3f cam_du(camera_dir_up[0],camera_dir_up[1],camera_dir_up[2]);

            Affine3f cam_pose = viz::makeCameraPose(cam_pos, cam_pos + cam_df, cam_du);
            viz::WCameraPosition cpw(0.5); // Coordinate axes
            viz::WCameraPosition cpw_frustum(Vec2f(1, 0.5)); // Camera frustum
            
            myWindow.showWidget("CPW", cpw, cam_pose);
            myWindow.showWidget("CPW_FRUSTUM", cpw_frustum, cam_pose);
        

            /*  log pose 
            {
                struct quaternion_s q;
                float dcm[3][3];
                for(int i = 0;i < 3; i++){
                    for(int j = 0;j < 3;j++){
                        dcm[i][j] = R.at<double>(i,j);
                    }
                }
                
                dcm_to_quaternion(&q,dcm);
                pose_info p;
                p.Q[0] = q.w;
                p.Q[1] = q.x;
                p.Q[2] = q.y;
                p.Q[3] = q.z;
                p.pos[0] = t.at<double>(0);
                p.pos[1] = t.at<double>(1);
                p.pos[2] = t.at<double>(2);
                p.timestamp = img[img_index].timestamp;
                pose_calced.push_back(p);
            }*/

            /* show point cloud */
            /*{  
                Mat tPoints;
                triangulation(good_kp_last,good_kp_now,R,t,tPoints);

                Mat TPCloud(1,tPoints.cols,CV_32FC3);
                Point3f* TPCdata = TPCloud.ptr<cv::Point3f>();
                Affine3f cloud_pose;

                for(int i = 0;i < tPoints.cols;i++){
                    cv::Mat x = tPoints.col(i);
                    x /= x.at<float>(3,0);
                    TPCdata[i].x = x.at<float>(0,0);
                    TPCdata[i].y = x.at<float>(1,0);
                    TPCdata[i].z = x.at<float>(2,0);
                }

                viz::WCloud cloud_widget(TPCloud, viz::Color::green());
                myWindow.showWidget("bunny", cloud_widget, cloud_pose);
            }*/

            myWindow.spinOnce(1,false);

            if(!vo.image_now.empty()){     
                drawMatchArrow(vo.image_now,vo.keypoints_now,vo.keypoints_last);
                imshow("image",vo.image_now);
            }
        }
        if(waitKey(1) == 'q'){
            break;
        }
    }
    
    //save_pos("../result/pose.csv",pose_calced);
    //cout << "pose saved" << endl;
    return 0;
}