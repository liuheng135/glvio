#include <iostream>
#include "lwlink.h"
#include "udp.h"
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include "image_lib.h"
#include "stdio.h"
#include "quaternion.h"

using namespace std;

struct udp_data_s udp0;
struct lwlink_data_handler_s link_handler;
char recvBuf[12000] = { 0 };

#define FLOW_IMAGE_WIDTH    60
#define FLOW_IMAGE_HEIGHT   60

void martix2Mat(cv::Mat *dst,struct matrix_s *src)
{
    int size = dst->cols * dst->rows;
    memcpy(dst->data,src->data,size);
}

int main(int argc, char *argv[])
{
    int i;
    int recved_len;
    unsigned char msg_type;
    char ip_addr[] = "192.168.0.1";
    char hello[] = "hello";
    struct lwlink_feature2D_s fp[4];

    cv::Vec3f cam_pos(0.f,0.f,0.f);
    cv::viz::Viz3d my3DWindow("3D sense");
    my3DWindow.showWidget("3D sense", cv::viz::WCoordinateSystem());

    cv::Mat cv_img(FLOW_IMAGE_WIDTH,FLOW_IMAGE_HEIGHT,CV_8UC1);
    cv::namedWindow("image",1);
   
    printf("viewer start...\r\n");

    udp_init(&udp0,ip_addr,3366);
    lwlink_data_handler_init(&link_handler,0x02);
    udp_send(&udp0,hello,5);

    cout << "server connected,waiting for message" << endl;

    while(1) {
        recved_len =  udp_recv(&udp0,recvBuf,10240);
        udp_send(&udp0,hello,5);

        if(recved_len > 0){
			for(i = 0; i < recved_len; i++){
                if(lwlink_data_handler_parse(&link_handler,recvBuf[i]) > 0){
                    uint8_t msg_type = lwlink_data_handler_get_type(&link_handler);

                    if(msg_type == MSG_TYPE_FEATURE2D){
                        uint8_t *img_data = lwlink_data_handler_get_data(&link_handler);
                        struct lwlink_feature2D_s *tfp = (struct lwlink_feature2D_s *)img_data;
                        if(tfp->feature_id < 4 ){
                            memcpy(&fp[tfp->feature_id],tfp,sizeof(struct lwlink_feature2D_s));
                        }
                    }

                    if(msg_type == MSG_TYPE_RAW_IMAGE){
                        uint8_t *img_data = lwlink_data_handler_get_data(&link_handler);
                        memcpy(cv_img.data,img_data,FLOW_IMAGE_WIDTH*FLOW_IMAGE_HEIGHT);

                        for(i = 0; i < 4;i++){
                            cv::Point2f pstart(fp[i].pos_x,fp[i].pos_y);
                            cv::Point2f pend(fp[i].pos_x + fp[i].vel_x,fp[i].pos_y + fp[i].vel_y);
                            cv::arrowedLine(cv_img,pstart,pend,cv::Scalar::all(-1),1,8,0,0.1);
                        }
                        cv::imshow("image",cv_img);
                    }

                    if(msg_type == MSG_TYPE_ATTITUDE){
                        uint8_t *att_data = lwlink_data_handler_get_data(&link_handler);
                        struct lwlink_attitude_s *att = (struct lwlink_attitude_s *)att_data;
                        struct quaternion_s q;
                        struct eulur_s e;
                        e.roll = att->roll;
                        e.pitch = att->pitch;
                        e.yaw   = att->yaw;
                        eulur_to_quater(&q,&e);
                        float camera_dir_up[3] = {0.f,0.f,1.f};
                        float camera_dir_front[3] = {1.f,0.f,0.f};
                        quater_rotate(camera_dir_up,camera_dir_up,&q);
                        quater_rotate(camera_dir_front,camera_dir_front,&q);
                        cv::Vec3f cam_df(camera_dir_front[0],camera_dir_front[1],camera_dir_front[2]);
                        cv::Vec3f cam_du(camera_dir_up[0],camera_dir_up[1],camera_dir_up[2]);

                        cv::Affine3f cam_pose = cv::viz::makeCameraPose(cam_pos, cam_pos + cam_df, cam_du);
                        cv::viz::WCameraPosition cpw(0.5); // Coordinate axes
                        cv::viz::WCameraPosition cpw_frustum(cv::Vec2f(1, 0.5)); // Camera frustum
                        
                        my3DWindow.showWidget("CPW", cpw, cam_pose);
                        my3DWindow.showWidget("CPW_FRUSTUM", cpw_frustum, cam_pose);
                        my3DWindow.spinOnce(1,false);
                        cout << "attitude recved" << endl;
                    }
                }     
            }
        }
        cv::waitKey(10); 
    }

    return 0;
}
