#include <iostream>
#include "lwlink.h"
#include "udp.h"
#include <opencv2/opencv.hpp>
#include "image_lib.h"
#include "stdio.h"

using namespace std;

struct udp_data_s udp0;
struct lwlink_data_handler_s link_handler;
char recvBuf[12000] = { 0 };

#define FLOW_IMAGE_WIDTH    64
#define FLOW_IMAGE_HEIGHT   64

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
    struct lwlink_feature2D_s fp1;

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
                        memcpy(&fp1,tfp,sizeof(fp1));
                        cout << "vel = " << fp1.vel_x << "," << fp1.vel_y  << ",   " << fp1.quality << endl;
                    }

                    if(msg_type == MSG_TYPE_RAW_IMAGE){
                        uint8_t *img_data = lwlink_data_handler_get_data(&link_handler);
                        memcpy(cv_img.data,img_data,FLOW_IMAGE_WIDTH*FLOW_IMAGE_HEIGHT);

                        cv::Point2f pstart(FLOW_IMAGE_WIDTH/2,FLOW_IMAGE_HEIGHT/2);
                        cv::Point2f pvel(fp1.vel_x,fp1.vel_y);
                        cv::Point2f pend = pstart + pvel * 2;
                        cv::arrowedLine(cv_img,pstart,pend,cv::Scalar::all(-1),1,8,0,0.1);

                        cv::imshow("image",cv_img);
                    }
                }     
            }
        }
        cv::waitKey(10); 
    }

    return 0;
}
