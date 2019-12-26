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

void martix2Mat(cv::Mat *dst,struct matrix_s *src)
{
    int size = dst->cols * dst->rows;
    memcpy(dst->data,src->data,size);
}

int main(int argc, char *argv[])
{
    int i;
    int recved_len;
    char ip_addr[] = "192.168.0.1";
    char hello[] = "hello";

    cv::Mat cv_img(100,100,CV_8UC1);
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
            cout << "data recvd:" << recved_len << endl;
            for(i = 0;i < 6;i++){
                printf("0x%02x ",recvBuf[i]);
            }
            printf(" ... ");
            for(i = recved_len - 3;i < recved_len;i++){
                printf("0x%02x ",recvBuf[i]);
            }
            printf("\r\n");

			for(i = 0; i < recved_len; i++){
                if(lwlink_data_handler_parse(&link_handler,recvBuf[i]) > 0){
                    uint8_t *img_data = lwlink_data_handler_get_data(&link_handler);
                    memcpy(cv_img.data,img_data,100*100);
                    cout << "msg recvd,type is " << lwlink_data_handler_get_type(&link_handler) << endl;
                    cv::imshow("image",cv_img);
                    
                }     
            }
            
        }
        
        cv::waitKey(10); 
    }

    return 0;
}
