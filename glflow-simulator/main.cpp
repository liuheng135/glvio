#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <optflow_lk.h>

using namespace std;
using namespace cv;

static unsigned char image_buffer_a[800*800] __attribute__((aligned(4)));
static unsigned char image_buffer_b[800*800] __attribute__((aligned(4)));

void matrix_show(String name,struct matrix_s *mat)
{
    namedWindow(name);
    Mat img(mat->rows,mat->cols,CV_8UC1,Scalar(0));
    memcpy(img.data,mat->data,mat->cols*mat->rows);
    imshow(name,img);
}

int main(int argc, char *argv[])
{

    float angle = 0;
    printf("simulator start...\r\n");
    Mat raw_image = imread("./1.jpg",IMREAD_GRAYSCALE);
    Mat Rotated_img;
    Mat R;
    Mat prev_show( 640,640, CV_8UC1,Scalar(0));
    Mat next_show( 640,640, CV_8UC1,Scalar(0));
    Size dst_sz(raw_image.cols,raw_image.rows );

    float             flow_err;
    unsigned char    *ptr;
    struct matrix_s   prev_image;
    struct matrix_s   next_image;
    struct point2f    prev_point;
    struct point2f    next_point;
    struct size2i     opt_win_size = {31,31};
    struct optflow_lk optflow;

    optflow_lk_create(&optflow,1,6,0.5,&opt_win_size);
    matrix_init(&prev_image,raw_image.cols,raw_image.rows,1,IMAGE_TYPE_8U,image_buffer_a);
    matrix_init(&next_image,raw_image.cols,raw_image.rows,1,IMAGE_TYPE_8U,image_buffer_b);
/*
    namedWindow("prev");
    namedWindow("next");
*/
    memcpy(prev_image.data,raw_image.data,raw_image.cols*raw_image.rows);

    while(1) {
        angle += 2.5f;
        R = getRotationMatrix2D(Point2f(320,320),angle,1);
        warpAffine(raw_image,Rotated_img,R,dst_sz,INTER_LINEAR,BORDER_REPLICATE);
        memcpy(next_image.data,Rotated_img.data,next_image.cols * next_image.rows);
        prev_point.x = 320;
        prev_point.y = 320;

        optflow_lk_calc(&optflow,&prev_image,&next_image,&prev_point,&next_point,&flow_err);

        printf("flow: %6.3f %6.3f %8.1f\r\n",next_point.x - prev_point.x,next_point.y - prev_point.y, flow_err);

        /*memcpy(prev_show.data,prev_image.data,prev_image.cols * prev_image.rows);
        memcpy(next_show.data,next_image.data,next_image.cols * next_image.rows);
        imshow("prev",prev_show);
        imshow("next",next_show);
        */
        matrix_show("prev",&prev_image);
        matrix_show("next",&next_image);


        ptr = prev_image.data;
        prev_image.data = next_image.data;
        next_image.data = ptr;

        if(cv::waitKey(100) == '1'){
            break;
        } 
    }


    return 0;
}