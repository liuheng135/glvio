#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include "quaternion.h"
#include <opencv2/viz.hpp>
#include "geometry.h"

using namespace std;
using namespace cv;

void drawMatchArrow(InputOutputArray& img,vector<Point2f> points1,vector<Point2f> points2)
{
    for(int i = 0; i < points1.size();i++){
        arrowedLine(img,points1[i],points2[i],Scalar::all(-1),1,8,0,0.1);
    }
}

void get_cross(Point3d &cross,Point3d ls,Point3d le,Point3d P0,Point3d nv)
{
    double tmp;
    
    double A = nv.x;
    double B = nv.y;
    double C = nv.z;
    double D = -A*P0.x - B*P0.y - C*P0.z;

    double m = le.x;
    double n = le.y;
    double p = le.z;
    
    double a = ls.x;
    double b = ls.y;
    double c = ls.z;

    double t = -(A*a + B*b + C*c + D);

    tmp = A*m + B*n + C*p;
    if(tmp != 0){
        t /= tmp;
    }else{
        cout << "ERROR:t = inf" << endl;
        t = 0;
    }

    cross.x = m * t + a;
    cross.y = n * t + b;
    cross.z = p * t + c;
}

/*
int main(void)
{
    viz::Viz3d myWindow("3D sense");
    myWindow.showWidget("3D frame", viz::WCoordinateSystem());

    Point3d P1(-10,20,70),P2(15,15,40);
    Point3d O1(0,0,0),O2(0.2,0.3,0.1);

    Point3d Camera1_P(O1.x,O1.y,O1.x + 0.01),Camera1_NV(0,0,1);
    Point3d Camera2_P(O2.x,O2.y,O2.z + 0.01),Camera2_NV(0,0,1);

    Point3d P1L,P1R,P2L,P2R;

    cout << "A" << endl;

    get_cross(P1L,O1,P1,Camera1_P,Camera1_NV);
    get_cross(P1R,O2,P1,Camera2_P,Camera2_NV);
    get_cross(P2L,O1,P2,Camera1_P,Camera1_NV);
    get_cross(P2R,O2,P2,Camera2_P,Camera2_NV);
    cout << "B" << endl;

    viz::WLine O1P1(O1,P1,viz::Color::red());
    viz::WLine O1P2(O1,P2,viz::Color::blue());
    viz::WLine O2P1(O2,P1,viz::Color::red());
    viz::WLine O2P2(O2,P2,viz::Color::blue());

    viz::WSphere SP1L(P1L,1,10,viz::Color::red());
    viz::WSphere SP1R(P1R,1,10,viz::Color::red());
    viz::WSphere SP2L(P2L,1,10,viz::Color::blue());
    viz::WSphere SP2R(P2R,1,10,viz::Color::blue());

    cout << "C" << endl;

    float camera_dir_up[3] = {0.f,0.f,1.f};
    float camera_dir_front[3] = {1.f,0.f,0.f};
    Vec3f cam_pos(0.f,0.f,0.f);
    Vec3f cam_df(camera_dir_front[0],camera_dir_front[1],camera_dir_front[2]);
    Vec3f cam_du(camera_dir_up[0],camera_dir_up[1],camera_dir_up[2]);

    Affine3f cam_pose = viz::makeCameraPose(cam_pos, cam_pos + cam_df, cam_du);
    viz::WCameraPosition cpw(0.5);
    viz::WCameraPosition cpw_frustum(Vec2f(1, 0.5)); 

    cout << "D" << endl;
    cpw.setRenderingProperty(viz::LINE_WIDTH, 4.0);
    myWindow.showWidget("CPW", cpw, cam_pose);
    myWindow.showWidget("CPW_FRUSTUM", cpw_frustum, cam_pose);

    myWindow.showWidget("O1P1",O1P1);
    myWindow.showWidget("O1P2",O1P2);
    myWindow.showWidget("O2P1",O2P1);
    myWindow.showWidget("O2P2",O2P2);

    myWindow.showWidget("P1L",SP1L);
    myWindow.showWidget("P1R",SP1R);
    myWindow.showWidget("P2L",SP2L);
    myWindow.showWidget("P2R",SP2R);

    cout << "E" << endl;

    while (!myWindow.wasStopped()){
        myWindow.spinOnce(1,true);
    }

    cout << "F" << endl;

    return 0;
}
*/



int main(int argc, char *argv[])
{
    cv::Vec3f cam_pos(0.f,0.f,0.f);
    cv::viz::Viz3d my3DWindow("3D sense");
    my3DWindow.showWidget("3D sense", cv::viz::WCoordinateSystem());
   
    printf("viewer start...\r\n");

    cout << "server connected,waiting for message" << endl;

    Point3d P1(50,10,-7),P2(40,-8,-5);
    Point3d O1(0,0,0),O2(1,0.5,-0.3);

    viz::WLine O1P1(O1,P1,viz::Color::red());
    viz::WLine O1P2(O1,P2,viz::Color::blue());
    viz::WLine O2P1(O2,P1,viz::Color::red());
    viz::WLine O2P2(O2,P2,viz::Color::blue());

    Point3d P1L,P1R,P2L,P2R;
    Point3d Camera1_P(O1.x + 0.5,O1.y,O1.z),Camera1_NV(1,0,0);
    Point3d Camera2_P(O2.x + 0.5,O2.y,O2.z),Camera2_NV(1,0,0);

    get_cross(P1L,O1,P1,Camera1_P,Camera1_NV);
    get_cross(P1R,O2,P1,Camera2_P,Camera2_NV);
    get_cross(P2L,O1,P2,Camera1_P,Camera1_NV);
    get_cross(P2R,O2,P2,Camera2_P,Camera2_NV);

    cout << "P1L = " << P1L << endl;

    viz::WSphere SP1L(P1L,0.1,20,viz::Color::red());
    viz::WSphere SP1R(P1R,0.1,20,viz::Color::red());
    viz::WSphere SP2L(P2L,0.1,20,viz::Color::blue());
    viz::WSphere SP2R(P2R,0.1,20,viz::Color::blue());

    struct geo_matches_s mp1,mp2;
    mp1.l.x = P1L.x;
    mp1.l.y = P1L.y;
    mp1.l.z = P1L.z;

    mp1.r.x = P1R.x;
    mp1.r.y = P1R.y;
    mp1.r.z = P1R.z;

    mp2.l.x = P2L.x;
    mp2.l.y = P2L.y;
    mp2.l.z = P2L.z;

    mp2.r.x = P2R.x;
    mp2.r.y = P2R.y;
    mp2.r.z = P2R.z;

    cout <<"x=" << P2R.x << "," << P2L.x << "," << P1L.x << endl;

    struct point3f T;
    struct point3f RP1;
    struct point3f RP2;

    geo_recovery_translation(&T,mp1,mp2);

    geo_recovery_depth(&RP1,mp1,T);
    geo_recovery_depth(&RP2,mp2,T);

    cout << "O2=" << O2 << endl;
    cout << "T =" << T.x << " , " << T.y << " , " << T.z << endl;
    cout << endl;
    cout << "P1 =" << P1 << endl;
    cout << "RP1=" << RP1.x << " , " << RP1.y << " , " << RP1.z << endl;
    cout << endl;
    cout << "P2 =" << P2 << endl;
    cout << "RP2=" << RP2.x << " , " << RP2.y << " , " << RP2.z << endl;

    while(1) {
        struct quaternion_s q;
        struct eulur_s e;
        e.roll = 0.0f;
        e.pitch = 0.01f;
        e.yaw   = 0.02f;
        eulur_to_quater(&q,&e);
        float camera_dir_up[3] = {0.f,0.f,1.f};
        float camera_dir_front[3] = {1.f,0.f,0.f};
        quater_rotate(camera_dir_up,camera_dir_up,&q);
        quater_rotate(camera_dir_front,camera_dir_front,&q);
        cv::Vec3f cam_df(camera_dir_front[0],camera_dir_front[1],camera_dir_front[2]);
        cv::Vec3f cam_du(camera_dir_up[0],camera_dir_up[1],camera_dir_up[2]);

        cv::Affine3f cam_pose = cv::viz::makeCameraPose(cam_pos, cam_pos + cam_df, cam_du);
        cv::viz::WCameraPosition cpw(0.5); // Coordinate axes

        my3DWindow.showWidget("CPW", cpw, cam_pose);

        my3DWindow.showWidget("O1P1",O1P1);
        my3DWindow.showWidget("O1P2",O1P2);
        my3DWindow.showWidget("O2P1",O2P1);
        my3DWindow.showWidget("O2P2",O2P2);

        
        my3DWindow.showWidget("P1L",SP1L);
        my3DWindow.showWidget("P1R",SP1R);
        my3DWindow.showWidget("P2L",SP2L);
        my3DWindow.showWidget("P2R",SP2R);
        
        my3DWindow.spinOnce(1,false);  
        if(cv::waitKey(1) == '1'){
            break;
        } 
    }

    return 0;
}