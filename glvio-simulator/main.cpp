#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include "quaternion.h"
#include <opencv2/viz.hpp>
#include "geometry.h"

using namespace std;
using namespace cv;

void get_cross(Point3d &cross,Point3d ls,Point3d le,Point3d P0,Point3d nv)
{
    double tmp;
    double t;
    Point3d ldv;

    ldv = le - ls;

    t = nv.x * (P0.x - ls.x) + nv.y * (P0.y - ls.y) + nv.z * (P0.z - ls.z);
    tmp = nv.x * ldv.x + nv.y * ldv.y + nv.z * ldv.z;
    if(tmp != 0.0){
        t /= tmp;
    }else{
        cout << "ERROR:t = inf" << endl;
        t = 0;
    }

    cross.x = ldv.x * t + ls.x;
    cross.y = ldv.y * t + ls.y;
    cross.z = ldv.z * t + ls.z;
}

int main(int argc, char *argv[])
{
    cv::Vec3f cam_pos(0.f,0.f,0.f);
    cv::viz::Viz3d my3DWindow("3D sense");
    my3DWindow.showWidget("3D sense", cv::viz::WCoordinateSystem());
   
    printf("simulator start...\r\n");

    Point3d P1(2000.0,500.0,400.0),P2(1500.0,-200.0,-100.0);
    Point3d O1(0.0,0.0,0.0),O2(-15.0,30.0,-10.0);

    viz::WLine O1P1(O1,P1,viz::Color::red());
    viz::WLine O1P2(O1,P2,viz::Color::blue());
    viz::WLine O2P1(O2,P1,viz::Color::red());
    viz::WLine O2P2(O2,P2,viz::Color::blue());

    Point3d P1L,P1R,P2L,P2R;
    Point3d Camera1_P(O1.x + 50.0,O1.y,O1.z),Camera1_NV(1.0,0.0,0.0);
    Point3d Camera2_P(O2.x + 50.0,O2.y,O2.z),Camera2_NV(1.0,0.0,0.0);

    get_cross(P1L,O1,P1,Camera1_P,Camera1_NV);
    get_cross(P1R,O2,P1,Camera2_P,Camera2_NV);
    get_cross(P2L,O1,P2,Camera1_P,Camera1_NV);
    get_cross(P2R,O2,P2,Camera2_P,Camera2_NV);

    cout << "O1 = " << O1 << endl;
    cout << "O2 = " << O2 << endl << endl;

    cout << "P1L = " << P1L << endl;
    cout << "P1R = " << P1R << endl << endl;
    cout << "P2L = " << P2L << endl;
    cout << "P2R = " << P2R << endl << endl;

    viz::WSphere SP1L(P1L,3,20,viz::Color::red());
    viz::WSphere SP1R(P1R,3,20,viz::Color::green());
    viz::WSphere SP2L(P2L,3,20,viz::Color::blue());
    viz::WSphere SP2R(P2R,3,20,viz::Color::yellow());
    viz::WSphere SO1(O1,3,20,viz::Color::white());
    viz::WSphere SO2(O2,3,20,viz::Color::white());

    struct geo_feature_s f1,f2;

    f1.observer_prev.point_pos_in_camera.x = P1L.x;
    f1.observer_prev.point_pos_in_camera.y = P1L.y;
    f1.observer_prev.point_pos_in_camera.z = P1L.z;
    f1.observer_prev.point_pos_valid = 1;

    f1.observer_next.point_pos_in_camera.x = P1R.x - O2.x;
    f1.observer_next.point_pos_in_camera.y = P1R.y - O2.y;
    f1.observer_next.point_pos_in_camera.z = P1R.z - O2.z;
    f1.observer_next.point_pos_valid = 1;

    f2.observer_prev.point_pos_in_camera.x = P2L.x;
    f2.observer_prev.point_pos_in_camera.y = P2L.y;
    f2.observer_prev.point_pos_in_camera.z = P2L.z;
    f2.observer_prev.point_pos_valid = 1;

    f2.observer_next.point_pos_in_camera.x = P2R.x - O2.x;
    f2.observer_next.point_pos_in_camera.y = P2R.y - O2.y;
    f2.observer_next.point_pos_in_camera.z = P2R.z - O2.z;
    f2.observer_next.point_pos_valid = 1;

    struct point3f T;

    geo_recovery_translation_2D2D(&f1,&f2);
    geo_recovery_depth(&f1);
    geo_recovery_depth(&f2);

    T.x = f1.observer_next.camera_pos_in_world.x;
    T.y = f1.observer_next.camera_pos_in_world.y;
    T.z = f1.observer_next.camera_pos_in_world.z;

    cout << "O2=" << O2 << endl;
    cout << "T2 =" << T.x << " , " << T.y << " , " << T.z << endl;
    
    cout << endl;
    cout << "P1 =" << P1 << endl;
    cout << "f1=" << f1.pos_in_world.x << " , " << f1.pos_in_world.y << " , " << f1.pos_in_world.z << endl;
    cout << endl;
    cout << "P2 =" << P2 << endl;
    cout << "f2=" << f2.pos_in_world.x << " , " << f2.pos_in_world.y << " , " << f2.pos_in_world.z << endl;

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
        cv::viz::WCameraPosition cpw(10); // Coordinate axes

        my3DWindow.showWidget("CPW", cpw, cam_pose);

        my3DWindow.showWidget("O1P1",O1P1);
        my3DWindow.showWidget("O1P2",O1P2);
        my3DWindow.showWidget("O2P1",O2P1);
        my3DWindow.showWidget("O2P2",O2P2);

        
        my3DWindow.showWidget("P1L",SP1L);
        my3DWindow.showWidget("P1R",SP1R);
        my3DWindow.showWidget("P2L",SP2L);
        my3DWindow.showWidget("P2R",SP2R);
        my3DWindow.showWidget("SO1",SO1);
        my3DWindow.showWidget("SO2",SO2);
        
        my3DWindow.spinOnce(1,false);  
        if(cv::waitKey(1) == '1'){
            break;
        } 
    }

    return 0;
}