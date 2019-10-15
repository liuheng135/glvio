#include "csv.hpp"
#include <fstream>


using namespace std;

int find_char(char *buf,char pattern)
{
    int i = 0;
    while(buf[i] != '\0'){
        if(buf[i] == pattern){
            return i;
        }
        i++;
    }
    return -1;
}

int read_csv(string filename,vector< vector< string > >& vv,int max_line)
{
    ifstream filein(filename);
    char linebuff[512];
    char strbuf[512];
    char *ptr;
    int line_num;
    int i,pos;

    if (filein.is_open()) 
	{
        for(i = 0; i < max_line; i++){
            if(filein.getline(linebuff,512)){
                ptr = &linebuff[0];
                while(1){
                    pos = find_char(ptr,',');
                    if(pos < 0){
                        int len = strlen(ptr);
                        strncpy(strbuf,ptr,len-1);
                        string *s = new string(strbuf);
                        vv[i].push_back(*s);
                        delete s;
                        break;
                    }else{
                        strncpy(strbuf,ptr,pos);
                        strbuf[pos] = '\0';
                        string *s = new string(strbuf);
                        vv[i].push_back(*s);
                        delete s;
                        ptr += pos+1;
                    }
                }
            }else{
                cout << "failed to read a line" << endl;
                break;
            }
        }
        return i-1;
    }else{
        cout << "can not open file:" << filename << endl;
    }
    return -1;
}



int load_imu_data(string path,vector<imu_data>& imu,int max_num)
{
    int i;
    int loaded_num;
    vector < vector < string > >  csv_value;
    
    csv_value.resize(max_num+1);
    
    loaded_num = read_csv(path,csv_value,max_num+1);

    cout << "imu title is :" << endl;
    for(int i = 0;i < csv_value[0].size();i++){
        cout << csv_value[0][i] << ",";
    }
    cout << endl;

    for(i = 0; i < loaded_num;i++){
        imu[i].timestamp = atoll(csv_value[i+1][0].c_str());
        imu[i].gyro[0]  = atof(csv_value[i+1][1].c_str());
        imu[i].gyro[1]  = atof(csv_value[i+1][2].c_str());
        imu[i].gyro[2]  = atof(csv_value[i+1][3].c_str());
        imu[i].acc[0] = atof(csv_value[i+1][4].c_str());
        imu[i].acc[1] = atof(csv_value[i+1][5].c_str());
        imu[i].acc[2] = atof(csv_value[i+1][6].c_str());
    }
    csv_value.clear();
    return loaded_num;
}

int load_img(string path,vector<img_info>& img,int max_num)
{
    int i;
    int loaded_num;
    vector< vector< string > >  csv_value;
    csv_value.resize(max_num+1);
    
    loaded_num = read_csv(path,csv_value,max_num+1);

    for(i = 0; i < loaded_num;i++){
        img[i].timestamp = atoll(csv_value[i+1][0].c_str());
        img[i].filename  = csv_value[i+1][1];
    }
    csv_value.clear();
    return loaded_num;
}

int load_pose(string path,std::vector<pose_info>& ap,int max_num)
{
    int i;
    int loaded_num;
    vector< vector< string > >  csv_value;
    csv_value.resize(max_num+1);
    
    loaded_num = read_csv(path,csv_value,max_num+1);

    for(i = 0; i < loaded_num;i++){
        ap[i].timestamp = atoll(csv_value[i+1][0].c_str());
        ap[i].pos[0]  = atof(csv_value[i+1][1].c_str());
        ap[i].pos[1]  = atof(csv_value[i+1][2].c_str());
        ap[i].pos[2]  = atof(csv_value[i+1][3].c_str());
        ap[i].Q[0]    = atof(csv_value[i+1][4].c_str());
        ap[i].Q[1]    = atof(csv_value[i+1][5].c_str());
        ap[i].Q[2]    = atof(csv_value[i+1][6].c_str());
        ap[i].Q[3]    = atof(csv_value[i+1][7].c_str());
        ap[i].vel[0]  = atof(csv_value[i+1][8].c_str());
        ap[i].vel[1]  = atof(csv_value[i+1][9].c_str());
        ap[i].vel[2]  = atof(csv_value[i+1][10].c_str());
    }
    csv_value.clear();
    return loaded_num;
}

void save_pos(string path,std::vector<pose_info> pos)
{
    ofstream fileout(path);

    if(fileout.is_open()){
        for(int i = 0;i < pos.size();i++){
            fileout << pos[i].timestamp << ","<<pos[i].pos[0] << "," << pos[i].pos[1] << "," << pos[i].pos[2]  \
            << "," << pos[i].Q[0] << "," << pos[i].Q[1] << "," << pos[i].Q[2] << "," << pos[i].Q[3] << "," \
            << endl;
        }
        fileout.close();
    }
}

void save_imu_att(std::string path,std::vector<pose_info> pos)
{
    ofstream fileout(path);

    if(fileout.is_open()){
        for(int i = 0;i < pos.size();i++){
            fileout << pos[i].timestamp << ","<<pos[i].pos[0] << "," << pos[i].pos[1] << "," << pos[i].pos[2]  \
            << "," << pos[i].eulur[0] << "," << pos[i].eulur[1] << "," << pos[i].eulur[2] << "," << endl;
        }
        fileout.close();
    }
}

void save_cloud(std::string path,std::vector<point4f> points)
{
    ofstream fileout(path);

    if(fileout.is_open()){
        for(int i = 0;i < points.size();i++){
            fileout << points[i].x << "," << points[i].y << "," << points[i].z << "," << points[i].s << endl;
        }
        fileout.close();
    }
}