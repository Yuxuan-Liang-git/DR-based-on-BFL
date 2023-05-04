#ifndef POSITION_DRIVER_H__
#define POSITION_DRIVER_H__

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <vector>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <algorithm>    // std::upper_bound
#include <cmath>
#include <proj.h>

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;

#define _MATH_DEFINES_DEFINED   //  使用PI需要cmath和定义这个

#define DYNAMIC_SEARCH

static double const_coef = 15.3;  //  方向盘到前轮转角的近似比值
static double wheelbase = 2.7;

static const char* off_line_data="../data/OfflineData.txt";
static const char* output_data="../output/output_data.csv";

static const char* yaw_rate_data_path="../output/yaw_rate_data.csv";

static const char* pos_path="../output/pos.csv";
static const char* pos_f_path="../output/pos_f.csv";
static const char* pos_speed_f_path="../output/pos_speed_f.csv";
static const char* integrate_pos_output_path="../output/Integrate_pos.csv";
static const char* gps_pos_path="../output/gps_pos.csv";
static const char* gps_data_path="../output/gps_data.csv";

static const char* ekf_data_path="../output/ekf_data.csv";
static const char* ekf_enu_data_path="../output/ekf_enu_data.csv";

struct vehicle_pos_s
{
    vector<double> X,Y,PHI;
};
struct steer2wheel_dic_item_s
{
    vector <double> SteeringWheelAngle;
    vector <double> DeltaF_rad;
    vector <double> DeltaF_deg;
};
struct steer2wheel_dic_s
{
    steer2wheel_dic_item_s item[5];
};

int my_split(const std::string& src, const char& delim,std::vector<std::string>& vec);

class Raw_data
{

    public:
        const char* data_address;
        vector <long int> time_stamp,gps_time_stamp,imu_time_stamp;
        vector <double> steering_wheel_angle;
        vector <double> yaw_rate;
        vector <double> speed;		//用于存放目标数据
        vector <double> lateral_acc;
        vector <double> longitude_acc;
        vector <double> deltaf;     //  前轮转角
        vector <double> gps_precision,gps_longitude,gps_lattitude,gps_orientation;

        vehicle_pos_s raw_pos;
        vehicle_pos_s imu_intergrate_pos;
        vehicle_pos_s pos,pos_speed_f;
        vehicle_pos_s gps_pos,gps_pos_tf;

        steer2wheel_dic_s steer2wheel_dic;

        Raw_data(const char* _data_path);
        void show(int i_min,int i_max);
        void save(const char* _file_path);
        void get_pos(vector <double> yaw_rate_t,vector <double> speed_t,vehicle_pos_s *pos);

        void save_intergrate_pos(const char* _file_path);
        void save_filtered_pos(const char* _file_path);

        void get_steer2wheel_dic(const char* _file_path);

        void get_const_coef_pos(const char* _file_path);
        void save_const_coef_pos(const char* _file_path);

        void save_pos(vehicle_pos_s pos,const char* _file_path);
        void save_enu(vector <double> gps_lon_t,vector <double> gps_lat_t,const char* _file_path);


        
        // void data_filtering(const char* _file_path);        
};

#endif // POSITION_DRIVER_H__