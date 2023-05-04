#ifndef FILTERED_DATA_H__
#define FILTERED_DATA_H__

#include "Raw_data.h"
#include <filter/extendedkalmanfilter.h>

#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>

#include <pdf/analyticconditionalgaussian.h>
#include <pdf/linearanalyticconditionalgaussian.h>
#include "nonlinearanalyticconditionalgaussianmobile.h"//added

using namespace MatrixWrapper;
using namespace BFL;
using namespace std;

class Filtered_data:public Raw_data
{
public:
        Filtered_data(const char* _data_address):Raw_data(_data_address)
        {

        }
        void get_ekf_data(void);
        void save_ekf_data(const char* _file_path);
        void save_speed_f_pos(const char* _file_path);
        void save_filtered_pos(const char* _file_path);
        void save_gps_data(const char* _file_path);
        void save_filtered_pos_enu(const char* _file_path);

        int gps_enu2utm(vector <double> gps_precision_t,vector <double>gps_longitude_t,
            vector <double> gps_lattitude_t,vehicle_pos_s *pos);
        int gps_utm2enu(vector <double> gps_enu_e_t,vector <double>gps_enu_n_t,
            vector <double> *gps_lon_t,vector <double> *gps_lat_t);
        void gps2pos_transform(void);
        void pos2gps_transform(void);
        void save_gps_pos(const char* _file_path);

        vector <double> yaw_rate_f;
        vector <double> speed_f;		//用于存放目标数据
        vector <double> lateral_acc_f;
        vector <double> longitude_acc_f;
        Eigen::Matrix3d R_Matrix;
        double gps_X_offset,gps_Y_offset,delta_PHI;
        vector <double> gps_ekf_lon,gps_ekf_lat,gps_odom_lon,gps_odom_lat;

        vehicle_pos_s pos_f;
};

#endif // FILTERED_DATA_H__