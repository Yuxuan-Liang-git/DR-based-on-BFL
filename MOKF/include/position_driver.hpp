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

#include <Kalman_Filter.hpp>

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;

#define _MATH_DEFINES_DEFINED   //  使用PI需要cmath和定义这个

#define DYNAMIC_SEARCH

static double const_coef = 16;  //  方向盘到前轮转角的近似比值
static double wheelbase = 2.7;

const char* off_line_data="../data/OfflineData.txt";
const char* yaw_rate_data_path="../data/yaw_rate_data.csv";

const char* data_output_path="../data/Raw_data.csv";
const char* data_f_path="../data/data_f.csv";
const char* integrate_pos_output_path="../data/Integrate_pos.csv";
const char* MOKF_pos_output_path="../data/MOKF_pos.csv";

const char* Matrix_output_path="../data/Matrix.csv";


const char* const_coef_pos_output_path="../data/Const_coef_pos.csv";
const char* steer2wheel_dic_path="../data/steer2wheel_dic.csv";



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



void save_pos(vehicle_pos_s &pos,const char* _file_path);
int my_split(const std::string& src, const char& delim,std::vector<std::string>& vec);

class Raw_data
{
        // vector <long int> _gps_time_stamp;
        // vector <double> _gps_a;
        // vector <double> _gps_b;
        // vector <double> _gps_c;
        // vector <double> _gps_d;
    public:
        const char* data_address;
        vector <long int> time_stamp;
        vector <double> steering_wheel_angle;
        vector <double> yaw_rate;
        vector <double> speed;		//用于存放目标数据
        vector <double> lateral_acc;
        vector <double> longitude_acc;

        vector <double> deltaf;     //  前轮转角
        vector <VectorXd> data_filtered;


        vehicle_pos_s imu_intergrate_pos;
        vehicle_pos_s const_coef_pos;

        steer2wheel_dic_s steer2wheel_dic;

        Raw_data(const char* _data_path);
        void show(int i_min,int i_max);
        void save(const char* _file_path);
        void get_pos(vector <double> yaw_rate_t,vehicle_pos_s *pos);

        void save_intergrate_pos(const char* _file_path);
        void get_steer2wheel_dic(const char* _file_path);

        void get_const_coef_pos(const char* _file_path);
        void save_const_coef_pos(const char* _file_path);


        // void data_filtering(const char* _file_path);        
};

Raw_data::Raw_data(const char* _data_address):data_address(_data_address)
{
    // 文件打开
    ifstream ifs;							//创建流对象
	ifs.open(data_address, ios::in);	//打开文件
	string temp;						//把文件中的一行数据作为字符串放入容器中
    while (getline(ifs, temp))          //利用getline（）读取每一行，并放入到 item 中
	{

        if (strstr(temp.c_str(), "[CAN]") != NULL)
        {
            istringstream istr(temp);                 //其作用是把字符串分解为单词(在此处就是把一行数据分为单个数据)	
            string str;
		    int count = 0;							 //统计一行数据中单个数据个数
            while (istr >> str)                      //以空格为界，把istringstream中数据取出放入到依次s中
            {
                if (count == 0)                      
                {
                    string str_temp;
                    str_temp=str.substr(10,10);
                    long int r = atol(str_temp.c_str());    
                    time_stamp.push_back(r);
                }
                //获取第2列数据
                else if (count == 3)
                {
                    double r = atof(str.c_str());
                    steering_wheel_angle.push_back(r);
                }
                count++;
            }
            count = 0;
            getline(ifs, temp);
            istringstream tstr(temp);                 //其作用是把字符串分解为单词(在此处就是把一行数据分为单个数据)	
            while (tstr >> str)                      //以空格为界，把istringstream中数据取出放入到依次s中
            {
                // cout << "wah"<<endl;
                //获取第1列数据
                if (count == 2)                      
                {
                    double r = atof(str.c_str())*M_PI/180.0;    //deg->rad   
                    yaw_rate.push_back(r);
                }
                //获取第2列数据
                else if (count == 3)
                {
                    double r = atof(str.c_str())/3.6; //  km/h->m/s
                    speed.push_back(r);
                }
                //获取第3列数据
                else if (count == 4)
                {
                    double r = atof(str.c_str());
                    lateral_acc.push_back(r);
                }
                //获取第3列数据
                else if (count == 5)
                {
                    double r = atof(str.c_str());
                    longitude_acc.push_back(r);
                }
                count++;
            }
            for(int j=0;j<3;j++)
            {
                getline(ifs, temp);
                // 跳过后面两行
            }
        }
        // if (strstr(temp.c_str(), "[GPS]") != NULL)
        // {
        //     istringstream istr(temp);                 //其作用是把字符串分解为单词(在此处就是把一行数据分为单个数据)	
        //     string str;
		//     int count = 0;							 //统计一行数据中单个数据个数
        //     while (istr >> str)                      //以空格为界，把istringstream中数据取出放入到依次s中
        //     {
        //         if (count == 0)                      
        //         {
        //             string str_temp;
        //             str_temp=str.substr(10,10);
        //             long int r = atol(str_temp.c_str());
        //             gps_time_stamp.push_back(r);
        //         }
        //         //获取第1列数据
        //         if (count == 2)                      
        //         {
        //             double r = atof(str.c_str());    
        //             gps_a.push_back(r);
        //         }
        //         //获取第2列数据
        //         else if (count == 3)
        //         {
        //             double r = atof(str.c_str());
        //             gps_b.push_back(r);
        //         }
        //                         //获取第2列数据
        //         else if (count == 4)
        //         {
        //             double r = atof(str.c_str());
        //             gps_c.push_back(r);
        //         }
        //                         //获取第2列数据
        //         else if (count == 5)
        //         {
        //             double r = atof(str.c_str());
        //             gps_d.push_back(r);
        //         }
        //         count++;
        //     }
        // }
        // sleep(1);
	}
    ifs.close();
}

void Raw_data::show(int i_min,int i_max)
{
    for(int i=i_min;i<i_max;i++)
    {
        cout<<"steering_wheel_angle:"<<steering_wheel_angle[i]<<'\t'<<
            "yaw_rate:"<<yaw_rate[i]<<'\t'<<
            "speed:"<<speed[i]<<'\t'<<
            "lateral_acc:"<<lateral_acc[i]<<'\t'<<
            "longitude_acc:"<<longitude_acc[i]<<endl;
        sleep(1);
    }
}

void Raw_data::save(const char* _file_path)
{
    vector<double> duration;
    double dur;
    for(int i=0;i<time_stamp.size();i++)
    {
        dur=(time_stamp[i]-time_stamp[i-1])/1000000.0;
        if(dur>40.0)
        {
            dur-=40.0;
        }
        duration.push_back(dur);
    }


    ofstream outFile;
    outFile.open(_file_path, ios::out);
    auto p1 = time_stamp.begin();
    auto p2 = steering_wheel_angle.begin();
    auto p3 = speed.begin();
    auto p4 = yaw_rate.begin();    //  单位不确定..估计是deg/s
    auto p5 = lateral_acc.begin();
    auto p6 = duration.begin();
    // auto p6 = raw_data.gps_time_stamp.begin();
    // auto p7 = raw_data.gps_a.begin();
    // auto p8 = raw_data.gps_b.begin();  
    // auto p9 = raw_data.gps_c.begin();
    // auto p10 = raw_data.gps_d.begin();

    outFile <<  "time_stamp" <<',' << "steering_wheel_angle" <<',' <<"speed"  <<',' 
        <<"yaw_rate"<<',' <<"lateral_acc" <<','<<"duration"<<endl;
    for (; p1 != time_stamp.end(); p1++,p2++,p3++,p4++,p5++,p6++)
    {
    outFile <<  *p1 <<',' << *p2 <<',' <<*p3  <<',' <<*p4<<',' 
        <<*p5 <<','<<*p6<<endl;

    // outFile <<  *p6 <<',' << *p7 <<',' <<*p8  <<',' <<*p9<<',' <<*p10 <<endl;
    // outFile << *p1<<','<<*p2  <<endl;
    // cout << *p3<<','<<*p4  <<endl;
    // sleep(1);

    // outFile << *p3<<','<<*p4  <<endl;


    }
    outFile.close();
}

void Raw_data::get_pos(vector <double> yaw_rate_t,vehicle_pos_s *pos)
{
    double X=0.0,Y=0.0,PHI=0.0;
    double beta=0.0;
    double dur;
    for(int i=1;i<time_stamp.size();i++)
    {
        dur=(time_stamp[i]-time_stamp[i-1])/1000000.0;
        if(dur>40.0)
        {
            dur-=40.0;
        }
        X+=(cos(PHI)*speed[i]*dur);
        Y+=(sin(PHI)*speed[i]*dur);
        PHI+=yaw_rate_t[i]*dur;   
        // if(abs(yaw_rate_t[i])>0.01)
        // {
        //     PHI+=yaw_rate_t[i]*dur;   
        // }

        if(PHI>M_PI)
        {
            PHI-=2*M_PI;
        }else if(PHI<-M_PI)
        {
            PHI+=2*M_PI;
        }
        pos->X.push_back(X);
        pos->Y.push_back(Y);
        pos->PHI.push_back(PHI);
    }
}

void Raw_data::save_intergrate_pos(const char* _file_path)
{
    vehicle_pos_s raw_pos;
    get_pos(yaw_rate,&raw_pos);
    save_pos(raw_pos,_file_path);
}

void Raw_data::get_steer2wheel_dic(const char* _file_path)
{
    vector<string> vecSplit;            //存放分割后的数据
    ifstream ifs;							//创建流对象
    string temp;
	ifs.open(_file_path, ios::in);	//打开文件
    if (!ifs)
    {
        cout << "打开文件失败！" << endl;
        exit(1);
    }
    getline(ifs, temp);                 //   第一行是标签，舍弃
    while (getline(ifs,temp))          //利用getline（）读取每一行，并放入到 item 中
	{
        int iRet = my_split(temp, ',', vecSplit);    //  分割每行的数据，数据含有空白字符
        if (0 == iRet){                             //  成功分割数据，将每行数据写入结构体中
            int idex = 0;
            //  存储静态转向表
            for(int i=0;i<5;i++)
            {
                if(vecSplit[3*i].c_str()!=std::string(""))
                {
                    steer2wheel_dic.item[i].SteeringWheelAngle.push_back(atof(vecSplit[3*i].c_str()));
                    steer2wheel_dic.item[i].DeltaF_rad.push_back(atof(vecSplit[3*i+1].c_str()));
                    steer2wheel_dic.item[i].DeltaF_deg.push_back(atof(vecSplit[3*i+2].c_str()));
                }
            }
        }
	}
    ifs.close();
}



// void Raw_data::data_filtering(const char* _file_path)
// {
//     x = VectorXd(2,1); // 初始化状态矩阵
//     x << 0,15.3;
//     P = MatrixXd(2,2); // 初始化不确定性协方差矩阵
//     P << 0.001,0.001,0.001,0.001;
//     A = MatrixXd(1, 1); // 状态转移矩阵初始化
//     A << 0,0,0,1;
//     H = MatrixXd(1, 1); // 测量矩阵
//     H << 1,0,0,1;
//     R = MatrixXd(1, 1); // 测量协方差矩阵 即测量噪声
//     R << 0.01,0,0,1;
//     Q = MatrixXd(1, 1); // 过程协方差矩阵 即模型噪声 想平滑曲线这个值取小一些
//     Q << 0.01,0,0,1;
//     B = MatrixXd(1,1);  //  控制量u到x的变换阵
//     B << 1,0,0,1;

    
//     // U.resize(1,yaw_rate.size());

//     VectorXd temp(1);
//     VectorXd u_data;
//     double s;   //  传动比
//     double temp_angle;
//     vector<double> z_i;
//     s=15.3;

//     u_data = VectorXd::Zero(1);

//     for(int i=0;i<yaw_rate.size();i++)
//     {
//         temp<<yaw_rate[i];
//         temp_angle=steering_wheel_angle[i]*M_PI/180;
//         u_data<<((speed[i]*tan(temp_angle/s))/wheelbase);
//         U.push_back(u_data);
//         Z.push_back(temp); 
//         if(speed[i]!=0)
//         {
//             if(atan(yaw_rate[i]*wheelbase/speed[i])!=0)
//             {
//                 double temp=temp_angle/(atan(yaw_rate[i]*wheelbase/speed[i]));
//                 if(temp>10&&temp<20)
//                 {
//                     s=temp;
//                 }
//             }
//         }
//         z_i.push_back(s);
//     }

//     Kalman_Filter k(A,B,H,Q,R,U,Z);
//     data_filtered=k.calculate(x,P);

//     ofstream outFile;
//     outFile.open(_file_path, ios::out);
//     auto p1 = z_i.begin();
//     outFile <<  "z_i"  <<endl;
//     for (; p1 != z_i.end(); p1++)
//     {
//         outFile <<  *p1  <<endl;
//     }
//     outFile.close();
//     // k.data_output(_file_path);
// }

void save_pos(vehicle_pos_s &pos,const char* _file_path)
{
    ofstream outFile;
    outFile.open(_file_path, ios::out);
    auto p1 = pos.X.begin();
    auto p2 = pos.Y.begin();
    auto p3 = pos.PHI.begin();

    outFile <<  "X" <<',' << "Y" <<',' <<"PHI" <<endl;
    for (; p1 != pos.X.end(); p1++,p2++,p3++)
    {
        outFile <<  *p1 <<',' << *p2 <<',' <<*p3  <<endl;
    }
    outFile.close();
}

/*
名称:my_split(const std::string& src, const char& delim,
         std::vector<std::string>& vec)

功能:用分隔符将源字符串分隔为多个子串并传出; n个分隔符, 分n+1个子串
    

参数:
     src-传入参数, 源字符串;
     delim-传入参数, 分隔符;
     vec-传出参数, 子串的集合;
  
返回值:
    0-成功;
    其它-失败;
 */

int my_split(const std::string& src, const char& delim,
		std::vector<std::string>& vec)
{
	int src_len = src.length();
	int find_cursor = 0;
	int read_cursor = 0;
	if (src_len <= 0) return -1;
	vec.clear();
	while (read_cursor < src_len){
		find_cursor = src.find(delim, find_cursor);
		//1.找不到分隔符
		if (-1 == find_cursor){
			if (read_cursor <= 0) return -1;
			//最后一个子串, src结尾没有分隔符
			if (read_cursor < src_len){
				vec.push_back(src.substr(read_cursor, src_len - read_cursor));
				return 0;
			}
		}
		//2.有连续分隔符的情况
		else if (find_cursor == read_cursor){
			//字符串开头为分隔符, 也按空子串处理, 如不需要可加上判断&&(read_cursor!=0)
			vec.push_back(std::string(""));
		}
		//3.找到分隔符
		else
			vec.push_back(src.substr(read_cursor, find_cursor - read_cursor));
		read_cursor = ++find_cursor;
		if (read_cursor == src_len){
			//字符串以分隔符结尾, 如不需要末尾空子串, 直接return
			vec.push_back(std::string(""));
			return 0;
		} 
	}//end while()
	return 0;
}

#endif // POSITION_DRIVER_H__