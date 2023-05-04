#ifndef POSITION_HANDLE_H__
#define POSITION_HANDLE_H__
#include <iostream>
#include <vector>
#include <position_driver.hpp>
#include <Kalman_Filter.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <iomanip>
#include <fstream>
using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;

class Data_handle : public Raw_data
{
    public:
        using Raw_data::Raw_data;
        void data2yaw_rate_const(void);
        void save_yaw_rate(const char* _file_path);
        void MOKFilter(void);
        void save_MOKF_pos(const char* _file_path);
        void save_const_coef_pos(const char* _file_path);
        void save_kf_pos(const char* _file_path);
        void save_const_kf_pos(const char* _file_path);

        void save_matrix(vector<MatrixXd> temp_matrix,const char* _file_path);
        void save_vector(vector<VectorXd> temp_vector,const char* _file_path);


        vector<double> yaw_rate_kf;
        vector<double> yaw_rate_const;
        vector<double> yaw_rate_const_kf;
        vector<double> yaw_rate_MOKF;

        vector<VectorXd> A_weight;
        vector<MatrixXd> P_weight;
        vector<MatrixXd> K_k_1;
        vector<MatrixXd> K_k_2;
       
        // MatrixXd G;   
        // void EKFilter(VectorXd x_start,MatrixXd P_start)

};

// 用车速、方向盘转角直接推yaw_rate 其中方向盘转角到前轮转角取定转向比
void Data_handle::data2yaw_rate_const(void)
{
    double temp=0.0;
    for(int i=0;i<time_stamp.size();++i)
    {
        // 滤去方向盘原位的奇怪的值
        if(abs(steering_wheel_angle[i])>5.0)
        {
            temp=speed[i]*tan((steering_wheel_angle[i]*M_PI)/
                (180*const_coef))/wheelbase;
        }
        else{
            temp=0.0;
        }

        yaw_rate_const.push_back(temp);
    }
}

void Data_handle::save_yaw_rate(const char* _file_path)
{
    ofstream outFile;
    outFile.open(_file_path, ios::out);
    auto p1 = yaw_rate_const.begin();
    auto p2 = yaw_rate_const_kf.begin();
    auto p3 = yaw_rate.begin();
    auto p4 = yaw_rate_kf.begin();
    auto p5 = yaw_rate_MOKF.begin();

    outFile <<  "yaw_rate_const" <<',' << "yaw_rate_const_kf" <<','<<"yaw_rate"<<','
        << "yaw_rate_kf"<< ','<< "yaw_rate_MOKF"<<endl;
    for (; p1 != yaw_rate_const.end(); p1++,p2++,p3++,p4++,p5++)
    {
        outFile <<  *p1 <<',' << *p2 <<','<<*p3<<','<<*p4<<','<<*p5<<endl;
    }
    outFile.close();

//     ofstream outFile;
//     outFile.open(_file_path, ios::out);
//     auto p1 = yaw_rate_const.begin();
//     auto p2 = yaw_rate_const_kf.begin();
//     auto p3 = yaw_rate.begin();
//     auto p4 = yaw_rate_kf.begin();
//     outFile <<  "yaw_rate_const" <<',' << "yaw_rate_const_kf" <<','<<"yaw_rate"<<','
//         << "yaw_rate_kf"<<endl;
//     for (; p1 != yaw_rate_const.end(); p1++,p2++,p3++,p4++)
//     {
//         outFile <<  *p1 <<',' << *p2 <<','<<*p3<<','<<*p4<<endl;
//     }
//     outFile.close();

}

void Data_handle::MOKFilter(void)
{
    vector<VectorXd> Z_1; // 轮速推算得到的yaw_rate
    vector<VectorXd> Z_2; // 直接测量得到的yaw_rate
    vector<VectorXd> U; // 控制量   都是0

    vector<VectorXd> x_hat_1;
    vector<VectorXd> x_hat_2;
    vector<VectorXd> x_result;




    double a_1,a_2;     //  两个传感器对应的系数

    VectorXd x_start = VectorXd(1,1); // 初始化状态矩阵
    x_start << 0;
    MatrixXd P_1 = MatrixXd(1,1); // 初始化不确定性协方差矩阵
    MatrixXd P_2 = MatrixXd(1,1); // 初始化不确定性协方差矩阵
    P_1 << 0.001;
    P_2 << 0.001;
    MatrixXd A = MatrixXd(1, 1); // 状态转移矩阵初始化
    A << 1;
    MatrixXd H = MatrixXd(1, 1); // 测量矩阵
    H << 1;
    MatrixXd R_1 = MatrixXd(1, 1); // 测量协方差矩阵 即测量噪声
    MatrixXd R_2 = MatrixXd(1, 1); // 测量协方差矩阵 即测量噪声
    R_1 << 0.5;
    R_2 << 0.05;
    MatrixXd Q = MatrixXd(1, 1); // 过程协方差矩阵 即模型噪声 想平滑曲线这个值取小一些
    Q << 0.1;
    MatrixXd B = MatrixXd(1,1);  //  控制量u到x的变换阵
    B << 1;

    VectorXd temp(1);
    VectorXd u_data;
    
    u_data = VectorXd::Zero(1);             //  没有控制量
    for(int i=0;i<yaw_rate_const.size();i++)
    {
        temp<<yaw_rate_const[i];
        Z_1.push_back(temp);
        U.push_back(u_data);
    }
    // cout<<Z_1.size()<<endl;
    Kalman_Filter kf_1(A,B,H,Q,R_1,U,Z_1);

    kf_1.calculate(x_start,P_1,&x_hat_1,&K_k_1);
    // cout<<x_hat_1.size()<<endl;

    for(int i=0;i<yaw_rate.size();i++)
    {
        temp<<yaw_rate[i];
        Z_2.push_back(temp);
    }
    Kalman_Filter kf_2(A,B,H,Q,R_2,U,Z_2);
    kf_2.calculate(x_start,P_2,&x_hat_2,&K_k_2);

    double double_temp;
    for(int i=0;i<x_hat_1.size();i++)
    {
        double_temp = x_hat_1[i](0);
        yaw_rate_const_kf.push_back(double_temp);
        // cout << i << endl;
        double_temp = x_hat_2[i](0);
        // cout << x_hat_1.size() << '\t' << x_hat_2.size() << endl;
        yaw_rate_kf.push_back(double_temp);
    }

    // cout<<x_hat_2.size()<<endl;

    // kf_2.data_output(kf2_data_path);
    VectorXd e(2);      //  MOKF用到的单位矩阵构成的“列向量”
    e << 1,1;
    MatrixXd I = MatrixXd::Identity(1,1);  
    MatrixXd P = 0.1 * MatrixXd::Identity(2,2);  //  两个传感器间的协方差矩阵
    VectorXd A_dash = VectorXd(2); // 两个传感器之间的权重

    int P_error_pos = 0;

    for(int k=0;k<yaw_rate.size();++k)
    {
        if(P.determinant()!=0)   //  P可能不可逆
        {
            A_dash=P.inverse()*e*(e.transpose()*P.inverse()*e).inverse();   //  一直有更新
        }
        else{
            cout<<"P_cov Error!"<<endl;
            P_error_pos++;
            cout<<P<<'\t'<<P_error_pos<<endl;
            // break;
        }
        A_weight.push_back(A_dash);
        P_weight.push_back(P);
        VectorXd temp = x_hat_1[k]*A_dash[0]+x_hat_2[k]*A_dash[1];
        double_temp = temp[0];
        //  将滤波后的值保存出来
        yaw_rate_MOKF.push_back(double_temp);
        // 更新协方差矩阵
        P(0,0)=(1-K_k_1[k](0,0))*(P(0,0)+Q(0,0))*(1-K_k_1[k](0,0));
        P(0,1)=(1-K_k_1[k](0,0))*(P(0,1)+Q(0,0))*(1-K_k_2[k](0,0));
        P(1,0)=(1-K_k_2[k](0,0))*(P(1,0)+Q(0,0))*(1-K_k_1[k](0,0));
        P(1,1)=(1-K_k_2[k](0,0))*(P(1,1)+Q(0,0))*(1-K_k_2[k](0,0));
        // if(k>128&&k<140)
        // {
        //     // A项里面有负数...不知道是不是对的...
        //     cout<< "K_k_1:"<<K_k_1[k](0,0)<<'\t'<<"K_k_2:"<<K_k_2[k](0,0)<<endl;
        //     // cout<<"P"<<'\n'<<P<<endl;
        //     cout<<A_dash[0]<<'\t'<<A_dash[1]<<'\t'<<A_dash[0]+A_dash[1]<<endl;

        //     // cout<<"yaw_rate_const"<< '\t' << "yaw_rate"<< endl
        //     //     <<yaw_rate_const[k]<< '\t' << yaw_rate[k]<< endl;

        //     cout<<"yaw_rate_const_kf"<< '\t' << "yaw_rate_kf"<< '\t' << "yaw_rate_MOKF"<< endl
        //         <<yaw_rate_const_kf[k]<< '\t' << yaw_rate_kf[k]<<'\t'<< yaw_rate_MOKF[k] << endl << endl;

        //     sleep(1);
        // }
    }
    
}

void Data_handle::save_MOKF_pos(const char* _file_path)
{

    vehicle_pos_s MOKF_pos;
    get_pos(yaw_rate_MOKF,&MOKF_pos);
    save_pos(MOKF_pos,_file_path);
}

void Data_handle::save_const_coef_pos(const char* _file_path)
{
    vehicle_pos_s const_coef_pos;
    get_pos(yaw_rate_const,&const_coef_pos);
    save_pos(const_coef_pos,_file_path);
}

void Data_handle::save_const_kf_pos(const char* _file_path)
{
    vehicle_pos_s const_kf_pos;
    get_pos(yaw_rate_const_kf,&const_kf_pos);
    save_pos(const_kf_pos,_file_path);
}

void Data_handle::save_kf_pos(const char* _file_path)
{
    vehicle_pos_s kf_pos;
    get_pos(yaw_rate_kf,&kf_pos);
    save_pos(kf_pos,_file_path);
}

void Data_handle::save_matrix(vector<MatrixXd> temp_matrix,const char* _file_path)
{
    ofstream outFile;
    outFile.open(_file_path, ios::out);

    for(int i=0;i<temp_matrix.size();++i) 
    {
        for(int j=0;j<temp_matrix[i].size();++j)
        {
            outFile<<temp_matrix[i](j)<<',';
        }
        outFile<<endl;

    }

    outFile.close();
}

void Data_handle::save_vector(vector<VectorXd> temp_vector,const char* _file_path)
{
    ofstream outFile;
    outFile.open(_file_path, ios::out);

    for(int i=0;i<temp_vector.size();++i) 
    {
        for(int j=0;j<temp_vector[i].size();++j)
        {
            outFile<<temp_vector[i](j)<<',';
        }
        outFile<<endl;

    }

    outFile.close();
}

// void Data_handle::EKFilter(VectorXd x_start,MatrixXd P_start)
// {
//     double v,delta_s,s,yaw_rate;
//     double a_temp,b_temp,c_temp,d_temp;     //  计算的中间值

//     VectorXd x_hat_pre=x_start;             //  先验值
//     VectorXd x_hat_temp;    //  后验值

//     MatrixXd P_pre=P_start;                 //  先验方差矩阵
//     MatrixXd P_temp;        //  后验方差矩阵

//     MatrixXd K_k; //  卡尔曼增益
//     for (unsigned int n = 1; n < z.size(); ++n) {

//         // cout << "n=" << n << endl;
//         /**
//          * KF Measurement update step
//          */
//         MatrixXd Ht = H.transpose();
//         MatrixXd S = H * P_pre * Ht + R;
//         MatrixXd Si = S.inverse();
//         MatrixXd K = P_pre * Ht * Si;
//         // cout << "K=" << endl
//         //     << K << endl;
//         // new state

//         VectorXd h = z[n-1]+H*(x_hat_pre-x_hat_temp);   //  参考状态下的理想观测值与雅可比阵得到的一阶偏差和
//         x_hat_temp = x_hat_pre + K * (z[n]-h);
//         x_hat.push_back(x_hat_temp);
//         MatrixXd I = MatrixXd::Identity(H.cols(),H.cols());
//         P_temp = (I - K * H) * P_pre;
//         // cout << "x_hat_temp=" << endl
//         //     << x_hat_temp << endl;
//         /**
//          * KF Prediction step
//          */
//         delta_s=x_hat_temp[0];
//         v=x_hat_temp[1];
//         yaw_rate=x_hat_temp[2];
//         s=x_hat_temp[3];       //   转向比
        
//         if(s!=0)
//         {
//             a_temp = delta_s/s;
//         }
//         if(v!=0)
//         {
//             b_temp = yaw_rate*wheelbase/v;
//         }
//         c_temp = pow(v,2)+pow(yaw_rate*wheelbase,2);
//         //  更新雅可比矩阵A 保证在不出现异常值时才更新
//         if(s!=0&&cos(a_temp)!=0&&atan(b_temp)!=0&&c_temp!=0)  
//         {
//             A[2][0]=v/(s*wheelbase*pow(cos(a_temp),2));
//             A[2][1]=tan(a_temp)/wheelbase;
//             A[2][3]=-(v*delta_s)/(wheelbase*pow(s*cos(a_temp),2));
//             A[3][0]=1/atan(b_temp);
//             A[3][1]=(delta_s*yaw_rate*wheelbase)/
//                 ((pow(v,2)+pow(yaw_rate*wheelbase,2))*pow(atan(b_temp),2));
//             A[3][2]=-(delta_s*v*wheelbase)/
//                 ((pow(v,2)+pow(yaw_rate*wheelbase,2)*pow(atan(),2)));
//         }
//         x_hat_pre = A * x_hat_temp + u[n];
//         MatrixXd At = A.transpose();
//         P_pre = A * P_temp * At + Q;
//         // cout << "x_hat_pre=" << endl
//         //     << x_hat_pre << endl;
//     }
// }


#endif //   POSITION_HANDLE_H__