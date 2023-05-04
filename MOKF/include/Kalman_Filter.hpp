#ifndef KALMAN_FILTER_H__
#define KALMAN_FILTER_H__

#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;


class Kalman_Filter
{
    private:
        vector<VectorXd> u,z;   //  输入量与观测值 
        MatrixXd Q,A,B,R,H;   
        vector<VectorXd> x_hat;
        vector<MatrixXd> K_k;
    public:

        Kalman_Filter(MatrixXd _A,MatrixXd _B,
            MatrixXd _H,MatrixXd _Q,MatrixXd _R,
            vector<VectorXd> _u,vector<VectorXd> _z):
            A(_A),B(move(_B)),H(move(_H)),
            Q(move(_Q)),R(move(_R)),u(move(_u)),z(move(_z))
        {
            cout <<"Initializing with Kalman_Filter!" << endl;
        }
        void calculate(VectorXd x_start,MatrixXd P_start,
            vector<VectorXd> *x_hat,vector<MatrixXd> *K_k);  
        void data_output(const char* _file_path);

};


void Kalman_Filter::calculate
    (VectorXd x_start,MatrixXd P_start,vector<VectorXd> *_x_hat,
        vector<MatrixXd> *_K_k)
{
    VectorXd x_hat_pre;             //  先验值
    VectorXd x_hat_temp=x_start;    //  后验值

    MatrixXd P_pre;                 //  先验方差矩阵
    MatrixXd P_temp=P_start;        //  后验方差矩阵

    for (unsigned int n = 0; n < z.size(); ++n) {
        // cout << "n=" << n << endl;
        /**
         * KF Prediction step
        */
        // cout << A.size()<< '\t' << B.size() << '\t' << u.size();

        x_hat_pre = A * x_hat_temp + B * u[n];
        MatrixXd At = A.transpose();
        P_pre = A * P_temp * At + Q;
        // cout << "x_hat_pre=" << endl
        //     << x_hat_pre << endl;
        /**
         * KF Measurement update step
         */
        // cout<<"z:"<<z[n]<<endl;
        MatrixXd Ht = H.transpose();
        MatrixXd S = H * P_pre * Ht + R;
        MatrixXd Si = S.inverse();
        MatrixXd K = P_pre * Ht * Si;
        _K_k->push_back(K);
        // new state
        MatrixXd I = MatrixXd::Identity(H.cols(),H.cols());
        VectorXd y = z[n] - H * x_hat_pre;
        x_hat_temp = x_hat_pre + (K * y);
        _x_hat->push_back(x_hat_temp);
        P_temp = (I - K * H) * P_pre;
        // cout << "x_hat_temp=" << endl
        //     << x_hat_temp << endl;

        // if(n>40)
        // {
        //     cout<<"n:"<<n<<endl;
        //     cout << "K=" << endl
        //         << K << endl;
        //     sleep(1);
        // }
    }

    //  留档给自身调用
    x_hat.assign(_x_hat->begin(), _x_hat->end());
    K_k.assign(_K_k->begin(), _K_k->end());

}

void Kalman_Filter::data_output(const char* _file_path)
{
    // for(int i=0;i<z.size();i++)
    // {
    //     cout<<"z:"<<z[i]<<'\n'<<"x_hat"<<endl<<x_hat[i]<<endl;
    //     sleep(1);
    // }

    ofstream outFile;
    outFile.open(_file_path, ios::out);
    auto p1 = z.begin();
    auto p2 = x_hat.begin();

    outFile <<  "Z" <<',' << "X_hat"  <<endl;
    for (; p1 != z.end(); p1++,p2++)
    {
        outFile <<  *p1 <<',' << *p2  <<endl;
    }
    outFile.close();
}


#endif //   KALMAN_FILTER_H__