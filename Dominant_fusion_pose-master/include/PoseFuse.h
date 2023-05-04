//
// Created by spring on 18-10-25.
//

#ifndef FUSIONPOSE_POSEFUSE_H
#define FUSIONPOSE_POSEFUSE_H
// bayesian filtering
#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>
//#include "nonlinearanalyticconditionalgaussianodo.h"
#include "datum.h"
#include "utm.h"
#include <sophus/se3.h>
#include <Eigen/Dense>
#include <mutex>

#include "nonlinearanalyticconditionalgaussianodo.h"

class Pose
{
public:
    Pose(Sophus::SE3 &tf, double t, bool valid = false)
    {
        Transform = tf;
        time = t;
        bValid = valid;
    }
    Pose() : time(-1), bValid(false)
    {
        ;
    }

    double time;
    Sophus::SE3 Transform;
    bool bValid;
};

class PoseFuse
{
public:
    PoseFuse();

    virtual ~PoseFuse();

    // Input sensor
    enum eSensor
    {
        gps = 0,
        imu = 1,
        vo = 2
    };

    bool update(const double filter_time);
    void initialize(const eSensor sensor);
    void addMeasurement(double speed, double steer, double time, double covar);
    void addMeasurement(const eSensor sensor, double x, double y, double z, double alpha1, double alpha2, double alpha3, double time, double covar);

    bool getInitialized()
    {
        return filter_initialized_;
    }

    bool getGPSInitialized()
    {
        return gps_initialized_;
    }

    bool getVOInitialized()
    {
        return vo_initialized_;
    }
    Sophus::SE3 getCurrentPose();
    Sophus::SE3 getCurrentPose(double &estimated_v);

    double getCurrentCov();

private:
    void GetStampedPose(Pose &T, Pose &oldT, double t);
    /// correct for angle overflow
    void angleOverflowCorrect(double &a, double ref);
    Sophus::SE3 Interpolate(Sophus::SE3 T1, Sophus::SE3 T2, float k);
    Sophus::SO3 SLERP(const Eigen::Quaterniond &q1, const Eigen::Quaterniond &q2, double k);

    // decompose Transform into x,y,z,Rx,Ry,Rz
    //    void decomposeTransform(const tf::StampedTransform& trans,
    //                            double& x, double& y, double&z, double&Rx, double& Ry, double& Rz);
    //    void decomposeTransform(const tf::Transform& trans,
    //                            double& x, double& y, double&z, double&Rx, double& Ry, double& Rz);

    // pdf / model / filter
    BFL::AnalyticSystemModelGaussianUncertainty *sys_model_;
    BFL::NonLinearAnalyticConditionalGaussianOdo *sys_pdf_;
    //    BFL::LinearAnalyticSystemModelGaussianUncertainty*            sys_model_;
    //    BFL::LinearAnalyticConditionalGaussian*           sys_pdf_;
    BFL::LinearAnalyticConditionalGaussian *odom_meas_pdf_;
    BFL::LinearAnalyticMeasurementModelGaussianUncertainty *odom_meas_model_;
    BFL::LinearAnalyticConditionalGaussian *imu_meas_pdf_;
    BFL::LinearAnalyticMeasurementModelGaussianUncertainty *imu_meas_model_;
    BFL::LinearAnalyticConditionalGaussian *vo_meas_pdf_;
    BFL::LinearAnalyticMeasurementModelGaussianUncertainty *vo_meas_model_;
    BFL::LinearAnalyticConditionalGaussian *gps_meas_pdf_;
    BFL::LinearAnalyticMeasurementModelGaussianUncertainty *gps_meas_model_;
    BFL::Gaussian *prior_;
    BFL::ExtendedKalmanFilter *filter_;
    MatrixWrapper::SymmetricMatrix odom_covariance_, imu_covariance_, vo_covariance_, gps_covariance_;

    // vars
    MatrixWrapper::ColumnVector vel_desi_, filter_estimate_old_vec_;
    MatrixWrapper::SymmetricMatrix filter_estimate_old_cov_;
    Pose filter_estimate_old_;
    Pose odom_meas_, odom_meas_old_, imu_meas_, imu_meas_old_, vo_meas_, vo_meas_old_, gps_meas_, gps_meas_old_;
    double filter_time_old_;
    bool filter_initialized_, odom_initialized_, imu_initialized_, vo_initialized_, gps_initialized_;

    // diagnostics
    double diagnostics_odom_rot_rel_, diagnostics_imu_rot_rel_;

    std::string output_frame_;
    std::string base_footprint_frame_;

    const double vehlen;
    const double kangle;
    const double kangleRight;
    std::mutex mMutex;

    double mSpeed;
    double mOmegaZ;
};

#endif //FUSIONPOSE_POSEFUSE_H
