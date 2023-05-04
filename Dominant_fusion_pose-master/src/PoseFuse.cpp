//
// Created by spring on 18-10-25.
//

#include "PoseFuse.h"
using namespace MatrixWrapper;
using namespace BFL;
using namespace std;
PoseFuse::PoseFuse() : prior_(NULL),
                       filter_(NULL),
                       filter_initialized_(false),
                       odom_initialized_(false),
                       imu_initialized_(false),
                       vo_initialized_(false),
                       gps_initialized_(false),
                       gps_covariance_(6),
                       imu_covariance_(3),
                       vo_covariance_(6),
                       odom_covariance_(6),
                       vehlen(2.65),
                       kangle(15.85),
                       kangleRight(15.98)
{
    gps_covariance_ = 0;
    imu_covariance_ = 0;
    vo_covariance_ = 0;
    odom_covariance_ = 0;
    // create SYSTEM MODEL
    ColumnVector sysNoise_Mu(8);
    sysNoise_Mu = 0;
    SymmetricMatrix sysNoise_Cov(8);
    sysNoise_Cov = 0;
    for (unsigned int i = 1; i <= 8; i++)
        sysNoise_Cov(i, i) = pow(1, 2);
    Gaussian system_Uncertainty(sysNoise_Mu, sysNoise_Cov);

    sys_pdf_ = new NonLinearAnalyticConditionalGaussianOdo(system_Uncertainty);
    sys_model_ = new AnalyticSystemModelGaussianUncertainty(sys_pdf_);

    Matrix A(6, 6);
    A = 0;
    A(1, 1) = 1;
    A(2, 2) = 1;
    A(3, 3) = 1;
    A(4, 4) = 1;
    A(5, 5) = 1;
    A(6, 6) = 1;

    Matrix B(6, 6);
    B = 0;
    vector<Matrix> AB(2);
    AB[0] = A;
    AB[1] = B;
    //sys_pdf_ = new LinearAnalyticConditionalGaussian(AB,system_Uncertainty);
    //sys_model_ = new LinearAnalyticSystemModelGaussianUncertainty(sys_pdf_);

    // create MEASUREMENT MODEL ODOM
    ColumnVector measNoiseOdom_Mu(6);
    measNoiseOdom_Mu = 0;
    SymmetricMatrix measNoiseOdom_Cov(6);
    measNoiseOdom_Cov = 0;
    for (unsigned int i = 1; i <= 6; i++)
        measNoiseOdom_Cov(i, i) = 1;
    Gaussian measurement_Uncertainty_Odom(measNoiseOdom_Mu, measNoiseOdom_Cov);
    Matrix Hodom(6, 8);
    Hodom = 0;
    Hodom(1, 1) = 1;
    Hodom(3, 3) = 1;
    Hodom(5, 5) = 1;
    odom_meas_pdf_ = new LinearAnalyticConditionalGaussian(Hodom, measurement_Uncertainty_Odom);
    odom_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(odom_meas_pdf_);

    // create MEASUREMENT MODEL IMU
    ColumnVector measNoiseImu_Mu(3);
    measNoiseImu_Mu = 0;
    SymmetricMatrix measNoiseImu_Cov(3);
    measNoiseImu_Cov = 0;
    for (unsigned int i = 1; i <= 3; i++)
        measNoiseImu_Cov(i, i) = 1;
    Gaussian measurement_Uncertainty_Imu(measNoiseImu_Mu, measNoiseImu_Cov);
    Matrix Himu(3, 8);
    Himu = 0;
    Himu(1, 4) = 1;
    Himu(2, 5) = 1;
    Himu(3, 6) = 1;
    imu_meas_pdf_ = new LinearAnalyticConditionalGaussian(Himu, measurement_Uncertainty_Imu);
    imu_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(imu_meas_pdf_);

    // create MEASUREMENT MODEL VO
    ColumnVector measNoiseVo_Mu(6);
    measNoiseVo_Mu = 0;
    SymmetricMatrix measNoiseVo_Cov(6);
    measNoiseVo_Cov = 0;
    for (unsigned int i = 1; i <= 6; i++)
        measNoiseVo_Cov(i, i) = 1;
    Gaussian measurement_Uncertainty_Vo(measNoiseVo_Mu, measNoiseVo_Cov);
    Matrix Hvo(6, 8);
    Hvo = 0;
    Hvo(1, 1) = 1;
    Hvo(2, 2) = 1;
    Hvo(3, 3) = 1;
    Hvo(4, 4) = 1;
    Hvo(5, 5) = 1;
    Hvo(6, 6) = 1;
    vo_meas_pdf_ = new LinearAnalyticConditionalGaussian(Hvo, measurement_Uncertainty_Vo);
    vo_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(vo_meas_pdf_);

    // create MEASUREMENT MODEL GPS
    //    ColumnVector measNoiseGps_Mu(3);  measNoiseGps_Mu = 0;
    //    SymmetricMatrix measNoiseGps_Cov(3);  measNoiseGps_Cov = 0;
    //    for (unsigned int i = 1; i <= 3; i++) measNoiseGps_Cov(i, i) = 1;
    //    Gaussian measurement_Uncertainty_GPS(measNoiseGps_Mu, measNoiseGps_Cov);
    //    Matrix Hgps(3, 6);  Hgps = 0;
    //    Hgps(1, 1) = 1;    Hgps(2, 2) = 1;    Hgps(3, 3) = 1;
    //    gps_meas_pdf_ = new LinearAnalyticConditionalGaussian(Hgps, measurement_Uncertainty_GPS);
    //    gps_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(gps_meas_pdf_);

    ColumnVector measNoiseGps_Mu(6);
    measNoiseGps_Mu = 0;
    SymmetricMatrix measNoiseGps_Cov(6);
    measNoiseGps_Cov = 0;
    for (unsigned int i = 1; i <= 6; i++)
        measNoiseGps_Cov(i, i) = 1;
    Gaussian measurement_Uncertainty_GPS(measNoiseGps_Mu, measNoiseGps_Cov);
    Matrix Hgps(6, 8);
    Hgps = 0;
    Hgps(1, 1) = 1;
    Hgps(3, 3) = 1;
    Hgps(5, 5) = 1;
    gps_meas_pdf_ = new LinearAnalyticConditionalGaussian(Hgps, measurement_Uncertainty_GPS);
    gps_meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(gps_meas_pdf_);
}

PoseFuse::~PoseFuse()
{
    if (filter_)
        delete filter_;
    if (prior_)
        delete prior_;
    delete odom_meas_model_;
    delete odom_meas_pdf_;
    delete imu_meas_model_;
    delete imu_meas_pdf_;
    delete vo_meas_model_;
    delete vo_meas_pdf_;
    delete gps_meas_model_;
    delete gps_meas_pdf_;
    delete sys_pdf_;
    delete sys_model_;
}

// initialize prior density of filter
void PoseFuse::initialize(const eSensor sensor)
{
    // set prior of filter
    ColumnVector prior_Mu(8);
    Pose prior;

    mMutex.lock();
    switch (sensor)
    {
    case gps:
        if (!gps_initialized_)
            return;
        prior = gps_meas_;
        break;
    case vo:
        if (!vo_initialized_)
            return;
        prior = vo_meas_;
        break;
    default:
        cerr << "other sensor can not initialize this system" << endl;
        return;
    }
    mMutex.unlock();

    Eigen::Matrix<double, 6, 1> EigenPrior = prior.Transform.log();
    Eigen::Vector3d PriorP = prior.Transform.translation();
    Eigen::Vector3d PriorR = prior.Transform.so3().log();
    prior_Mu(1) = PriorP[0];
    prior_Mu(2) = PriorP[1];
    prior_Mu(3) = PriorP[2];
    prior_Mu(4) = PriorR[0];
    prior_Mu(5) = PriorR[1];
    prior_Mu(6) = PriorR[2];
    prior_Mu(7) = 0.0;
    prior_Mu(8) = 0.0;

    SymmetricMatrix prior_Cov(8);
    for (unsigned int i = 1; i <= 8; i++)
    {
        for (unsigned int j = 1; j <= 8; j++)
        {
            if (i == j)
                prior_Cov(i, j) = pow(1, 2);
            else
                prior_Cov(i, j) = 0;
        }
    }
    prior_ = new Gaussian(prior_Mu, prior_Cov);
    filter_ = new ExtendedKalmanFilter(prior_);

    // remember prior

    filter_estimate_old_vec_ = prior_Mu;
    filter_estimate_old_ = prior;
    filter_time_old_ = prior.time;

    imu_meas_old_ = filter_estimate_old_;
    gps_meas_old_ = filter_estimate_old_;
    vo_meas_old_ = filter_estimate_old_;
    odom_meas_old_ = filter_estimate_old_;

    imu_meas_.bValid = false;
    gps_meas_.bValid = false;
    vo_meas_.bValid = false;
    odom_meas_.bValid = false;
    // filter initialized
    filter_initialized_ = true;

    mSpeed = 0.0;
    mOmegaZ = 0.0;
}

bool PoseFuse::update(const double filter_time)
{
    // only update filter when it is initialized
    if (!filter_initialized_)
    {
        cerr << "Cannot update filter when filter was not initialized first." << endl;
        return false;
    }

    // only update filter for time later than current filter time
    double dt = (filter_time - filter_time_old_);
    filter_time_old_ = filter_time;
    if (dt == 0)
        return false;
    if (dt < 0)
    {
        cerr << "Will not update robot pose with time" << dt << "sec in the past" << endl;
        return false;
    }

    // system update filter
    // --------------------
    // for now only add system noise
    //ColumnVector vel_desi(6); vel_desi = 0; //BFL
    ColumnVector vel_desi(2);
    vel_desi(1) = dt;
    vel_desi(2) = dt;
    filter_->Update(sys_model_, vel_desi);

    mMutex.lock();
    /// process odom measurement
    if (odom_meas_.bValid)
    {
        //todo use filtered pose???
        GetStampedPose(odom_meas_, odom_meas_old_, filter_time);
        ColumnVector odom_rel(6);
        Eigen::Vector3d tempP = odom_meas_.Transform.translation();
        Eigen::Vector3d tempR = odom_meas_.Transform.so3().log();
        odom_rel(1) = tempP[0];
        odom_rel(2) = tempP[1];
        odom_rel(3) = tempP[2];
        odom_rel(4) = tempR[0];
        odom_rel(6) = tempR[2];
        angleOverflowCorrect(tempR[1], filter_estimate_old_vec_(5));
        odom_rel(5) = tempR[1];

        odom_meas_pdf_->AdditiveNoiseSigmaSet(odom_covariance_ * pow(dt, 2));

        filter_->Update(odom_meas_model_, odom_rel);
        // todo
        diagnostics_odom_rot_rel_ = odom_rel(6);
        odom_meas_old_ = odom_meas_;
        odom_meas_.bValid = false;
    }

    /// process imu measurement
    if (imu_meas_.bValid)
    {
        GetStampedPose(imu_meas_, imu_meas_old_, filter_time);
        ColumnVector imu_rel(3);
        Eigen::Vector3d tempR = imu_meas_.Transform.so3().log();
        imu_rel(1) = tempR[0];
        imu_rel(3) = tempR[2];
        angleOverflowCorrect(tempR[1], filter_estimate_old_vec_(5));
        imu_rel(2) = tempR[1];
        imu_meas_pdf_->AdditiveNoiseSigmaSet(imu_covariance_ * pow(dt, 2));
        filter_->Update(imu_meas_model_, imu_rel);
        imu_meas_old_ = imu_meas_;
        imu_meas_.bValid = false;
    }

    /// process gps measurement
    if (gps_meas_.bValid)
    {
        GetStampedPose(gps_meas_, gps_meas_old_, filter_time);
        ColumnVector gps_rel(6);
        Eigen::Vector3d tempP = gps_meas_.Transform.translation();
        Eigen::Vector3d tempR = gps_meas_.Transform.so3().log();
        gps_rel(1) = tempP[0];
        gps_rel(2) = tempP[1];
        gps_rel(3) = tempP[2];
        gps_rel(4) = tempR[0];
        gps_rel(6) = tempR[2];
        angleOverflowCorrect(tempR[1], filter_estimate_old_vec_(5));
        gps_rel(5) = tempR[1];

        gps_meas_pdf_->AdditiveNoiseSigmaSet(gps_covariance_ * pow(dt, 2));
        filter_->Update(gps_meas_model_, gps_rel);

        gps_meas_old_ = gps_meas_;
        gps_meas_.bValid = false;
    }

    /// process vo measurement
    if (vo_meas_.bValid)
    {
        GetStampedPose(vo_meas_, vo_meas_old_, filter_time);
        ColumnVector vo_rel(6);
        Eigen::Vector3d tempP = vo_meas_.Transform.translation();
        Eigen::Vector3d tempR = vo_meas_.Transform.so3().log();
        vo_rel(1) = tempP[0];
        vo_rel(2) = tempP[1];
        vo_rel(3) = tempP[2];
        vo_rel(4) = tempR[0];
        vo_rel(6) = tempR[2];
        angleOverflowCorrect(tempR[1], filter_estimate_old_vec_(5));
        vo_rel(5) = tempR[1];

        vo_meas_pdf_->AdditiveNoiseSigmaSet(vo_covariance_ * pow(dt, 2));
        filter_->Update(vo_meas_model_, vo_rel);
        vo_meas_old_ = vo_meas_;
        vo_meas_.bValid = false;
    }

    // remember last estimate
    filter_estimate_old_vec_ = filter_->PostGet()->ExpectedValueGet();
    filter_estimate_old_cov_ = filter_->PostGet()->CovarianceGet();
    mSpeed = filter_estimate_old_vec_(7);
    Eigen::Vector3d tempP, tempR;
    for (int j = 0; j < 3; ++j)
    {
        tempP[j] = filter_estimate_old_vec_(j + 1);
        tempR[j] = filter_estimate_old_vec_(j + 4);
    }

    filter_estimate_old_.Transform = Sophus::SE3(Sophus::SO3(tempR), tempP);
    filter_estimate_old_.time = filter_time;
    filter_estimate_old_.bValid = true;

    odom_meas_old_ = filter_estimate_old_;
    mMutex.unlock();
}

void PoseFuse::addMeasurement(const eSensor sensor, double x, double y, double z, double alpha1, double alpha2,
                              double alpha3, double time, double covar)
{
    mMutex.lock();
    switch (sensor)
    {
    case gps:
        if (!gps_initialized_)
            gps_initialized_ = true;
        gps_covariance_(1, 1) = covar;
        gps_covariance_(2, 2) = covar;
        gps_covariance_(3, 3) = covar;
        gps_covariance_(4, 4) = covar;
        gps_covariance_(5, 5) = covar;
        gps_covariance_(6, 6) = covar;

        gps_meas_.bValid = true;
        gps_meas_.time = time;
        gps_meas_.Transform = Sophus::SE3(Sophus::SO3(Eigen::Vector3d(alpha1, alpha2, alpha3)), Eigen::Vector3d(x, y, z));
        break;
    case vo:
        if (!vo_initialized_)
            vo_initialized_ = true;
        vo_covariance_(1, 1) = covar;
        vo_covariance_(2, 2) = covar;
        vo_covariance_(3, 3) = covar;
        vo_covariance_(4, 4) = covar;
        vo_covariance_(5, 5) = covar;
        vo_covariance_(6, 6) = covar;

        vo_meas_.bValid = true;
        vo_meas_.time = time;
        vo_meas_.Transform = Sophus::SE3(Sophus::SO3(Eigen::Vector3d(alpha1, alpha2, alpha3)), Eigen::Vector3d(x, y, z));
        break;
    case imu:
        if (!imu_initialized_)
            imu_initialized_ = true;
        imu_covariance_(1, 1) = covar;
        imu_covariance_(2, 2) = covar;
        imu_covariance_(3, 3) = covar;

        imu_meas_.bValid = true;
        imu_meas_.time = time;
        imu_meas_.Transform = Sophus::SE3(Sophus::SO3(Eigen::Vector3d(alpha1, alpha2, alpha3)), Eigen::Vector3d(x, y, z));
        break;
    default:
        cerr << "for now, other sensor is not supported" << endl;
        break;
    }
    mMutex.unlock();
}

void PoseFuse::addMeasurement(double speed, double steer, double time, double covar)
{
    if (!filter_initialized_)
        return;
    double dX, dY, dThe;
    double speedx, speedy, speedTheta;
    double realspeed, accInterval, IMUtimestamp, f_tyre_angle;
    double move_R;

    realspeed = speed / 3.6;
    steer = -steer;

    if (abs(speed) <= 1)
        realspeed *= 1.1;
    else if (speed <= 2)
        realspeed *= 1.1;
    else if (speed <= 5)
        realspeed *= 1.05;
    else if (speed < 9)
        realspeed *= 1.02;
    else
        realspeed *= 1;

    mSpeed = realspeed;

    if (steer < 0)
        f_tyre_angle = steer / kangle / (180 / M_PI);
    else
        f_tyre_angle = steer / kangleRight / (180 / M_PI);

    mMutex.lock();
    accInterval = time - odom_meas_old_.time;
    if (!odom_meas_old_.bValid)
        cerr << "odom_old is not valid?" << endl;
    if (accInterval < 0)
        cerr << "accInterval < 0???" << endl;

    if (abs(f_tyre_angle) > 0.0000001)
    {
        move_R = vehlen / tan(f_tyre_angle);
        dThe = accInterval * realspeed / move_R;
        mOmegaZ = realspeed / move_R;
        dY = move_R * sin(dThe);
        dX = move_R * (1.0 - cos(dThe));
    }
    else
    {
        dThe = accInterval * realspeed * tan(f_tyre_angle) / (double)vehlen;
        mOmegaZ = realspeed * tan(f_tyre_angle) / (double)vehlen;
        dY = accInterval * realspeed;
        dX = 0.0;
    }
    Sophus::SE3 deltaT(Sophus::SO3(Eigen::Vector3d(0, dThe, 0)), Eigen::Vector3d(dX, 0, dY));
    odom_meas_.bValid = true;
    odom_meas_.time = time;
    odom_meas_.Transform = odom_meas_old_.Transform * deltaT;

    odom_covariance_(1, 1) = covar;
    odom_covariance_(2, 2) = covar;
    odom_covariance_(3, 3) = covar;
    odom_covariance_(4, 4) = covar;
    odom_covariance_(5, 5) = covar * 10;
    odom_covariance_(6, 6) = covar;
    mMutex.unlock();
}

//  T 此次测量值 oldT 上次测量值 t 更新模型时的时间
void PoseFuse::GetStampedPose(Pose &T, Pose &oldT, double t)
{
    if (!T.bValid)
    {
        cerr << "this pose is not valid" << endl;
        return;
    }
    Sophus::SE3 deltaT = oldT.Transform.inverse() * T.Transform;
    double deltaTime = T.time - oldT.time;
    if (deltaTime < 0)
    {
        cerr << "deltaT is < 0" << endl;
        return;
    }

    //    auto vectorDeltaP_R = deltaT.so3().log();
    //    auto vectorDeltaP_t = deltaT.translation();
    //    vectorDeltaP_R *= ((t-oldT.time)/deltaTime);
    //    vectorDeltaP_t *= ((t-oldT.time)/deltaTime);
    //
    //    T.time = t;
    //    T.Transform = oldT.Transform*Sophus::SE3(Sophus::SO3(vectorDeltaP_R),vectorDeltaP_t);
    T.time = t; //  测量值时间更新为当前时间
    if (deltaTime > 10)
    {
        cerr << "1000!!,kkkkkkkkkj!!!!!!!!!!!!" << endl;
        return;
    }
    //  t是现在的时间辍 oldT和T是测量时对应的时间辍
    double k = (t - oldT.time) / deltaTime;
    //  
    T.Transform = Interpolate(oldT.Transform, T.Transform, k);
}

Sophus::SE3 PoseFuse::getCurrentPose()
{
    //    cout<<"filter_estimate_old:"<<filter_estimate_old_.Transform.log()<<endl;
    mMutex.lock();
    Sophus::SE3 temp = gps_meas_old_.Transform;
    mMutex.unlock();
    //    cout<<"measurement:"<<temp.log()<<endl;
    return filter_estimate_old_.Transform;
    //    return temp;
}

Sophus::SE3 PoseFuse::getCurrentPose(double &estimated_v)
{
    //    cout<<"filter_estimate_old:"<<filter_estimate_old_.Transform.log()<<endl;
    mMutex.lock();
    Sophus::SE3 temp = gps_meas_old_.Transform;
    mMutex.unlock();
    //    cout<<"measurement:"<<temp.log()<<endl;
    estimated_v = mSpeed;
    return filter_estimate_old_.Transform;
    //    return temp;
}

double PoseFuse::getCurrentCov()
{
    double lAccuracy = (filter_estimate_old_cov_(1, 1) + filter_estimate_old_cov_(3, 3)) / 2.0;
    return sqrt(lAccuracy);
}

// correct for angle overflow
void PoseFuse::angleOverflowCorrect(double &a, double ref)
{
    while ((a - ref) > M_PI)
        a -= 2 * M_PI;
    while ((a - ref) < -M_PI)
        a += 2 * M_PI;
};

Sophus::SO3 PoseFuse::SLERP(const Eigen::Quaterniond &q1, const Eigen::Quaterniond &q2, double k)
{
    assert(k >= 0 && k <= 1);
    vector<double> start{q1.w(), q1.x(), q1.y(), q1.z()};
    vector<double> end{q2.w(), q2.x(), q2.y(), q2.z()};
    double cosa = start[0] * end[0] + start[1] * end[1] + start[2] * end[2] + start[3] * end[3];

    // If the dot product is negative, the quaternions have opposite handed-ness and slerp won't take
    // the shorter path. Fix by reversing one quaternion.
    if (cosa < 0.0f)
    {
        end[0] = -end[0];
        end[1] = -end[1];
        end[2] = -end[2];
        end[3] = -end[3];
        cosa = -cosa;
    }

    double k0, k1;

    // If the inputs are too close for comfort, linearly interpolate
    if (cosa > 0.9995f)
    {
        k0 = 1.0 - k;
        k1 = k;
    }
    else
    {
        double sina = sqrt(1.0f - cosa * cosa);
        double a = atan2(sina, cosa);
        k0 = sin((1.0f - k) * a) / sina;
        k1 = sin(k * a) / sina;
    }
    vector<double> result(4);
    result[0] = start[0] * k0 + end[0] * k1;
    result[1] = start[1] * k0 + end[1] * k1;
    result[2] = start[2] * k0 + end[2] * k1;
    result[3] = start[3] * k0 + end[3] * k1;
    return Sophus::SO3(Eigen::Quaterniond(result[0], result[1], result[2], result[3]));
}

Sophus::SE3 PoseFuse::Interpolate(Sophus::SE3 T1, Sophus::SE3 T2, float k)
{
    assert(k >= 0);
    Sophus::SE3 result;
    if (k > 1)
    {
        Sophus::SE3 deltaT = T1.inverse() * T2;
        //  循环插值直到T1 T2时间间隔小于
        while (k - 1 > 1)
        {
            T1 = T2;
            T2 = T2 * deltaT;
            k--;
        }
        Sophus::SO3 tempR = SLERP(T1.unit_quaternion(), T2.unit_quaternion(), k - 1);
        Sophus::SO3 deltaR = T1.so3().inverse() * tempR;
        Sophus::SO3 resultR = T2.so3() * deltaR;
        Eigen::Vector3d deltaP = (T2.translation() - T1.translation()) * (k - 1);
        Eigen::Vector3d resultP = T2.translation() + deltaP;
        result = Sophus::SE3(resultR, resultP);
    }
    else
    {
        //  四元数球面插值
        Sophus::SO3 resultR = SLERP(T1.unit_quaternion(), T2.unit_quaternion(), k);
        Eigen::Vector3d deltaP = (T2.translation() - T1.translation()) * k;
        Eigen::Vector3d resultP = T1.translation() + deltaP;
        result = Sophus::SE3(resultR, resultP);
    }
    return result;
}