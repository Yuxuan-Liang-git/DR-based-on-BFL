#include <iostream>
#include <fstream>
#include <thread>
#include <sys/time.h>
#include <opencv2/opencv.hpp>
#include "config.h"
#include "CommonApi/RPCInterface.h"
#include "CommonApi/CommonThreadLock.h"
#include "CommonApi/CommonThread.h"
#include "CommonApi/CommonTimer.h"
#include "PoseFuse.h"

/// params for rpc
DEFINE_string(rpc_provider, "ZCM", "RPC provider");
DEFINE_string(rpc_url, "udpm://239.255.76.67:7667", "RPC URL");

/// sub and pub channels
DEFINE_string(rpc_sub_IMU, "IMU", "name of imu channel");
DEFINE_string(rpc_sub_GPS, "GPS", "name of gps channel");
DEFINE_string(rpc_sub_CANMsg, "CANMsg", "name of CAN message channel");
DEFINE_string(rpc_sub_SlamPos, "SlamPos", "name of Slam position channel");
DEFINE_string(rpc_pub_FusionPos, "FusionPos", "name of fusion position channel");

/// params for GPS
DEFINE_double(InitLatitude, 31.59283903958974, "zero point latitude");
DEFINE_double(InitLongitude, 120.77434997436357, "zero point longitude");

/// params for fusion
DEFINE_bool(UseVehicleOdom, true, "use vehicle odom or not");
DEFINE_bool(UseIMU, false, "use imu or not");
DEFINE_bool(UseVSLAMPos, true, "use slam pos or not");
DEFINE_bool(UseGPS, false, "use gps or not");

/// params for debug
DEFINE_string(saving_dir, "/dominant/Project_Dominant/data_analysis/gpsdata/", "pos saving dir");
DEFINE_bool(SavePos, false, "save pos for debug");

using namespace MatrixWrapper;

// using namespace std;
#define EPSINON 0.0000000001

class ttt
{
public:
    ttt();
    ~ttt() {}
    static double GetCurrentMilliSecTime();
};

double ttt::GetCurrentMilliSecTime()
{
    struct timeval nTime;
    gettimeofday(&nTime, NULL);
    return ((double)nTime.tv_sec + (double)nTime.tv_usec * 1e-6) * 1e3;
}

float ComputeGPSDistance(long double lat1, long double lon1, long double lat2, long double lon2)
{
    float dis;
    dis = (float)(6378.140 * 1000 * 2 * asin(sqrt((sin((lat1 - lat2) / 2)) * (sin((lat1 - lat2) / 2)) + cos(lat1) * cos(lat2) * (sin((lon1 - lon2) / 2)) * (sin((lon1 - lon2) / 2)))));
    dis = (float)floor(dis * 10000 + 0.5) / 10000;
    return dis;
}

void Global2_Local(double &goalPx, double &goalPy, /*目标点GPS*/ long double latGoal, long double lonGoal, /*当前GPS*/ long double lat, long double lon, long double head)
{
    double angle;
    float dis, disla, dislo;
    dis = ComputeGPSDistance(latGoal, lonGoal, lat, lon);
    disla = ComputeGPSDistance(latGoal, lonGoal, lat, lonGoal);
    dislo = ComputeGPSDistance(lat, lonGoal, lat, lon);

    if ((lon - lonGoal) > EPSINON)
    {
        if ((lat - latGoal) > EPSINON)
            angle = 180 * atan((dislo) / (disla)) / M_PI;
        else if ((lat - latGoal) < (-EPSINON))
            angle = (180 + 180 * atan((dislo) / (-disla)) / M_PI);
        else
            angle = 90;
    }
    else if ((lon - lonGoal) < (-EPSINON))
    {
        if ((lat - latGoal) > EPSINON)
            angle = 180 * atan((-dislo) / (disla)) / M_PI;
        else if ((lat - latGoal) < (-EPSINON))
            angle = (-180 + 180 * atan((dislo) / (disla)) / M_PI);
        else
            angle = -90;
    }
    else
    {
        //angle = 0;
        if ((lat - latGoal) > EPSINON)
            angle = 0;
        else if ((lat - latGoal) < (-EPSINON))
            angle = 180;
        else
            angle = 0;
    }
    double dHeadAngle;
    angle = angle / (long double)180 * M_PI;
    dHeadAngle = (angle - head);
    goalPx = -1 * dis * sin(dHeadAngle);
    goalPy = -1 * dis * cos(dHeadAngle);
}

/// changshu tech
long double InitLat = FLAGS_InitLatitude;
long double InitLot = FLAGS_InitLongitude;

// /// tongji university
// long double InitLat = 31.29050741906509;
// long double InitLot = 121.20598910714992;

/// Karman Filter instance
PoseFuse filter;

/// timestamp
double timestamp = 0.0;
double last_timestamp = 0.0;
double d_timestamp = 0.0;

double gps_valid_count = 0.0;

std::ofstream SlamPosFile, GPSPosFile, FusionPosFile;

void SignalHandle(const char *data, int size)
{
    std::ofstream fs("/log/FusionPose_glog_dump.log", std::ios::app);
    std::string str = std::string(data, size);
    fs << str;
    fs.close();
    LOG(ERROR) << str;
}

/**
 * @brief main function for fusion pose.
 */
int main(int argc, char *argv[])
{
    // fprintf(stdout, "%s v%ld\n", argv[0], FUSION_POSE_TIME_VERSION);
    // std::cout << "Using " << FLAGS_rpc_provider << " as backend.." << std::endl;
    google::SetVersionString(std::to_string(FUSION_POSE_TIME_VERSION));
    google::SetUsageMessage("");
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::InstallFailureWriter(&SignalHandle);
    google::SetStderrLogging(google::GLOG_WARNING);
    FLAGS_log_dir = "/log";
    FLAGS_log_prefix = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_logbufsecs = 0;
    FLAGS_max_log_size = 10;
    FLAGS_stop_logging_if_full_disk = true;

    bool odom_active_ = FLAGS_UseVehicleOdom;
    bool imu_active_ = FLAGS_UseIMU;
    bool vo_active_ = FLAGS_UseVSLAMPos;
    bool gps_active_ = FLAGS_UseGPS;

    std::thread t_canmsg, t_slampos, t_imu, t_gps;

    /// create rpc instance
    dominant::RPCInterface rpc(FLAGS_rpc_provider, FLAGS_rpc_url);

    double startTime = CommonTimer::GetCurrentMilliSecTime();

    /// rpc subscribe and start listening
    if (odom_active_)
    {
        rpc.subscribe<PacketCanMsg>(FLAGS_rpc_sub_CANMsg);
        std::cout << "Using vehicle odom." << std::endl;
    }
    if (gps_active_)
    {
        rpc.subscribe<PacketGPS>(FLAGS_rpc_sub_GPS);
        std::cout << "Using GPS." << std::endl;
    }
    if (imu_active_)
    {
        rpc.subscribe<PacketIMU>(FLAGS_rpc_sub_IMU);
        std::cout << "Using IMU." << std::endl;
    }
    if (vo_active_)
    {
        rpc.subscribe<PacketSlamPos>(FLAGS_rpc_sub_SlamPos);
        std::cout << "Using VSLAM Pos." << std::endl;
    }
    rpc.startListening();

    if (FLAGS_SavePos)
    {
        std::cout << "Start to save position result." << std::endl;
        SlamPosFile.open(FLAGS_saving_dir + "slampos.txt");
        GPSPosFile.open(FLAGS_saving_dir + "gpspos.txt");
        FusionPosFile.open(FLAGS_saving_dir + "fusionpos.txt");
    }

    double estimated_v = 0;
    double real_v = 0;

    while (true)
    {
        if (odom_active_ && !t_canmsg.joinable())
        {
            t_canmsg = std::thread([&]() {
                while (true)
                {
                    double interval = 0;
                    PacketCanMsg msg;
                    int ret = rpc.waitData<PacketCanMsg>(FLAGS_rpc_sub_CANMsg, msg, interval, -1);
                    double time = (CommonTimer::GetCurrentMilliSecTime() - startTime) / 1000.0;
                    double accu;
                    if (msg.speed < 0.1)
                        accu = 0.05;
                    else
                        accu = 0.05;
                    real_v = msg.speed;
                    filter.addMeasurement(msg.speed, msg.steer, time, accu);
                }
            });
        }
        if (gps_active_ && !t_gps.joinable())
        {
            t_gps = std::thread([&]() {
                while (true)
                {
                    double interval = 0;
                    PacketGPS msg;
                    int ret = rpc.waitData<PacketGPS>(FLAGS_rpc_sub_GPS, msg, interval, -1);
                    double time = (CommonTimer::GetCurrentMilliSecTime() - startTime) / 1000.0;
                    double x, y, heading;
                    Global2_Local(x, y, msg.lat * M_PI / 180.0, msg.lon * M_PI / 180.0, InitLat * M_PI / 180.0, InitLot * M_PI / 180.0, 0);
                    heading = msg.heading * M_PI / 180.0;
                    if (msg.accuracy > 0.06)
                        gps_valid_count = 110;
                    if (gps_valid_count < 1)
                    {
                        filter.addMeasurement(PoseFuse::gps, x, 0, y, 0, heading, 0, time, 0.05);
                        if (FLAGS_SavePos)
                        {
                            GPSPosFile << time << " " << x << " " << y << " " << msg.heading << std::endl;
                        }
                    }
                    else
                    {
                        // std::cout << gps_valid_count << std::endl;
                        gps_valid_count--;
                    }
                }
            });
        }
        if (vo_active_ && !t_slampos.joinable())
        {
            t_slampos = std::thread([&]() {
                while (true)
                {
                    double interval = 0;
                    PacketSlamPos msg;
                    int ret = rpc.waitData<PacketSlamPos>(FLAGS_rpc_sub_SlamPos, msg, interval, -1);
                    double time = (CommonTimer::GetCurrentMilliSecTime() - startTime) / 1000.0;
                    double heading = msg.heading * M_PI / 180.0;
                    filter.addMeasurement(PoseFuse::vo, msg.x, 0, msg.y, 0, heading, 0, time, 10000.0);
                    // filter.addMeasurement(PoseFuse::vo, msg.x, 0, msg.y, 0, heading, 0, time, msg.accuracy);
                    if (FLAGS_SavePos)
                    {
                        SlamPosFile << time << " " << msg.x << " " << msg.y << " " << msg.heading << std::endl;
                    }
                }
            });
        }
        if (imu_active_ && !t_imu.joinable())
        {
            t_imu = std::thread([&]() {
                while (true)
                {
                    double interval = 0;
                    PacketIMU msg;
                    int ret = rpc.waitData<PacketIMU>(FLAGS_rpc_sub_IMU, msg, interval, -1);
                    double time = (CommonTimer::GetCurrentMilliSecTime() - startTime) / 1000.0;
                    filter.addMeasurement(PoseFuse::imu, 0, 0, 0, 0, msg.heading, 0, time, 1.0);
                }
            });
        }

        if (filter.getInitialized())
        {
            timestamp = (CommonTimer::GetCurrentMilliSecTime() - startTime) / 1000.0;
            filter.update(timestamp);
            Sophus::SE3 CurrentPose = filter.getCurrentPose(estimated_v);
            d_timestamp = timestamp - last_timestamp;
            last_timestamp = timestamp;
            // std::cout << "e: " << estimated_v * 3.6 << " "
            //           << "r:" << real_v << std::endl;
            /// publish fusion result
            Eigen::Vector3d p = CurrentPose.translation();
            Eigen::Vector3d theta = CurrentPose.so3().log();
            PacketFusionPos my_data;
            float heading = theta[1] * 180.0 / M_PI;
            if (heading < 0)
                heading += 360;
            my_data.heading = heading;
            my_data.accuracy = 0.05;
            // my_data.accuracy = filter.getCurrentCov() > 0.05 ? filter.getCurrentCov() : 0.05;
            my_data.x = p[0];
            my_data.y = p[2];
            rpc.publish(FLAGS_rpc_pub_FusionPos, &my_data);
            // std::cout << "x:" << my_data.x << ",y:" << my_data.y << ",heading:" << my_data.heading << std::endl;
            // std::cout << filter.getCurrentCov() << std::endl;
            if (FLAGS_SavePos)
            {
                double time = (CommonTimer::GetCurrentMilliSecTime() - startTime) / 1000.0;
                FusionPosFile << time << " " << my_data.x << " " << my_data.y << " " << my_data.heading << std::endl;
            }
        }
        else
        {
            if (filter.getGPSInitialized())
                filter.initialize(PoseFuse::gps);
            else if (filter.getVOInitialized())
                filter.initialize(PoseFuse::vo);
        }
        usleep(10000);
    }
    if (FLAGS_SavePos)
    {
        SlamPosFile.close();
        FusionPosFile.close();
        GPSPosFile.close();
    }
    return 0;
}
