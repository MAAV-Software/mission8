#include <csignal>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>

#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <zcm/zcm-cpp.hpp>

#include <common/messages/MsgChannels.hpp>
#include <common/messages/global_update_t.hpp>
#include <common/messages/imu_t.hpp>
#include <common/messages/lidar_t.hpp>
#include <common/messages/plane_fit_t.hpp>
#include <common/messages/state_t.hpp>
#include <common/utils/GetOpt.hpp>
#include <common/utils/ZCMHandler.hpp>
#include <gnc/Constants.hpp>
#include <gnc/Estimator.hpp>
#include <gnc/measurements/ImuMeasurement.hpp>
#include <gnc/measurements/Measurement.hpp>
#include <gnc/utils/ZcmConversion.hpp>

using maav::GLOBAL_UPDATE_CHANNEL;
using maav::HEIGHT_LIDAR_CHANNEL;
using maav::IMU_CHANNEL;
using maav::PLANE_FIT_CHANNEL;
using maav::SIM_GLOBAL_UPDATE_CHANNEL;
using maav::SIM_PLANE_FIT_CHANNEL;
using maav::STATE_CHANNEL;
using maav::gnc::convertGlobalUpdate;
using maav::gnc::convertImu;
using maav::gnc::convertLidar;
using maav::gnc::convertPlaneFit;
using maav::gnc::ConvertState;
using maav::gnc::Estimator;
using maav::gnc::State;
using maav::gnc::measurements::GlobalUpdateMeasurement;
using maav::gnc::measurements::ImuMeasurement;
using maav::gnc::measurements::LidarMeasurement;
using maav::gnc::measurements::MeasurementSet;
using maav::gnc::measurements::PlaneFitMeasurement;

using namespace std;

sig_atomic_t KILL = 0;

void sighandler(int sig) { KILL = sig; }
void applyMagnetometerCalibration(imu_t &msg, const Eigen::Vector3d &offset,
    const Eigen::Vector3d &scale, const Eigen::Matrix3d &rotM);
int main(int argc, char **argv)
{
    signal(SIGINT, sighandler);
    signal(SIGABRT, sighandler);

    GetOpt gopt;
    gopt.addBool('h', "help", false, "This message");
    gopt.addBool('v', "verbose", false, "Print the state at each iteration");
    gopt.addString('c', "config", "", "Path to config.");
    gopt.addString('i', "imucalibfile", "../config/imu-calib.yaml", "Path to imu calibration file");

    if (!gopt.parse(argc, argv, 1) || gopt.getBool("help"))
    {
        cout << "Usage: " << argv[0] << " [options]" << endl;
        gopt.printHelp();
        return 1;
    }

    bool verbose = gopt.getBool("verbose");
    if (verbose)
    {
        ios::sync_with_stdio(false);
        std::cout << std::showpos << std::setprecision(3) << fixed;
    }

    cout << "Starting MAAV Estimator" << endl;

    // Init ZCM
    zcm::ZCM zcm{"ipc"};
    zcm::ZCM zcm_udp{"udpm://239.255.76.67:7667?ttl=1"};
    if (!zcm.good())
    {
        throw "Bad ZCM";
    }
    if (!zcm_udp.good())
    {
        throw "Bad ZCM UDP";
    }

    cout << "ZCM Good" << endl;

    YAML::Node config = YAML::LoadFile(gopt.getString("config"));

    ZCMHandler<imu_t> imu_handler;
    ZCMHandler<lidar_t> lidar_handler;
    ZCMHandler<plane_fit_t> plane_fit_handler;
    ZCMHandler<global_update_t> global_update_handler;

    std::cout << "==========================" << std::endl;

    std::cout << "IMU:              ";
    if (config["sim_imu"].as<bool>())
    {
        zcm.subscribe(maav::SIM_IMU_CHANNEL, &ZCMHandler<imu_t>::recv, &imu_handler);
        std::cout << "SIM" << std::endl;
    }
    else
    {
        zcm.subscribe(maav::IMU_CHANNEL, &ZCMHandler<imu_t>::recv, &imu_handler);
        std::cout << "REGULAR" << std::endl;
    }

    std::cout << "Lidar:            ";
    if (config["sim_lidar"].as<bool>())
    {
        zcm.subscribe(maav::SIM_HEIGHT_LIDAR_CHANNEL, &ZCMHandler<lidar_t>::recv, &lidar_handler);
        std::cout << "SIM" << std::endl;
    }
    else
    {
        zcm.subscribe(maav::HEIGHT_LIDAR_CHANNEL, &ZCMHandler<lidar_t>::recv, &lidar_handler);
        std::cout << "REGULAR" << std::endl;
    }

    std::cout << "Plane Fit:        ";
    if (config["sim_planefit"].as<bool>())
    {
        zcm.subscribe(
            maav::SIM_PLANE_FIT_CHANNEL, &ZCMHandler<plane_fit_t>::recv, &plane_fit_handler);
        std::cout << "SIM" << std::endl;
    }
    else
    {
        zcm.subscribe(maav::PLANE_FIT_CHANNEL, &ZCMHandler<plane_fit_t>::recv, &plane_fit_handler);
        std::cout << "REGULAR" << std::endl;
    }

    std::cout << "Global Update:    ";
    if (config["sim_global_update"].as<bool>())
    {
        zcm.subscribe(maav::SIM_GLOBAL_UPDATE_CHANNEL, &ZCMHandler<global_update_t>::recv,
            &global_update_handler);
        std::cout << "SIM" << std::endl;
    }
    else
    {
        zcm.subscribe(maav::GLOBAL_UPDATE_CHANNEL, &ZCMHandler<global_update_t>::recv,
            &global_update_handler);
        std::cout << "REGULAR" << std::endl;
    }

    std::cout << "--------------------------" << std::endl;

    Estimator estimator(config);

    std::cout << "==========================" << std::endl;

    // Main Loop
    KILL = false;
    zcm.start();

    cout << "Calibrating IMU..." << endl;

    Eigen::Vector3d avg_acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d avg_angular_rate = Eigen::Vector3d::Zero();
    int num_samples = 50;
    for (int count = 0; count < num_samples && !KILL;)
    {
        if (imu_handler.ready())
        {
            const imu_t msg = imu_handler.msg();
            imu_handler.pop();
            auto msg_ptr = convertImu(msg);
            avg_acceleration += msg_ptr->acceleration;
            avg_angular_rate += msg_ptr->angular_rates;
            count++;
        }
        this_thread::sleep_for(2ms);
    }
    avg_acceleration /= static_cast<double>(num_samples);
    avg_angular_rate /= static_cast<double>(num_samples);

    Eigen::Vector3d true_accel = {0, 0, -maav::gnc::constants::STANDARD_GRAVITY};
    Eigen::Vector3d accel_bias = avg_acceleration - true_accel;
    Eigen::Vector3d true_ang_rate = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias = avg_angular_rate - true_ang_rate;

    cout << "Calibrated. Starting biases:\n";
    cout << "Gyro: " << gyro_bias.transpose() << std::endl;
    cout << "Accel: " << accel_bias.transpose() << std::endl;
    estimator.setBiases(gyro_bias, accel_bias);

    YAML::Node mag_calib = YAML::LoadFile(gopt.getString("imucalibfile"));
    Eigen::Vector3d offset(mag_calib["offset"].as<Eigen::Vector3d>());
    Eigen::Vector3d scale(mag_calib["scale"].as<Eigen::Vector3d>());
    Eigen::Matrix3d rotM(mag_calib["rotM"].as<Eigen::Matrix3d>());

    cout << "Starting estimator loop" << endl;

    while (!KILL)
    {
        std::this_thread::sleep_for(2ms);

        // Kalman Update Rate = IMU's Update rate,
        // so we ONLY update the Kalman Filter when the IMU Updates.
        if (!imu_handler.ready()) continue;

        MeasurementSet set;

        imu_t msg = imu_handler.msg();
        imu_handler.pop();

        applyMagnetometerCalibration(msg, offset, scale, rotM);

        set.imu = convertImu(msg);

        if (lidar_handler.ready())
        {
            const lidar_t msg = lidar_handler.msg();
            lidar_handler.pop();

            set.lidar = convertLidar(msg);
        }

        if (plane_fit_handler.ready())
        {
            const plane_fit_t msg = plane_fit_handler.msg();
            plane_fit_handler.pop();

            set.plane_fit = convertPlaneFit(msg);
        }

        if (global_update_handler.ready())
        {
            const global_update_t msg = global_update_handler.msg();
            global_update_handler.pop();

            set.global_update = convertGlobalUpdate(msg);
        }

        const State &state = estimator.add_measurement_set(set);

        state_t zcm_state = ConvertState(state);

        if (verbose)
        {
            std::cout << *(set.imu);
            if (set.lidar) std::cout << *(set.lidar);
            if (set.plane_fit) std::cout << *(set.plane_fit);
            if (set.global_update) std::cout << *(set.global_update);
            std::cout << state;
        }

        zcm.publish(STATE_CHANNEL, &zcm_state);
        zcm_udp.publish(STATE_CHANNEL, &zcm_state);

        this_thread::sleep_for(2ms);
    }

    zcm.stop();

    return 0;
}

void applyMagnetometerCalibration(imu_t &msg, const Eigen::Vector3d &offset,
    const Eigen::Vector3d &scale, const Eigen::Matrix3d &rotM)
{
    for (int i = 0; i < 3; i++)
    {
        msg.magnetometer[i] -= offset(i);
    }

    Eigen::RowVector3d prod =
        Eigen::RowVector3d(msg.magnetometer[0], msg.magnetometer[1], msg.magnetometer[2]) * rotM;

    for (int i = 0; i < 3; i++)
    {
        msg.magnetometer[i] = prod(i) / scale(i);
    }
}
