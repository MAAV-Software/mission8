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
#include <gnc/constants.hpp>
#include <gnc/estimator.hpp>
#include <gnc/measurements/Imu.hpp>
#include <gnc/measurements/Measurement.hpp>
#include <gnc/utils/ZcmConversion.hpp>

using maav::HEIGHT_LIDAR_CHANNEL;
using maav::GLOBAL_UPDATE_CHANNEL;
using maav::SIM_GLOBAL_UPDATE_CHANNEL;
using maav::IMU_CHANNEL;
using maav::PLANE_FIT_CHANNEL;
using maav::SIM_PLANE_FIT_CHANNEL;
using maav::STATE_CHANNEL;
using maav::gnc::ConvertState;
using maav::gnc::convertLidar;
using maav::gnc::convertGlobalUpdate;
using maav::gnc::convertImu;
using maav::gnc::convertPlaneFit;
using maav::gnc::Estimator;
using maav::gnc::State;
using maav::gnc::measurements::ImuMeasurement;
using maav::gnc::measurements::LidarMeasurement;
using maav::gnc::measurements::MeasurementSet;
using maav::gnc::measurements::PlaneFitMeasurement;
using maav::gnc::measurements::GlobalUpdateMeasurement;

using namespace std;

sig_atomic_t KILL = 0;

void sighandler(int sig) { KILL = sig; }
int main(int argc, char** argv)
{
    signal(SIGINT, sighandler);
    signal(SIGABRT, sighandler);

    GetOpt gopt;
    gopt.addBool('h', "help", false, "This message");
    gopt.addBool('v', "verbose", false, "Print the state at each iteration");
    gopt.addString('c', "config", "", "Path to config.");

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
        std::cout << std::showpos << std::setprecision(4) << fixed;
    }

    cout << "Starting MAAV Estimator" << endl;

    // Init ZCM
    zcm::ZCM zcm{"ipc"};
    if (!zcm.good())
    {
        throw "Bad ZCM";
    }

    cout << "ZCM Good" << endl;

    YAML::Node config = YAML::LoadFile(gopt.getString("config"));
    Estimator estimator(config);

    ZCMHandler<imu_t> imu_handler;
    ZCMHandler<lidar_t> lidar_handler;
    ZCMHandler<plane_fit_t> plane_fit_handler;
    ZCMHandler<global_update_t> global_update_handler;

    zcm.subscribe(IMU_CHANNEL, &ZCMHandler<imu_t>::recv, &imu_handler);
    zcm.subscribe(HEIGHT_LIDAR_CHANNEL, &ZCMHandler<lidar_t>::recv, &lidar_handler);
    zcm.subscribe(PLANE_FIT_CHANNEL, &ZCMHandler<plane_fit_t>::recv, &plane_fit_handler);

    // Use simulated planefit?
    bool sim_planefit = config["sim_planefit_update"].as<bool>();
    if (sim_planefit)
        zcm.subscribe(SIM_PLANE_FIT_CHANNEL, &ZCMHandler<plane_fit_t>::recv, &plane_fit_handler);
    else
        zcm.subscribe(PLANE_FIT_CHANNEL, &ZCMHandler<plane_fit_t>::recv, &plane_fit_handler);

    // Use simulated slam?
    bool sim_slam = config["sim_global_update"].as<bool>();
    if (sim_slam)
        zcm.subscribe(
            SIM_GLOBAL_UPDATE_CHANNEL, &ZCMHandler<global_update_t>::recv, &global_update_handler);
    else
        zcm.subscribe(
            GLOBAL_UPDATE_CHANNEL, &ZCMHandler<global_update_t>::recv, &global_update_handler);

    zcm.start();

    // Main Loop
    KILL = false;

    cout << "Starting estimator loop" << endl;

    while (!KILL)
    {
        std::this_thread::sleep_for(2ms);

        // Kalman Update Rate = IMU's Update rate,
        // so we ONLY update the Kalman Filter when the IMU Updates.
        if (!imu_handler.ready()) continue;

        MeasurementSet set;

        if (lidar_handler.ready())
        {
            const lidar_t msg = lidar_handler.msg();
            lidar_handler.pop();

            set.lidar = convertLidar(msg);

            std::cout << "Lidar Dist: " << set.lidar->distance() << std::endl;
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

            std::cout << "Global Pos: " << set.global_update->position().transpose() << std::endl;
        }

        const imu_t msg = imu_handler.msg();
        imu_handler.pop();

        set.imu = convertImu(msg);

        if (set.global_update && set.imu)
        {
            std::cout << "Time difference: " << set.imu->time_usec - set.global_update->timeUSec()
                      << std::endl;
        }

        std::cout << "Acc z: " << set.imu->acceleration.z() << std::endl;

        const State& state = estimator.add_measurement_set(set);

        state_t zcm_state = ConvertState(state);

        if (verbose)
        {
            std::cout << state;
        }

        zcm.publish(STATE_CHANNEL, &zcm_state);

        this_thread::sleep_for(2ms);
    }

    zcm.stop();

    return 0;
}
