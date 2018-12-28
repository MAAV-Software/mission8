#include <chrono>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>

#include <yaml-cpp/yaml.h>
#include <zcm/zcm-cpp.hpp>

#include <common/utils/GetOpt.hpp>
#include <imu/ImuDevice.hpp>
#include <imu/Microstrain.hpp>

using namespace zcm;
using namespace std;
using namespace std::chrono;
using namespace std::chrono_literals;

sig_atomic_t RUNNING = 1;

void sigHandler(int) { RUNNING = 0; }
std::unique_ptr<maav::ImuDevice> getDevice(const std::string& device_name);

int main(int argc, char** argv)
{
    signal(SIGINT, sigHandler);
    signal(SIGABRT, sigHandler);
    signal(SIGSEGV, sigHandler);
    signal(SIGTERM, sigHandler);
    signal(SIGQUIT, sigHandler);

    GetOpt gopt;
    gopt.addBool('h', "help", false, "This message");
    gopt.addString('c', "config", "../config/imu-config.yaml", "Path to config.");
    gopt.addBool('v', "verbose", false, "Print readings.");

    if (!gopt.parse(argc, argv, 1) || gopt.getBool("help"))
    {
        std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
        gopt.printHelp();
        return 1;
    }

    bool verbose = gopt.getBool("verbose");
    YAML::Node config = YAML::LoadFile(gopt.getString("config"));

    cout << fixed << setprecision(4) << setw(4);

    cout << "Starting MAAV IMU Driver.\nConnecting IMU" << endl;

    ZCM zcm{"ipc"};
    if (!zcm.good()) throw runtime_error("maav-controller: Invalid ZCM");

    cout << "ZCM good" << endl;

    imu_t dummy;
    dummy.utime = 0;
    zcm.publish(maav::IMU_CHANNEL, &dummy);

    this_thread::sleep_for(150ms);

    std::string device_name = config["device"].as<std::string>();
    std::unique_ptr<maav::ImuDevice> device = getDevice(device_name);

    if (!device)
    {
        return 0;
    }

    cout << "IMU Connected. Begin reading data and publishing messages.\n";

    imu_t msg;
    constexpr auto period = 10000us;
    // constexpr auto period = 1000000us;
    if (verbose)
    {
        ios::sync_with_stdio(false);
        cout << std::showpos << std::setprecision(3);
    }

    while (RUNNING)
    {
        auto start_time = duration_cast<microseconds>(system_clock::now().time_since_epoch());

        device->read(msg);

        zcm.publish(maav::IMU_CHANNEL, &msg);

        if (verbose)
        {
            cout << "Time: " << msg.utime << '\n';
            cout << "Acceleration: x: " << msg.acceleration[0] << ", y: " << msg.acceleration[1]
                 << ", z: " << msg.acceleration[2] << '\n';
            cout << "AngularRates: x: " << msg.angular_rates[0] << ", y: " << msg.angular_rates[1]
                 << ", z: " << msg.angular_rates[2] << endl;
        }

        this_thread::sleep_for(
            period -
            (duration_cast<microseconds>(system_clock::now().time_since_epoch()) - start_time));
    }

    cout << "Exiting MAAV IMU Driver" << endl;
}

std::unique_ptr<maav::ImuDevice> getDevice(const std::string& device_name)
{
    if (device_name == "Microstrain")
    {
        return std::unique_ptr<maav::ImuDevice>(new maav::MicrostrainImu());
    }
    else if (device_name == "VectorNav")
    {
        throw runtime_error("VectorNav IMU has not been implemented yet");
    }
    else if (device_name == "BNO055")
    {
        return nullptr;
    }
    else
    {
        throw runtime_error("Unknown IMU device");
    }
}
