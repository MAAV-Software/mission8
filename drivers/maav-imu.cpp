#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
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
void calibrateMagnetometer(
    std::unique_ptr<maav::ImuDevice>& device, const std::string& imu_calib_file);

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
    gopt.addBool('m', "magcal", false, "Run magnetometer calibration");
    gopt.addString('i', "imucalibfile", "../config/imu-calib.yaml", "Path to imu calibration file");

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
    ZCM zcm_udp{"udpm://239.255.76.67:7667?ttl=1"};
    if (!zcm.good()) throw runtime_error("maav-controller: Invalid ZCM");
    if (!zcm_udp.good()) throw runtime_error("maav-controller: Invalid ZCM UDP");

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

    if (gopt.getBool("magcal"))
    {
        calibrateMagnetometer(device, gopt.getString("imucalibfile"));
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

    YAML::Node mag_config = YAML::LoadFile(gopt.getString("imucalibfile"));

    double offset_x = mag_config["offset_x"].as<double>();
    double offset_y = mag_config["offset_y"].as<double>();
    double offset_z = mag_config["offset_z"].as<double>();
    double scale_x = mag_config["scale_x"].as<double>();
    double scale_y = mag_config["scale_y"].as<double>();
    double scale_z = mag_config["scale_z"].as<double>();

    while (RUNNING)
    {
        auto start_time = duration_cast<microseconds>(system_clock::now().time_since_epoch());

        device->read(msg);

        // Scale magnetometer readings from calibration constants
        msg.magnetometer[0] = (msg.magnetometer[0] - offset_x) * scale_x;
        msg.magnetometer[1] = (msg.magnetometer[1] - offset_y) * scale_y;
        msg.magnetometer[2] = (msg.magnetometer[2] - offset_z) * scale_z;

        zcm.publish(maav::IMU_CHANNEL, &msg);
        zcm_udp.publish(maav::IMU_CHANNEL, &msg);

        if (verbose)
        {
            cout << "Time: " << msg.utime << '\n';
            cout << "Acceleration: x: " << msg.acceleration[0] << ", y: " << msg.acceleration[1]
                 << ", z: " << msg.acceleration[2] << '\n';
            cout << "AngularRates: x: " << msg.angular_rates[0] << ", y: " << msg.angular_rates[1]
                 << ", z: " << msg.angular_rates[2] << endl;
            cout << "Magnetometer: x: " << msg.magnetometer[0] << ", y: " << msg.magnetometer[1]
                 << ", z: " << msg.magnetometer[2] << endl;
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

void calibrateMagnetometer(
    std::unique_ptr<maav::ImuDevice>& device, const std::string& imu_calib_file)
{
    cout << "Starting magnetometer calibration.\n";

    imu_t msg;
    vector<vector<double>> magvals;

    std::clock_t start;
    double duration;

    start = std::clock();

    cout << "Move the IMU in many different directions for 1 minute.\n";

    while (RUNNING)
    {
        device->read(msg);
        magvals.push_back(
            vector<double>{msg.magnetometer[0], msg.magnetometer[1], msg.magnetometer[2]});
        duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
        if (duration >= 60) break;
        std::this_thread::sleep_for(1ms);
    }

    if (!RUNNING) return;

    // Calibration calculations: https://appelsiini.net/2018/calibrate-magnetometer/

    // Find max, min
    double x_max, y_max, z_max;
    x_max = y_max = z_max = std::numeric_limits<double>::min();
    double x_min, y_min, z_min;
    x_min = y_min = z_min = std::numeric_limits<double>::max();
    for (auto& v : magvals)
    {
        if (v[0] > x_max) x_max = v[0];
        if (v[1] > y_max) y_max = v[1];
        if (v[2] > z_max) z_max = v[2];
        if (v[0] < x_min) x_min = v[0];
        if (v[1] < y_min) y_min = v[1];
        if (v[2] < z_min) z_min = v[2];
    }

    double offset_x = (x_max + x_min) / 2.0;
    double offset_y = (y_max + y_min) / 2.0;
    double offset_z = (z_max + z_min) / 2.0;

    double avg_delta_x = (x_max - x_min) / 2.0;
    double avg_delta_y = (y_max - y_min) / 2.0;
    double avg_delta_z = (z_max - z_min) / 2.0;

    double avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3.0;

    double scale_x = avg_delta / avg_delta_x;
    double scale_y = avg_delta / avg_delta_y;
    double scale_z = avg_delta / avg_delta_z;

    // Save constants:
    // offset x,y,z
    // scale x,y,z
    YAML::Node node, _baseNode = YAML::LoadFile(imu_calib_file);
    _baseNode["offset_x"] = offset_x;
    _baseNode["offset_y"] = offset_y;
    _baseNode["offset_z"] = offset_z;
    _baseNode["scale_x"] = scale_x;
    _baseNode["scale_y"] = scale_y;
    _baseNode["scale_z"] = scale_z;

    remove(imu_calib_file.c_str());

    ofstream fout(imu_calib_file);
    fout << _baseNode;
    fout.close();

    cout << "Magnetometer calibration completed.\n";
}