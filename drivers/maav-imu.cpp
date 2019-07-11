#include <Eigen/Dense>
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
#include <common/utils/yaml_matrix.hpp>
#include <zcm/zcm-cpp.hpp>

#include <common/utils/GetOpt.hpp>
#include <imu/ImuDevice.hpp>
#include <imu/Microstrain.hpp>

#include <gnc/utils/MagnetometerEllipsoidFit.hpp>

using namespace zcm;
using namespace std;
using namespace std::chrono;
using namespace std::chrono_literals;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::SelfAdjointEigenSolver;
using Eigen::Vector3d;
using Eigen::VectorXd;
using maav::gnc::runMagnetometerCalibration;

sig_atomic_t RUNNING = 1;

void sigHandler(int) { RUNNING = 0; }
std::unique_ptr<maav::ImuDevice> getDevice(const std::string& device_name);
void calibrateMagnetometer(
    std::unique_ptr<maav::ImuDevice>& device, const std::string& imu_calib_file);
std::pair<double, size_t> getAxis(const string& str);

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

    const std::string x_str = config["x"].as<std::string>();
    auto x_axis = getAxis(x_str);
    const double x_sign = x_axis.first;
    const size_t x_index = x_axis.second;

    const std::string y_str = config["y"].as<std::string>();
    auto y_axis = getAxis(y_str);
    const double y_sign = y_axis.first;
    const size_t y_index = y_axis.second;

    const std::string z_str = config["z"].as<std::string>();
    auto z_axis = getAxis(z_str);
    const double z_sign = z_axis.first;
    const size_t z_index = z_axis.second;

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
        std::cout << "Exiting...\n";
        return 0;
    }

    cout << "IMU Connected. Begin reading data and publishing messages.\n";

    imu_t msg;
    constexpr auto period = 10000us;
    if (verbose)
    {
        ios::sync_with_stdio(false);
        cout << std::showpos << std::setprecision(3);
    }

    while (RUNNING)
    {
        auto start_time = duration_cast<microseconds>(system_clock::now().time_since_epoch());

        device->read(msg);
        imu_t msg_corrected = msg;
        msg_corrected.angular_rates.data[0] = x_sign * msg.angular_rates.data[x_index];
        msg_corrected.angular_rates.data[1] = y_sign * msg.angular_rates.data[y_index];
        msg_corrected.angular_rates.data[2] = z_sign * msg.angular_rates.data[z_index];

        msg_corrected.acceleration.data[0] = x_sign * msg.acceleration.data[x_index];
        msg_corrected.acceleration.data[1] = y_sign * msg.acceleration.data[y_index];
        msg_corrected.acceleration.data[2] = z_sign * msg.acceleration.data[z_index];

        zcm.publish(maav::IMU_CHANNEL, &msg_corrected);
        zcm_udp.publish(maav::IMU_CHANNEL, &msg_corrected);

        if (verbose)
        {
            cout << "Time: " << msg.utime << '\n';
            cout << "Acceleration: x: " << msg.acceleration.data[0]
                 << ", y: " << msg.acceleration.data[1] << ", z: " << msg.acceleration.data[2]
                 << '\n';
            cout << "AngularRates: x: " << msg.angular_rates.data[0]
                 << ", y: " << msg.angular_rates.data[1] << ", z: " << msg.angular_rates.data[2]
                 << endl;
            cout << "Magnetometer: x: " << msg.magnetometer.data[0]
                 << ", y: " << msg.magnetometer.data[1] << ", z: " << msg.magnetometer.data[2]
                 << endl;
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
        throw runtime_error("BNO055 IMU has not been implemented yet");
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

    auto start = duration_cast<seconds>(system_clock::now().time_since_epoch());

    cout << "Rotate the drone to trace an imaginary sphere\n";

    while (RUNNING)
    {
        device->read(msg);
        magvals.push_back(vector<double>{
            msg.magnetometer.data[0], msg.magnetometer.data[1], msg.magnetometer.data[2]});
        if ((duration_cast<seconds>(system_clock::now().time_since_epoch()) - start) >= 60s) break;
        std::this_thread::sleep_for(1ms);
    }

    if (!RUNNING) return;

    VectorXd x(magvals.size());
    VectorXd y(magvals.size());
    VectorXd z(magvals.size());

    int idx = 0;
    for (auto& v : magvals)
    {
        x(idx) = v[0];
        y(idx) = v[1];
        z(idx) = v[2];
        idx++;
    }

    MagParams mp = maav::gnc::runMagnetometerCalibration(x, y, z);

    YAML::Node node, _baseNode = YAML::LoadFile(imu_calib_file);
    _baseNode["offset"] = mp.offset;
    _baseNode["scale"] = mp.scale;
    _baseNode["rotM"] = mp.rotM;

    remove(imu_calib_file.c_str());

    ofstream fout(imu_calib_file);
    fout << _baseNode;
    fout.close();

    cout << "Magnetometer calibration completed.\n";
}

std::pair<double, size_t> getAxis(const string& str)
{
    if (str.size() > 2 || str.empty() ||
        (str.find('x') == std::string::npos && str.find('y') == std::string::npos &&
            str.find('z') == std::string::npos))
    {
        throw runtime_error("Bad config. Needs to be in the form of \"<axis>\" or \"-<axis>\"");
    }

    std::pair<double, size_t> axis{1, 0};

    if (str.find('-') != std::string::npos)
    {
        axis.first = -1;
    }

    if (str.find('x') != std::string::npos)
        axis.second = 0;
    else if (str.find('y') != std::string::npos)
        axis.second = 1;
    else if (str.find('z') != std::string::npos)
        axis.second = 2;

    return axis;
}