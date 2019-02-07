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

using namespace zcm;
using namespace std;
using namespace std::chrono;
using namespace std::chrono_literals;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::SelfAdjointEigenSolver;
using Eigen::Vector3d;
using Eigen::VectorXd;

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
        std::cout << "Exiting...\n";
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

    auto start = duration_cast<microseconds>(system_clock::now().time_since_epoch());

    cout << "Rotate the drone to trace an imaginary sphere\n";

    while (RUNNING)
    {
        device->read(msg);
        magvals.push_back(
            vector<double>{msg.magnetometer[0], msg.magnetometer[1], msg.magnetometer[2]});
        if ((duration_cast<microseconds>(system_clock::now().time_since_epoch()) - start) >= 6e7us)
            break;
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

    // Algorithm from https://github.com/martindeegan/drone/blob/master/copter/matlab/ellipsoid_fit.m

    MatrixXd D(magvals.size(), 9);

    for (size_t i = 0; i < magvals.size(); i++)
    {
        D(i, 0) = (x(i) * x(i));
        D(i, 1) = (y(i) * y(i));
        D(i, 2) = (z(i) * z(i));
        D(i, 3) = 2 * x(i) * y(i);
        D(i, 4) = 2 * z(i) * x(i);
        D(i, 5) = 2 * y(i) * z(i);
        D(i, 6) = 2 * x(i);
        D(i, 7) = 2 * y(i);
        D(i, 8) = 2 * z(i);
    }

    MatrixXd v;

    v = (D.transpose() * D)
            .colPivHouseholderQr()
            .solve(D.transpose() * Eigen::MatrixXd::Ones(D.rows(), 1));

    Matrix4d A;
    A(0, 0) = v(0);
    A(0, 1) = v(3);
    A(0, 2) = v(4);
    A(0, 3) = v(6);
    A(1, 0) = v(3);
    A(1, 1) = v(1);
    A(1, 2) = v(5);
    A(1, 3) = v(7);
    A(2, 0) = v(4);
    A(2, 1) = v(5);
    A(2, 2) = v(2);
    A(2, 3) = v(8);
    A(3, 0) = v(6);
    A(3, 1) = v(7);
    A(3, 2) = v(8);
    A(3, 3) = -1;

    Vector3d v_col;
    v_col << v(6), v(7), v(8);

    Vector3d offset = -A.block(0, 0, 3, 3).colPivHouseholderQr().solve(v_col);

    MatrixXd Tmtx = MatrixXd::Identity(4, 4);

    Tmtx(3, 0) = offset(0);
    Tmtx(3, 1) = offset(1);
    Tmtx(3, 2) = offset(2);

    Matrix4d AT = Tmtx * A * Tmtx.transpose();

    SelfAdjointEigenSolver<MatrixXd> es((AT.block(0, 0, 3, 3) / -AT(3, 3)));

    MatrixXd ev = es.eigenvalues().real();

    MatrixXd rotM = es.eigenvectors().real();

    double scale_x = sqrt(std::abs(1 / ev(0, 0)));
    double scale_y = sqrt(std::abs(1 / ev(1, 0)));
    double scale_z = sqrt(std::abs(1 / ev(2, 0)));

    YAML::Node node, _baseNode = YAML::LoadFile(imu_calib_file);
    _baseNode["offset_x"] = offset(0);
    _baseNode["offset_y"] = offset(1);
    _baseNode["offset_z"] = offset(2);
    _baseNode["scale_x"] = scale_x;
    _baseNode["scale_y"] = scale_y;
    _baseNode["scale_z"] = scale_z;
    _baseNode["rotM"] = rotM;

    remove(imu_calib_file.c_str());

    ofstream fout(imu_calib_file);
    fout << _baseNode;
    fout.close();

    cout << "Magnetometer calibration completed.\n";
}