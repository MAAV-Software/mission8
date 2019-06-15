#include "QualisysZCM.hpp"

#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>

using std::string;
using std::endl;
using std::cout;
using std::cerr;
using std::isnan;

namespace qualisys
{
static constexpr float deg2rad = M_PI / 180.f;
static constexpr float QTM_FREQ = 100.f;  // QTM data comes at 100 Hz
static constexpr float DT = 1.f / QTM_FREQ;

// Offsets used for zero-ing different bodies
struct Offset
{
    bool set = false;
    double x = 0, y = 0, z = 0;
};
std::vector<Offset> offs;

// Tare all bodies' positions
void sig_handler(int)
{
    for (auto& off : offs)
    {
        off.set = false;
    }
}

bool QualisysZCM::init(const YAML::Node& yconf)
{
    string server_addr = yconf["Qualisys"]["server_address"].as<string>();
    int base_port = yconf["Qualisys"]["base_port"].as<int>();
    unsigned short udp_port = yconf["Qualisys"]["udp_port"].as<int>();
    int maj_ver = yconf["Qualisys"]["major_version"].as<int>();
    int min_ver = yconf["Qualisys"]["minor_version"].as<int>();
    print_pos_ = yconf["Info"]["position"].as<bool>();
    print_vel_ = yconf["Info"]["velocity"].as<bool>();
    print_ar_ = yconf["Info"]["angular_velocity"].as<bool>();
    print_period_ = yconf["Info"]["period"].as<int>();
    string pub_ip = yconf["ZMQ"]["pub_ip"].as<string>();
    int pub_port = yconf["ZMQ"]["port"].as<int>();

    cout << "Connecting to Qualisys cameras at " << server_addr << ":" << base_port;
    if (!port_protocol_.Connect(server_addr.data(), base_port, &udp_port, maj_ver, min_ver))
    {
        cerr << "\tFail" << endl;
        cerr << port_protocol_.GetErrorString() << endl;
        return false;
    }
    cout << "\tSuccess" << endl;

    bool dummy;  // why qualisys >:(
    port_protocol_.Read6DOFSettings(dummy);

    // Connect publisher
    pub_.connect(pub_ip, pub_port);

    // install signal handler for taring the position using Ctrl + Z (SIGTERM)
    signal(SIGTERM, sig_handler);

    return true;
}

void QualisysZCM::disconnect()
{
    cout << "Disconnecting from Qualisys cameras" << endl;
    port_protocol_.StreamFramesStop();
    port_protocol_.Disconnect();
}

void QualisysZCM::handle_packet_data(CRTPacket* packet)
{
    int body_count = packet->Get6DOFEulerBodyCount();
    for (int i = 0; i < body_count; ++i)
    {
        float x, y, z, roll, pitch, yaw;
        packet->Get6DOFEulerBody(i, x, y, z, roll, pitch, yaw);

        // Create a zero offset for each body. We will set this offset on SIGTERM (Ctrl + Z)
        if (offs.size() <= i)
        {
            offs.emplace_back();
        }

        if (isnan(x) || isnan(y) || isnan(z) || isnan(roll) || isnan(pitch) || isnan(yaw))
        {
            cerr << "Rigid body " << i << " not detected" << endl;
            continue;
        }

        // Set the offset of this body to its first known position
        if (!offs[i].set)
        {
            offs[i] = {true, x, y, z};
            cout << "Rigid body " << i << " offset by (" << x / 1000 << ", " << y / 1000 << ", "
                 << z / 1000 << ")" << endl;
        }

        // Qualisys sometimes flips 180 degrees around X axis, fix that
        if (roll > 90)
        {
            roll -= 180;
        }
        else if (roll < -90)
        {
            roll += 180;
        }

        StateMsg state = make_state(i, x, y, z, roll, pitch, yaw);
        static bool print = print_ar_ || print_pos_ || print_vel_;
        if (print) info(state);  // Print position, velocity, and angular velocity if requested
        pub_.send(state);
    }
}

float update(std::deque<float>& d, float data) 
{
    if (d.size() > 9) {
        d.pop_front();
    }
    d.push_back(data);

    float sum = 0;
    for (const auto& a : d) {
        sum += a;
    }
    return sum / (float)d.size();
}

StateMsg QualisysZCM::make_state(int i, float x, float y, float z, float roll, float pitch, float yaw)
{
    static float x_prev = 0;
    static float y_prev = 0;
    static float z_prev = 0;
    StateMsg state;
    state.utime = std::chrono::duration_cast<std::chrono::microseconds>(
                      std::chrono::system_clock::now().time_since_epoch())
                      .count();

    // Scale and offset position based on last zero
    x = (x - offs[i].x) / 1000.0;
    y = (y - offs[i].y) / 1000.0;
    z = (z - offs[i].z) / 1000.0;
    state.position.data[0] = x;
    state.position.data[1] = y;
    state.position.data[2] = z;

    // Populate velocities
    state.velocity.data[0] = update(prev_state_.x, (x - x_prev) / DT);
    state.velocity.data[1] = update(prev_state_.y, (y - y_prev) / DT);
    state.velocity.data[2] = update(prev_state_.z, (z - z_prev) / DT);
    x_prev = x;
    y_prev = y;
    z_prev = z;
    state.angular_velocity.data[0] = 0;//(roll - prev_state_.roll) / DT;
    state.angular_velocity.data[1] = 0;//(pitch - prev_state_.pitch) / DT;
    state.angular_velocity.data[2] = 0;//(yaw - prev_state_.yaw) / DT;

    // Populate accelerations
    state.acceleration.data[0] = 0;//(state.velocity.data[0] - prev_state_.dx);
    state.acceleration.data[1] = 0;//(state.velocity.data[1] - prev_state_.dy);
    state.acceleration.data[2] = 0;//(state.velocity.data[2] - prev_state_.dz);

    // scale rpy
    roll = roll * M_PI / 360.0;
    pitch = pitch * M_PI / 360.0;
    yaw = yaw * M_PI / 360.0;

    // calculate quaternion from rpy
    state.attitude.data[0] =
        (cos(roll) * cos(pitch) * cos(yaw)) + (sin(roll) * sin(pitch) * sin(yaw));
    state.attitude.data[1] =
        (sin(roll) * cos(pitch) * cos(yaw)) - (cos(roll) * sin(pitch) * sin(yaw));
    state.attitude.data[2] =
        (sin(roll) * cos(pitch) * sin(yaw)) + (cos(roll) * sin(pitch) * cos(yaw));
    state.attitude.data[0] =
        (cos(roll) * cos(pitch) * sin(yaw)) - (sin(roll) * sin(pitch) * cos(yaw));

    return state;
}

void QualisysZCM::run()
{
    CRTPacket* packet = port_protocol_.GetRTPacket();
    CRTPacket::EPacketType e_type;
    port_protocol_.GetCurrentFrame(CRTProtocol::cComponent6dEuler);

    if (port_protocol_.ReceiveRTPacket(e_type, true))
    {
        switch (e_type)
        {
            case CRTPacket::PacketError:
                cerr << "Error when streaming frames: "
                     << port_protocol_.GetRTPacket()->GetErrorString() << endl;
                break;

            case CRTPacket::PacketNoMoreData:
                cerr << "No more data" << endl;
                break;

            case CRTPacket::PacketData:
                handle_packet_data(packet);
                break;

            default:
                cerr << "Unknown packet case" << endl;
        }
    }

    return;
}

void QualisysZCM::info(const StateMsg& state)
{
    static int ctr = 0;
    if (ctr == print_period_)
    {
        if (print_pos_)
        {
            cout << " X: " << std::fixed << std::setprecision(6) << state.position.data[0];
            cout << "  Y: " << std::fixed << std::setprecision(6) << state.position.data[1];
            cout << "  Z: " << std::fixed << std::setprecision(6) << state.position.data[2];
            cout << endl;
        }
        if (print_vel_)
        {
            cout << "dX: " << std::fixed << std::setprecision(6) << state.velocity.data[0];
            cout << " dY: " << std::fixed << std::setprecision(6) << state.velocity.data[1];
            cout << " dZ: " << std::fixed << std::setprecision(6) << state.velocity.data[2];
            cout << endl;
        }
        if (print_ar_)
        {
            cout << "gX: " << std::fixed << std::setprecision(6) << state.angular_velocity.data[0];
            cout << " gY: " << std::fixed << std::setprecision(6) << state.angular_velocity.data[1];
            cout << " gZ: " << std::fixed << std::setprecision(6) << state.angular_velocity.data[2];
            cout << endl;
        }
        ctr = 0;
    }
    else
    {
        ++ctr;
    }
}
}
