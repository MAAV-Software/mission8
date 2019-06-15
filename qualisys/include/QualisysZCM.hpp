#ifndef MAAV_QUALISYS_DRIVER_HPP
#define MAAV_QUALISYS_DRIVER_HPP

#include <atomic>
#include <deque>
#include <iostream>

#include <yaml-cpp/yaml.h>

#include <groundtruth_inertial_t.hpp>

#include "Publisher.hpp"
#include "RTProtocol.h"

/**
 * @brief A namespace for all things that interface with the Qualisys Motion Tracking System
 * directly
 *
 */
namespace qualisys
{
using StateMsg = groundtruth_inertial_t;

/**
 * @brief Manages receiving data from QTM and publishing it over ZCM in a StateMsg type.
 *
 */
class QualisysZCM
{
public:
    // Default construct
    QualisysZCM() = default;
    // Not copy-able
    QualisysZCM(const QualisysZCM&) = delete;
    QualisysZCM& operator==(const QualisysZCM&) = delete;

    /**
     * @brief Configure this object using data from a YAML config. This QualisysZCm object is
     * invalid unless init is initialized by calling `init` first.
     *
     * @param yconf The YAML config Node used to initialize this object
     * @return true Initialization was successful
     * @return false Initialization failed
     */
    bool init(const YAML::Node& yconf);

    /**
     * @brief Read a packet of data from QTM and publish its info over ZCM
     *
     */
    void run();

    /**
     * @brief Disconnect from the QTM server
     *
     */
    void disconnect();

private:
    /**
     * @brief Helper function for dispatching a ZCM message
     *
     * @param packet The packet obtained from QTM
     */
    void handle_packet_data(CRTPacket* packet);

    /**
     * @brief Construct a StateMsg object for dispatch with the given info.
     *
     * @param i The body index
     * @param x The X position of the body
     * @param y The Y position of the body
     * @param z The Z position of the body
     * @param roll The roll Euler angle of the body
     * @param pitch The pitch Euler angle of the body
     * @param yaw The yaw Euler angle of the body
     * @return StateMsg The constructed StateMsg object.
     */
    StateMsg make_state(int i, float x, float y, float z, float roll, float pitch, float yaw);

    // Protocol for communicating with QTM
    CRTProtocol port_protocol_;

    Publisher<StateMsg> pub_;

    // Store previous state data to get velocities
    struct
    {
        std::deque<float> x, y, z;
    } prev_state_;

    // Printing
    void info(const StateMsg& state);
    bool print_pos_;
    bool print_vel_;
    bool print_ar_;
    int print_period_;  // how often to print
};
}  // namespace qualisys

#endif
