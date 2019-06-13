/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file autopilot_interface.cpp
 *
 * @brief Autopilot interface functions
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#define MAVLINK_HELPER static inline
#include <mavlink/v2.0/mavlink_conversions.h>
#include <common/mavlink/AutopilotInterface.hpp>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;

namespace maav
{
namespace mavlink
{
// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
uint64_t get_time_usec()
{
    static struct timeval _time_stamp;
    gettimeofday(&_time_stamp, NULL);
    return _time_stamp.tv_sec * 1000000 + _time_stamp.tv_usec;
}

// ----------------------------------------------------------------------------------
//   Setpoint Helper Functions
// ----------------------------------------------------------------------------------

// choose one of the next three

/*
 * Set target local ned position
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target XYZ locations
 * in the Local NED frame, in meters.
 */
void set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp)
{
    sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;

    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

    sp.x = x;
    sp.y = y;
    sp.z = z;

    printf("POSITION SETPOINT XYZ = [ %.4f , %.4f , %.4f ] \n", sp.x, sp.y, sp.z);
}

/*
 * Set target local ned velocity
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target VX VY VZ
 * velocities in the Local NED frame, in meters per second.
 */
void set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp)
{
    sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY;

    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

    sp.vx = vx;
    sp.vy = vy;
    sp.vz = vz;

    // printf("VELOCITY SETPOINT UVW = [ %.4f , %.4f , %.4f ] \n", sp.vx, sp.vy, sp.vz);
}

/*
 * Set target local ned acceleration
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target AX AY AZ
 * accelerations in the Local NED frame, in meters per second squared.
 */
void set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t &sp)
{
    // NOT IMPLEMENTED
    fprintf(stderr, "set_acceleration doesn't work yet \n");
    throw 1;

    sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION &
                   MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY;

    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

    sp.afx = ax;
    sp.afy = ay;
    sp.afz = az;
}

// the next two need to be called after one of the above

/*
 * Set target local ned yaw
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw
 * in the Local NED frame, in radians.
 */
void set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp)
{
    sp.type_mask &= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;

    sp.yaw = yaw;

    printf("POSITION SETPOINT YAW = %.4f \n", sp.yaw);
}

/*
 * Set target local ned yaw rate
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with a target yaw rate
 * in the Local NED frame, in radians per second.
 */
void set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp)
{
    sp.type_mask &= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;

    sp.yaw_rate = yaw_rate;
}

// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------

AutopilotInterface::AutopilotInterface(CommunicationType com_type) : com_type_(com_type)
{
    switch (com_type)
    {
        case CommunicationType::UART:
            std::cout << "No default uart constructor" << std::endl;
            throw EXIT_FAILURE;
            break;
        case CommunicationType::UDP:

            setDefaults();

            udp_socket = socket(AF_INET, SOCK_DGRAM, 0);

            local_address.sin_family = AF_INET;
            local_address.sin_addr.s_addr = INADDR_ANY;
            local_address.sin_port = htons(14540);

            bind(udp_socket, (struct sockaddr *)&local_address, sizeof(sockaddr));
            fcntl(udp_socket, F_SETFL, O_NONBLOCK | O_ASYNC);

            remote_address.sin_family = AF_INET;
            remote_address.sin_addr.s_addr = inet_addr("127.0.0.1");
            remote_address.sin_port = htons(14557);
            break;
    }
}

AutopilotInterface::AutopilotInterface(SerialPort *serial_port_)
    : com_type_(CommunicationType::UART)
{
    setDefaults();
    serial_port = serial_port_;  // serial port management object
}

AutopilotInterface::~AutopilotInterface() {}

void AutopilotInterface::setDefaults()
{
    // initialize attributes
    write_count = 0;

    reading_status = 0;  // whether the read thread is running
    writing_status = 0;  // whether the write thread is running
    control_status = 0;  // whether the autopilot is in offboard control mode
    KILL = false;        // flag to signal thread exit

    system_id = 0;     // system id
    autopilot_id = 0;  // autopilot component id
    companion_id = 0;  // companion computer component id

    current_messages.sysid = system_id;
    current_messages.compid = autopilot_id;

    armed_ = false;
    offboard_ = false;
    requested_arm_ = false;
    requested_disarm_ = false;
}

// ------------------------------------------------------------------------------
//   Update Setpoint
// ------------------------------------------------------------------------------
void AutopilotInterface::update_setpoint(InnerLoopSetpoint &setpoint)
{
    std::lock_guard<std::mutex> guard(setpoint_mutex_);

    assert(setpoint.thrust >= 0 && setpoint.thrust <= 1);

    // current_setpoint.time_boot_ms = current_messages_in.system_time.time_boot_ms;
    current_setpoint.target_system = system_id;
    current_setpoint.target_component = autopilot_id;
    current_setpoint.type_mask = 0;  // default typemask ignores nothing

    mavlink_euler_to_quaternion(setpoint.roll, setpoint.pitch, setpoint.yaw, current_setpoint.q);
    current_setpoint.body_roll_rate = setpoint.roll_rate;
    current_setpoint.body_pitch_rate = setpoint.pitch_rate;
    current_setpoint.body_yaw_rate = setpoint.yaw_rate;
    current_setpoint.thrust = setpoint.thrust;
}

// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void AutopilotInterface::read_messages()
{
    bool success = false;       // receive success flag
    bool received_all = false;  // receive only one message
    TimeStamps this_timestamps;

    // Blocking wait for new data
    while (!received_all && !KILL)
    {
        // ----------------------------------------------------------------------
        //   READ MESSAGE
        // ----------------------------------------------------------------------
        mavlink_message_t message;
        switch (com_type_)
        {
            case CommunicationType::UART:
                success = serial_port->read_message(message);
                break;
            case CommunicationType::UDP:
                char buffer[BUFFER_SIZE];
                memset(buffer, 0, BUFFER_SIZE);
                int recsize = read(udp_socket, buffer, BUFFER_SIZE);

                mavlink_status_t status;
                bool message_received = false;
                // using status so it doesnt through an error (bullshit short term fix)
                if (status.buffer_overrun > 0) message_received = false;

                if (recsize > 0)
                {
                    for (int i = 0; i < recsize; ++i)
                    {
                        message_received =
                            mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &message, &status);
                    }
                }
                success = message_received;
                break;
        }

        // ----------------------------------------------------------------------
        //   HANDLE MESSAGE
        // ----------------------------------------------------------------------
        if (success)
        {
            // Store message sysid and compid.
            // Note this doesn't handle multiple message sources.
            current_messages.sysid = message.sysid;
            current_messages.compid = message.compid;

            // Handle Message ID
            switch (message.msgid)
            {
                case MAVLINK_MSG_ID_COMMAND_ACK:
                    mavlink_msg_command_ack_decode(&message, &(current_messages.ack));
                    current_messages.time_stamps.ack = get_time_usec();
                    this_timestamps.ack = current_messages.time_stamps.ack;

                    switch (current_messages.ack.command)
                    {
                        case MAV_CMD_COMPONENT_ARM_DISARM:
                            std::cout << "ARM_DISARM ACK" << std::endl;
                            if (current_messages.ack.result == MAV_RESULT_ACCEPTED)
                            {
                                if (requested_disarm_)
                                {
                                    std::cout << "DISARMED" << std::endl;
                                    requested_disarm_ = false;
                                    armed_ = false;
                                }
                                if (requested_arm_)
                                {
                                    std::cout << "ARMED" << std::endl;
                                    requested_arm_ = false;
                                    armed_ = true;
                                }
                            }
                            else
                            {
                                if (requested_disarm_)
                                {
                                    std::cout << "DISARMING DENIED" << std::endl;
                                }
                                if (requested_arm_)
                                {
                                    std::cout << "ARMING DENIED" << std::endl;
                                }
                            }
                            break;
                        case MAV_CMD_NAV_GUIDED_ENABLE:
                            std::cout << "OFFBOARD ACK" << std::endl;
                            if (current_messages.ack.result == MAV_RESULT_ACCEPTED)
                            {
                                if (offboardMode())
                                {
                                    std::cout << "OFFBOARD MODE DISABLED" << std::endl;
                                    offboard_ = false;
                                }
                                else
                                {
                                    std::cout << "OFFBOARD MODE ENABLED" << std::endl;
                                    offboard_ = true;
                                }
                            }
                            else
                            {
                                if (offboardMode())
                                {
                                    std::cout << "OFFBOARD MODE SWITCH DENIED" << std::endl;
                                }
                                else
                                {
                                    std::cout << "OFFBOARD MODE DENIED" << std::endl;
                                }
                            }

                            break;
                    }
                    break;

                case MAVLINK_MSG_ID_HEARTBEAT:
                {
                    // printf("MAVLINK_MSG_ID_HEARTBEAT\n");
                    mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
                    current_messages.time_stamps.heartbeat = get_time_usec();
                    this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
                    break;
                }

                case MAVLINK_MSG_ID_SYS_STATUS:
                {
                    // printf("MAVLINK_MSG_ID_SYS_STATUS\n");
                    mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
                    current_messages.time_stamps.sys_status = get_time_usec();
                    this_timestamps.sys_status = current_messages.time_stamps.sys_status;
                    break;
                }

                case MAVLINK_MSG_ID_BATTERY_STATUS:
                {
                    // printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
                    mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
                    current_messages.time_stamps.battery_status = get_time_usec();
                    this_timestamps.battery_status = current_messages.time_stamps.battery_status;
                    break;
                }

                case MAVLINK_MSG_ID_RADIO_STATUS:
                {
                    // printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
                    mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
                    current_messages.time_stamps.radio_status = get_time_usec();
                    this_timestamps.radio_status = current_messages.time_stamps.radio_status;
                    break;
                }

                case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                {
                    // printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
                    mavlink_msg_local_position_ned_decode(
                        &message, &(current_messages.local_position_ned));
                    current_messages.time_stamps.local_position_ned = get_time_usec();
                    this_timestamps.local_position_ned =
                        current_messages.time_stamps.local_position_ned;
                    break;
                }

                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                {
                    // printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
                    mavlink_msg_global_position_int_decode(
                        &message, &(current_messages.global_position_int));
                    current_messages.time_stamps.global_position_int = get_time_usec();
                    this_timestamps.global_position_int =
                        current_messages.time_stamps.global_position_int;
                    break;
                }

                case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
                {
                    // printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
                    mavlink_msg_position_target_local_ned_decode(
                        &message, &(current_messages.position_target_local_ned));
                    current_messages.time_stamps.position_target_local_ned = get_time_usec();
                    this_timestamps.position_target_local_ned =
                        current_messages.time_stamps.position_target_local_ned;
                    break;
                }

                case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
                {
                    // printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
                    mavlink_msg_position_target_global_int_decode(
                        &message, &(current_messages.position_target_global_int));
                    current_messages.time_stamps.position_target_global_int = get_time_usec();
                    this_timestamps.position_target_global_int =
                        current_messages.time_stamps.position_target_global_int;
                    break;
                }

                case MAVLINK_MSG_ID_HIGHRES_IMU:
                {
                    // printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
                    mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
                    current_messages.time_stamps.highres_imu = get_time_usec();
                    this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
                    break;
                }

                case MAVLINK_MSG_ID_ATTITUDE:
                {
                    // printf("MAVLINK_MSG_ID_ATTITUDE\n");
                    mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
                    current_messages.time_stamps.attitude = get_time_usec();
                    this_timestamps.attitude = current_messages.time_stamps.attitude;
                    break;
                }

                default:
                {
                    // printf("Warning, did not handle message id %i\n",message.msgid);
                    break;
                }

            }  // end: switch msgid

        }  // end: if read message

        // Check for receipt of all items
        received_all = this_timestamps.heartbeat &&
                       //				this_timestamps.battery_status             &&
                       //				this_timestamps.radio_status               &&
                       //				this_timestamps.local_position_ned         &&
                       //				this_timestamps.global_position_int        &&
                       //				this_timestamps.position_target_local_ned  &&
                       //				this_timestamps.position_target_global_int &&
                       //				this_timestamps.highres_imu                &&
                       //				this_timestamps.attitude                   &&
                       this_timestamps.sys_status;

        // give the write thread time to use the port
        if (writing_status > false)
        {
            usleep(100);  // look for components of batches at 10kHz
        }

    }  // namespace mavlink

    return;
}  // namespace maav

// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int AutopilotInterface::write_message(mavlink_message_t message)
{
    std::lock_guard<std::mutex> guard(write_mutex_);

    int len = 0;
    switch (com_type_)
    {
        case CommunicationType::UART:
            // do the write
            len = serial_port->write_message(message);
            // book keep
            write_count++;
            break;

        case CommunicationType::UDP:
            char buffer[BUFFER_SIZE];
            mavlink_msg_to_send_buffer((uint8_t *)buffer, &message);

            try
            {
                len = sendto(udp_socket, buffer, BUFFER_SIZE, 0, (struct sockaddr *)&remote_address,
                    sizeof(struct sockaddr_in));
            }
            catch (int e)
            {
            }
            break;
    }

    return len;
}

// ------------------------------------------------------------------------------
//   Write Setpoint Message
// ------------------------------------------------------------------------------
void AutopilotInterface::write_setpoint()
{
    // --------------------------------------------------------------------------
    //   PACK PAYLOAD
    // --------------------------------------------------------------------------

    // pull from position target
    mavlink_set_attitude_target_t sp;
    {
        std::lock_guard<std::mutex> guard(setpoint_mutex_);
        sp = current_setpoint;
    }

    // double check some system parameters
    if (not sp.time_boot_ms) sp.time_boot_ms = (uint32_t)(get_time_usec() / 1000);
    sp.target_system = system_id;
    sp.target_component = autopilot_id;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------

    mavlink_message_t message;
    mavlink_msg_set_attitude_target_encode(system_id, companion_id, &message, &sp);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------

    // do the write
    int len = write_message(message);

    // check the write
    if (len <= 0) fprintf(stderr, "WARNING: could not send attitude set point \n");
    //	else
    //		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x,
    // position_target.y, position_target.z);

    return;
}

// ------------------------------------------------------------------------------
//   Start Off-Board Mode
// ------------------------------------------------------------------------------

bool AutopilotInterface::offboardMode() const { return offboard_; }

void AutopilotInterface::enable_offboard_control()
{
    // Should only send this command once
    if (control_status == false)
    {
        std::cout << "ENABLE OFFBOARD MODE" << std::endl;
        // printf("ENABLE OFFBOARD MODE\n");

        // ----------------------------------------------------------------------
        //   TOGGLE OFF-BOARD MODE
        // ----------------------------------------------------------------------

        // Sends the command to go off-board
        int success = toggle_offboard_control(true);

        // Check the command was written
        if (success)
            control_status = true;
        else
        {
            fprintf(stderr, "Error: off-board mode not set, could not write message\n");
            // throw EXIT_FAILURE;
        }

        printf("\n");

    }  // end: if not offboard_status
}

// ------------------------------------------------------------------------------
//   Stop Off-Board Mode
// ------------------------------------------------------------------------------
void AutopilotInterface::disable_offboard_control()
{
    // Should only send this command once
    if (control_status == true)
    {
        printf("DISABLE OFFBOARD MODE\n");

        // ----------------------------------------------------------------------
        //   TOGGLE OFF-BOARD MODE
        // ----------------------------------------------------------------------

        // Sends the command to stop off-board
        int success = toggle_offboard_control(false);

        // Check the command was written
        if (success)
            control_status = false;
        else
        {
            fprintf(stderr, "Error: off-board mode not set, could not write message\n");
            // throw EXIT_FAILURE;
        }

        printf("\n");

    }  // end: if offboard_status
}

// ------------------------------------------------------------------------------
//   Toggle Off-Board Mode
// ------------------------------------------------------------------------------
int AutopilotInterface::toggle_offboard_control(bool flag)
{
    // Prepare command for off-board mode
    mavlink_command_long_t com = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    com.target_system = system_id;
    com.target_component = autopilot_id;
    com.command = MAV_CMD_NAV_GUIDED_ENABLE;
    com.confirmation = true;
    com.param1 = (float)flag;  // flag >0.5 => start, <0.5 => stop

    // Encode
    mavlink_message_t message;
    mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

    // Send the message
    return write_message(message);
}

bool AutopilotInterface::armed() const { return armed_; }

bool AutopilotInterface::arm()
{
    if (!armed()) requested_arm_ = true;
    return arm_disarm(true);
}

bool AutopilotInterface::disarm()
{
    if (armed()) requested_disarm_ = true;
    return arm_disarm(false);
}

bool AutopilotInterface::arm_disarm(const bool arm)
{
    mavlink_command_long_t command = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    command.command = MAV_CMD_COMPONENT_ARM_DISARM;
    command.target_system = system_id;
    command.target_component = autopilot_id;
    command.confirmation = true;  // idk what this is doing

    // 1 arm, 0 disarm
    if (arm)
        command.param1 = 1.;
    else
        command.param1 = 0.;

    mavlink_message_t message;
    mavlink_msg_command_long_encode(system_id, companion_id, &message, &command);

    return write_message(message);
}

// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void AutopilotInterface::start()
{
    // --------------------------------------------------------------------------
    //   CHECK SERIAL PORT
    // --------------------------------------------------------------------------

    if (com_type_ == CommunicationType::UART)
    {
        if (serial_port->status != 1)  // SERIAL_PORT_OPEN
        {
            fprintf(stderr, "ERROR: serial port not open\n");
            throw 1;
        }
    }
    else
    {
    }

    // --------------------------------------------------------------------------
    //   READ THREAD
    // --------------------------------------------------------------------------

    printf("START READ THREAD \n");

    read_thread_ = std::thread(&AutopilotInterface::start_read_thread, this);

    // now we're reading messages
    printf("\n");

    // --------------------------------------------------------------------------
    //   CHECK FOR MESSAGES
    // --------------------------------------------------------------------------

    printf("CHECK FOR MESSAGES\n");

    const auto TIMEOUT_DURATION = 2s;

    // Try to connect for 2 seconds
    auto start = std::chrono::system_clock::now();
    while (not current_messages.sysid)
    {
        auto now = std::chrono::system_clock::now();
        if (now - start > TIMEOUT_DURATION || KILL) throw EXIT_FAILURE;
        std::this_thread::sleep_for(500ms);
    }

    printf("Found\n");

    // now we know autopilot is sending messages
    printf("\n");

    // --------------------------------------------------------------------------
    //   GET SYSTEM and COMPONENT IDs
    // --------------------------------------------------------------------------

    // This comes from the heartbeat, which in theory should only come from
    // the autopilot we're directly connected to it.  If there is more than one
    // vehicle then we can't expect to discover id's like this.
    // In which case set the id's manually.

    // System ID
    if (not system_id)
    {
        system_id = current_messages.sysid;
        printf("GOT VEHICLE SYSTEM ID: %i\n", system_id);
    }

    // Component ID
    if (not autopilot_id)
    {
        autopilot_id = current_messages.compid;
        printf("GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
        printf("\n");
    }

    // --------------------------------------------------------------------------
    //   GET INITIAL POSITION
    // --------------------------------------------------------------------------

    // Wait for initial position ned
    start = std::chrono::system_clock::now();
    while (not(
        current_messages.time_stamps.local_position_ned && current_messages.time_stamps.attitude))
    {
        auto now = std::chrono::system_clock::now();
        if (now - start > TIMEOUT_DURATION || KILL) throw EXIT_FAILURE;
        std::this_thread::sleep_for(500ms);
    }

    // copy initial position ned
    MavlinkMessages local_data = current_messages;
    initial_position.x = local_data.local_position_ned.x;
    initial_position.y = local_data.local_position_ned.y;
    initial_position.z = local_data.local_position_ned.z;
    initial_position.vx = local_data.local_position_ned.vx;
    initial_position.vy = local_data.local_position_ned.vy;
    initial_position.vz = local_data.local_position_ned.vz;
    initial_position.yaw = local_data.attitude.yaw;
    initial_position.yaw_rate = local_data.attitude.yawspeed;

    printf("INITIAL POSITION XYZ = [ %.4f , %.4f , %.4f ] \n", initial_position.x,
        initial_position.y, initial_position.z);
    printf("INITIAL POSITION YAW = %.4f \n", initial_position.yaw);
    printf("\n");

    // we need this before starting the write thread

    // --------------------------------------------------------------------------
    //   WRITE THREAD
    // --------------------------------------------------------------------------
    printf("START WRITE THREAD \n");

    write_thread_ = std::thread(&AutopilotInterface::start_write_thread, this);

    // wait for it to be started
    while (!writing_status) std::this_thread::sleep_for(100ms);  // 10Hz

    // now we're streaming setpoint commands
    printf("STARTED WRITING\n");

    // Done!
    return;
}

// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void AutopilotInterface::stop()
{
    if (com_type_ == CommunicationType::UDP)
    {
        close(udp_socket);
    }
    // --------------------------------------------------------------------------
    //   CLOSE THREADS
    // --------------------------------------------------------------------------
    printf("CLOSE THREADS\n");

    // signal exit
    KILL = true;

    // now the read and write threads are closed
    printf("\n");

    // still need to close the serial_port separately
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void AutopilotInterface::start_read_thread()
{
    if (reading_status != 0)
    {
        fprintf(stderr, "read thread already running\n");
        return;
    }
    else
    {
        read_thread();
        return;
    }
}

// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void AutopilotInterface::start_write_thread(void)
{
    if (not writing_status == false)
    {
        fprintf(stderr, "write thread already running\n");
        return;
    }

    else
    {
        write_thread();
        return;
    }
}

// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void AutopilotInterface::handle_quit(int sig)
{
    disable_offboard_control();

    try
    {
        stop();
    }
    catch (int error)
    {
        fprintf(stderr, "Warning, could not stop autopilot interface\n");
    }
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void AutopilotInterface::read_thread()
{
    reading_status = true;

    while (!KILL)
    {
        read_messages();
        std::this_thread::sleep_for(100ms);
    }

    reading_status = false;

    return;
}

// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void AutopilotInterface::write_thread(void)
{
    // signal startup
    writing_status = 2;

    // prepare an initial setpoint, just stay put
    mavlink_set_attitude_target_t sp;
    sp.body_pitch_rate = 0;
    sp.body_roll_rate = 0;
    sp.body_yaw_rate = 0;
    sp.q[0] = 1;
    sp.q[0] = 0;
    sp.q[0] = 0;
    sp.q[0] = 0;
    sp.thrust = 0;

    // set position target
    current_setpoint = sp;

    // write a message and signal writing
    write_setpoint();
    writing_status = true;

    // Pixhawk needs to see off-board commands at minimum 2Hz,
    // otherwise it will go into fail safe
    while (!KILL)
    {
        write_setpoint();

        // Write at 4Hz while on standby
        // 100Hz during flight
        // if (STANDBY)
        // std::this_thread::sleep_for(250ms);
        // else
        std::this_thread::sleep_for(10ms);
    }

    // signal end
    writing_status = false;

    return;
}

void AutopilotInterface::setStandby(bool stby) { STANDBY = stby; }

bool AutopilotInterface::simConnection() { return com_type_ == CommunicationType::UDP; }

}  // namespace mavlink
}  // namespace maav
