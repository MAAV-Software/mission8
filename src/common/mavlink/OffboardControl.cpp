#include <unistd.h>
#include <atomic>
#include <cassert>
#include <chrono>
#include <csignal>
#include <iostream>
#include <mutex>

#include <common/mavlink/OffboardControl.hpp>
#include <common/messages/MsgChannels.hpp>
#include <common/messages/attitude_target_t.hpp>

using namespace std::chrono;
using std::cout;
using std::this_thread::sleep_for;
using std::thread;

namespace maav
{
namespace mavlink
{
std::atomic<bool> END{false};

// Creates thread and passing read_messages loop into thread
OffboardControl::OffboardControl(const CommunicationType com_type, const std::string& port_path)
    : com_port(com_type, port_path)
{
    offboard_control_active = false;
    read_tid = thread(&OffboardControl::read_thread, this);
    attitude_target_t at;
}

OffboardControl::~OffboardControl()
{
    END = true;
    if (read_tid.joinable()) read_tid.join();
}

void OffboardControl::read_thread()
{
    while (!END)
    {
        readMessage();  // read incomming messages

        send_heartbeat();  // maintain link with pixhawk, if link lost pixhawk -> failsafe
        // This heartbeat is probably too fast, consider lowering rate (what is the minimum?)
    }
}

bool OffboardControl::readMessage()
{
    mavlink_message_t message;
    attitude_target_t at;
    bool heartbeat_received = false;

    // Try to read 100 messages
    for (int i = 0; i < 100; ++i)
    {
        bool message_received = false;
        message_received = com_port.read_message(message);

        if (message_received)
        {
            switch (message.msgid)
            {
                case MAVLINK_MSG_ID_HEARTBEAT:
                    mavlink_msg_heartbeat_decode(&message, &(current_messages_in.heartbeat));
                    check_offboard_control();
                    heartbeat_received = true;
                    break;
                case MAVLINK_MSG_ID_SYSTEM_TIME:
                    mavlink_msg_system_time_decode(&message, &(current_messages_in.system_time));
                    ping(current_messages_in.system_time.time_boot_ms * 1000);  // convert to us
                    break;
                case MAVLINK_MSG_ID_ALTITUDE:
                    altitude_last = alt_filter.getState();
                    usec_last = ems_state.usec;
                    z_velocity_last = ems_state.z_velocity;

                    mavlink_msg_altitude_decode(&message, &(current_messages_in.altitude));
                    ems_state.usec = current_messages_in.altitude.time_usec;
                    alt_filter.run(current_messages_in.altitude.altitude_monotonic);

                    dt = static_cast<double>(ems_state.usec - usec_last) / 1.e6;

                    vel_filter.run(-1 * (alt_filter.getState() - altitude_last) / dt);
                    ems_state.z_velocity = vel_filter.getState();
                    break;
                case MAVLINK_MSG_ID_ATTITUDE_TARGET:
                    mavlink_msg_attitude_target_decode(
                        &message, &(current_messages_in.attitude_target));
                    at.time_boot_ms = current_messages_in.attitude_target.time_boot_ms;
                    at.q[0] = current_messages_in.attitude_target.q[0];
                    at.q[1] = current_messages_in.attitude_target.q[1];
                    at.q[2] = current_messages_in.attitude_target.q[2];
                    at.q[3] = current_messages_in.attitude_target.q[3];
                    at.roll_rate = current_messages_in.attitude_target.body_roll_rate;
                    at.pitch_rate = current_messages_in.attitude_target.body_pitch_rate;
                    at.yaw_rate = current_messages_in.attitude_target.body_yaw_rate;
                    at.thrust = current_messages_in.attitude_target.thrust;
                    break;
                default:
                    break;
            }
        }
    }

    return heartbeat_received;  // return this so we can look for first heartbeat
}

void OffboardControl::write_message(const mavlink_message_t& message)
{
    // Currently doesnt provide much additional functionality but it
    // abstracts away the com port write
    com_port.write_message(message);
}

bool OffboardControl::check_offboard_control()
{
    if (current_messages_in.heartbeat.custom_mode != custom_mode && offboard_control_active)
    {
        offboard_control_active = false;
        cout << "Lost offboard control on pixhawk\n";
        return false;
    }
    return true;
}

bool OffboardControl::offboardControlActive() { return offboard_control_active; }

void OffboardControl::holdZeroAttitude(const std::chrono::seconds hold_period)
{
    auto timeout = std::chrono::system_clock::now() + hold_period;
    while (std::chrono::system_clock::now() < timeout)
    {
        set_attitude_target(InnerLoopSetpoint());
        sleep_for(1ms);
    }
}

void OffboardControl::ping(const uint64_t boot_timestamp)
{
    mavlink_ping_t ping_message;
    ping_message.target_system = system_id;
    ping_message.target_component = autopilot_id;
    ping_message.seq = 1234;
    ping_message.time_usec = boot_timestamp;

    mavlink_message_t message;
    mavlink_msg_ping_encode(system_id, companion_id, &message, &ping_message);

    write_message(message);
}

void OffboardControl::send_heartbeat()
{
    mavlink_heartbeat_t heartbeat;
    heartbeat.type = MAV_TYPE_GCS;
    heartbeat.autopilot = MAV_AUTOPILOT_INVALID;  // identify as GCS/offboard control
    heartbeat.base_mode = 0;                      // no relavent mode for gcs, set to 0
    heartbeat.system_status = MAV_STATE_ACTIVE;
    heartbeat.custom_mode = 0;

    mavlink_message_t message;
    mavlink_msg_heartbeat_encode(system_id, companion_id, &message, &heartbeat);
    write_message(message);
}

bool OffboardControl::activate_offboard_control()
{
    // This command will fail if the pixhawk is not
    // receiving offboard position/attitude commands
    // at a rate >2 Hz
    mavlink_command_long_t command;
    command.command = MAV_CMD_NAV_GUIDED_ENABLE;
    command.target_system = system_id;
    command.target_component = autopilot_id;
    command.confirmation = 0;  // idk what this is doing, true seems bad, 2 seems better
    command.param1 = 1;        // >0.5 activate, <0.5 deactivate

    mavlink_message_t message;
    mavlink_msg_command_long_encode(system_id, companion_id, &message, &command);

    set_attitude_target(InnerLoopSetpoint());
    sleep_for(100ms);

    write_message(message);

    sleep_for(100ms);

    if (current_messages_in.heartbeat.custom_mode == custom_mode)
    {
        return offboard_control_active = true;
    }

    return offboard_control_active = false;
}

bool OffboardControl::arm() { return arm_disarm(true); }

bool OffboardControl::disarm() { return arm_disarm(false); }

bool OffboardControl::arm_disarm(const bool arm)
{
    mavlink_command_long_t command;
    command.command = MAV_CMD_COMPONENT_ARM_DISARM;
    command.target_system = system_id;
    command.target_component = autopilot_id;
    command.confirmation = 0;  // idk what this is doing
    if (arm)
        command.param1 = 1.;  // 1 arm, 0 disarm
    else
        command.param1 = 0.;

    mavlink_message_t message;
    mavlink_msg_command_long_encode(system_id, companion_id, &message, &command);

    write_message(message);
    if (is_armed())
    {
        return true;
    }

    return false;
}

bool OffboardControl::is_armed()
{
    return current_messages_in.heartbeat.base_mode == armed_base_mode;
}

void OffboardControl::set_attitude_target(
    const InnerLoopSetpoint& new_setpoint, const uint8_t type_mask)
{
    assert(new_setpoint.thrust >= 0 && new_setpoint.thrust <= 1);

    mavlink_set_attitude_target_t setpoint;
    setpoint.time_boot_ms = current_messages_in.system_time.time_boot_ms;
    setpoint.target_system = system_id;
    setpoint.target_component = autopilot_id;
    setpoint.type_mask = type_mask;  // default typemask ignores nothing

    setpoint.q[0] = new_setpoint.q.w();
    setpoint.q[1] = new_setpoint.q.x();
    setpoint.q[2] = new_setpoint.q.y();
    setpoint.q[3] = new_setpoint.q.z();
    setpoint.body_roll_rate = new_setpoint.roll_rate;
    setpoint.body_pitch_rate = new_setpoint.pitch_rate;
    setpoint.body_yaw_rate = new_setpoint.yaw_rate;
    setpoint.thrust = new_setpoint.thrust;

    mavlink_message_t message;
    mavlink_msg_set_attitude_target_encode(system_id, companion_id, &message, &setpoint);
    write_message(message);
}

}  // maav
}  // gnc
