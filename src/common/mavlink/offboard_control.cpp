#include <common/mavlink/offboard_control.hpp>

#include <unistd.h>
#include <atomic>
#include <cassert>
#include <chrono>
#include <iostream>
#include <mutex>

using namespace std::chrono;
using std::this_thread::sleep_for;
using std::thread;
using std::cout;

namespace maav
{
namespace mavlink
{
std::atomic<bool> KILL{false};

// Creates thread and passing read_messages loop into thread
OffboardControl::OffboardControl(const CommunicationType com_type, const std::string& port_path)
    : com_port(com_type, port_path)
{
    offboard_control_active = false;

    // Read messages until we get a heartbeat
    cout << "Checking for heartbeat...\n";
    while (!read_message())
        ;

    cout << "Heartbeat received\n";

    // Start read thread and heartbeat (TODO: datalink like Qgroundcontrol)
    read_tid = thread(&OffboardControl::read_thread, this);

    // Spam zero attitudes to pixhawk to make it happy
    cout << "Zeroing attitude setpoints...\n";
    hold_zero_attitude(1);

    cout << "Requesting offboard control...\n";
    if (activate_offboard_control())
    {
        cout << "Established offboard control on pixhawk\n";
    }
    else
    {
        cout << "Unable to establish off board control!\n";
    }

    // Spam more setpoints
    hold_zero_attitude(1);

    // Arm quad, this is for real.
    cout << "Arming system\n";
    if (arm())
    {
        cout << "System armed\n";
    }
    else
    {
        cout << "System failed to arm\n";
    }
}

OffboardControl::~OffboardControl()
{
    KILL = true;
    read_tid.join();
}

void OffboardControl::read_thread()
{
    while (!KILL)
    {
        read_message();  // read incomming messages

        send_heartbeat();  // maintain link with pixhawk, if link lost pixhawk -> failsafe
        // This heartbeat is probably too fast, consider lowering rate (what is the minimum?)
    }
}

bool OffboardControl::read_message()
{
    mavlink_message_t message;
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
                    check_offboard_control();  // check each heartbeat
                    // TODO: handle loss of heartbeat
                    heartbeat_received = true;
                    break;
                case MAVLINK_MSG_ID_SYSTEM_TIME:
                    mavlink_msg_system_time_decode(&message, &(current_messages_in.system_time));
                    ping(current_messages_in.system_time.time_boot_ms * 1000);  // convert to us
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

void OffboardControl::check_offboard_control()
{
    if (current_messages_in.heartbeat.custom_mode != custom_mode && offboard_control_active)
    {
        offboard_control_active = false;
        cout << "Lost offboard control on pixhawk\n";
    }
}

void OffboardControl::hold_zero_attitude(const uint64_t seconds)
{
    for (uint64_t i = 0; i < 100 * seconds; ++i)
    {
        set_attitude_target(InnerLoopSetpoint());
        ;
        sleep_for(10ms);
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

    auto timeout = system_clock::now() + 10s;
    while (!offboard_control_active)
    {
        set_attitude_target(InnerLoopSetpoint());
        sleep_for(100ms);

        write_message(message);

        sleep_for(100ms);

        if (current_messages_in.heartbeat.custom_mode == custom_mode)
        {
            offboard_control_active = true;
            break;
        }

        // Timeout
        if (system_clock::now() > timeout)
        {
            cout << "Timeout establishing offboard control of pixhawk\n\n";
            return false;
        }
    }

    return true;
}

bool OffboardControl::arm()
{
    mavlink_command_long_t command;
    command.command = MAV_CMD_COMPONENT_ARM_DISARM;
    command.target_system = system_id;
    command.target_component = autopilot_id;
    command.confirmation = 0;  // idk what this is doing
    command.param1 = 1.;       // 1 arm, 0 disarm

    mavlink_message_t message;
    mavlink_msg_command_long_encode(system_id, companion_id, &message, &command);

    auto timeout = system_clock::now() + 10s;
    while (system_clock::now() < timeout)
    {
        write_message(message);
        set_attitude_target(InnerLoopSetpoint());
        if (current_messages_in.heartbeat.base_mode == armed_base_mode)
        {
            return true;
        }
    }

    return false;
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

void OffboardControl::takeoff(const float takeoff_altitude)
{
    assert(false);
    // NOTE: auto:takeoff requires gps, implement using height controller
}

}  // maav
}  // gnc
