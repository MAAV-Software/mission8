#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <string>

#include <yaml-cpp/yaml.h>
#include <zcm/zcm-cpp.hpp>

#include <common/mavlink/AutopilotInterface.hpp>
#include <common/mavlink/SerialPort.hpp>
#include <common/utils/GetOpt.hpp>
#include <gnc/control/StateMachine.hpp>

using maav::gnc::control::StateMachine;
using maav::mavlink::AutopilotInterface;
using maav::mavlink::SerialPort;

using namespace std::chrono_literals;

std::atomic<bool> KILL{false};
void sig_handler(int) { KILL = true; }
bool findConnection(YAML::Node config, SerialPort*& uart_port, AutopilotInterface*& interface);

int main(int argc, char** argv)
{
    /*
     *      Signal handlers
     */
    signal(SIGINT, sig_handler);
    signal(SIGABRT, sig_handler);
    signal(SIGSEGV, sig_handler);
    signal(SIGTERM, sig_handler);

    /*
     *      Command line arguments
     */
    GetOpt gopt;
    gopt.addBool('h', "help", false, "This message");
    gopt.addString('c', "config", "../config/gnc/control-config.yaml", "Path to control config");
    gopt.addBool('v', "verbose", true, "Print information");
    if (!gopt.parse(argc, argv, 1) || gopt.getBool("help"))
    {
        std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
        gopt.printHelp();
        return 1;
    }

    YAML::Node config;
    const std::string control_config_file = gopt.getString("config");
    /*
     *  Try to open the control config before passing it into
     *  the classes.  This alerts user to pesky cryptic error
     *  when invalid file is passed
     */
    try
    {
        config = YAML::LoadFile(control_config_file);
    }
    catch (...)
    {
        std::cout << "Could not find config file\nPlease provide command line option  \"-c  "
                     "<path-to-config>\"\n";
        return 2;
    }

    SerialPort* uart_port = nullptr;
    AutopilotInterface* autopilot_interface = nullptr;

    bool success = findConnection(config, uart_port, autopilot_interface);
    if (success)
    {
        StateMachine state_machine(control_config_file, autopilot_interface);
        state_machine.run(KILL);
    }

    // Clean up
    if (autopilot_interface)
    {
        // autopilot_interface->disable_offboard_control();
        autopilot_interface->stop();
        delete autopilot_interface;
    }
    if (uart_port)
    {
        uart_port->stop();
        delete uart_port;
    }
}

bool findConnection(YAML::Node config, SerialPort*& uart_port, AutopilotInterface*& interface)
{
    constexpr int OFFBOARD_BAUDRATE = 921600;
    constexpr std::array<const char*, 8> possible_files = {"/dev/ttyUSB0", "/dev/ttyUSB1",
        "/dev/ttyUSB2", "/dev/ttyUSB3", "/dev/ttyUSB4", "/dev/ttyUSB5", "/dev/ttyUSB6",
        "/dev/ttyUSB7"};

    std::cout << "Checking for serial connection..." << std::endl;
    for (const char* serial_port : possible_files)
    {
        std::cout << "Checking " << std::string(serial_port) << std::endl;
        uart_port = new SerialPort(serial_port, OFFBOARD_BAUDRATE);
        interface = new AutopilotInterface(uart_port);

        try
        {
            uart_port->start();
            interface->start();
            std::cout << "UART connection found!" << std::endl;
            return true;
        }
        catch (int e)
        {
            delete uart_port;
            delete interface;
        }
    }

    std::cout << "Serial connection not found." << std::endl;
    std::cout << "Checking for UDP connection (simulator)..." << std::endl;

    interface = new AutopilotInterface(maav::mavlink::CommunicationType::UDP);
    try
    {
        interface->start();
        std::cout << "UDP connection found!" << std::endl;
        return true;
    }
    catch (int e)
    {
        interface->stop();
        delete interface;
    }

    std::cout << "UDP connection not found." << std::endl;

    return false;
}
