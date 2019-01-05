#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#include <common/messages/MsgChannels.hpp>
#include <common/messages/killswitch_t.hpp>
#include <zcm/zcm-cpp.hpp>

std::atomic<bool> KILL{false};
void sigHandler(int) { KILL = true; }
int main()
{
    signal(SIGINT, sigHandler);
    signal(SIGABRT, sigHandler);
    signal(SIGSEGV, sigHandler);
    signal(SIGTERM, sigHandler);

    zcm::ZCM zcm{"ipc"};
    killswitch_t killswitch;
    killswitch.kill = false;
    killswitch.utime = 0;
    zcm.publish(maav::KILLSWITCH_CHANNEL, &killswitch);

    std::string a;
    std::cout << "Press ENTER to activate killswitch\n";
    while (!KILL && getline(std::cin, a))
    {
        killswitch.kill = true;
        std::cout << "Kill sent to controller\n";
        while (!KILL) zcm.publish(maav::KILLSWITCH_CHANNEL, &killswitch);
    }
}