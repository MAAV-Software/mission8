#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <string>
#include <sstream>

#include <zcm/zcm-cpp.hpp>

#include <common/messages/MsgChannels.hpp>
#include <common/messages/heartbeat_t.hpp>
#include <common/messages/state_t.hpp>
#include <common/messages/camera_pose_t.hpp>
#include <common/utils/GetOpt.hpp>
#include <vision/core/utilities.hpp>

using std::atomic;
using std::condition_variable;
using std::mutex;
using std::string;
using std::unique_lock;
using std::unique_ptr;
using std::vector;
using std::stringstream;
using zcm::ZCM;
using std::shared_ptr;
using pcl::PointCloud;
using pcl::PointXYZ;
using std::chrono::milliseconds;
using std::thread;
using std::string;

// Used for synchronization with the kill signal
mutex mtx;
condition_variable cond_var;

class Handler
{
public:
    explicit Handler(zcm::ZCM& zcm) : zcm_ {zcm} {}

    // Forwards all relevant information over the state channel
    void handle(const zcm::ReceiveBuffer*, const std::string&,
        const camera_pose_t* message)
    {
        state_t gnc_state;
        gnc_state.utime = message->utime;

        // Copy over position data
        gnc_state.position.data[0] = message->x_translation_;
        gnc_state.position.data[1] = message->y_translation_;
        gnc_state.position.data[2] = message->z_translation_;

        // Copy over all velocity data
        gnc_state.velocity.data[0] = message->x_velocity_;
        gnc_state.velocity.data[1] = message->y_velocity_;
        gnc_state.velocity.data[2] = message->z_velocity_;

        // Copy over all acceleration data
        gnc_state.acceleration.data[0] = message->x_acceleration_;
        gnc_state.acceleration.data[1] = message->y_acceleration_;
        gnc_state.acceleration.data[2] = message->z_acceleration_;

        // Copy over all attitude data
        gnc_state.attitude.data[0] = message->Qi_rotation_;
        gnc_state.attitude.data[1] = message->Qj_rotation_;
        gnc_state.attitude.data[2] = message->Qk_rotation_;
        gnc_state.attitude.data[3] = message->Qr_rotation_;

        gnc_state.covariance.rows = 0;
        gnc_state.covariance.cols = 0;

        zcm_.publish(maav::STATE_CHANNEL, &gnc_state);
    }
private:
    zcm::ZCM& zcm_;
};

// Keeps track of whether the kill signal has been received
atomic<bool> KILL{false};

void sigHandler(int)
{
    unique_lock<mutex> lck(mtx);
    KILL = true;
    cond_var.notify_one();
}

// Sends heartbeats every 100 milliseconds
// Also the main thread waits for this to end
// to know when the kill signal has been sent
void runHeartbeat(zcm::ZCM* zcm)
{
    heartbeat_t msg;
    msg.alive = true;
    while (!KILL)
    {
        zcm->publish(maav::STATE_FORWARD_HEARTBEAT_CHANNEL, &msg);
        std::this_thread::sleep_for(milliseconds(100));
    }
}

int main(int argc, char** argv)
{
    // Bind sigHandler
    signal(SIGINT, sigHandler);
    signal(SIGABRT, sigHandler);
    signal(SIGSEGV, sigHandler);
    signal(SIGTERM, sigHandler);

    GetOpt gopt;
    gopt.addBool('h', "help", false, "This message");

    if (!gopt.parse(argc, argv, 1) || gopt.getBool("help"))
    {
        std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
        gopt.printHelp();
        return 1;
    }

    zcm::ZCM zcm {"ipc"};
    Handler handler(zcm);
    zcm.subscribe(maav::CAMERA_POS_CHANNEL, &Handler::handle, &handler);
    zcm.start();
    // start heartbeat thread
    thread heartbeat(runHeartbeat, &zcm);

    // Wait until the kill signal is received
    unique_lock<mutex> lck(mtx);
    while (!KILL)
    {
        cond_var.wait(lck);
    }
    zcm.stop();
    heartbeat.join();
}
