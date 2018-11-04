#include <chrono>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include <zcm/zcm-cpp.hpp>

#include "common/messages/MsgChannels.hpp"
#include "common/messages/laser_t.hpp"
#include "common/messages/obstacle_list_t.hpp"
#include "common/messages/obstacle_t.hpp"
#include "common/messages/state_t.hpp"
#include "common/utils/ZCMHandler.hpp"

#include "lidar-viewer.cpp"
#include "obstacles/ObstacleDetector.hpp"

using namespace std;

// Lidar mount rotation in rads. Rot 0 is the long part pointing in the x+ direction
double LIDAR_ROTATION = -M_PIl / 4;  // M_PIl is a higher precision PI constant
double LIDAR_MAX_RANGE = 4.0;

class LidarHandler
{
   public:
    LidarHandler(shared_ptr<zcm::ZCM> zcmptr_in, shared_ptr<ZCMSingleHandler<state_t>> state_ptr_in)
        : zcmptr{zcmptr_in}, state_ptr{state_ptr_in}, myViewer{false}

    {
    }

    void recv(const zcm::ReceiveBuffer*, const string& channel, const laser_t* msg)
    {
        // Get the most recent messages in a normal form
        state_t thisState = state_ptr->msg();
        thisLaser = *msg;

        // Rotate lidar data to correct orientation
        thisLaser.rad0 += LIDAR_ROTATION;

        // cout << setprecision(2);

        // Filter the noisy extreme values, by zeroing them out
        for (int i = 0; i < thisLaser.nranges; i++)
        {
            if (thisLaser.ranges[i] > LIDAR_MAX_RANGE || thisLaser.ranges[i] < 0)
            {
                thisLaser.ranges[i] = 0;
            }
        }

        // Run the obstacle detection
        maav::ObstacleDetector ot(thisLaser.rad0, thisLaser.radstep);

        // maav::ObstacleDetector::obstacle_data testStruct;
        vector<maav::ObstacleDetector::obstacle_data> ob_data;
        ob_data = ot.detect(thisLaser.ranges);

        // make sure that the rotations are correct

        for (size_t i = 0; i < ob_data.size(); i++)
        {
            // cout << ob_data.size() << "\t" << ob_data[i].midpoint << "\t"	<< setprecision(8);
            cout << ob_data[i].width << endl;
            // ob_data[i].midpoint.x() *= -1;
            ob_data[i].midpoint.y() *= -1;
        }

        myViewer.view(thisLaser, ob_data);  // TODO: comment this out in final

        double x = thisState.state[0];
        double y = thisState.state[1];

        // these values not needed for trivial calculation
        // double state[10]; // [x, y, z, q0, q1, q2, q3, xdot, ydot, zdot]
        // double z = thisState.state[2];
        // double q0 = thisState.state[3];
        // double q1 = thisState.state[4];
        // double q2 = thisState.state[5];
        // double q3 = thisState.state[6];
        // double xdot = thisState.state[7];
        // double ydot = thisState.state[8];
        // double zdot = thisState.state[9];

        obstacle_list_t obstacleList;
        obstacleList.num_obstacles = ob_data.size();
        obstacleList.obstacles.resize(ob_data.size());

        // Iterate over through our detected obstacles and add them to the
        // obstacleList message
        for (size_t i = 0; i < ob_data.size(); i++)
        {
            obstacleList.obstacles[i].x = x + ob_data[i].midpoint.x();
            obstacleList.obstacles[i].y = y + ob_data[i].midpoint.y();
        }

        // obstacle_t outputObstacle;
        // TODO: publish this message

        zcmptr->publish(maav::OBSTS_CHANNEL, &obstacleList);
    }

   private:
    shared_ptr<zcm::ZCM> zcmptr;
    shared_ptr<ZCMSingleHandler<state_t>> state_ptr;

    laser_t thisLaser;

    LidarViewer myViewer;
};

int main(int argc, char** argv)
{
    shared_ptr<zcm::ZCM> zcm = make_shared<zcm::ZCM>("ipc");

    if (!zcm->good()) return 1;

    state_t emptystate;

    // Populate emptyState with a real timestamp
    emptystate.utime = static_cast<int64_t>(
        chrono::time_point_cast<chrono::microseconds>(chrono::system_clock::now())
            .time_since_epoch()
            .count());

    // Construct a ZCMSingleHandler with an empty state
    shared_ptr<ZCMSingleHandler<state_t>> zs = make_shared<ZCMSingleHandler<state_t>>(emptystate);

    LidarHandler lidarHandler{zcm, zs};

    // Set up subscriptions to receive the vehicle's localized state as well
    // as LIDAR data
    zcm->subscribe(maav::STATE_CHANNEL, &ZCMSingleHandler<state_t>::recv, &*zs);
    zcm->subscribe(maav::LASER, &LidarHandler::recv, &lidarHandler);

    zcm->run();

    return 0;
}
