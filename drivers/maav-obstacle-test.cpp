#include <vector>
#include <iostream>
#include <condition_variable>
#include <mutex>

#include <common/messages/point_cloud_t.hpp>
#include <common/messages/point_t.hpp>
#include <common/messages/MsgChannels.hpp>

#include <vision/obstacle/NaiveObstacle.hpp>

#include <vision/core/utilities.hpp>

#include <zcm/zcm-cpp.hpp>

using std::cout;
using std::endl;
using std::condition_variable;
using std::unique_lock;
using std::mutex;

using pcl::PointCloud;
using pcl::PointXYZ;

using maav::vision::NaiveObstacle;

class Handler
{
public:
    Handler() = default;
    void handle(const zcm::ReceiveBuffer*, const std::string&,
        const point_cloud_t* message)
    {
        PointCloud<PointXYZ>::Ptr cloud = maav::vision::zcmTypeToPCLPointCloud(*message);
        if (!obstacle_detector_.detectObstacles(cloud).empty())
        {
            cout << "There is an obstacle directly ahead!" << endl;
        }
    }
private:
    NaiveObstacle obstacle_detector_;
};

int main(int argc, char** argv)
{
    Handler handler;
    zcm::ZCM zcm {"ipc"};
    zcm.subscribe(maav::FORWARD_CAMERA_POINT_CLOUD_CHANNEL,
        &Handler::handle, &handler);
    zcm.start();
    mutex mtx;
    unique_lock<mutex> lck(mtx);
    condition_variable cv;
    cv.wait(lck);
}