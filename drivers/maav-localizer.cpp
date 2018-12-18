#include <atomic>
#include <csignal>
#include <iostream>

#include <yaml-cpp/yaml.h>
#include <zcm/zcm-cpp.hpp>

#include <common/messages/MsgChannels.hpp>
#include <common/messages/map_t.hpp>
#include <common/messages/rgbd_image_t.hpp>
#include <common/utils/GetOpt.hpp>
#include <common/utils/ZCMHandler.hpp>

#include <gnc/slam/System.h>
#include <gnc/localizer.hpp>

using maav::MAP_CHANNEL;
using maav::RGBD_CHANNEL;
using maav::gnc::Localizer;
using maav::gnc::SlamInitializer;
using maav::gnc::slam::System;
using std::cout;
using std::chrono::duration_cast;
using std::chrono::system_clock;

cv::Mat convertRgb(const rgb_image_t&);
cv::Mat convertDepth(const depth_image_t&);

std::atomic<bool> KILL{false};
void sigHandler(int) { KILL = true; }
int main(int argc, char** argv)
{
    cout << "SLAM driver\n";

    signal(SIGINT, sigHandler);
    signal(SIGABRT, sigHandler);
    signal(SIGSEGV, sigHandler);
    signal(SIGTERM, sigHandler);

    GetOpt gopt;
    gopt.addBool('h', "help", false, "This message");
    gopt.addString('c', "config", "", "Path to config.");
    gopt.addString('v', "vocab", "", "Path to vocab");

    if (!gopt.parse(argc, argv, 1) || gopt.getBool("help"))
    {
        std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
        gopt.printHelp();
        return 1;
    }

    zcm::ZCM zcm{"ipc"};
    zcm.start();

    ZCMHandler<rgbd_image_t> image_handler;
    zcm.subscribe(RGBD_CHANNEL, &ZCMHandler<rgbd_image_t>::recv, &image_handler);

    SlamInitializer slam_init;
    slam_init.vocabulary_file = gopt.getString("vocab");
    slam_init.config_file = gopt.getString("config");
    slam_init.sensor = System::eSensor::RGBD;
    slam_init.use_viewer = true;

    Localizer localizer(slam_init);

    while (!KILL)
    {
        // This might not work, this is currently just a
        // framework to work off of
        if (image_handler.ready())
        {
            rgbd_image_t img = image_handler.msg();
            image_handler.pop();
            localizer.addImage(convertRgb(img.rgb_image), convertDepth(img.depth_image), img.utime);
            // map_t map = localizer.getMap();
            // zcm.publish(MAP_CHANNEL, &map);
        }

        std::this_thread::sleep_for(1ms);
    }

    zcm.stop();
}

/*
 * Converts RGB image message into cv::Mat for localizer
 */
cv::Mat convertRgb(const rgb_image_t& rgb_image)
{
    return cv::Mat(cv::Size(640, 480), CV_8UC3, (uint8_t*)rgb_image.raw_image.data(),
               cv::Mat::AUTO_STEP)
        .clone();
}

/*
 * Converts depth image message into cv::Mat for localizer
 */
cv::Mat convertDepth(const depth_image_t& depth_image)
{
    return cv::Mat(cv::Size(640, 480), CV_16SC1, (int16_t*)depth_image.raw_image.data(),
               cv::Mat::AUTO_STEP)
        .clone();
}
