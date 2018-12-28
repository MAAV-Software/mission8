#include <atomic>
#include <csignal>
#include <iostream>

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <zcm/zcm-cpp.hpp>

#include <common/messages/MsgChannels.hpp>
#include <common/messages/global_update_t.hpp>
#include <common/messages/map_t.hpp>
#include <common/messages/rgbd_image_t.hpp>
#include <common/utils/GetOpt.hpp>
#include <common/utils/ZCMHandler.hpp>

#include <gnc/slam/System.h>
#include <gnc/localizer.hpp>

using maav::MAP_CHANNEL;
using maav::RGBD_FORWARD_CHANNEL;
using maav::GLOBAL_UPDATE_CHANNEL;
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
    gopt.addString('c', "config", "../config/gnc/slam-config.yaml", "Path to config.");
    gopt.addString('v', "vocab", "../config/gnc/ORBvoc.txt", "Path to vocab");

    if (!gopt.parse(argc, argv, 1) || gopt.getBool("help"))
    {
        std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
        gopt.printHelp();
        return 1;
    }

    zcm::ZCM zcm{"ipc"};
    zcm.start();

    ZCMHandler<rgbd_image_t> image_handler;
    zcm.subscribe(maav::RGBD_FORWARD_CHANNEL, &ZCMHandler<rgbd_image_t>::recv, &image_handler);

    SlamInitializer slam_init;
    slam_init.vocabulary_file = gopt.getString("vocab");
    slam_init.config_file = gopt.getString("config");
    slam_init.sensor = System::eSensor::RGBD;
    slam_init.use_viewer = true;

    Localizer localizer(slam_init);
    cv::Mat pose;

    ios::sync_with_stdio(false);
    std::cout << std::showpos << std::setprecision(4);

    global_update_t msg;

    while (!KILL)
    {
        if (image_handler.ready())
        {
            rgbd_image_t img = image_handler.msg();
            image_handler.pop();

            cv::Mat rgb_image = convertRgb(img.rgb_image);
            cv::Mat depth_image = convertDepth(img.depth_image);
            localizer.addImage(rgb_image, depth_image, img.utime);
            pose = localizer.getPose();

            if (!pose.empty())
            {
                Eigen::Matrix3d attitude;
                Eigen::Vector3d position;
                for (size_t i = 0; i < 3; ++i)
                {
                    for (size_t j = 0; j < 3; ++j)
                    {
                        attitude(i, j) = static_cast<double>(pose.at<float>(i, j));
                    }
                    position(i) = static_cast<double>(pose.at<float>(i, 3));
                }

                const Eigen::Quaterniond qatt(attitude);
                msg.attitude[0] = qatt.w();
                msg.attitude[1] = -qatt.z();
                msg.attitude[2] = -qatt.x();
                msg.attitude[2] = -qatt.y();

                msg.position[0] = -position.z();
                msg.position[1] = -position.x();
                msg.position[2] = -position.y();

                msg.utime = img.utime;

                zcm.publish(GLOBAL_UPDATE_CHANNEL, &msg);
            }
        }

        std::this_thread::sleep_for(2ms);
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
    return cv::Mat(cv::Size(640, 480), CV_16UC1, (uint16_t*)depth_image.raw_image.data(),
               cv::Mat::AUTO_STEP)
        .clone();
}
