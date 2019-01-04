#include <atomic>
#include <csignal>
#include <iostream>

#include <yaml-cpp/node/detail/bool_type.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <zcm/zcm-cpp.hpp>

#include <common/messages/MsgChannels.hpp>
#include <common/messages/global_update_t.hpp>
#include <common/messages/map_t.hpp>
#include <common/messages/rgbd_image_t.hpp>
#include <common/messages/slam_localization_mode_t.hpp>
#include <common/messages/slam_reset_t.hpp>
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
    gopt.addString('b', "vocab", "../config/gnc/ORBvoc.txt", "Path to vocab");
    gopt.addBool('v', "verbose", false, "Print extra info");

    if (!gopt.parse(argc, argv, 1) || gopt.getBool("help"))
    {
        std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
        gopt.printHelp();
        return 1;
    }

    zcm::ZCM zcm{"ipc"};
    zcm::ZCM zcm_udp{"udpm://239.255.76.67:7667?ttl=1"};
    if (!zcm.good())
    {
        throw "Bad ZCM";
    }
    if (!zcm_udp.good())
    {
        throw "Bad ZCM UDP";
    }

    bool verbose = gopt.getBool("verbose");
    if (verbose)
    {
        ios::sync_with_stdio(false);
        std::cout << std::showpos << std::setprecision(4) << fixed;
    }

    ZCMHandler<rgbd_image_t> image_handler;
    zcm.subscribe(maav::RGBD_FORWARD_CHANNEL, &ZCMHandler<rgbd_image_t>::recv, &image_handler);

    ZCMHandler<slam_localization_mode_t> loc_mode_handler;
    ZCMHandler<slam_reset_t> reset_handler;
    ZCMHandler<slam_localization_mode_t> loc_mode_handler_udp;
    ZCMHandler<slam_reset_t> reset_handler_udp;
    zcm.subscribe(maav::SLAM_LOCALIZATION_MODE_CHANNEL, &ZCMHandler<slam_localization_mode_t>::recv,
        &loc_mode_handler);
    zcm.subscribe(maav::SLAM_RESET_CHANNEL, &ZCMHandler<slam_reset_t>::recv, &reset_handler);
    zcm_udp.subscribe(maav::SLAM_LOCALIZATION_MODE_CHANNEL,
        &ZCMHandler<slam_localization_mode_t>::recv, &loc_mode_handler_udp);
    zcm_udp.subscribe(
        maav::SLAM_RESET_CHANNEL, &ZCMHandler<slam_reset_t>::recv, &reset_handler_udp);

    YAML::Node config = YAML::LoadFile(gopt.getString("config"));

    SlamInitializer slam_init;
    slam_init.vocabulary_file = gopt.getString("vocab");
    slam_init.config_file = gopt.getString("config");
    slam_init.sensor = System::eSensor::RGBD;
    slam_init.zcm_url = config["zcm_url"].as<std::string>();
    slam_init.send_images = config["send_images"].as<bool>();

    Localizer localizer(slam_init);
    cv::Mat pose;

    ios::sync_with_stdio(false);
    std::cout << std::showpos << std::setprecision(4);

    global_update_t msg;

    zcm.start();

    while (!KILL)
    {
        if (loc_mode_handler.ready())
        {
            auto msg = loc_mode_handler.msg();
            loc_mode_handler.pop();
            if (msg.mode)
            {
                localizer.slam.ActivateLocalizationMode();
            }
            else
            {
                localizer.slam.DeactivateLocalizationMode();
            }
        }
        if (reset_handler.ready())
        {
            auto msg = reset_handler.msg();
            reset_handler.pop();

            if (msg.reset)
            {
                localizer.slam.Reset();
            }
        }
        if (loc_mode_handler_udp.ready())
        {
            auto msg = loc_mode_handler_udp.msg();
            loc_mode_handler_udp.pop();
            if (msg.mode)
            {
                localizer.slam.ActivateLocalizationMode();
            }
            else
            {
                localizer.slam.DeactivateLocalizationMode();
            }
        }
        if (reset_handler_udp.ready())
        {
            auto msg = reset_handler_udp.msg();
            reset_handler_udp.pop();

            if (msg.reset)
            {
                localizer.slam.Reset();
            }
        }
        if (image_handler.ready())
        {
            rgbd_image_t img = image_handler.msg();
            image_handler.pop();

            cv::Mat rgb_image = convertRgb(img.rgb_image);
            cv::Mat depth_image = convertDepth(img.depth_image);
            cv::rotate(rgb_image, rgb_image, cv::ROTATE_180);
            cv::rotate(depth_image, depth_image, cv::ROTATE_180);
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
                msg.attitude[3] = -qatt.y();

                msg.position[0] = -position.z();
                msg.position[1] = -position.x();
                msg.position[2] = -position.y();

                msg.utime = img.utime;

                if (verbose)
                {
                    std::cout << "Attitude Corr: " << msg.attitude[0] << ' ' << msg.attitude[1]
                              << ' ' << msg.attitude[2] << ' ' << msg.attitude[3] << ' ' << '\n';
                    std::cout << "Attitude UCor: " << qatt.w() << ' ' << qatt.x() << ' ' << qatt.y()
                              << ' ' << qatt.z() << ' ' << '\n';
                    std::cout << "Position: " << msg.position[0] << ' ' << msg.position[1] << ' '
                              << msg.position[2] << ' ' << std::endl;
                }

                zcm.publish(GLOBAL_UPDATE_CHANNEL, &msg);
                zcm_udp.publish(GLOBAL_UPDATE_CHANNEL, &msg);
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
