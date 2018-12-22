/*
 * rs2_cap.cpp
 * Cheng Jiang
 * chengjia@umich.edu
 * May 2018
 *
 * MAAV Camera Calibration
 * Captures 10 frames with librealsense2. Saves images in the build dir.
 * RGB image saved as png, depth saved as csv. Frame by frame preview is shown.
 *
 * This document is supplied as is without expressed or implied warranties
 * of any kind. Nice
 */

#include <chrono>
#include <fstream>
#include <iostream>
#include <csignal>
#include <atomic>
#include <algorithm>
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <zcm/zcm-cpp.hpp>

#include <common/messages/rgbd_image_t.hpp>
#include <common/messages/MsgChannels.hpp>

using namespace rs2;
using namespace std::chrono;

float get_depth_scale(rs2::device dev) {
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors()) {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>()) {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}

rs2_stream find_stream_to_align(
    const std::vector<rs2::stream_profile>& streams) {
    // Given a vector of streams, we try to find a depth stream and
    // another stream to align depth with.
    // We prioritize color streams to make the view look better.
    // If color is not available, we take another stream that (other than depth)
    rs2_stream align_to = RS2_STREAM_ANY;
    bool depth_stream_found = false;
    bool color_stream_found = false;
    for (rs2::stream_profile sp : streams) {
        rs2_stream profile_stream = sp.stream_type();
        if (profile_stream != RS2_STREAM_DEPTH) {
            if (!color_stream_found)  // Prefer color
                align_to = profile_stream;

            if (profile_stream == RS2_STREAM_COLOR) {
                color_stream_found = true;
            }
        } else {
            depth_stream_found = true;
        }
    }

    if (!depth_stream_found)
        throw std::runtime_error("No Depth stream available");

    if (align_to == RS2_STREAM_ANY)
        throw std::runtime_error("No stream found to align with Depth");

    return align_to;
}

bool profile_changed(const std::vector<rs2::stream_profile>& current,
                     const std::vector<rs2::stream_profile>& prev) {
    for (auto&& sp : prev) {
        // If previous profile is in current (maybe just added another)
        auto itr =
            std::find_if(std::begin(current), std::end(current),
                         [&sp](const rs2::stream_profile& current_sp) {
                             return sp.unique_id() == current_sp.unique_id();
                         });
        if (itr ==
            std::end(current))  // If it previous stream wasn't found in current
        {
            return true;
        }
    }
    return false;
}

std::atomic<bool> KILL{false};
void sigHandler(int) { KILL = true; }

int main(int, char**) {
    signal(SIGINT, sigHandler);
    signal(SIGABRT, sigHandler);
    signal(SIGSEGV, sigHandler);
    signal(SIGTERM, sigHandler);

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe_forward;
    rs2::pipeline pipe_downward;
    rs2::config cfg_forward;
    rs2::config cfg_downward;

    // Forward - 819112072448
    // Donward - 819112070694
    cfg_forward.enable_device("819112072448");
    cfg_downward.enable_device("819112070694");

    cfg_forward.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg_forward.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    cfg_downward.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg_downward.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    rs2::pipeline_profile profile_forward = pipe_forward.start(cfg_forward);
    rs2::pipeline_profile profile_downward = pipe_downward.start(cfg_downward);

    float depth_scale_forward = get_depth_scale(profile_forward.get_device());
    float depth_scale_downward = get_depth_scale(profile_downward.get_device());

    auto streams_forward = profile_forward.get_streams();
    auto streams_downward = profile_downward.get_streams();

    rs2_stream align_to_forward = find_stream_to_align(streams_forward);
    rs2_stream align_to_downward = find_stream_to_align(streams_downward);

    rs2::align align_forward(align_to_forward);
    rs2::align align_downward(align_to_downward);

    // Disabling auto exposure
    rs2::sensor sensor_1(profile_forward.get_device().query_sensors()[0]);
    rs2::sensor sensor_2(profile_forward.get_device().query_sensors()[1]);
    rs2::sensor sensor_3(profile_downward.get_device().query_sensors()[0]);
    rs2::sensor sensor_4(profile_downward.get_device().query_sensors()[1]);

    sensor_1.set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
    sensor_2.set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
    sensor_3.set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
    sensor_4.set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);

    std::cout << "Depth scale: " << depth_scale_forward << std::endl;
    std::cout << "Depth scale: " << depth_scale_downward << std::endl;

    zcm::ZCM zcm{"ipc"};
    rgbd_image_t rgbd_image_forward;
    rgbd_image_forward.rgb_image.width = 640;
    rgbd_image_forward.rgb_image.height = 480;
    rgbd_image_forward.rgb_image.size = 640 * 480 * 3;
    rgbd_image_forward.rgb_image.raw_image.resize(rgbd_image_forward.rgb_image.size);
    rgbd_image_forward.depth_image.width = 640;
    rgbd_image_forward.depth_image.height = 480;
    rgbd_image_forward.depth_image.size = 640*480;
    rgbd_image_forward.depth_image.raw_image.resize(rgbd_image_forward.depth_image.size);

    rgbd_image_t rgbd_image_downward;
    rgbd_image_downward.rgb_image.width = 640;
    rgbd_image_downward.rgb_image.height = 480;
    rgbd_image_downward.rgb_image.size = 640 * 480 * 3;
    rgbd_image_downward.rgb_image.raw_image.resize(rgbd_image_downward.rgb_image.size);
    rgbd_image_downward.depth_image.width = 640;
    rgbd_image_downward.depth_image.height = 480;
    rgbd_image_downward.depth_image.size = 640*480;
    rgbd_image_downward.depth_image.raw_image.resize(rgbd_image_downward.depth_image.size);


    // Capture 10 frames to give autoexposure, etc. a chance to settle
    for (auto i = 0; i < 10; ++i){ 
        pipe_forward.wait_for_frames();
        pipe_downward.wait_for_frames();
    }
    
    // int count = 0;
    while (!KILL) {
        // count++; 
        rs2::frameset frames_forward = pipe_forward.wait_for_frames();
        if (profile_changed(pipe_forward.get_active_profile().get_streams(),
                            profile_forward.get_streams())) {
            // If the profile was changed, update the align object,
            // and also get the new device's depth scale
            profile_forward = pipe_forward.get_active_profile();
            align_to_forward = find_stream_to_align(profile_forward.get_streams());
            align_forward = rs2::align(align_to_forward);
            depth_scale_forward = get_depth_scale(profile_forward.get_device());
        }

        rs2::frameset frames_downward = pipe_downward.wait_for_frames();
        if (profile_changed(pipe_downward.get_active_profile().get_streams(),
                            profile_downward.get_streams())) {
            // If the profile was changed, update the align object,
            // and also get the new device's depth scale
            profile_downward = pipe_downward.get_active_profile();
            align_to_downward = find_stream_to_align(profile_downward.get_streams());
            align_downward = rs2::align(align_to_downward);
            depth_scale_downward = get_depth_scale(profile_downward.get_device());
        }

        auto processed_forward = align_forward.process(frames_forward);
        auto processed_downward = align_downward.process(frames_downward);

        // Trying to get both color and aligned depth frames
        rs2::video_frame other_frame_forward = processed_forward.first_or_default(align_to_forward);
        rs2::depth_frame aligned_depth_frame_forward = processed_forward.get_depth_frame();
        rs2::video_frame other_frame_downward = processed_downward.first_or_default(align_to_downward);
        rs2::depth_frame aligned_depth_frame_downward = processed_downward.get_depth_frame();

        unsigned long tframe;

        // If one of them is unavailable, continue iteration, otherwise process
        if (aligned_depth_frame_forward && other_frame_forward) {
            cv::Mat color_forward(cv::Size(640, 480), CV_8UC3,
                        (void*)other_frame_forward.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat depth_forward(cv::Size(640, 480), CV_16SC1,
                        (void*)aligned_depth_frame_forward.get_data(),
                        cv::Mat::AUTO_STEP);

            tframe = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();

            rgbd_image_forward.utime = tframe;
            memcpy(rgbd_image_forward.rgb_image.raw_image.data(), other_frame_forward.get_data(), rgbd_image_forward.rgb_image.size * sizeof(int8_t));
            memcpy(rgbd_image_forward.depth_image.raw_image.data(), aligned_depth_frame_forward.get_data(), rgbd_image_forward.depth_image.size * sizeof(int16_t));
            zcm.publish(maav::RGBD_FORWARD_CHANNEL, &rgbd_image_forward);
        } 


        if (aligned_depth_frame_downward && other_frame_downward) {
            cv::Mat color_downward(cv::Size(640, 480), CV_8UC3,
                        (void*)other_frame_downward.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat depth_downward(cv::Size(640, 480), CV_16SC1,
                        (void*)aligned_depth_frame_downward.get_data(),
                        cv::Mat::AUTO_STEP);

            tframe = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();

            rgbd_image_downward.utime = tframe;
            memcpy(rgbd_image_downward.rgb_image.raw_image.data(), other_frame_downward.get_data(), rgbd_image_downward.rgb_image.size * sizeof(int8_t));
            memcpy(rgbd_image_downward.depth_image.raw_image.data(), aligned_depth_frame_downward.get_data(), rgbd_image_downward.depth_image.size * sizeof(int16_t));
            zcm.publish(maav::RGBD_DOWNWARD_CHANNEL, &rgbd_image_downward);
        } 

    }

    return EXIT_SUCCESS;
}
