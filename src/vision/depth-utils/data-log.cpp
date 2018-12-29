#include <sys/stat.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <librealsense/rs.hpp>
#include <librealsense2/rs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "vision/depth-utils/CameraInterfaceBase.hpp"
#include "vision/depth-utils/D400CameraInterface.hpp"
#include "vision/depth-utils/LegacyCameraInterface.hpp"

using std::atomic;
using std::condition_variable;
using std::mutex;
using std::shared_ptr;
using std::thread;
using std::vector;

using maav::vision::CameraInterfaceBase;
using maav::vision::D400CameraInterface;
using maav::vision::LegacyCameraInterface;

atomic<bool> terminate = false;
std::mutex mtx;
std::condition_variable cond_var;
std::string directory;

void sigHandler(int);
void camera_thread(CameraInterfaceBase *cam);
void createDirectories();
void createDirectories(std::string);

int main(int argc, char **argv)
{
    std::cout << "Usage: data-log <# rs1 cams> <# rs2 cams> <directory (optional)>\n";
    if (argc == 4)
    {
        directory = argv[3];
        if (directory.back() == '/') directory.pop_back();
        createDirectories(directory);
    }
    else
    {
        createDirectories();
        directory = "ImageData";
    }

    unsigned int old_cams = static_cast<unsigned>(std::stoi(argv[1]));
    unsigned int new_cams = static_cast<unsigned>(std::stoi(argv[2]));

    // Initialize cameras
    vector<CameraInterfaceBase *> cameras;
    cameras.reserve(old_cams + new_cams);

    if (old_cams > 0)
    {
        LegacyCameraInterface *cam1 = new LegacyCameraInterface(0);
        cameras.push_back(cam1);
        std::cout << "First rs1 camera made" << std::endl;
        // Retrieve rs::context
        shared_ptr<rs::context> ctx = cam1->getContext();
        // Continue creating cameras
        for (unsigned i = 1; i < old_cams; ++i)
        {
            LegacyCameraInterface *next_cam = new LegacyCameraInterface(static_cast<int>(i), ctx);
            cameras.push_back(next_cam);
        }
        std::cout << "rs1 Cameras created" << std::endl;
    }

    if (new_cams > 0)
    {
        rs2::context ctx;
        auto list = ctx.query_devices();
        std::cout << list.size() << " rs2 devices connected.\n";
        for (unsigned int i = 0; i < new_cams; ++i)
        {
            D400CameraInterface *next_cam = new D400CameraInterface(
                list[i].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER), 640, 480, 30);
            cameras.push_back(next_cam);
        }
        std::cout << "rs2 Cameras created" << std::endl;
    }

    // drop first 10 frames
    for (int i = 0; i < 10; ++i)
    {
        for (auto &cam : cameras)
        {
            cam->loadNext();
        }
    }

    // cameras now contains LegacyCameraInterface instances for each of the connected cameras
    // Initialize variables used for thread control
    vector<thread> threads;
    // Set up sighandlers
    signal(SIGINT, sigHandler);
    signal(SIGSEGV, sigHandler);
    signal(SIGABRT, sigHandler);
    std::cout << "Got to thread creation" << std::endl;
    // Create threads
    for (unsigned i = 0; i < old_cams + new_cams; ++i)
    {
        threads.emplace_back(camera_thread, cameras[i]);
    }
    // Wait until awoken by sig handler
    std::unique_lock<std::mutex> lck(mtx);
    while (terminate == false)
    {
        cond_var.wait(lck);
    }
    // Join the threads back to main
    for (unsigned i = 0; i < old_cams + new_cams; ++i)
    {
        threads[i].join();
    }

    for (CameraInterfaceBase *cam : cameras) delete cam;
    return 0;
}

void sigHandler(int)
{
    // Set terminate to true
    terminate = true;
    cond_var.notify_one();
    cond_var.notify_one();
}

void camera_thread(CameraInterfaceBase *cam)
{
    cv::Mat rgb;
    cv::Mat depth;
    std::string fname = directory;
    fname += "/Timestamps/time";
    fname += std::to_string(cam->getTag());
    fname += ".txt";
    std::ofstream timestamps(fname);
    for (int frame = 0; !(terminate); ++frame)
    {
        int64_t start_time = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch())
                                 .count();

        rgb = cam->getRGB();
        depth = cam->getDepth();

        std::string fname = directory;
        fname += "/RGB";
        fname += std::to_string(cam->getTag());
        fname += "/";
        fname += std::to_string(frame);
        fname += ".bin";
        std::ofstream outfileRGB(fname, std::ios::out | std::ios::binary);
        std::cout << rgb.elemSize() << " " << rgb.total() << "\n";
        outfileRGB.write((char *)rgb.data, (long int)(rgb.elemSize() * rgb.total()));
        outfileRGB.close();

        fname = directory;
        fname += "/Depth";
        fname += std::to_string(cam->getTag());
        fname += "/";
        fname += std::to_string(frame);
        fname += ".bin";
        std::ofstream outfileDepth(fname, std::ios::out | std::ios::binary);
        std::cout << depth.elemSize() << " " << depth.total() << "\n";
        outfileDepth.write((char *)depth.data, (long int)(depth.elemSize() * depth.total()));
        outfileDepth.close();

        timestamps << start_time << '\n';

        cam->loadNext();
    }
}

void createDirectories(std::string dir_name)
{
    bool result = true;
    std::string command = "rm -rf ";
    command += dir_name;
    result = result && system(command.c_str());
    command = "mkdir ";
    command += dir_name;
    result = result && system(command.c_str());
    command += "/Timestamps";
    result = result && system(command.c_str());

    for (int i = 0; i < 5; ++i)
    {
        std::string command = "mkdir ";
        command += dir_name;
        command += "/RGB";
        command += std::to_string(i);
        result = result && system(command.c_str());
        command = "mkdir ";
        command += dir_name;
        command += "/Depth";
        command += std::to_string(i);
        result = result && system(command.c_str());
    }
    try
    {
        if (!result)
        {
            throw std::runtime_error("a system call returned failure");
        }
    }
    catch (std::runtime_error err)
    {
        throw err;
    }
}

void createDirectories()
{
    struct stat stat_struct;
    bool result = true;
    stat("ImageData", &stat_struct);

    // if the ImageData directory exists, tar the existing data
    if (S_ISDIR(stat_struct.st_mode))
    {
        int i = 0;
        std::string filename = "ImageData0.tar.gz";
        std::ifstream file(filename);
        while (file)
        {
            ++i;
            filename = "ImageData";
            filename += std::to_string(i);
            filename += ".tar.gz";
            file = std::ifstream(filename);
        }
        std::string command = "tar -czf ";
        command += filename;
        command += " ImageData/*";
        std::cout << command << std::endl;
        result = result && system(command.c_str());
        result = result && system("rm -rf ImageData");
    }

    // create the new directories for storage
    result = result && system("mkdir ImageData");
    result = result && system("mkdir ImageData/Timestamps");

    for (int i = 0; i < 5; ++i)
    {
        std::string command = "mkdir ImageData/RGB";
        command += std::to_string(i);
        result = result && system(command.c_str());
        command = "mkdir ImageData/Depth";
        command += std::to_string(i);
        result = result && system(command.c_str());
    }
    try
    {
        if (!result)
        {
            throw std::runtime_error("a system call returned failure");
        }
    }
    catch (std::runtime_error err)
    {
        throw err;
    }
}
