#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/plugins/DepthCameraPlugin.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/sensors/sensors.hh>
#include <ignition/math.hh>

#include <functional>
#include <string>
#include <vector>
#include <Eigen/Core>

#include <common/messages/MsgChannels.hpp>
#include <common/messages/point_cloud_t.hpp>
#include <common/messages/rgbd_image_t.hpp>
#include <zcm/zcm-cpp.hpp>

using std::vector;
using zcm::ZCM;
using Eigen::Vector3d;
using Eigen::Matrix3d;

namespace gazebo
{
class MaavCameraPlugin : public DepthCameraPlugin
{
public:
    MaavCameraPlugin() : zcm("ipc")
    {
        rgb_image.size = -1;
        depth_image.size = -1;
    }

    void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
    {
        DepthCameraPlugin::Load(sensor, sdf);

        sensor_ = sensor;

        std::string name = sensor->ScopedName();
        std::cout << "Camera scoped name: " << name << std::endl;
        if (name.find("Forward") != std::string::npos)
        {
            image_channel_name_ = maav::RGBD_FORWARD_CHANNEL;
            pointcloud_channel_name_ = maav::FORWARD_CAMERA_POINT_CLOUD_CHANNEL;
        }
        else if (name.find("Downward") != std::string::npos)
        {
            image_channel_name_ = maav::RGBD_DOWNWARD_CHANNEL;
            pointcloud_channel_name_ = maav::DOWNWARD_CAMERA_POINT_CLOUD_CHANNEL;
        }

        depth_image.width = 640;
        depth_image.height = 480;
        depth_image.size = 640 * 480;
        depth_image.raw_image = std::vector<int16_t>(depth_image.size, 0);

        std::cout << "[Camera Plugin] loaded!" << std::endl;
    }

    void OnNewDepthFrame(const float* _image, unsigned int _width, unsigned int _height,
        unsigned int /*_depth*/, const std::string& /*_format*/) override
    {
        std::cout << "New depth" << std::endl;
        // depth_image.height = static_cast<int32_t>(camera->ImageHeight());
        // depth_image.width = static_cast<int32_t>(camera->ImageWidth());
        // depth_image.size = depth_image.height * depth_image.width;
        // if (depth_image.raw_image.size() != static_cast<size_t>(depth_image.size))
        // {
        //     depth_image.raw_image.resize(depth_image.size);
        // }

        // // Convert Float depth data to RealSense depth data
        // const float* depth_data_float = camera->DepthData();
        // for (int32_t i = 0; i < depth_image.size; ++i)
        // {
        //     float pixel_depth = depth_data_float[i];
        //     depth_image.raw_image[i] =
        //         (pixel_depth > 10) ? 0 : static_cast<int16_t>(pixel_depth * 5000);
        // }

        // depth_valid = true;
        // if (rgbd_valid)
        // {
        //     SendMessage();
        // }
    }

    void OnNewImageFrame(const unsigned char* _image, unsigned int _width, unsigned int _height,
        unsigned int _depth, const std::string& _format) override
    {
        rgb_image.height = _height;
        rgb_image.width = _width;
        rgb_image.size = _width * _height;
        rgb_image.raw_image = vector<int8_t>(_image, _image + rgb_image.size);

        SendMessage();
    }

    // The point cloud is in the image frame: Z into page, x to the right and y down
    void OnNewRGBPointCloud(const float* _pcd, unsigned int _width, unsigned int _height,
        unsigned int _depth, const std::string& _format) override
    {
        point_cloud_t cloud;
        cloud.size = _width * _height;
        cloud.point_cloud.reserve(cloud.size);

        for (size_t i = 0; i < _width; i++)
        {
            for (size_t j = 0; j < _height; j++)
            {
                size_t index = (j * _width) + i;
                point_t point;
                point.x = _pcd[4 * index];
                point.y = _pcd[4 * index + 1];
                point.z = _pcd[4 * index + 2];

                cloud.point_cloud.push_back(point);
            }
        }

        auto time = sensor_->LastMeasurementTime();
        uint64_t usec = time.sec * 1000000;
        usec += time.nsec / 1000;

        cloud.utime = usec;

        zcm.publish(pointcloud_channel_name_, &cloud);
    }

    void SendMessage()
    {
        auto time = sensor_->LastMeasurementTime();
        uint64_t usec = time.sec * 1000000;
        usec += time.nsec / 1000;

        // // Convert Gazebo time format to microseconds
        // auto time =sensor_->world->SimTime();
        // uint64_t usec = time.sec * 1000000;
        // usec += time.nsec / 1000;

        msg.utime = usec;

        msg.rgb_image = rgb_image;
        msg.depth_image = depth_image;

        zcm.publish(image_channel_name_, &msg);
    }

private:
    rgbd_image_t msg;
    rgb_image_t rgb_image;
    depth_image_t depth_image;

    sensors::SensorPtr sensor_;

    ZCM zcm;
    std::string image_channel_name_;
    std::string pointcloud_channel_name_;

    rendering::DynamicLines* lines;
};

GZ_REGISTER_SENSOR_PLUGIN(MaavCameraPlugin)
}  // namespace gazebo
