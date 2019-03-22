#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/sensors/sensors.hh>
#include <ignition/math.hh>
#include <string>
#include <vector>

#include <common/messages/MsgChannels.hpp>
#include <common/messages/rgbd_image_t.hpp>
#include <zcm/zcm-cpp.hpp>

using std::vector;
using zcm::ZCM;

namespace gazebo
{
class MaavCameraPlugin : public SensorPlugin
{
public:
    MaavCameraPlugin() : zcm("ipc")
    {
        rgb_image.size = -1;
        depth_image.size = -1;
    }

    void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
    {
        camera_sensor = std::dynamic_pointer_cast<sensors::DepthCameraSensor>(_sensor);

        std::string name = _sensor->ScopedName();
        std::cout << "Camera scoped name: " << name << std::endl;
        if (name.find("Forward") != std::string::npos)
        {
            channel_name = maav::RGBD_FORWARD_CHANNEL;
        }
        else if (name.find("Downward") != std::string::npos)
        {
            channel_name = maav::RGBD_DOWNWARD_CHANNEL;
        }

        if (!camera_sensor)
        {
            gzthrow("Not a camera sensor");
        }

        camera = camera_sensor->DepthCamera();

        update_connection_depth =
            camera->ConnectNewDepthFrame(std::bind(&MaavCameraPlugin::DepthUpdate, this));
        update_connection_color =
            camera->ConnectNewImageFrame(std::bind(&MaavCameraPlugin::ColorUpdate, this));

        std::cout << "Camera plugin loaded!" << std::endl;
    }

    void DepthUpdate()
    {
        depth_image.height = static_cast<int32_t>(camera->ImageHeight());
        depth_image.width = static_cast<int32_t>(camera->ImageWidth());
        depth_image.size = depth_image.height * depth_image.width;
        if (depth_image.raw_image.size() != static_cast<size_t>(depth_image.size))
        {
            depth_image.raw_image.resize(depth_image.size);
        }

        // Convert Float depth data to RealSense depth data
        const float* depth_data_float = camera->DepthData();
        for (int32_t i = 0; i < depth_image.size; ++i)
        {
            float pixel_depth = depth_data_float[i];
            depth_image.raw_image[i] =
                (pixel_depth > 10) ? 0 : static_cast<int16_t>(pixel_depth * 5000);
        }

        depth_valid = true;
        if (rgbd_valid)
        {
            SendMessage();
        }
    }

    void ColorUpdate()
    {
        rgb_image.height = static_cast<int32_t>(camera->ImageHeight());
        rgb_image.width = static_cast<int32_t>(camera->ImageWidth());
        rgb_image.size = rgb_image.height * rgb_image.width * 3;
        rgb_image.raw_image =
            vector<int8_t>(camera->ImageData(), camera->ImageData() + rgb_image.size);

        rgbd_valid = true;
        if (depth_valid)
        {
            SendMessage();
        }
    }

    void SendMessage()
    {
        auto time = camera_sensor->LastUpdateTime();
        uint64_t usec = time.sec * 1000000;
        usec += time.nsec / 1000;
        msg.utime = usec;

        msg.rgb_image = rgb_image;
        msg.depth_image = depth_image;

        zcm.publish(channel_name, &msg);

        // Set image size back to null
        rgbd_valid = false;
        depth_valid = false;
    }

private:
    rendering::DepthCameraPtr camera;
    sensors::DepthCameraSensorPtr camera_sensor;
    event::ConnectionPtr update_connection_depth;
    event::ConnectionPtr update_connection_color;

    rgbd_image_t msg;
    rgb_image_t rgb_image;
    depth_image_t depth_image;

    bool rgbd_valid = false;
    bool depth_valid = false;

    sdf::ElementPtr sdf;
    ZCM zcm;
    std::string channel_name;
};

GZ_REGISTER_SENSOR_PLUGIN(MaavCameraPlugin)
}
