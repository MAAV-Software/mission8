#include <functional>
#include <random>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <ignition/math.hh>

#include <common/messages/MsgChannels.hpp>
#include <common/messages/lidar_t.hpp>
#include <zcm/zcm-cpp.hpp>

using zcm::ZCM;
namespace gazebo
{
class MaavLidarPlugin : public SensorPlugin
{
public:
    MaavLidarPlugin()
        : zcm("ipc"),
          rd_{},
          gen_{rd_()},
          bias_generator_{0.09, 0.01},
          noise_generator_{0, 0.025},
          outlier_generator_{9.0, 1.0},
          outlier_trigger_generator_{0, 0},
          outlier_count_generator_{0, 10},
          outlier_mode_(false),
          last_distance_(0.0),
          outlier_count_(0)
    {
    }

    void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
    {
        parentSensor = std::dynamic_pointer_cast<sensors::RaySensor>(_sensor);
        noise_density_ = _sdf->Get<double>("noise_density");
        bias_mean_ = _sdf->Get<double>("bias_mean");
        bias_std_ = _sdf->Get<double>("bias_std");
        outlier_mean_ = _sdf->Get<double>("outlier_mean");
        outlier_std_ = _sdf->Get<double>("outlier_std");
        outlier_freq_ = _sdf->Get<int>("outlier_freq");
        outlier_chi_ = _sdf->Get<int>("outlier_chi");

        bias_generator_ = std::normal_distribution<double>{bias_mean_, bias_std_};

        // zero mean white gaussian noise
        double noise_mean = 0;
        noise_generator_ = std::normal_distribution<double>(noise_mean, noise_density_);

        outlier_generator_ = std::normal_distribution<double>(outlier_mean_, outlier_std_);
        outlier_trigger_generator_ = std::uniform_int_distribution<>(0, outlier_freq_);

        bias_ = bias_generator_(gen_);

        if (!parentSensor)
        {
            gzthrow("Not a ray sensor");
        }

        this->updateConnection =
            this->parentSensor->ConnectUpdated(std::bind(&MaavLidarPlugin::OnUpdate, this));

        std::cout << "[Lidar Plugin] loaded!" << std::endl;
    }

    void add_noise()
    {
        if (outlier_trigger_generator_(gen_) == 0 && !outlier_mode_ && msg.distance.data[0] > 0.3)
        {
            outlier_mode_ = true;
            outlier_max_count_ = outlier_count_generator_(gen_);
        }

        if (outlier_mode_ && outlier_count_ < outlier_max_count_)
        {
            outlier_count_++;
            double outlier = outlier_generator_(gen_);
            double dist = std::abs(outlier - outlier_mean_) / outlier_std_;
            if (dist < outlier_chi_)
                msg.distance.data[0] = outlier;
            else
                msg.distance.data[0] = 0;
        }
        else
        {
            outlier_mode_ = false;
            outlier_count_ = 0;

            double noise = noise_generator_(gen_);
            msg.distance.data[0] = msg.distance.data[0] + bias_ + noise;
        }

        // Round to cm precision
        msg.distance.data[0] = std::round(msg.distance.data[0] * 100.0) / 100.0;
        last_distance_ = msg.distance.data[0];
    }

    void OnUpdate()
    {
        double distance = parentSensor->Range(0);
        auto time = parentSensor->LastMeasurementTime();
        uint64_t usec = time.sec * 1000000;
        usec += time.nsec / 1000;

        msg.utime = usec;
        msg.distance.data[0] = distance;

        zcm.publish(maav::SIM_HEIGHT_LIDAR_CHANNEL, &msg);

        add_noise();

        zcm.publish(maav::HEIGHT_LIDAR_CHANNEL, &msg);
    }

private:
    gazebo::sensors::RaySensorPtr parentSensor;
    event::ConnectionPtr updateConnection;

    lidar_t msg;
    sdf::ElementPtr sdf;
    ZCM zcm;

    std::random_device rd_;
    std::mt19937 gen_;
    std::normal_distribution<double> bias_generator_;
    std::normal_distribution<double> noise_generator_;
    std::normal_distribution<double> outlier_generator_;
    std::uniform_int_distribution<> outlier_trigger_generator_;
    std::uniform_int_distribution<> outlier_count_generator_;
    bool outlier_mode_;
    double last_distance_;
    size_t outlier_count_;
    size_t outlier_max_count_;
    int outlier_freq_;
    int outlier_chi_;
    double outlier_mean_;
    double outlier_std_;

    double noise_density_;
    double bias_mean_;
    double bias_std_;
    double bias_;
};

GZ_REGISTER_SENSOR_PLUGIN(MaavLidarPlugin)
}