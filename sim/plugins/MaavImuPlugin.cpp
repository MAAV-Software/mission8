#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <ignition/math.hh>
#include <random>

#include <Eigen/Dense>

#include <common/messages/MsgChannels.hpp>
#include <common/messages/imu_t.hpp>
#include <zcm/zcm-cpp.hpp>

using zcm::ZCM;

namespace gazebo
{
class MaavImuPlugin : public ModelPlugin
{
public:
    MaavImuPlugin()
        : last_time_(0),
          zcm("ipc"),
          gyro_(Eigen::Vector3d::Zero()),
          accel_(Eigen::Vector3d::Zero()),
          mag_(Eigen::Vector3d::Zero()),
          rd{},
          gen{rd()}
    {
    }

    void Load(physics::ModelPtr model, sdf::ElementPtr _sdf)
    {
        world_ = model->GetWorld();
        model_ = model;
        link_ = model_->GetLinks()[0];

        auto gyro = _sdf->GetElement("gyro");
        auto accel = _sdf->GetElement("accel");
        auto mag = _sdf->GetElement("mag");
        gyro_noise_density_ = gyro->Get<double>("noise_density");
        gyro_bias_stability_ = gyro->Get<double>("bias_stability");
        gyro_initial_bias_error_ = gyro->Get<double>("initial_bias_error");
        accel_noise_density_ = gyro->Get<double>("noise_density");
        accel_bias_stability_ = gyro->Get<double>("bias_stability");
        accel_initial_bias_error_ = gyro->Get<double>("initial_bias_error");
        mag_noise_density_ = gyro->Get<double>("noise_density");
        mag_bias_stability_ = gyro->Get<double>("bias_stability");
        mag_initial_bias_error_ = gyro->Get<double>("initial_bias_error");

        gyro_noise_generator = std::normal_distribution<double>(0, gyro_noise_density_);
        accel_noise_generator = std::normal_distribution<double>(0, accel_noise_density_);
        mag_noise_generator = std::normal_distribution<double>(0, mag_noise_density_);

        gyro_bias_generator = std::normal_distribution<double>(0, gyro_bias_stability_);
        accel_bias_generator = std::normal_distribution<double>(0, accel_bias_stability_);
        mag_bias_generator = std::normal_distribution<double>(0, mag_bias_stability_);

        gyro_bias_initial_generator = std::normal_distribution<double>(0, gyro_initial_bias_error_);
        accel_bias_initial_generator =
            std::normal_distribution<double>(0, accel_initial_bias_error_);
        mag_bias_initial_generator = std::normal_distribution<double>(0, mag_initial_bias_error_);

        gyro_bias_ = Eigen::Vector3d{gyro_bias_initial_generator(gen),
            gyro_bias_initial_generator(gen), gyro_bias_initial_generator(gen)};
        accel_bias_ = Eigen::Vector3d{accel_bias_initial_generator(gen),
            accel_bias_initial_generator(gen), accel_bias_initial_generator(gen)};
        mag_bias_ = Eigen::Vector3d{mag_bias_initial_generator(gen),
            mag_bias_initial_generator(gen), mag_bias_initial_generator(gen)};

        updateConnection =
            event::Events::ConnectWorldUpdateBegin(std::bind(&MaavImuPlugin::OnUpdate, this));

        std::cout << "IMU plugin loaded!" << std::endl;
    }

    void add_noise()
    {
        gyro_bias_ += Eigen::Vector3d{
            gyro_bias_generator(gen), gyro_bias_generator(gen), gyro_bias_generator(gen)};
        accel_bias_ += Eigen::Vector3d{
            gyro_bias_generator(gen), gyro_bias_generator(gen), gyro_bias_generator(gen)};

        gyro_noise_ = Eigen::Vector3d{
            gyro_bias_generator(gen), gyro_bias_generator(gen), gyro_bias_generator(gen)};
        accel_noise_ = Eigen::Vector3d{
            accel_bias_generator(gen), accel_bias_generator(gen), accel_bias_generator(gen)};
        mag_noise_ = Eigen::Vector3d{
            mag_bias_generator(gen), mag_bias_generator(gen), mag_bias_generator(gen)};

        gyro_ += gyro_bias_ + gyro_noise_;
        accel_ += accel_bias_ + accel_noise_;
        mag_ += mag_bias_ + mag_noise_;
    }

    void OnUpdate()
    {
        auto time = world_->SimTime();
        uint64_t usec = time.sec * 1000000;
        usec += time.nsec / 1000;
        uint64_t dt = usec - last_time_;

        auto pose = link_->WorldPose();
        auto gravity = world_->Gravity();
        // World frame readings
        auto accel = link_->WorldLinearAccel() + gravity;
        auto angular_rate = link_->WorldAngularVel();
        auto mag = world_->MagneticField();

        // World to imu rotation
        auto R_w_i = pose.Rot();

        // Converted to local frame readings
        accel = R_w_i.RotateVector(accel);
        angular_rate = R_w_i.RotateVector(angular_rate);
        mag = R_w_i.RotateVector(mag);

        gyro_ += Eigen::Vector3d{angular_rate.X(), angular_rate.Y(), angular_rate.Z()};
        accel_ += Eigen::Vector3d{accel.X(), accel.Y(), accel.Z()};
        mag_ += Eigen::Vector3d{mag.X(), mag.Y(), mag.Z()};
        count++;

        if (dt < 1000000 / 100) return;

        msg.utime = usec;
        last_time_ = usec;

        gyro_ *= (1.0 / static_cast<double>(count));
        accel_ *= (1.0 / static_cast<double>(count));
        mag_ *= (1.0 / static_cast<double>(count));

        msg.angular_rates.data[0] = gyro_.x();
        msg.angular_rates.data[1] = gyro_.y();
        msg.angular_rates.data[2] = gyro_.z();

        msg.acceleration.data[0] = accel_.x();
        msg.acceleration.data[1] = accel_.y();
        msg.acceleration.data[2] = accel_.z();

        msg.magnetometer.data[0] = mag_.x();
        msg.magnetometer.data[1] = mag_.y();
        msg.magnetometer.data[2] = mag_.z();

        zcm.publish(maav::SIM_IMU_CHANNEL, &msg);

        // add_noise();

        msg.angular_rates.data[0] = gyro_.x();
        msg.angular_rates.data[1] = gyro_.y();
        msg.angular_rates.data[2] = gyro_.z();

        msg.acceleration.data[0] = accel_.x();
        msg.acceleration.data[1] = accel_.y();
        msg.acceleration.data[2] = accel_.z();

        msg.magnetometer.data[0] = mag_.x();
        msg.magnetometer.data[1] = mag_.y();
        msg.magnetometer.data[2] = mag_.z();

        zcm.publish(maav::IMU_CHANNEL, &msg);

        gyro_ = Eigen::Vector3d::Zero();
        accel_ = Eigen::Vector3d::Zero();
        mag_ = Eigen::Vector3d::Zero();
        count = 0;
    }

private:
    gazebo::physics::WorldPtr world_;
    physics::ModelPtr model_;
    physics::LinkPtr link_;
    gazebo::sensors::ImuSensorPtr imu_sensor_;
    gazebo::sensors::MagnetometerSensorPtr mag_sensor_;
    event::ConnectionPtr updateConnection;

    uint64_t last_time_;

    imu_t msg;
    sdf::ElementPtr sdf;
    ZCM zcm;

    double gyro_noise_density_;
    double gyro_bias_stability_;
    double gyro_initial_bias_error_;

    double accel_noise_density_;
    double accel_bias_stability_;
    double accel_initial_bias_error_;

    double mag_noise_density_;
    double mag_bias_stability_;
    double mag_initial_bias_error_;

    Eigen::Vector3d gyro_;
    Eigen::Vector3d accel_;
    Eigen::Vector3d mag_;

    Eigen::Vector3d gyro_bias_;
    Eigen::Vector3d accel_bias_;
    Eigen::Vector3d mag_bias_;

    Eigen::Vector3d gyro_noise_;
    Eigen::Vector3d accel_noise_;
    Eigen::Vector3d mag_noise_;

    std::random_device rd;
    std::mt19937 gen;

    std::normal_distribution<double> gyro_bias_generator;
    std::normal_distribution<double> gyro_noise_generator;
    std::normal_distribution<double> gyro_bias_initial_generator;

    std::normal_distribution<double> accel_bias_generator;
    std::normal_distribution<double> accel_noise_generator;
    std::normal_distribution<double> accel_bias_initial_generator;

    std::normal_distribution<double> mag_bias_generator;
    std::normal_distribution<double> mag_noise_generator;
    std::normal_distribution<double> mag_bias_initial_generator;

    int count;
};

GZ_REGISTER_MODEL_PLUGIN(MaavImuPlugin)
}