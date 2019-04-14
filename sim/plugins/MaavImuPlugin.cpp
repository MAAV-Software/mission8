/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "MaavImuPlugin.hpp"

#include <stdio.h>
#include <chrono>
#include <cmath>
#include <iostream>

#include <boost/bind.hpp>

namespace gazebo
{
MaavImuPlugin::MaavImuPlugin() : ModelPlugin(), velocity_prev_W_(0, 0, 0), zcm("ipc") {}

MaavImuPlugin::~MaavImuPlugin() { updateConnection_->~Connection(); }

void MaavImuPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // Store the pointer to the model
    model_ = _model;
    world_ = model_->GetWorld();

    // default params
    namespace_.clear();

    if (_sdf->HasElement("robotNamespace"))
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    else
        gzerr << "[gazebo_imu_plugin] Please specify a robotNamespace.\n";
    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init(namespace_);

    if (_sdf->HasElement("linkName"))
        link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
    else
        gzerr << "[gazebo_imu_plugin] Please specify a linkName.\n";
    // Get the pointer to the link
    link_ = model_->GetLink(link_name_);
    if (link_ == NULL)
        gzthrow("[gazebo_imu_plugin] Couldn't find specified link \"" << link_name_ << "\".");

    frame_id_ = link_name_;

    getSdfParam<std::string>(_sdf, "imuTopic", imu_topic_, kDefaultImuTopic);
    getSdfParam<double>(_sdf, "gyroscopeNoiseDensity", imu_parameters_.gyroscope_noise_density,
        imu_parameters_.gyroscope_noise_density);
    getSdfParam<double>(_sdf, "gyroscopeRandomWalk", imu_parameters_.gyroscope_random_walk,
        imu_parameters_.gyroscope_random_walk);
    getSdfParam<double>(_sdf, "gyroscopeBiasCorrelationTime",
        imu_parameters_.gyroscope_bias_correlation_time,
        imu_parameters_.gyroscope_bias_correlation_time);
    assert(imu_parameters_.gyroscope_bias_correlation_time > 0.0);
    getSdfParam<double>(_sdf, "gyroscopeTurnOnBiasSigma",
        imu_parameters_.gyroscope_turn_on_bias_sigma, imu_parameters_.gyroscope_turn_on_bias_sigma);
    getSdfParam<double>(_sdf, "accelerometerNoiseDensity",
        imu_parameters_.accelerometer_noise_density, imu_parameters_.accelerometer_noise_density);
    getSdfParam<double>(_sdf, "accelerometerRandomWalk", imu_parameters_.accelerometer_random_walk,
        imu_parameters_.accelerometer_random_walk);
    getSdfParam<double>(_sdf, "accelerometerBiasCorrelationTime",
        imu_parameters_.accelerometer_bias_correlation_time,
        imu_parameters_.accelerometer_bias_correlation_time);
    assert(imu_parameters_.accelerometer_bias_correlation_time > 0.0);
    getSdfParam<double>(_sdf, "accelerometerTurnOnBiasSigma",
        imu_parameters_.accelerometer_turn_on_bias_sigma,
        imu_parameters_.accelerometer_turn_on_bias_sigma);
    getSdfParam<double>(_sdf, "frequency", imu_parameters_.frequency, imu_parameters_.frequency);

    last_time_ = world_->SimTime();
    last_publish_time_ = world_->SimTime();

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection_ =
        event::Events::ConnectWorldUpdateBegin(boost::bind(&MaavImuPlugin::OnUpdate, this, _1));

    gravity_W_ = world_->Gravity();
    imu_parameters_.gravity_magnitude = gravity_W_.Length();

    standard_normal_distribution_ = std::normal_distribution<double>(0.0, 1.0);

    double sigma_bon_g = imu_parameters_.gyroscope_turn_on_bias_sigma;
    double sigma_bon_a = imu_parameters_.accelerometer_turn_on_bias_sigma;
    for (int i = 0; i < 3; ++i)
    {
        gyroscope_turn_on_bias_[i] = sigma_bon_g * standard_normal_distribution_(random_generator_);
        accelerometer_turn_on_bias_[i] =
            sigma_bon_a * standard_normal_distribution_(random_generator_);
    }

    // TODO(nikolicj) incorporate steady-state covariance of bias process
    gyroscope_bias_.setZero();
    accelerometer_bias_.setZero();

    std::cout << "[IMU Plugin] Loaded!" << std::endl;
}

/// \brief This function adds noise to acceleration and angular rates for
///        accelerometer and gyroscope measurement simulation.
void MaavImuPlugin::addNoise(
    Eigen::Vector3d* linear_acceleration, Eigen::Vector3d* angular_velocity, const double dt)
{
    // CHECK(linear_acceleration);
    // CHECK(angular_velocity);
    assert(dt > 0.0);

    // Gyrosocpe
    double tau_g = imu_parameters_.gyroscope_bias_correlation_time;
    // Discrete-time standard deviation equivalent to an "integrating" sampler
    // with integration time dt.
    double sigma_g_d = 1 / sqrt(dt) * imu_parameters_.gyroscope_noise_density;
    double sigma_b_g = imu_parameters_.gyroscope_random_walk;
    // Compute exact covariance of the process after dt [Maybeck 4-114].
    double sigma_b_g_d =
        sqrt(-sigma_b_g * sigma_b_g * tau_g / 2.0 * (exp(-2.0 * dt / tau_g) - 1.0));
    // Compute state-transition.
    double phi_g_d = exp(-1.0 / tau_g * dt);
    // Simulate gyroscope noise processes and add them to the true angular rate.
    for (int i = 0; i < 3; ++i)
    {
        gyroscope_bias_[i] = phi_g_d * gyroscope_bias_[i] +
                             sigma_b_g_d * standard_normal_distribution_(random_generator_);
        (*angular_velocity)[i] = (*angular_velocity)[i] + gyroscope_bias_[i] +
                                 sigma_g_d * standard_normal_distribution_(random_generator_) +
                                 gyroscope_turn_on_bias_[i];
    }

    // Accelerometer
    double tau_a = imu_parameters_.accelerometer_bias_correlation_time;
    // Discrete-time standard deviation equivalent to an "integrating" sampler
    // with integration time dt.
    double sigma_a_d = 1 / sqrt(dt) * imu_parameters_.accelerometer_noise_density;
    double sigma_b_a = imu_parameters_.accelerometer_random_walk;
    // Compute exact covariance of the process after dt [Maybeck 4-114].
    double sigma_b_a_d =
        sqrt(-sigma_b_a * sigma_b_a * tau_a / 2.0 * (exp(-2.0 * dt / tau_a) - 1.0));
    // Compute state-transition.
    double phi_a_d = exp(-1.0 / tau_a * dt);
    // Simulate accelerometer noise processes and add them to the true linear
    // acceleration.
    for (int i = 0; i < 3; ++i)
    {
        accelerometer_bias_[i] = phi_a_d * accelerometer_bias_[i] +
                                 sigma_b_a_d * standard_normal_distribution_(random_generator_);
        (*linear_acceleration)[i] = (*linear_acceleration)[i] + accelerometer_bias_[i] +
                                    sigma_a_d * standard_normal_distribution_(random_generator_) +
                                    accelerometer_turn_on_bias_[i];
    }
}

// This gets called by the world update start event.
void MaavImuPlugin::OnUpdate(const common::UpdateInfo& _info)
{
    common::Time current_time = world_->SimTime();

    double dt = (current_time - last_time_).Double();

    double publish_period = (current_time - last_publish_time_).Double();
    bool should_publish = publish_period >= 1 / imu_parameters_.frequency;

    last_time_ = current_time;

    // Get microseconds for zcm message
    uint64_t usec = current_time.sec * 1000000;
    usec += current_time.nsec / 1000;
    imu_msg_.utime = usec;
    gt_bias_msg_.utime = usec;

    ignition::math::Pose3d T_W_I = link_->WorldPose();  // TODO(burrimi): Check tf.

    ignition::math::Quaterniond C_W_I = T_W_I.Rot();

    // Copy ignition::math::Quaterniond to gazebo::msgs::Quaternion
    gazebo::msgs::Quaternion* orientation = new gazebo::msgs::Quaternion();
    orientation->set_x(C_W_I.X());
    orientation->set_y(C_W_I.Y());
    orientation->set_z(C_W_I.Z());
    orientation->set_w(C_W_I.W());

    ignition::math::Vector3d acceleration_I =
        link_->RelativeLinearAccel() - C_W_I.RotateVectorReverse(gravity_W_);

    ignition::math::Vector3d angular_vel_I = link_->RelativeAngularVel();

    Eigen::Vector3d linear_acceleration_I(
        acceleration_I.X(), acceleration_I.Y(), acceleration_I.Z());
    Eigen::Vector3d angular_velocity_I(angular_vel_I.X(), angular_vel_I.Y(), angular_vel_I.Z());

    imu_msg_.angular_rates.data[0] = angular_velocity_I.y();
    imu_msg_.angular_rates.data[1] = angular_velocity_I.x();
    imu_msg_.angular_rates.data[2] = -angular_velocity_I.z();

    imu_msg_.acceleration.data[0] = linear_acceleration_I.y();
    imu_msg_.acceleration.data[1] = linear_acceleration_I.x();
    imu_msg_.acceleration.data[2] = -linear_acceleration_I.z();

    // Publish only if the period is greater than the desired sampling period
    if (should_publish)
    {
        // Publish noise free imu message
        zcm.publish(maav::SIM_IMU_CHANNEL, &imu_msg_);
    }

    addNoise(&linear_acceleration_I, &angular_velocity_I, dt);

    imu_msg_.angular_rates.data[0] = angular_velocity_I.y();
    imu_msg_.angular_rates.data[1] = angular_velocity_I.x();
    imu_msg_.angular_rates.data[2] = -angular_velocity_I.z();

    imu_msg_.acceleration.data[0] = linear_acceleration_I.y();
    imu_msg_.acceleration.data[1] = linear_acceleration_I.x();
    imu_msg_.acceleration.data[2] = -linear_acceleration_I.z();

    // Publish only if the period is greater than the desired sampling period
    if (should_publish)
    {
        // Publish noisy imu message
        zcm.publish(maav::IMU_CHANNEL, &imu_msg_);
    }

    gt_bias_msg_.gyro_bias.data[0] = gyroscope_bias_.y();
    gt_bias_msg_.gyro_bias.data[1] = gyroscope_bias_.x();
    gt_bias_msg_.gyro_bias.data[2] = -gyroscope_bias_.z();

    gt_bias_msg_.accel_bias.data[0] = accelerometer_bias_.y();
    gt_bias_msg_.accel_bias.data[1] = accelerometer_bias_.x();
    gt_bias_msg_.accel_bias.data[2] = -accelerometer_bias_.z();

    // Publish only if the period is greater than the desired sampling period
    if (should_publish)
    {
        // Publish ground truth biases
        zcm.publish(maav::GT_IMU_CHANNEL, &gt_bias_msg_);
    }

    // Copy Eigen::Vector3d to gazebo::msgs::Vector3d
    gazebo::msgs::Vector3d* linear_acceleration = new gazebo::msgs::Vector3d();
    linear_acceleration->set_x(linear_acceleration_I[0]);
    linear_acceleration->set_y(linear_acceleration_I[1]);
    linear_acceleration->set_z(linear_acceleration_I[2]);

    // Copy Eigen::Vector3d to gazebo::msgs::Vector3d
    gazebo::msgs::Vector3d* angular_velocity = new gazebo::msgs::Vector3d();
    angular_velocity->set_x(angular_velocity_I[0]);
    angular_velocity->set_y(angular_velocity_I[1]);
    angular_velocity->set_z(angular_velocity_I[2]);

    // Reset only if published
    if (should_publish)
    {
        last_publish_time_ = current_time;
    }
}

GZ_REGISTER_MODEL_PLUGIN(MaavImuPlugin)

}  // namespace gazebo
