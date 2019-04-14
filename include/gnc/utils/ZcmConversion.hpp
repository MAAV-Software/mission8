#pragma once

#include <memory>

#include <Eigen/Eigen>

#include <common/messages/global_update_t.hpp>
#include <common/messages/groundtruth_inertial_t.hpp>
#include <common/messages/imu_t.hpp>
#include <common/messages/lidar_t.hpp>
#include <common/messages/matrix_t.hpp>
#include <common/messages/plane_fit_t.hpp>
#include <common/messages/quaternion_t.hpp>
#include <common/messages/state_t.hpp>
#include <common/messages/vector1_t.hpp>
#include <common/messages/vector2_t.hpp>
#include <common/messages/vector3_t.hpp>
#include <common/messages/vector4_t.hpp>
#include <common/messages/waypoint_t.hpp>
#include <gnc/State.hpp>
#include <gnc/measurements/GlobalUpdateMeasurement.hpp>
#include <gnc/measurements/ImuMeasurement.hpp>
#include <gnc/measurements/LidarMeasurement.hpp>
#include <gnc/measurements/PlaneFitMeasurement.hpp>
#include <gnc/measurements/Waypoint.hpp>
namespace maav
{
namespace gnc
{
state_t ConvertState(const State& state);
State ConvertState(const state_t& state);

State ConvertGroundTruthState(const groundtruth_inertial_t& state);

Waypoint ConvertWaypoint(const waypoint_t& zcm_waypoint);

std::shared_ptr<measurements::LidarMeasurement> convertLidar(const lidar_t& zcm_lidar);
// lidar_t convertLidar(const measurements::LidarMeasurement& lidar);

std::shared_ptr<measurements::ImuMeasurement> convertImu(const imu_t& zcm_imu);
// imu_t convertImu(const measurements::ImuMeasurement& imu);

std::shared_ptr<measurements::PlaneFitMeasurement> convertPlaneFit(
    const plane_fit_t& zcm_plane_fit);
// plane_fit_t convertPlaneFit(const measurements::PlaneFitMeasurement& plane_fit);

std::shared_ptr<measurements::GlobalUpdateMeasurement> convertGlobalUpdate(
    const global_update_t& zcm_global);
// global_update_t convertGlobalUpdate(const measurements::GlobalUpdateMeasurement& global);

vector1_t convertVector1d(double vec);
double convertVector1d(const vector1_t& zcm_vec);

vector2_t convertVector2d(const Eigen::Vector2d& vec);
Eigen::Vector2d convertVector2d(const vector2_t& zcm_vec);

vector3_t convertVector3d(const Eigen::Vector3d& vec);
Eigen::Vector3d convertVector3d(const vector3_t& zcm_vec);

vector4_t convertVector4d(const Eigen::Vector4d& vec);
Eigen::Vector4d convertVector4d(const vector4_t& zcm_vec);

quaternion_t convertQuaternion(const Sophus::SO3d& att);
Sophus::SO3d convertQuaternion(const quaternion_t& zcm_att);

template <class EigenDerived>
void convertMatrix(const Eigen::MatrixBase<EigenDerived>& mat, matrix_t& zcm_mat)
{
    zcm_mat.rows = static_cast<int32_t>(mat.rows());
    zcm_mat.cols = static_cast<int32_t>(mat.cols());
    zcm_mat.data =
        std::vector<std::vector<double>>(zcm_mat.rows, std::vector<double>(zcm_mat.cols));
    for (size_t i = 0; i < static_cast<size_t>(mat.rows()); i++)
    {
        for (size_t j = 0; j < static_cast<size_t>(mat.cols()); j++)
        {
            zcm_mat.data[i][j] = mat(i, j);
        }
    }
}

template <class EigenDerived>
void convertMatrix(Eigen::MatrixBase<EigenDerived>& mat, const matrix_t& zcm_mat)
{
    for (size_t i = 0; i < static_cast<size_t>(zcm_mat.rows); i++)
    {
        for (size_t j = 0; j < static_cast<size_t>(zcm_mat.cols); j++)
        {
            mat(i, j) = zcm_mat.data[i][j];
        }
    }
}
}  // namespace gnc
}  // namespace maav
