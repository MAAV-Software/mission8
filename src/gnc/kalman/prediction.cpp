#include <common/utils/yaml_matrix.hpp>
#include <gnc/constants.hpp>
#include <gnc/kalman/prediction.hpp>

namespace maav
{
namespace gnc
{
namespace kalman
{
UkfPrediction::UkfPrediction(YAML::Node config) : transformation(config["UT"])
{
    using std::placeholders::_1;
    std::function<KalmanState(const KalmanState&)> functor =
        std::bind(&UkfPrediction::predict, this, _1);
    transformation.set_transformation(functor);

    constexpr size_t IMU_DoF = 6;
    using ImpulseVector = Eigen::Matrix<double, IMU_DoF, 1>;
    ImpulseVector Q_i_vec = config["Q_i"].as<ImpulseVector>();
    Eigen::DiagonalMatrix<double, IMU_DoF> Q_i{Q_i_vec};

    Eigen::Matrix<double, KalmanState::DoF, IMU_DoF> F_i =
        Eigen::Matrix<double, KalmanState::DoF, IMU_DoF>::Zero();

    F_i.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    F_i.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity();

    Q = F_i * Q_i * F_i.transpose();
}

void UkfPrediction::operator()(const History::ConstIterator prev, const History::Iterator next)
{
    // Set internal references for use in the transition function
    _prev = prev;
    _next = next;

    // Propagate state
    _next->state = transformation(prev->state);

    // Add process noise
    _next->state.covariance() += Q;
}

KalmanState UkfPrediction::predict(const KalmanState& state)
{
    KalmanState& next_state = _next->state;
    double dt = static_cast<double>(_next->get_time() - _prev->get_time()) * constants::USEC_TO_SEC;

    // Propagate unchanged biases
    next_state.gyro_bias() = _prev->state.gyro_bias();
    next_state.accel_bias() = _prev->state.accel_bias();

    const Eigen::Vector3d& w_prev =
        _prev->measurement.imu->angular_rates - _prev->state.gyro_bias();
    const Eigen::Vector3d& w_next =
        _next->measurement.imu->angular_rates - _next->state.gyro_bias();
    const Eigen::Vector3d w_mid = (w_next + w_prev) / 2.0;
    const Sophus::SO3d dR = Sophus::SO3d::exp(w_mid * dt);
    next_state.attitude() = _prev->state.attitude() * dR;

    // TODO: need to check if we should invert attitude here
    const Eigen::Vector3d gravity = {0, 0, -constants::STANDARD_GRAVITY};

    /**
     * Remove biases
     * Rotate to global frame
     * Remove gravity
     */
    const Eigen::Vector3d& a_prev =
        state.attitude() * (_prev->measurement.imu->acceleration - _prev->state.accel_bias()) -
        gravity;
    const Eigen::Vector3d& a_next =
        next_state.attitude() * (_next->measurement.imu->acceleration - _next->state.accel_bias()) -
        gravity;
    const Eigen::Vector3d a_mid = (a_next + a_prev) / 2.0;
    next_state.velocity() = _prev->state.velocity() + (a_mid * dt);

    const Eigen::Vector3d& v_prev = _prev->state.velocity();
    const Eigen::Vector3d& v_next = next_state.velocity();
    const Eigen::Vector3d v_mid = (v_next + v_prev) / 2.0;
    next_state.position() = _prev->state.position() + (v_mid * dt);

    // Move bias corrected sensor readings
    next_state.angular_velocity() = w_next;
    next_state.acceleration() = a_next;

    return next_state;
}
}  // namespace kalman
}  // namespace gnc
}  // namespace maav