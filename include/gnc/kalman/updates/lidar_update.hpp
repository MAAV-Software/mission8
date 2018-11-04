#include <gnc/kalman/base_update.hpp>
#include <gnc/measurements/Lidar.hpp>

namespace maav
{
namespace gnc
{
namespace kalman
{
struct LidarMeasurement : measurements::LidarMeasurement
{
    constexpr static size_t DoF = 1;
    LidarMeasurement() = default;

    using CovarianceMatrix = Eigen::Matrix<double, DoF, DoF>;
    using ErrorStateVector = Eigen::Matrix<double, DoF, DoF>;

    const CovarianceMatrix& covariance() const;
    CovarianceMatrix& covariance();

    ErrorStateVector operator-(const LidarMeasurement& other) const;

    LidarMeasurement& operator+=(const ErrorStateVector& error_state);

    CovarianceMatrix _covariance;

    static LidarMeasurement compute_gaussian(
        const std::array<LidarMeasurement, KalmanState::N>& points,
        const std::array<double, KalmanState::N>& m_weights,
        const std::array<double, KalmanState::N>& c_weights)
    {
        LidarMeasurement meas;
        meas.covariance() = CovarianceMatrix::Identity();
        meas.distance = 0;
        return meas;
    }
};

class LidarUpdate : BaseUpdate<LidarMeasurement>
{
    LidarMeasurement predicted(const KalmanState& state);
    LidarMeasurement measured(const measurements::Measurement& meas);

    public:
    using BaseUpdate::operator();
};
}
}
}