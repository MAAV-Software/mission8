#ifndef MAAV_MATH_HPP
#define MAAV_MATH_HPP

#include <stddef.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <random>
#include <string>
#include <utility>
#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

/**
 * @file math.hpp
 * @brief Convenience math functions
 *
 * @details A bunch of general math algorithms are implemented here.
 * A fair amount of the functions are templated, and those that aren't
 * eschew templates because they're not needed or not wanted.
 */

/**
 * @brief Default maav namespace
 */
namespace maav
{
/**
 * @brief Globally defined pi
 */
constexpr double PI = 3.14159265358979;

/**
 * @brief See if two numbers are close to each other
 * @param d1 A number
 * @param d2 A number
 * @param res Resolution (how close the numbers have to be e.g. .001)
 * @return true if (abs(d1 - d2) < res), false otherwise
 */
template <typename T>
constexpr bool is_approx_equal(T d1, T d2, double res)
{
    double tmp = fabs(d1 - d2);
    if (tmp < res) return true;
    return false;
}

/**
 * @brief Overloaded for vector support, tests each pair of elements
 * @see is_approx_equal(T, T, double)
 * @param d1 A vector whose elements will be compared
 * @param d2 A vector whose elements will be compared
 * @param res Resolution, or maximum difference between the numbers
 * @return true if (abs(d1[i] - d2[i]) < res) for i = 0:d1.length-1
 */
template <typename T>
constexpr bool is_approx_equal(const std::vector<T> &d1, const std::vector<T> &d2, double res)
{
    bool ret = true;
    size_t i = 0;
    size_t size = std::min(d1.size(), d2.size());
    while (ret && (i < size))
    {
        ret &= is_approx_equal(d1[i], d2[i], res);
        ++i;
    }
    return ret;
}

/**
 * @brief Overloaded for Vector3d support, tests each pair of elements
 * @see is_approx_equal(T, T, double)
 * @param d1 A Vector3d whose elements will be compared
 * @param d2 A Vector3d whose elements will be compared
 * @param res Resolution, or maximum difference between the numbers
 * @return true if (abs(d1[i] - d2[i]) < res) for i = 0:2
 */
inline bool is_approx_equal(const Eigen::Vector3d &d1, const Eigen::Vector3d &d2, double res)
{
    for (int i = 0; i < 3; i++)
    {
        if (!is_approx_equal(d1(i), d2(i), res)) return false;
    }
    return true;
}

/**
 * @brief Converts an angle from degrees to radians
 * @param angle Angle, in degrees
 * @return Angle in radians
 */
constexpr double deg_to_rad(double angle) { return angle / 180.0 * PI; }
/**
 * @brief Converts an angle from radians to degrees
 * @param angle Angle, in radians
 * @return Angle in degrees
 */
constexpr double rad_to_deg(double angle) { return angle / PI * 180.0; }
/**
 * @brief Calculates the hypotenuse of a right triangle with given leg lengths.
 * @return Length of the triangle's hypotenuse.
 */
constexpr double calc_hypotenuse(double l1, double l2)
{
    return std::sqrt(std::pow(l1, 2) + std::pow(l2, 2));
}

/**
 * @brief Computes the modulus of a number
 * @param num Number used for computation
 * @param div Divisor used for computation
 * @return The modulus -> (num % div)
 */
template <typename T>
constexpr T mod(T num, T div)
{
    if (div == 0) return 0;
    T tmp = floor(num / div);
    return num - (div * tmp);
}

/**
 * @brief A Pseudo-Random Number Generator type
 *
 * @details Objects of this type can be used to produce random numbers in a much
 * more reliable and modular way than `std::rand` and `std::srand` and a much
 * more convenient way than using `<random>` directly. A simple use of this
 * class looks something like this:
 *
 *     //this random number generator is automatically randomly seeded, and thus
 *     //non-deterministic
 *     maav::PRNG rand;
 *
 *     //this will print some random value in the range [0.0, 1.0)
 *     cout << rand(0.0, 1.0) << endl;
 *
 *     //this one is seeded with a particular string, so it will produce the
 *     //same sequence each time this program is run
 *     maav::PRNG seeded_rand{"lolz so random xd"};
 *
 *     //this also prints a value in [0.0, 1.0), but this value doesn't change
 *     //between program invocations
 *     cout << seeded_rand(0.0, 1.0) << endl;
 */
class PRNG
{
    // this internal random engine used to generate random numbers
    // this type is chosen because it has a really high period, even though it
    // also has a lot of state
    std::mt19937 random_engine;

   public:
    /**
     * @brief Creates a randomly-seeded PRNG
     */
    PRNG();

    /**
     * @brief Creates a PRNG seeded with a certain string
     * @param seed The seed to use
     */
    explicit PRNG(const char *seed);

    /**
     * @brief Creates a PRNG seeded with a certain string
     * @param seed The seed to use
     */
    explicit PRNG(const std::string &seed);

    /**
     * @brief Produces a (uniformly distributed) random number in the range
     * [left, right)
     * @param left The left endpoint
     * @param right The right endpoint
     * @pre left < right
     */
    double operator()(double left, double right);
};

/**
 * @brief Returns increments for moving dist along an axis
 * @param rpy The orientation of an object in space
 * @param dist Distance to move with each step
 * @return The direction vector, in an Eigen::Vector3d
 *
 * @details Calculates the xyz increments you need to traverse along
 * the axis that is rotated.
 */
inline Eigen::Vector3d dir_vec(const Eigen::Vector3d &rpy, double dist)
{
    Eigen::Vector3d ret;
    Eigen::Vector3d rot(rpy);

    ret(0) = 1;
    ret(1) = 0;
    ret(2) = 0;

    Eigen::Quaterniond tmp = Eigen::AngleAxisd(rot(0), Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(rot(1), Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(rot(2), Eigen::Vector3d::UnitZ());

    tmp.normalize();

    ret = tmp.toRotationMatrix() * ret;
    ret *= dist;
    return ret;
}

/**
 * @brief Creates a theshold array
 * @param hl Low h
 * @param hh High h
 * @param sl Low s
 * @param sh High s
 * @param vl Low v
 * @param vh High v
 * @return Threshold array
 */
constexpr std::array<uint8_t, 6> make_thresh(
    uint8_t hl, uint8_t hh, uint8_t sl, uint8_t sh, uint8_t vl, uint8_t vh)
{
    return {hl, hh, sl, sh, vl, vh};
}

/**
 * @param csv CSV string
 * @see make_thresh(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t)
 */
inline std::array<uint8_t, 6> make_thresh(const std::string &csv)
{
    std::string tmp[6];
    size_t tmp_i = 0;
    for (size_t i = 0; i < csv.size(); ++i)
    {
        if (csv[i] == ',')
            tmp_i++;
        else
            tmp[tmp_i].push_back(csv[i]);
    }
    if (tmp_i != 5) return make_thresh(0, 0, 0, 0, 0, 0);
    return make_thresh(atoi(tmp[0].c_str()), atoi(tmp[1].c_str()), atoi(tmp[2].c_str()),
        atoi(tmp[3].c_str()), atoi(tmp[4].c_str()), atoi(tmp[5].c_str()));
}

/**
 * @brief Gives the yaw necessary to travel from start to end
 * @param start Starting position
 * @param end Ending position
 * @return Yaw angle needed to travel from start and end
 */
inline double yaw_between(Eigen::Vector3d &start, Eigen::Vector3d &end)
{
    Eigen::Vector3d diff = end - start;
    if (diff(1) == 0)
    {
        if (diff(0) < 0)
            return maav::PI;
        else
            return 0.0;
    }
    else if (diff(0) == 0)
    {
        if (diff(1) < 0)
            return -1 * maav::PI / 2;
        else
            return maav::PI / 2;
    }

    double tmp = atan(diff(1) / diff(0));

    if (diff(0) < 0 && diff(1) > 0)
        return maav::PI + tmp;
    else if (diff(0) < 0 && diff(1) < 0)
        return (-1 * maav::PI) + tmp;

    return tmp;
}

/**
 * @brief Creates an Eigen::Vector3 with the required type
 * @param x 0th value
 * @param y 1st value
 * @param z 2nd value
 * @return An Eigen::Vector3 that looks like [x, y, z]
 */
template <typename T>
constexpr Eigen::Matrix<T, 3, 1> create_vec(T x, T y, T z)
{
    Eigen::Matrix<T, 3, 1> ret;
    ret(0) = x;
    ret(1) = y;
    ret(2) = z;
    return ret;
}

constexpr std::pair<double, double> polar_to_cart(double r, double theta)
{
    return std::make_pair(r * cos(theta), r * sin(theta));
}

/**
 * @brief Maps a real (x,y) location to a pixel location
 * @param x x location
 * @param y y location
 * @param max_dim maximum possible absolute value of x or y
 * @param dimension of the image
 */
constexpr std::pair<unsigned int, unsigned int> point_to_pixel(
    double x, double y, double max_dim, unsigned int image_dim)
{
    double scale = static_cast<double>(image_dim / 2) / max_dim;

    return std::make_pair(static_cast<unsigned int>(x * scale + image_dim / 2),
        image_dim - static_cast<unsigned int>(y * scale + image_dim / 2));
}

/**
 * Matches and reorders sets of expected corner coordinates and camera corner
 * coordinates. After this function is done, 'camera[i]' will be the camera
 * corner that matches with the expected corner 'expected[i]'. Also will trim
 * off unmatched corners so after it's done: expected.size() == camera.size()
 *
 * @param expected The set of expected corner coordinates
 * @param camera The set of camera corner coordinates
 */
void matchCorners(std::vector<Eigen::Vector2d> &expected, std::vector<Eigen::Vector2d> &camera);

/**
 * Calculates and returns an affine transformation matrix from expected corner
 * coordinates to camera corner coordinates.
 *
 * NOTE: Requires at least 3 points for each
 *
 * @param expected A vector of expected corner coordinate points
 * @param camera A vector of coordinate points from the camera
 * @return The transformation matrix
 */
Eigen::Matrix3d getTransformMatrix(const std::vector<Eigen::Vector2d> &expected,
    const std::vector<Eigen::Vector2d> &camera, double yaw);

/**
 * @brief Returns all the points on lines in the given image
 * @param src Source image
 * @return An array of array of cv::Point. Each array of points is a single
 * "contour", or line
 *
 * @details This function does not do any color thresholding. Pass in an
 * already thresholded image if that is desired.
 */
std::vector<Eigen::Vector2d> get_line_points(const cv::Mat &src);

/**
 * @brief Updates current vehicle position using localization
 * @param pos Current position
 * @param expected Expected feature image coordinates based on current position
 * @param camera Feature image coordinates from computer vision
 * @return The localized position
 *
 * @details Requires that expected & camera coordinates are already matched up.
 * Throws maav::err::LocalizationError if localization is not possible.
 */
Eigen::Vector3d localizePosition(const Eigen::Vector3d &pos, double yaw,
    const std::vector<Eigen::Vector2d> &expected, const std::vector<Eigen::Vector2d> &camera);
}  // namespace maav

template <size_t T>
double MahalanobisDistance(const Eigen::Matrix<double, T, 1> &x,
    const Eigen::Matrix<double, T, 1> &y, const Eigen::Matrix<double, T, T> &P)
{
    return ((x - y).transpose() * P.inverse() * (x - y))(0);
}

#endif  // MAAV_MATH_HPP
