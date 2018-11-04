#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <random>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>

#include "vision/depth-utils/PlaneFitter.hpp"
#include "vision/depth-utils/Point3f.hpp"

using namespace pf;
typedef Point3f Point;

using Eigen::MatrixXf;
using std::vector;
using std::cout;
using std::cerr;
using Eigen::BDCSVD;
using Eigen::Vector4f;
using std::rand;

// Number of iterations to run ransac for
constexpr unsigned int numIterations = 20;
// Minimum number of points to not immediately return failure
// also number of points used in SVD
constexpr unsigned int minPoints = 25;
// Maximum number of points used to keep down runtime
// Any amount greater than this will result in some
// dynamic subsampling
constexpr unsigned int maxPoints = 10000;
// Portion of points that should be inliers to return success
constexpr float percentPoints = 0.65f;
// Minimum number of iterations before returning success
constexpr unsigned int minIterations = 8;

PlaneFitter::PlaneFitter(float inlierDistanceIn) : inlierDistance{inlierDistanceIn} {}
double PlaneFitter::calculateError(const Eigen::Vector4f &sampledCoefficients)
{
    // Orthogonal projection least squares method
    // Calculate a basis of the plane represented by sampledCoefficients
    // Points used to calculate plane basis
    Eigen::Vector3f bPoints[3] = {{0, 0, 0}, {1, 1, 0}, {1, 2, 0}};
    for (unsigned i = 0; i < 3; ++i)
    {
        bPoints[i](2) = (sampledCoefficients(0) * bPoints[i](0) +
                            sampledCoefficients(1) * bPoints[i](1) - sampledCoefficients(3)) /
                        (-1 * sampledCoefficients(2));
    }
    // Basis vectors
    Eigen::Vector3f bVectors[2];
    for (unsigned i = 0; i < 2; ++i)
    {
        bVectors[i] = bPoints[0] - bPoints[i + 1];
    }
    // Matrix A is just the change of basis matrix from b coordinates
    // to standard coordinates of R3
    Eigen::MatrixXf A(3, 2);
    for (unsigned x = 0; x < 2; ++x)
    {
        for (unsigned y = 0; y < 3; ++y)
        {
            A(y, x) = bVectors[x](y);
        }
    }
    // Use A to compute the orthogonal projection matrix onto
    // the plane spanned by the bVectors and store the result in A
    // The transpose of A since eigen is weird with transposes
    Eigen::MatrixXf ATranspose = A.transpose();
    A = A * ((ATranspose * A).inverse()) * ATranspose;
    vector<Eigen::Vector3f> projectedPoints(pointsVectors.size());
    double squaredError = 0;
    for (unsigned i = 0; i < pointsVectors.size(); ++i)
    {
        projectedPoints[i] = A * pointsVectors[i];
        Eigen::Vector3f difference = projectedPoints[i] - pointsVectors[i];
        squaredError += difference.dot(difference);
    }
    return squaredError;
}

MatrixXf PlaneFitter::fitPlane(vector<Point> &points)
{
    // Check if the min number of points exists
    if (points.size() < minPoints)
    {
        return MatrixXf();
    }
    // Check if too many points exist
    if (points.size() > maxPoints)
    {
        unsigned int divideBy = points.size() / maxPoints + 1;
        subSample(points, divideBy);
    }
    // Create an ordered set of point vectors for this set of points
    pointsVectors.resize(points.size());
    for (unsigned i = 0; i < points.size(); ++i)
    {
        pointsVectors[i](0) = points[i].x;
        pointsVectors[i](1) = points[i].y;
        pointsVectors[i](2) = points[i].z;
    }
    // Initialize variables used in later steps of fitting
    const unsigned int pointsSize = points.size();
    Vector4f bestVn(0, 0, 0, 0);
    double bestError = std::numeric_limits<double>::max();
    // Run numIterations of ransac
    for (unsigned int i = 0; i < numIterations; ++i)
    {
        // Randomly Sample minPoints points to compute the SVD of
        MatrixXf sampledPoints = samplePoints(points);
        // Compute the svd
        BDCSVD<MatrixXf> svd(sampledPoints, Eigen::ComputeFullV);
        // Take the right most right singular vector
        MatrixXf v = svd.matrixV();
        Vector4f vn(0, 0, 0, 0);
        for (unsigned row = 0; row < 4; ++row)
        {
            vn(row) = v(row, v.cols() - 1);
        }
        // Find out how many points are inliers
        unsigned numInliers = 0;
        for (unsigned idx = 0; idx < pointsSize; ++idx)
        {
            Point &p = points[idx];
            float distance = (vn(0) * p.x + vn(1) * p.y + vn(2) * p.z + vn(3)) /
                             sqrt(vn(0) * vn(0) + vn(1) * vn(1) + vn(2) * vn(2));
            if (distance <= inlierDistance)
            {
                ++numInliers;
            }
        }
        // If there are enough inliers return the plane coefficients
        if ((float)numInliers > (float)pointsSize * percentPoints)
        {
            float error = calculateError(vn);
            if (error < bestError)
            {
                bestError = error;
                bestVn = vn;
            }
            if (i > minIterations)
            {
                return bestVn;
            }
        }
    }
    if (bestError == std::numeric_limits<double>::max())
    {
        return MatrixXf(0, 0);
    }
    else
    {
        return bestVn;
    }
}

// Done I think
void PlaneFitter::subSample(vector<Point> &points, unsigned int divideBy)
{
    const size_t pointsSize = points.size();
    unsigned int pointsIdx = 0;
    // Set the first n points based on the xth point of points
    // then shrink to the first n elements
    for (unsigned int i = 0; i < pointsSize; i += divideBy)
    {
        points[pointsIdx++] = points[i];
    }
    points.resize(pointsIdx);
    return;
}

// Done I think
MatrixXf PlaneFitter::samplePoints(const std::vector<Point> &points)
{
    std::uniform_int_distribution<unsigned int> sample(0, points.size() - 1);
    std::random_device rd;
    std::mt19937 gen(rd());
    MatrixXf toReturn(minPoints, 4);
    const size_t pointsSize = points.size();
    vector<bool> isPicked(pointsSize, false);
    // Sample points without replacement
    for (unsigned int i = 0; i < minPoints;)
    {
        unsigned int idx = sample(gen);
        if (!isPicked[idx])
        {
            isPicked[idx] = true;
            toReturn(i, 0) = points[idx].x;
            toReturn(i, 1) = points[idx].y;
            toReturn(i, 2) = points[idx].z;
            toReturn(i, 3) = 1;
            ++i;
        }
    }
    return toReturn;
}
