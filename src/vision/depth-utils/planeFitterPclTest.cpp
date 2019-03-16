#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <random>
#include <string>

#include <eigen3/Eigen/Dense>

#include "vision/core/PlaneFitter.hpp"

using Eigen::MatrixXf;
using pcl::PointCloud;
using pcl::PointXYZ;
using std::cerr;
using std::cout;
using std::ofstream;
using std::vector;

constexpr float distanceTolerance = 0.3f;

double calculateError(
    const Eigen::Vector4f &sampledCoefficients, vector<Eigen::Vector3f> &pointsVectors)
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

PointCloud<PointXYZ> generatePoints(const float x, const float y, const float z, const float d,
    const float noise, const unsigned numPoints)
{
    std::normal_distribution<float> sample;
    std::random_device rd;
    std::mt19937 gen(rd());
    const unsigned limit = static_cast<unsigned>(sqrt(numPoints));
    PointCloud<PointXYZ> points(numPoints, 1);
    unsigned i = 0;
    for (unsigned outer = 0; outer < limit; ++outer)
    {
        for (unsigned inner = 0; inner < limit; ++inner)
        {
            if (x == 0.f)
            {
                points[i].x = 0;
            }
            else
            {
                points[i].x = (static_cast<float>(inner) + sample(gen) * noise);
            }
            if (y == 0.f)
            {
                points[i].y = 0;
            }
            else
            {
                points[i].y = static_cast<float>(outer) + sample(gen) * noise;
            }
            points[i].z = (-1.f * x * points[i].x - y * points[i].y - d) / z;
            ++i;
        }
    }
    return points;
}

struct coefficients_t
{
    float x;
    float y;
    float z;
    float d;
};

bool checkCoefficients(coefficients_t ground, coefficients_t test)
{
    std::uniform_real_distribution<float> sample(-1.0, 1.0);
    std::random_device rd;
    std::mt19937 gen(rd());
    for (unsigned i = 0; i < 100; ++i)
    {
        float testX = sample(gen) * 10;
        float testY = sample(gen) * 10;
        PointXYZ groundPoint(testX, testY, 0.f);
        PointXYZ testPoint(testX, testY, 0.f);
        groundPoint.z =
            (-1 * groundPoint.x * ground.x - ground.y * groundPoint.y - ground.d) / ground.z;
        testPoint.z = (-1 * testPoint.x * test.x - test.y * testPoint.y - test.d) / test.z;
        if (sqrt((testPoint.z - groundPoint.z) * (testPoint.z - groundPoint.z)) > distanceTolerance)
        {
            cout << "Failing z: " << testPoint.z << " " << groundPoint.z << '\n';
            return false;
        }
    }
    return true;
}

void randomTest(std::ofstream &foutPCL, std::ofstream &foutErrorPCL)
{
    std::uniform_real_distribution<float> sample;
    std::uniform_real_distribution<float> sampleWithNegatives(-1.0, 1.0);
    std::random_device rd;
    std::mt19937 gen(rd());
    const unsigned numPoints = 1000;
    const float inlierDistance = sample(gen) * 0.1f;
    const float noise = static_cast<float>(sample(gen) * 0.1);
    const float x = sampleWithNegatives(gen);
    const float y = sampleWithNegatives(gen);
    const float d = sampleWithNegatives(gen);
    const float z = -1 * x - y - d;
    maav::vision::PlaneFitter planeFitterPCL(inlierDistance);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    *cloud = generatePoints(x, y, z, d, noise, numPoints);
    vector<Eigen::Vector3f> pointsVectors(cloud->size());
    for (unsigned i = 0; i < cloud->size(); ++i)
    {
        pointsVectors[i](0) = (*cloud)[i].x;
        pointsVectors[i](1) = (*cloud)[i].y;
        pointsVectors[i](2) = (*cloud)[i].z;
    }
    long long time1 = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch())
                          .count();
    auto coefs = planeFitterPCL.fitPlane(cloud);
    long long time2 = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch())
                          .count();
    vector1_t zdot;
    vector1_t zdepth;
    vector1_t roll;
    vector1_t pitch;
    unsigned long long utime = 0;
    planeFitterPCL.runPlaneFitting(cloud, zdot, zdepth, roll, pitch, utime);
    if (coefs.size() == 0)
    {
        foutErrorPCL << std::numeric_limits<double>::max();
    }
    else
    {
        foutErrorPCL << calculateError(coefs, pointsVectors) /
                            static_cast<double>(pointsVectors.size())
                     << std::endl;
    }
    foutPCL << (time2 - time1) << std::endl;
}

int main()
{
    ofstream foutPCL("fitting-times-pcl.txt");
    ofstream foutErrorPCL("fitting-error-pcl.txt");
    for (unsigned i = 0; i < 500; ++i)
    {
        randomTest(foutPCL, foutErrorPCL);
    }
    cout << "tests completed successfully" << std::endl;
    return 0;
}
