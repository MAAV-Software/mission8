#ifndef __PLANEFITTER_HPP__
#define __PLANEFITTER_HPP__

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

#include "Point3f.hpp"

class PlaneFitter
{
    public:
    PlaneFitter() = delete;
    explicit PlaneFitter(float inlierDistanceIn);
    // Pass in a matrix that stores all the points
    // in the point cloud, will return information
    // from plane fitting
    Eigen::MatrixXf fitPlane(std::vector<pf::Point3f> &points);
    // Keep only ever divideBy'th point used in case there
    // is much more data than needed
    void subSample(std::vector<pf::Point3f> &pointsIn, unsigned int divideBy);
    // Sample points and return an eigen matrix of
    // size minPoints by 4 containing them
    Eigen::MatrixXf samplePoints(const std::vector<pf::Point3f> &pointsIn);
    // Calculate the least mean squares error
    double calculateError(const Eigen::Vector4f &sampledCoefficients);

    private:
    float inlierDistance;
    std::vector<Eigen::Vector3f> pointsVectors;
};

#endif
