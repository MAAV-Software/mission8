#include <iostream>
#include <string>
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <random>
#include <chrono>
#include <fstream>

#include "vision/depth-utils/Point3f.hpp"
#include "vision/depth-utils/PlaneFitter.hpp"
#include "vision/depth-utils/PlaneFitterPCL.hpp"

using std::vector;
using std::cout;
using std::cerr;
using pf::Point3f;
using Eigen::MatrixXf;
using std::ofstream;

constexpr float distanceTolerance = 0.3f;

double calculateError(const Eigen::Vector4f &sampledCoefficients,
	vector<Eigen::Vector3f> &pointsVectors)
{
	// Orthogonal projection least squares method
	// Calculate a basis of the plane represented by sampledCoefficients
	// Points used to calculate plane basis
	Eigen::Vector3f bPoints[3] =
	{
		{0, 0, 0},
		{1, 1, 0},
		{1, 2, 0}
	};
	for (unsigned i = 0; i < 3; ++i)
	{
		bPoints[i](2) = (sampledCoefficients(0) * bPoints[i](0) +
			sampledCoefficients(1) * bPoints[i](1) -
			sampledCoefficients(3)) /
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

vector<Point3f> generatePoints(const float x, const float y,
	const float z, const float d,
	const float noise, const unsigned numPoints)
{
	std::normal_distribution<float> sample;
	std::random_device rd;
	std::mt19937 gen(rd());
	const unsigned limit = sqrt(numPoints);
	vector<Point3f> points(numPoints);
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
				points[i].x = inner + sample(gen) * noise;
			}
			if (y == 0.f)
			{
				points[i].y = 0;
			}
			else
			{
				points[i].y = outer + sample(gen) * noise;
			}
			points[i].z = (-1.f * x * points[i].x - y * points[i].y - d) / z;
			++i;
		}
	}
	return points;
}

void oldTest()
{
	float inlierDistance = 0.3;
	float x = 2.4;
	float y = 1.4;
	float z = -2.5;
	float d = 3;
	float noise = 0.2;
	unsigned numPoints = 200;
	PlaneFitter planeFitter(inlierDistance);
	vector<Point3f> points = generatePoints(x, y, z, d, noise, numPoints);
	for (auto &point : points)
	{
		cout << point.x << " " << point.y << " " << point.z << std::endl;
	}
	MatrixXf coefs = planeFitter.fitPlane(points);
	cout << "x = " << coefs(0, 0) << ", y = " << coefs(1, 0) << ", z = " << coefs(2, 0);
	cout << ", d = " << coefs(3, 0) << std::endl;
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
		Point3f groundPoint = {testX, testY, 0.f};
		Point3f testPoint = {testX, testY, 0.f};
		groundPoint.z = (-1 * groundPoint.x * ground.x - ground.y * groundPoint.y -
			ground.d) / ground.z;
		testPoint.z = (-1 * testPoint.x * test.x - test.y * testPoint.y -
			test.d) / test.z;
		if (sqrt((testPoint.z - groundPoint.z) * (testPoint.z - groundPoint.z))
			> distanceTolerance)
		{
			cout << "Failing z: " << testPoint.z << " " << groundPoint.z << '\n';
			return false;
		}
	}
	return true;
}

void randomTest(std::ofstream &fout, std::ofstream &foutError,
	std::ofstream &foutPCL, std::ofstream &foutErrorPCL)
{
	std::uniform_real_distribution<float> sample;
	std::uniform_real_distribution<float> sampleWithNegatives(-1.0, 1.0);
	std::random_device rd;
	std::mt19937 gen(rd());
	const unsigned numPoints = 1000;
	const float inlierDistance = sample(gen) * 0.1f;
	const float noise = sample(gen) * 0.1;
	const float x = sampleWithNegatives(gen);
	const float y = sampleWithNegatives(gen);
	const float d = sampleWithNegatives(gen);
	const float z = -1 * x - y - d;
	PlaneFitter planeFitter(inlierDistance);
	PlaneFitterPCL planeFitterPCL(inlierDistance);
	vector<Point3f> points = generatePoints(x, y, z, d, noise, numPoints);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	planeFitterPCL.convertCloud(cloud, points);
	vector<Eigen::Vector3f> pointsVectors(points.size());
	for (unsigned i = 0; i < points.size(); ++i)
	{
		pointsVectors[i](0) = points[i].x;
		pointsVectors[i](1) = points[i].y;
		pointsVectors[i](2) = points[i].z;
	}
	long long time1 = std::chrono::duration_cast<std::chrono::
		microseconds>(std::chrono::system_clock::now().
		time_since_epoch()).count();
	MatrixXf coefs = planeFitter.fitPlane(points);
	long long time2 = std::chrono::duration_cast<std::chrono::
		microseconds>(std::chrono::system_clock::now().
		time_since_epoch()).count();
	if (coefs.size() == 0)
	{
		foutError << std::numeric_limits<double>::max();
	}
	else
	{
		foutError << calculateError(coefs, pointsVectors) /
			pointsVectors.size() << std::endl;
	}
	fout << (time2 - time1) / 1000 << std::endl;
	// Now perform the same thing but with pcl
	time1 = std::chrono::duration_cast<std::chrono::
		microseconds>(std::chrono::system_clock::now().
		time_since_epoch()).count();
	coefs = planeFitterPCL.fitPlane(cloud);
	time2 = std::chrono::duration_cast<std::chrono::
		microseconds>(std::chrono::system_clock::now().
		time_since_epoch()).count();
	float zdot;
	float zdepth;
	planeFitterPCL.runPlaneFitting(cloud, zdot, zdepth);
	if (coefs.size() == 0)
	{
		foutErrorPCL << std::numeric_limits<double>::max();
	}
	else
	{
		foutErrorPCL << calculateError(coefs, pointsVectors) /
			pointsVectors.size() << std::endl;
	}
	foutPCL << (time2 - time1) << std::endl;
}

int main()
{
	ofstream fout("fitting-times.txt");
	ofstream foutPCL("fitting-times-pcl.txt");
	ofstream foutError("fitting-error.txt");
	ofstream foutErrorPCL("fitting-error-pcl.txt");
	for (unsigned i = 0; i < 500; ++i)
	{
		randomTest(fout, foutError, foutPCL, foutErrorPCL);
	}
	cout << "tests completed successfully" << std::endl;
	return 0;
}
