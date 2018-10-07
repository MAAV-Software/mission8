#include "common/math/math.hpp"
#include <Eigen/Dense>
#include "common/utils/LocalizationError.hpp"

#include <cassert>
#include <cstring>
#include <iterator>

constexpr double MAX_PIXEL_DIFF = 5.0;

using Eigen::ComputeThinU;
using Eigen::ComputeThinV;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Rotation2D;
using Eigen::Vector2d;
using Eigen::Vector2i;
using Eigen::Vector3d;
using Eigen::VectorXd;
using cv::Canny;
using cv::GaussianBlur;
using cv::Mat;
using cv::Point;
using cv::Scalar;
using cv::findContours;
using cv::inRange;
using cv::Size;
using std::vector;
using std::random_device;
using std::seed_seq;
using std::strlen;
using std::string;
using std::begin;
using std::end;
using std::uniform_real_distribution;
using std::min;

namespace maav
{
PRNG::PRNG() : random_engine{random_device{}()} {}
PRNG::PRNG(const char *seed)
{
	seed_seq sseq(seed, seed + strlen(seed));
	random_engine.seed(sseq);
}

PRNG::PRNG(const string &seed)
{
	seed_seq sseq(begin(seed), end(seed));
	random_engine.seed(sseq);
}

double PRNG::operator()(double left, double right)
{
	assert(left < right);
	return uniform_real_distribution<>{left, right}(random_engine);
}
}

void maav::matchCorners(vector<Vector2d> &expected, vector<Vector2d> &camera)
{
	if (expected.empty() || camera.empty())
	{
		return;
	}

	vector<Vector2d> newExpected;
	vector<Vector2d> newCamera;

	for (unsigned i = 0; i < expected.size(); ++i)
	{
		int bestIdx = 0;
		double bestDist = DBL_MAX;

		// find the closest camera point to this expected point
		for (unsigned j = 0; j < camera.size(); ++j)
		{
			double currDist = (camera[j] - expected[i]).squaredNorm();
			if (currDist < bestDist)
			{
				bestIdx = j;
				bestDist = currDist;
			}
		}

		if ((bestDist - pow(MAX_PIXEL_DIFF, 2.0)) < 0.0)
		{
			newExpected.push_back(expected[i]);
			newCamera.push_back(camera[bestIdx]);
		}
	}

	expected = newExpected;
	camera = newCamera;
}

Matrix3d maav::getTransformMatrix(const vector<Vector2d> &expected, const vector<Vector2d> &camera,
								  double yaw)
{
	int totalPoints = min(expected.size(), camera.size());

	MatrixXd A(totalPoints * 2, 3);

	double M1 = cos(yaw);
	double M2 = -sin(yaw);
	double M4 = sin(yaw);
	double M5 = cos(yaw);

	for (int i = 0; i < totalPoints * 2; ++i)
	{
		if (i % 2 == 0)
		{
			A(i, 0) = expected[i / 2][0] * M1 + expected[i / 2][1] * M2;
			A(i, 1) = 1;
			A(i, 2) = 0;
		}
		else
		{
			A(i, 0) = expected[i / 2][0] * M4 + expected[i / 2][1] * M5;
			A(i, 1) = 0;
			A(i, 2) = 1;
		}
	}

	// Get b matrix from camera corner coords
	VectorXd b(totalPoints * 2);

	for (int i = 0; i < totalPoints; ++i)
	{
		b[i * 2] = camera[i][0];
		b[(i * 2) + 1] = camera[i][1];
	}

	// Solve Ax = b for x
	VectorXd x = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);

	// Get transformation matrix from x
	Matrix3d transform;
	transform << M1, M2, x[1], M4, M5, x[2], 0, 0, 1;

	return transform;
}

vector<Vector2d> maav::get_line_points(const Mat &src)
{
	vector<vector<Point>> all_contours;
	vector<Vector2d> contours;
	Mat tmp;

	GaussianBlur(src, tmp, Size(9, 9), 2, 2);
	Canny(tmp, tmp, 50, 50);
	findContours(tmp, all_contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	for (const auto &contour : all_contours)
	{
		for (const auto &pt : contour)
		{
			contours.push_back(Vector2d{pt.x, pt.y});
		}
	}

	return contours;
}

Eigen::Vector3d maav::localizePosition(const Eigen::Vector3d &pos, double yaw,
									   const std::vector<Eigen::Vector2d> &expected,
									   const std::vector<Eigen::Vector2d> &camera)
{
	Eigen::Vector3d newPos;

	// Calculate a new transformation matrix only if we have enough points
	if (expected.size() < 3 || camera.size() < 3)
	{
		throw err::LocalizationError{"not enough points"};
	}

	Matrix3d transform = getTransformMatrix(expected, camera, yaw);

	if (transform.hasNaN())
	{
		throw err::LocalizationError{"transform matrix has NaNs"};
	}

	// get translation from matrix
	Vector2d translation = {transform(0, 2), transform(1, 2)};

	// transform translation vector back to world reference frame
	Rotation2D<double> rot(yaw);
	translation = rot * translation;

	// Apply translation to current position.
	// translation is from the set of expected corners to set of camera
	// corners, so it's opposite of the actual change in current pos
	newPos = pos;
	newPos[0] -= translation[0];
	newPos[1] -= translation[1];

	return newPos;
}
