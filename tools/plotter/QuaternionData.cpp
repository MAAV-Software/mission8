#include "QuaternionData.hpp"

#include <Eigen/Dense>

#include <cstdlib>
#include <iostream>

QuaternionData::QuaternionData(std::vector<std::string> legend) : AbstractData(3, legend) {}
