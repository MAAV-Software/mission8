#pragma once

#include <map>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

// Until we have real data
#include <random>

#include <yaml-cpp/yaml.h>

#include "AbstractData.hpp"
#include "qcustomplot.h"

// enum class PlotType
// {
//     LINE,
//     PARAMETRIC2D,
//     PARAMETRIC3D,
//     SCATTER2D,
//     SCATTER3D
// };

class QuaternionData : public AbstractData
{
public:
    QuaternionData() = delete;
    QuaternionData(const QuaternionData&) = delete;

    QuaternionData(std::vector<std::string> legend);

protected:
    using AbstractData::abstract_data_mutex;
    using AbstractData::dim_;
    using AbstractData::graphs_;
    using AbstractData::legend_;
    using AbstractData::name;
    using AbstractData::new_data_;
};