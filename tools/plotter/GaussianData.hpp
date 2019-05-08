#pragma once

#include <Eigen/Dense>

#include "AbstractData.hpp"

class GaussianData : public AbstractData
{
public:
    GaussianData() = delete;

    GaussianData(int dim, std::vector<std::string> legend);

    using AbstractData::addData;

    virtual void addData(std::vector<double>&& data, const Eigen::MatrixXd& covariance);

    virtual bool createLinePlots(QCustomPlot* plot) override;

    virtual void plotLines() override;

    // virtual void purgeGraphs() override;

protected:
    size_t getDataLine(size_t i);
    size_t getUpperBound(size_t i);
    size_t getLowerBound(size_t i);

    using AbstractData::graphs_;

    using DataPair = std::pair<std::vector<double>, Eigen::MatrixXd>;
    std::queue<DataPair> new_data_;

    std::mutex covariance_mutex_;
};