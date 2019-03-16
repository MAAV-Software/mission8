#pragma once

#include <map>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

// Until we have real data
#include <random>

#include <yaml-cpp/yaml.h>

#include "qcustomplot.h"

enum class PlotType
{
    LINE,
    PARAMETRIC2D,
    PARAMETRIC3D,
    SCATTER2D,
    SCATTER3D
};

class AbstractData
{
public:
    AbstractData() = delete;
    AbstractData(const AbstractData&) = delete;

    AbstractData(int dim, std::vector<std::string> legend);

    virtual void addData(std::vector<double>&& data);

    virtual void purgeGraphs();

    virtual bool createLinePlots(QCustomPlot* plot);

    virtual void destroyLinePlots(QCustomPlot* plot);

    virtual void createScatterPlots(QCustomPlot* plot);

    virtual void createParametricPlots(QCustomPlot* plot);

    /**
     * @brief Plots all dimensions as lines vs time
     */
    virtual void plotLines();

    /**
     * @brief Plots 2 dimensions as a parametric curve. dim must be 2
     */
    virtual void plot2DParametric();

    /**
     * @brief Plots 3 dimensions as a parametric curve. dim must be 3
     */
    virtual void plot3DParametric();

    /**
     * @brief Plots 2 dimensions as points. dim must be 2
     */
    virtual void Plot2DScatter();

    /**
     * @brief Plots 3 dimensions as points. dim must be 3
     */
    virtual void Plot3DScatter();

protected:
    int dim_;
    QString name;
    std::vector<QString> legend_;

    /**
     * A map of (Key: window ptr, Value: list of graphs inside that window).
     * This list of graphs has size = dim_.
     */
    std::map<QCustomPlot*, std::vector<QCPGraph*>> graphs_;

    std::queue<std::vector<double>> new_data_;

    std::mutex abstract_data_mutex;
};