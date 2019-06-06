#include <cmath>
#include <iostream>

#include "GaussianData.hpp"

GaussianData::GaussianData(int dim, std::vector<std::string> legend) : AbstractData(dim, legend) {}
size_t GaussianData::getDataLine(size_t i) { return 3 * i; }
size_t GaussianData::getUpperBound(size_t i) { return 3 * i + 1; }
size_t GaussianData::getLowerBound(size_t i) { return 3 * i + 2; }
void GaussianData::addData(std::vector<double>&& data, const Eigen::MatrixXd& covariance)
{
    std::lock_guard<std::mutex> guard(covariance_mutex_);
    new_data_.emplace(data, covariance);
}

bool GaussianData::createLinePlots(QCustomPlot* plot)
{
    std::lock_guard<std::mutex> guard(abstract_data_mutex);
    // Check if window is not already being plotted on
    if (graphs_.find(plot) == graphs_.end())
    {
        graphs_[plot] = std::vector<QCPGraph*>(3 * dim_);

        // For each dimension, add a line and two covariance bounds
        for (int i = 0; i < dim_; i++)
        {
            QCPGraph* new_graph = plot->addGraph();
            QCPGraph* cov_upper = plot->addGraph();
            QCPGraph* cov_lower = plot->addGraph();

            // Set legend
            new_graph->setName(legend_[i]);

            // Set looks
            QPen graphPen;
            int cutoff = 50;
            QColor color(QColor(rand() % (255 - cutoff) + cutoff, rand() % (255 - cutoff) + cutoff,
                rand() % (255 - cutoff) + cutoff));
            static int discard = 0;
            discard = (discard + 1) % 3;
            switch (discard)
            {
                case 0:
                    color.setBlue(0);
                    break;
                case 1:
                    color.setRed(0);
                    break;
                case 2:
                    color.setGreen(0);
                    break;
            }
            graphPen.setColor(color);
            graphPen.setWidthF(1);
            new_graph->setPen(graphPen);

            QPen boundsPen;
            boundsPen.setColor(color);
            boundsPen.setStyle(Qt::DotLine);
            cov_upper->setPen(boundsPen);
            cov_lower->setPen(boundsPen);

            graphs_[plot][getDataLine(i)] = new_graph;
            graphs_[plot][getUpperBound(i)] = cov_upper;
            graphs_[plot][getLowerBound(i)] = cov_lower;
        }

        return true;
    }
    else
    {
        std::cerr << "Warning! trying to plot same data twice!" << std::endl;
        return false;
    }
}

void GaussianData::plotLines()
{
    std::lock_guard<std::mutex> guard(covariance_mutex_);
    while (!new_data_.empty())
    {
        std::vector<double>& data_vec = new_data_.front().first;
        Eigen::MatrixXd covariance = new_data_.front().second;
        for (auto& pair : graphs_)
        {
            std::vector<QCPGraph*>& graphs = pair.second;
            for (int i = 0; i < dim_; i++)
            {
                double variance = covariance(i, i);
                double sigma3 = 3 * std::sqrt(variance);
                double data = data_vec[i + 1];
                graphs[getDataLine(i)]->addData(data_vec[0], data);
                graphs[getUpperBound(i)]->addData(data_vec[0], data + sigma3);
                graphs[getLowerBound(i)]->addData(data_vec[0], data - sigma3);
            }
        }
        new_data_.pop();
    }
}
