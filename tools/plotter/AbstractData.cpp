#include "AbstractData.hpp"

#include <cstdlib>
#include <iostream>

AbstractData::AbstractData(int dim, std::vector<std::string> legend) : dim_(dim)
{
    for (const std::string& str : legend)
    {
        legend_.emplace_back(str.c_str());
    }
}

bool AbstractData::createLinePlots(QCustomPlot* plot)
{
    std::lock_guard<std::mutex> guard(abstract_data_mutex);
    // Check if window is not already being plotted on
    if (graphs_.find(plot) == graphs_.end())
    {
        graphs_[plot] = std::vector<QCPGraph*>(dim_);

        // For each dimension, add a graph
        for (int i = 0; i < dim_; i++)
        {
            QCPGraph* new_graph = plot->addGraph();

            // Set legend
            new_graph->setName(legend_[i]);

            // Set looks
            QPen graphPen;
            int cutoff = 50;
            graphPen.setColor(QColor(rand() % (255 - cutoff) + cutoff,
                rand() % (255 - cutoff) + cutoff, rand() % (255 - cutoff) + cutoff));
            graphPen.setWidthF(1);
            new_graph->setPen(graphPen);

            graphs_[plot][i] = new_graph;
        }

        return true;
    }
    else
    {
        std::cerr << "Warning! trying to plot same data twice!" << std::endl;
        return false;
    }
}

void AbstractData::destroyLinePlots(QCustomPlot* plot)
{
    std::lock_guard<std::mutex> guard(abstract_data_mutex);

    // Check that this window is being plotted on
    if (graphs_.find(plot) != graphs_.end())
    {
        // For each dimension, remove the plots
        for (auto* graph : graphs_[plot])
        {
            plot->removeGraph(graph);
        }
        // Remove plots from map
        graphs_.erase(plot);
    }
    else
    {
        throw("Error! We are not plotting on this window!");
    }
}

void AbstractData::createScatterPlots(QCustomPlot* plot) {}

void AbstractData::createParametricPlots(QCustomPlot* plot) {}

void AbstractData::addData(std::vector<double>&& data)
{
    std::lock_guard<std::mutex> guard(abstract_data_mutex);
    new_data_.push(data);
    // std::cout << "Size: " << ?graphs_.back().begin()->second->data()->size() << std::endl;
}

void AbstractData::purgeGraphs()
{
    // graphs_ is vector with each dimension holding
    // maps which map plots to the graph
    for (auto window : graphs_)
    {
        for (auto dim_plot : window.second)
        {
            dim_plot->data()->clear();
        }
    }
}

void AbstractData::plotLines()
{
    std::lock_guard<std::mutex> guard(abstract_data_mutex);
    while (!new_data_.empty())
    {
        std::vector<double>& data_vec = new_data_.front();
        for (auto& pair : graphs_)
        {
            std::vector<QCPGraph*>& graphs = pair.second;
            for (int i = 0; i < dim_; i++)
            {
                graphs[i]->addData(data_vec[0], data_vec[i + 1]);
            }
        }
        new_data_.pop();
    }
}

void AbstractData::plot2DParametric()
{
    if (dim_ != 2) throw("Error bad DIM");
    // TODO: Finish
}

void AbstractData::plot3DParametric()
{
    if (dim_ != 3) throw("Error bad DIM");
    // TODO: Finish
}
void AbstractData::Plot2DScatter()
{
    if (dim_ != 2) throw("Error bad DIM");
    // TODO: Finish
}

void AbstractData::Plot3DScatter()
{
    if (dim_ != 3) throw("Error bad DIM");
    // TODO: Finish
}
