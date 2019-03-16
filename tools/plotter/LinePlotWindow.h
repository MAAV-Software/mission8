#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <memory>
#include <string>

#include <qcustomplot.h>
#include <QMainWindow>
#include <QTimer>

#include <yaml-cpp/node/detail/bool_type.h>
#include <yaml-cpp/yaml.h>
#include <zcm/zcm-cpp.hpp>

#include <common/messages/global_update_t.hpp>
#include <common/messages/imu_t.hpp>
#include <common/messages/lidar_t.hpp>
#include <common/messages/pid_error_t.hpp>
#include <common/messages/plane_fit_t.hpp>
#include <common/messages/state_t.hpp>
#include <common/utils/ZCMHandler.hpp>

#include "AbstractData.hpp"
#include "DataDict.hpp"
#include "ListWindow.h"

class ListWindow;

namespace Ui
{
class LinePlotWindow;
}

class DerivedQCustomPlot : public QCustomPlot
{
public:
    DerivedQCustomPlot(bool *dragging, QWidget *parent = 0);
    void mousePressEvent(QMouseEvent *event);

private:
    bool *dragging_;
};

class LinePlotWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit LinePlotWindow(const YAML::Node &config_in, std::shared_ptr<DataDict> dict,
        ListWindow *const list_window, QListWidget *listWidget, QWidget *parent = 0);
    ~LinePlotWindow();

    virtual void closeEvent(QCloseEvent *event);

protected:
    void dragEnterEvent(QDragEnterEvent *event);
    void dropEvent(QDropEvent *event);
    void keyPressEvent(QKeyEvent *event);

private slots:
    void real_time_data_slot();
    void draw_slot();
    void setDrag();

private:
    void setup();

    void update_y_axis();
    void update_x_axis();

    double time_manager(uint64_t);
    double time();
    void check_value_range(double);
    void time_value_init();

    Ui::LinePlotWindow *ui;
    QTimer dataTimer;
    QTimer drawTimer;

    YAML::Node config;

    zcm::ZCM zcm;
    ZCMHandler<lidar_t> lidar_handler;
    ZCMHandler<imu_t> imu_handler;
    ZCMHandler<global_update_t> global_update_handler;
    ZCMHandler<plane_fit_t> plane_fit_handler;
    ZCMHandler<state_t> state_handler;
    ZCMHandler<pid_error_t> pid_error_handler;

    uint64_t start_utime;
    uint64_t max_utime;
    uint64_t pause_utime;

    double time_range;
    double time_med;
    bool dragging_;
    double value_range;
    double value_med;
    double max_value;
    double min_value;

    std::shared_ptr<DataDict> dict_;
    ListWindow *const parent_window_;
    QListWidget *listWidget_;

    std::vector<std::shared_ptr<AbstractData>> current_graphs_;

    DerivedQCustomPlot *custom_plot_;
};

#endif  // MAINWINDOW_H
