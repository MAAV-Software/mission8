#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <string>

#include <qcustomplot.h>
#include <QMainWindow>
#include <QTimer>

#include <yaml-cpp/yaml.h>
#include <zcm/zcm-cpp.hpp>

#include <common/messages/global_update_t.hpp>
#include <common/messages/imu_t.hpp>
#include <common/messages/lidar_t.hpp>
#include <common/messages/pid_error_t.hpp>
#include <common/messages/plane_fit_t.hpp>
#include <common/messages/state_t.hpp>
#include <common/utils/ZCMHandler.hpp>

namespace Ui
{
class MainWindow;
}

enum class Graph
{
    LIDAR = 0,
    IMU_X,
    IMU_Y,
    IMU_Z,
    IMU_YAW,
    IMU_ROLL,
    IMU_PITCH,
    PF_ROLL,
    PF_PITCH,
    PF_Z,
    PF_Z_DOT,
    STATE_POS_X,
    STATE_POS_Y,
    STATE_POS_Z,
    STATE_VEL_X,
    STATE_VEL_Y,
    STATE_VEL_Z,
    IMU_MAG_X,
    IMU_MAG_Y,
    IMU_MAG_Z,
    GLOBAL_UPDATE_X,
    GLOBAL_UPDATE_Y,
    GLOBAL_UPDATE_Z,
    PID_POS_X,
    PID_POS_Y,
    PID_POS_Z,
    PID_VEL_X,
    PID_VEL_Y,
    PID_VEL_Z,
    COM_THRUST,
    COM_ROLL,
    COM_PITCH,
    STATE_POS_X_U,
    STATE_POS_X_L,
    STATE_POS_Y_U,
    STATE_POS_Y_L,
    STATE_POS_Z_U,
    STATE_POS_Z_L,
    STATE_VEL_X_U,
    STATE_VEL_X_L,
    STATE_VEL_Y_U,
    STATE_VEL_Y_L,
    STATE_VEL_Z_U,
    STATE_VEL_Z_L
};

const int NUM_GRAPHS = 44;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(const YAML::Node &config_in, QWidget *parent = 0);
    ~MainWindow();

private slots:
    void real_time_data_slot();
    void draw_slot();

    void on_pause_button_clicked();
    void on_auto_y_button_toggled();
    void on_unselect_all_button_clicked();
    void on_select_all_button_clicked();
    void on_reset_plots_button_clicked();
    void on_log_button_clicked();

private:
    void setup(QCustomPlot *customPlot);
    void add_graphs(QCustomPlot *customPlot);
    void color_graph(QWidget *, const Graph &, const std::vector<int> rgb);
    void color_graph_bounds(QWidget *, const Graph &, const std::vector<int>);
    void set_sliders();
    void update_y_axis();
    void update_x_axis();

    void plot_lidar();
    void plot_imu();
    void plot_plane_fit();
    void plot_state();
    void plot_global_update();
    void plot_pid();
    void plot(const Graph &graph, double key, double value);

    double time_manager(uint64_t);
    double time();
    void check_value_range(double);
    bool auto_y();
    void time_value_init();

    Ui::MainWindow *ui;
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
    double value_range;
    double value_med;

    double max_value;
    double min_value;
};

#endif  // MAINWINDOW_H
