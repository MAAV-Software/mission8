#include <fstream>
#include <string>
#include <vector>

#include <mainwindow.h>
#include <ui_mainwindow.h>
#include <QDebug>
#include <QDesktopWidget>
#include <QMessageBox>
#include <QMetaEnum>
#include <QScreen>
#include <QStatusBar>

#include <common/messages/MsgChannels.hpp>

/*
 *
 *        CONSTRUCTOR, DESTRUCTOR, SETUP
 *
 */
void MainWindow::time_value_init()
{
    max_utime = 0;
    pause_utime = 0;
    time_range = 5;
    time_med = 0;
    value_range = 2;
    value_med = 0;
    max_value = time_med + value_range;
    min_value = time_med - value_range;
    start_utime = 0;
    --start_utime;
}

MainWindow::~MainWindow() { delete ui; }
MainWindow::MainWindow(const YAML::Node &config_in, QWidget *parent)
    : QMainWindow(parent),
      ui(new Ui::MainWindow),
      config(config_in),
      zcm(config["zcm-url"].as<std::string>())
{
    time_value_init();

    ui->setupUi(this);
    setup(ui->customPlot);
    setWindowTitle("Maav Plotter");
    statusBar()->clearMessage();
    ui->customPlot->replot();
    ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->customPlot->setOpenGl(true);
    ui->customPlot->legend->setVisible(true);
    ui->customPlot->legend->setBrush(QBrush(QColor(255, 255, 255, 200)));

    zcm.subscribe(maav::HEIGHT_LIDAR_CHANNEL, &ZCMHandler<lidar_t>::recv, &lidar_handler);
    zcm.subscribe(maav::IMU_CHANNEL, &ZCMHandler<imu_t>::recv, &imu_handler);
    zcm.subscribe(
        maav::GLOBAL_UPDATE_CHANNEL, &ZCMHandler<global_update_t>::recv, &global_update_handler);
    zcm.subscribe(maav::PLANE_FIT_CHANNEL, &ZCMHandler<plane_fit_t>::recv, &plane_fit_handler);
    zcm.subscribe(maav::STATE_CHANNEL, &ZCMHandler<state_t>::recv, &state_handler);
    zcm.start();
}

void MainWindow::setup(QCustomPlot *customPlot)
{
    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%m:%s");
    customPlot->xAxis->setTicker(timeTicker);
    customPlot->axisRect()->setupFullAxesBox();

    add_graphs(customPlot);
    set_sliders();
    ui->customPlot->xAxis->setRange(time_med - time_range, time_med + time_range);
    ui->customPlot->yAxis->setRange(value_med - value_range, value_med + value_range);

    connect(customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->xAxis2,
        SLOT(setRange(QCPRange)));
    connect(customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->yAxis2,
        SLOT(setRange(QCPRange)));

    connect(&dataTimer, SIGNAL(timeout()), this, SLOT(real_time_data_slot()));
    dataTimer.start(0);

    connect(&drawTimer, SIGNAL(timeout()), this, SLOT(draw_slot()));
    drawTimer.start(config["draw-timeout"].as<double>());
}

void MainWindow::add_graphs(QCustomPlot *customPlot)
{
    // Add proper number of graphs
    for (int i = 0; i < NUM_GRAPHS; ++i) customPlot->addGraph();

    const YAML::Node &color = config["color"];

    // Set graph and button colors
    color_graph(ui->lidar_button, Graph::LIDAR, color["lidar"].as<std::vector<int>>());
    color_graph(ui->imu_acc_x, Graph::IMU_X, color["imu-x"].as<std::vector<int>>());
    color_graph(ui->imu_acc_y, Graph::IMU_Y, color["imu-y"].as<std::vector<int>>());
    color_graph(ui->imu_acc_z, Graph::IMU_Z, color["imu-z"].as<std::vector<int>>());
    color_graph(ui->imu_yaw_rate, Graph::IMU_YAW, color["imu-yaw"].as<std::vector<int>>());
    color_graph(ui->imu_pitch_rate, Graph::IMU_PITCH, color["imu-pitch"].as<std::vector<int>>());
    color_graph(ui->imu_roll_rate, Graph::IMU_ROLL, color["imu-roll"].as<std::vector<int>>());
    color_graph(ui->pf_pitch, Graph::PF_PITCH, color["pf-pitch"].as<std::vector<int>>());
    color_graph(ui->pf_roll, Graph::PF_ROLL, color["pf-roll"].as<std::vector<int>>());
    color_graph(ui->pf_z, Graph::PF_Z, color["pf-z"].as<std::vector<int>>());
    color_graph(ui->pf_z_dot, Graph::PF_Z_DOT, color["pf-z-dot"].as<std::vector<int>>());
    color_graph(ui->state_pos_x, Graph::STATE_POS_X, color["state-pos-x"].as<std::vector<int>>());
    color_graph(ui->state_pos_y, Graph::STATE_POS_Y, color["state-pos-y"].as<std::vector<int>>());
    color_graph(ui->state_pos_z, Graph::STATE_POS_Z, color["state-pos-z"].as<std::vector<int>>());
    color_graph(ui->state_vel_x, Graph::STATE_VEL_X, color["state-vel-x"].as<std::vector<int>>());
    color_graph(ui->state_vel_y, Graph::STATE_VEL_Y, color["state-vel-y"].as<std::vector<int>>());
    color_graph(ui->state_vel_z, Graph::STATE_VEL_Z, color["state-vel-z"].as<std::vector<int>>());
    color_graph(ui->global_update_x, Graph::GLOBAL_UPDATE_X,
        color["global-update-x"].as<std::vector<int>>());
    color_graph(ui->global_update_y, Graph::GLOBAL_UPDATE_Y,
        color["global-update-y"].as<std::vector<int>>());
    color_graph(ui->global_update_z, Graph::GLOBAL_UPDATE_Z,
        color["global-update-z"].as<std::vector<int>>());
}

void MainWindow::color_graph(QWidget *button, const Graph &graph, const std::vector<int> rgb)
{
    QPen graph_pen(QColor(rgb[0], rgb[1], rgb[2]));
    graph_pen.setWidthF(config["line-width"].as<float>());
    ui->customPlot->graph(static_cast<int>(graph))->setPen(graph_pen);
    ui->customPlot->graph(static_cast<int>(graph))->setName(button->objectName());
}

/*
 *
 *      REALTIME PLOTTING FUNCTIONS
 *
 */
void MainWindow::real_time_data_slot()
{
    plot_lidar();
    plot_imu();
    plot_plane_fit();
    plot_state();
    plot_global_update();

    // make key axis range scroll with the data (at a constant range size of 8):
    if (!ui->pause_button->isChecked())
    {
        time_med = time();
        set_sliders();
    }
}

void MainWindow::draw_slot()
{
    if (!(ui->pause_button->isChecked())) update_x_axis();
    ui->customPlot->replot();
}

void MainWindow::plot_lidar()
{
    if (lidar_handler.ready())
    {
        double elapsed_time = time_manager(lidar_handler.msg().utime);
        if (ui->lidar_button->isChecked())
        {
            ui->customPlot->graph(static_cast<int>(Graph::LIDAR))
                ->addData(elapsed_time, lidar_handler.msg().distance);
            if (auto_y()) check_value_range(lidar_handler.msg().distance);
        }

        // Pop every time message is received to prevent buildup of old messages
        lidar_handler.pop();
    }
}

void MainWindow::plot_imu()
{
    if (imu_handler.ready())
    {
        double elapsed_time = time_manager(imu_handler.msg().utime);
        if (ui->imu_button->isChecked())
        {
            if (ui->imu_acc_x->isChecked())
            {
                ui->customPlot->graph(static_cast<int>(Graph::IMU_X))
                    ->addData(elapsed_time, imu_handler.msg().acceleration[0]);
                if (auto_y()) check_value_range(imu_handler.msg().acceleration[0]);
            }
            if (ui->imu_acc_y->isChecked())
            {
                ui->customPlot->graph(static_cast<int>(Graph::IMU_Y))
                    ->addData(elapsed_time, imu_handler.msg().acceleration[1]);
                if (auto_y()) check_value_range(imu_handler.msg().acceleration[1]);
            }
            if (ui->imu_acc_z->isChecked())
            {
                ui->customPlot->graph(static_cast<int>(Graph::IMU_Z))
                    ->addData(elapsed_time, imu_handler.msg().acceleration[2]);
                if (auto_y()) check_value_range(imu_handler.msg().acceleration[2]);
            }
        }

        if (ui->imu_ang_button->isChecked())
        {
            if (ui->imu_yaw_rate->isChecked())
            {
                ui->customPlot->graph(static_cast<int>(Graph::IMU_YAW))
                    ->addData(elapsed_time, imu_handler.msg().angular_rates[2]);
                if (auto_y()) check_value_range(imu_handler.msg().angular_rates[2]);
            }
            if (ui->imu_pitch_rate->isChecked())
            {
                ui->customPlot->graph(static_cast<int>(Graph::IMU_PITCH))
                    ->addData(elapsed_time, imu_handler.msg().angular_rates[1]);
                if (auto_y()) check_value_range(imu_handler.msg().angular_rates[1]);
            }
            if (ui->imu_roll_rate->isChecked())
            {
                ui->customPlot->graph(static_cast<int>(Graph::IMU_ROLL))
                    ->addData(elapsed_time, imu_handler.msg().angular_rates[0]);
                if (auto_y()) check_value_range(imu_handler.msg().angular_rates[0]);
            }
        }

        // Pop every time message is received to prevent buildup of old messages
        imu_handler.pop();
    }
}

void MainWindow::plot_plane_fit()
{
    if (plane_fit_handler.ready())
    {
        double elapsed_time = time_manager(plane_fit_handler.msg().utime);
        if (ui->pf_angle_button->isChecked())
        {
            if (ui->pf_pitch->isChecked())
            {
                ui->customPlot->graph(static_cast<int>(Graph::PF_PITCH))
                    ->addData(elapsed_time, plane_fit_handler.msg().pitch);
                if (auto_y()) check_value_range(plane_fit_handler.msg().pitch);
            }
            if (ui->pf_roll->isChecked())
            {
                ui->customPlot->graph(static_cast<int>(Graph::PF_ROLL))
                    ->addData(elapsed_time, plane_fit_handler.msg().roll);
                if (auto_y()) check_value_range(plane_fit_handler.msg().roll);
            }
        }

        if (ui->pf_z_button->isChecked())
        {
            if (ui->pf_z->isChecked())
            {
                ui->customPlot->graph(static_cast<int>(Graph::PF_Z))
                    ->addData(elapsed_time, plane_fit_handler.msg().z);
                if (auto_y()) check_value_range(plane_fit_handler.msg().z);
            }
            if (ui->pf_z_dot->isChecked())
            {
                ui->customPlot->graph(static_cast<int>(Graph::PF_Z_DOT))
                    ->addData(elapsed_time, plane_fit_handler.msg().z_dot);
                if (auto_y()) check_value_range(plane_fit_handler.msg().z_dot);
            }
        }

        plane_fit_handler.pop();
    }
}

void MainWindow::plot_state()
{
    if (state_handler.ready())
    {
        double elapsed_time = time_manager(state_handler.msg().utime);

        if (ui->state_pos_button->isChecked())
        {
            if (ui->state_pos_x->isChecked())
            {
                ui->customPlot->graph(static_cast<int>(Graph::STATE_POS_X))
                    ->addData(elapsed_time, state_handler.msg().position[0]);
                if (auto_y()) check_value_range(state_handler.msg().position[0]);
            }
            if (ui->state_pos_y->isChecked())
            {
                ui->customPlot->graph(static_cast<int>(Graph::STATE_POS_Y))
                    ->addData(elapsed_time, state_handler.msg().position[1]);
                if (auto_y()) check_value_range(state_handler.msg().position[1]);
            }
            if (ui->state_pos_z->isChecked())
            {
                ui->customPlot->graph(static_cast<int>(Graph::STATE_POS_Z))
                    ->addData(elapsed_time, state_handler.msg().position[2]);
                if (auto_y()) check_value_range(state_handler.msg().position[2]);
            }
        }

        if (ui->state_vel_button->isChecked())
        {
            if (ui->state_vel_x->isChecked())
            {
                ui->customPlot->graph(static_cast<int>(Graph::STATE_VEL_X))
                    ->addData(elapsed_time, state_handler.msg().velocity[0]);
                if (auto_y()) check_value_range(state_handler.msg().velocity[0]);
            }
            if (ui->state_vel_y->isChecked())
            {
                ui->customPlot->graph(static_cast<int>(Graph::STATE_VEL_Y))
                    ->addData(elapsed_time, state_handler.msg().velocity[1]);
                if (auto_y()) check_value_range(state_handler.msg().velocity[1]);
            }
            if (ui->state_vel_z->isChecked())
            {
                ui->customPlot->graph(static_cast<int>(Graph::STATE_VEL_Z))
                    ->addData(elapsed_time, state_handler.msg().velocity[2]);
                if (auto_y()) check_value_range(state_handler.msg().velocity[2]);
            }
        }

        state_handler.pop();
    }
}

void MainWindow::plot_global_update()
{
    if (global_update_handler.ready())
    {
        double elapsed_time = time_manager(global_update_handler.msg().utime);
        if (ui->global_update_button->isChecked())
        {
            if (ui->global_update_x->isChecked())
            {
                ui->customPlot->graph(static_cast<int>(Graph::GLOBAL_UPDATE_X))
                    ->addData(elapsed_time, global_update_handler.msg().position[0]);
            }
            if (ui->global_update_y->isChecked())
            {
                ui->customPlot->graph(static_cast<int>(Graph::GLOBAL_UPDATE_Y))
                    ->addData(elapsed_time, global_update_handler.msg().position[1]);
            }
            if (ui->global_update_z->isChecked())
            {
                ui->customPlot->graph(static_cast<int>(Graph::GLOBAL_UPDATE_Z))
                    ->addData(elapsed_time, global_update_handler.msg().position[2]);
            }
        }

        global_update_handler.pop();
    }
}

/*
 *
 *      SLOTS
 *
 */

void MainWindow::on_pause_button_clicked()
{
    if (ui->pause_button->isChecked())
        pause_utime = max_utime;
    else if (!ui->pause_button->isChecked())
        max_utime = pause_utime;
}

void MainWindow::on_auto_y_button_toggled()
{
    value_med = 0;
    value_range = 2;
    max_value = 2;
    min_value = -2;
}

static void uncheck(QAbstractButton *button)
{
    if (button->isChecked()) button->click();
}

static void check(QAbstractButton *button)
{
    if (!button->isChecked()) button->click();
}

void MainWindow::on_unselect_all_button_clicked()
{
    uncheck(ui->lidar_button);
    uncheck(ui->imu_button);
    uncheck(ui->imu_ang_button);
    uncheck(ui->pf_angle_button);
    uncheck(ui->pf_z_button);
    uncheck(ui->state_pos_button);
    uncheck(ui->state_vel_button);
    uncheck(ui->global_update_button);
}

void MainWindow::on_select_all_button_clicked()
{
    check(ui->lidar_button);
    check(ui->imu_button);
    check(ui->imu_ang_button);
    check(ui->pf_angle_button);
    check(ui->pf_z_button);
    check(ui->state_pos_button);
    check(ui->state_vel_button);
    check(ui->global_update_button);
}
void MainWindow::on_reset_plots_button_clicked()
{
    for (int i = 0; i < NUM_GRAPHS; ++i)
    {
        ui->customPlot->graph(i)->data()->clear();
    }
    time_value_init();
}

#include <iostream>
void MainWindow::on_log_button_clicked() {}
/*
 *
 *        HELPERS
 *
 */
double MainWindow::time_manager(uint64_t msg_time)
{
    // Sets min time for elapsed time measurement
    // Sets max time for graph range
    if (msg_time < start_utime) start_utime = msg_time;
    if (msg_time > max_utime) max_utime = msg_time;

    // Return elapsed time
    return static_cast<double>(msg_time - start_utime) / 1e6;
}

double MainWindow::time() { return static_cast<double>(max_utime - start_utime) / 1e6; }
void MainWindow::check_value_range(double value)
{
    if (value < min_value)
    {
        min_value = value;
        update_y_axis();
    }
    else if (value > max_value)
    {
        max_value = value;
        update_y_axis();
    }
}

void MainWindow::set_sliders()
{
    ui->bottom_scroll_bar->setRange(-time_range, static_cast<int>(time()) + time_range);
    ui->bottom_scroll_bar->setValue(static_cast<int>(time()));
}

void MainWindow::update_x_axis()
{
    ui->customPlot->xAxis->setRange(time_med - time_range, time_med + time_range);
}

void MainWindow::update_y_axis()
{
    value_med = (max_value + min_value) / 2;
    value_range = max_value - value_med;
    ui->customPlot->yAxis->setRange(min_value, max_value);
}

bool MainWindow::auto_y() { return ui->auto_y_button->isChecked(); }