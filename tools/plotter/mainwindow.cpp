#include <experimental/filesystem>
#include <fstream>
#include <iostream>
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
    ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->customPlot->setOpenGl(true);

    ui->customPlot->legend->setVisible(true);
    ui->customPlot->legend->setBrush(QBrush(QColor(255, 255, 255, 200)));

    ui->customPlot->replot();

    zcm.subscribe(maav::HEIGHT_LIDAR_CHANNEL, &ZCMHandler<lidar_t>::recv, &lidar_handler);
    zcm.subscribe(maav::IMU_CHANNEL, &ZCMHandler<imu_t>::recv, &imu_handler);
    zcm.subscribe(
        maav::GLOBAL_UPDATE_CHANNEL, &ZCMHandler<global_update_t>::recv, &global_update_handler);
    zcm.subscribe(maav::PLANE_FIT_CHANNEL, &ZCMHandler<plane_fit_t>::recv, &plane_fit_handler);
    zcm.subscribe(maav::STATE_CHANNEL, &ZCMHandler<state_t>::recv, &state_handler);
    zcm.subscribe(maav::PID_ERROR_CHANNEL, &ZCMHandler<pid_error_t>::recv, &pid_error_handler);
    zcm.start();
}

void MainWindow::setup(QCustomPlot *customPlot)
{
    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%m:%s");
    customPlot->xAxis->setTicker(timeTicker);
    customPlot->axisRect()->setupFullAxesBox();

    add_graphs(customPlot);
    customPlot->graph(static_cast<int>(Graph::STATE_POS_X))->setAdaptiveSampling(true);
    customPlot->graph(static_cast<int>(Graph::STATE_POS_X_U))->setAdaptiveSampling(true);
    customPlot->graph(static_cast<int>(Graph::STATE_POS_X_L))->setAdaptiveSampling(true);
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
    drawTimer.start(1000. / config["refresh-rate"].as<double>());
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
    color_graph(ui->imu_mag_x, Graph::IMU_MAG_X, color["imu-mag-x"].as<std::vector<int>>());
    color_graph(ui->imu_mag_y, Graph::IMU_MAG_Y, color["imu-mag-y"].as<std::vector<int>>());
    color_graph(ui->imu_mag_z, Graph::IMU_MAG_Z, color["imu-mag-z"].as<std::vector<int>>());
    color_graph_bounds(
        ui->state_pos_x, Graph::STATE_POS_X_U, color["state-pos-x"].as<std::vector<int>>());
    color_graph_bounds(
        ui->state_pos_x, Graph::STATE_POS_X_L, color["state-pos-x"].as<std::vector<int>>());
    color_graph_bounds(
        ui->state_pos_y, Graph::STATE_POS_Y_U, color["state-pos-y"].as<std::vector<int>>());
    color_graph_bounds(
        ui->state_pos_y, Graph::STATE_POS_Y_L, color["state-pos-y"].as<std::vector<int>>());
    color_graph_bounds(
        ui->state_pos_z, Graph::STATE_POS_Z_U, color["state-pos-z"].as<std::vector<int>>());
    color_graph_bounds(
        ui->state_pos_z, Graph::STATE_POS_Z_L, color["state-pos-z"].as<std::vector<int>>());
    color_graph_bounds(
        ui->state_vel_x, Graph::STATE_VEL_X_U, color["state-vel-x"].as<std::vector<int>>());
    color_graph_bounds(
        ui->state_vel_x, Graph::STATE_VEL_X_L, color["state-vel-x"].as<std::vector<int>>());
    color_graph_bounds(
        ui->state_vel_y, Graph::STATE_VEL_Y_U, color["state-vel-y"].as<std::vector<int>>());
    color_graph_bounds(
        ui->state_vel_y, Graph::STATE_VEL_Y_L, color["state-vel-y"].as<std::vector<int>>());
    color_graph_bounds(
        ui->state_vel_z, Graph::STATE_VEL_Z_U, color["state-vel-z"].as<std::vector<int>>());
    color_graph_bounds(
        ui->state_vel_z, Graph::STATE_VEL_Z_L, color["state-vel-z"].as<std::vector<int>>());
    color_graph(ui->global_update_x, Graph::GLOBAL_UPDATE_X,
        color["global-update-x"].as<std::vector<int>>());
    color_graph(ui->global_update_y, Graph::GLOBAL_UPDATE_Y,
        color["global-update-y"].as<std::vector<int>>());
    color_graph(ui->global_update_z, Graph::GLOBAL_UPDATE_Z,
        color["global-update-z"].as<std::vector<int>>());
    color_graph(ui->pid_pos_x, Graph::PID_POS_X, color["pid-pos-x"].as<std::vector<int>>());
    color_graph(ui->pid_pos_y, Graph::PID_POS_Y, color["pid-pos-y"].as<std::vector<int>>());
    color_graph(ui->pid_pos_z, Graph::PID_POS_Z, color["pid-pos-z"].as<std::vector<int>>());
    color_graph(ui->pid_vel_x, Graph::PID_VEL_X, color["pid-vel-x"].as<std::vector<int>>());
    color_graph(ui->pid_vel_y, Graph::PID_VEL_Y, color["pid-vel-y"].as<std::vector<int>>());
    color_graph(ui->pid_vel_z, Graph::PID_VEL_Z, color["pid-vel-z"].as<std::vector<int>>());
    color_graph(ui->com_thrust, Graph::COM_THRUST, color["com-thrust"].as<std::vector<int>>());
    color_graph(ui->com_roll, Graph::COM_ROLL, color["com-roll"].as<std::vector<int>>());
    color_graph(ui->com_pitch, Graph::COM_PITCH, color["com-pitch"].as<std::vector<int>>());
}

void MainWindow::color_graph(QWidget *button, const Graph &graph, const std::vector<int> rgb)
{
    QPen graph_pen(QColor(rgb[0], rgb[1], rgb[2]));
    graph_pen.setWidthF(config["line-width"].as<float>());
    ui->customPlot->graph(static_cast<int>(graph))->setPen(graph_pen);
    ui->customPlot->graph(static_cast<int>(graph))->setName(button->objectName());
}

void MainWindow::color_graph_bounds(QWidget *button, const Graph &graph, const std::vector<int> rgb)
{
    QPen graph_pen(QColor(rgb[0], rgb[1], rgb[2]));
    graph_pen.setStyle(Qt::DashLine);
    graph_pen.setWidthF(config["line-width"].as<float>());
    ui->customPlot->graph(static_cast<int>(graph))->setPen(graph_pen);
    ui->customPlot->graph(static_cast<int>(graph))->removeFromLegend();
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
    plot_pid();

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
    ui->customPlot->replot(QCustomPlot::rpQueuedReplot);
}

void MainWindow::plot_lidar()
{
    if (lidar_handler.ready())
    {
        double et = time_manager(lidar_handler.msg().utime);
        if (ui->lidar_button->isChecked())
        {
            plot(Graph::LIDAR, et, lidar_handler.msg().distance);
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
        const imu_t &msg = imu_handler.msg();
        double et = time_manager(imu_handler.msg().utime);
        const std::vector<double> mag = {msg.magnetometer[0], msg.magnetometer[1], msg.magnetometer[2]};
        if (ui->imu_button->isChecked())
        {
            if (ui->imu_acc_x->isChecked())
            {
                plot(Graph::IMU_X, et, msg.acceleration[0]);
                if (auto_y()) check_value_range(msg.acceleration[0]);
            }
            if (ui->imu_acc_y->isChecked())
            {
                plot(Graph::IMU_Y, et, msg.acceleration[1]);
                if (auto_y()) check_value_range(msg.acceleration[1]);
            }
            if (ui->imu_acc_z->isChecked())
            {
                plot(Graph::IMU_Z, et, msg.acceleration[2]);
                if (auto_y()) check_value_range(msg.acceleration[2]);
            }
        }

        if (ui->imu_ang_button->isChecked())
        {
            if (ui->imu_yaw_rate->isChecked())
            {
                plot(Graph::IMU_YAW, et, msg.angular_rates[2]);
                if (auto_y()) check_value_range(msg.angular_rates[2]);
            }
            if (ui->imu_pitch_rate->isChecked())
            {
                plot(Graph::IMU_PITCH, et, msg.angular_rates[1]);
                if (auto_y()) check_value_range(msg.angular_rates[1]);
            }
            if (ui->imu_roll_rate->isChecked())
            {
                plot(Graph::IMU_ROLL, et, msg.angular_rates[0]);
                if (auto_y()) check_value_range(msg.angular_rates[0]);
            }
        }

        if (ui->imu_mag_button->isChecked())
        {
            if (ui->imu_mag_x->isChecked())
            {
                plot(Graph::IMU_MAG_X, et, mag[0]);
                if (auto_y()) check_value_range(mag[0]);
            }
            if (ui->imu_mag_y->isChecked())
            {
                plot(Graph::IMU_MAG_Y, et, mag[1]);
                if (auto_y()) check_value_range(mag[1]);
            }
            if (ui->imu_mag_z->isChecked())
            {
                plot(Graph::IMU_MAG_Z, et, mag[2]);
                if (auto_y()) check_value_range(mag[2]);
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
        const plane_fit_t &msg = plane_fit_handler.msg();
        double et = time_manager(plane_fit_handler.msg().utime);
        if (ui->pf_angle_button->isChecked())
        {
            if (ui->pf_pitch->isChecked())
            {
                plot(Graph::PF_PITCH, et, msg.pitch);
                if (auto_y()) check_value_range(msg.pitch);
            }
            if (ui->pf_roll->isChecked())
            {
                plot(Graph::PF_ROLL, et, msg.roll);
                if (auto_y()) check_value_range(msg.roll);
            }
        }

        if (ui->pf_z_button->isChecked())
        {
            if (ui->pf_z->isChecked())
            {
                plot(Graph::PF_Z, et, msg.z);
                if (auto_y()) check_value_range(msg.z);
            }
            if (ui->pf_z_dot->isChecked())
            {
                plot(Graph::PF_Z_DOT, et, msg.z_dot);
                if (auto_y()) check_value_range(msg.z_dot);
            }
        }

        plane_fit_handler.pop();
    }
}

void MainWindow::plot_state()
{
    if (state_handler.ready())
    {
        const state_t &msg = state_handler.msg();
        double et = time_manager(state_handler.msg().utime);
        const std::vector<double> pos = {msg.position[0], msg.position[1], msg.position[2]};
        const std::vector<double> vel = {msg.velocity[0], msg.velocity[1], msg.velocity[2]};
        const std::vector<double> pos_sd = {
            sqrt(msg.covariance[3][3]), sqrt(msg.covariance[4][4]), sqrt(msg.covariance[5][5])};
        const std::vector<double> vel_sd = {
            sqrt(msg.covariance[6][6]), sqrt(msg.covariance[7][7]), sqrt(msg.covariance[8][8])};

        double bound_factor = 3;

        if (ui->state_pos_button->isChecked())
        {
            if (ui->state_pos_x->isChecked())
            {
                plot(Graph::STATE_POS_X, et, pos[0]);

                if (auto_y()) check_value_range(pos[0]);

                if (ui->pos_error_bounds->isChecked())
                {
                    plot(Graph::STATE_POS_X_U, et, pos[0] + bound_factor * pos_sd[0]);
                    plot(Graph::STATE_POS_X_L, et, pos[0] - bound_factor * pos_sd[0]);
                }
            }
            if (ui->state_pos_y->isChecked())
            {
                plot(Graph::STATE_POS_Y, et, pos[1]);
                if (auto_y()) check_value_range(pos[1]);

                if (ui->pos_error_bounds->isChecked())
                {
                    plot(Graph::STATE_POS_Y_U, et, pos[1] + bound_factor * pos_sd[1]);
                    plot(Graph::STATE_POS_Y_L, et, pos[1] - bound_factor * pos_sd[1]);
                }
            }
            if (ui->state_pos_z->isChecked())
            {
                plot(Graph::STATE_POS_Z, et, pos[2]);
                if (auto_y()) check_value_range(pos[2]);

                if (ui->pos_error_bounds->isChecked())
                {
                    plot(Graph::STATE_POS_Z_U, et, pos[2] + bound_factor * pos_sd[2]);
                    plot(Graph::STATE_POS_Z_L, et, pos[2] - bound_factor * pos_sd[2]);
                }
            }
        }

        if (ui->state_vel_button->isChecked())
        {
            if (ui->state_vel_x->isChecked())
            {
                plot(Graph::STATE_VEL_X, et, vel[0]);
                if (auto_y()) check_value_range(vel[0]);

                if (ui->vel_error_bounds->isChecked())
                {
                    plot(Graph::STATE_VEL_X_U, et, vel[0] + (bound_factor * vel_sd[0]));
                    plot(Graph::STATE_VEL_X_L, et, vel[0] - (bound_factor * vel_sd[0]));
                }
            }
            if (ui->state_vel_y->isChecked())
            {
                plot(Graph::STATE_VEL_Y, et, vel[1]);
                if (auto_y()) check_value_range(vel[1]);

                if (ui->vel_error_bounds->isChecked())
                {
                    plot(Graph::STATE_VEL_Y_U, et, vel[1] + bound_factor * vel_sd[1]);
                    plot(Graph::STATE_VEL_Y_L, et, vel[1] - bound_factor * vel_sd[1]);
                }
            }
            if (ui->state_vel_z->isChecked())
            {
                plot(Graph::STATE_VEL_Z, et, vel[2]);
                if (auto_y()) check_value_range(vel[2]);

                if (ui->vel_error_bounds->isChecked())
                {
                    plot(Graph::STATE_VEL_Z_U, et, vel[2] + bound_factor * vel_sd[2]);
                    plot(Graph::STATE_VEL_Z_L, et, vel[2] - bound_factor * vel_sd[2]);
                }
            }
        }

        state_handler.pop();
    }
}

void MainWindow::plot_global_update()
{
    if (global_update_handler.ready())
    {
        const global_update_t &msg = global_update_handler.msg();
        double et = time_manager(global_update_handler.msg().utime);
        if (ui->global_update_button->isChecked())
        {
            if (ui->global_update_x->isChecked())
            {
                plot(Graph::GLOBAL_UPDATE_X, et, msg.position[0]);
                if (auto_y()) check_value_range(msg.position[0]);
            }
            if (ui->global_update_y->isChecked())
            {
                plot(Graph::GLOBAL_UPDATE_Y, et, msg.position[1]);
                if (auto_y()) check_value_range(msg.position[1]);
            }
            if (ui->global_update_z->isChecked())
            {
                plot(Graph::GLOBAL_UPDATE_Z, et, msg.position[2]);
                if (auto_y()) check_value_range(msg.position[2]);
            }
        }

        global_update_handler.pop();
    }
}

void MainWindow::plot_pid()
{
    if (pid_error_handler.ready())
    {
        const pid_error_t &msg = pid_error_handler.msg();
        double et = time_manager(pid_error_handler.msg().utime);
        if (ui->pid_pos_button->isChecked())
        {
            if (ui->pid_pos_x->isChecked())
            {
                plot(Graph::PID_POS_X, et, msg.pos_error[0]);
                if (auto_y()) check_value_range(msg.pos_error[0]);
            }
            if (ui->pid_pos_y->isChecked())
            {
                plot(Graph::PID_POS_Y, et, msg.pos_error[1]);
                if (auto_y()) check_value_range(msg.pos_error[1]);
            }
            if (ui->pid_pos_z->isChecked())
            {
                plot(Graph::PID_POS_Z, et, msg.pos_error[2]);
                if (auto_y()) check_value_range(msg.pos_error[2]);
            }
        }

        if (ui->pid_vel_button->isChecked())
        {
            if (ui->pid_vel_x->isChecked())
            {
                plot(Graph::PID_VEL_X, et, msg.vel_error[0]);
                if (auto_y()) check_value_range(msg.vel_error[0]);
            }
            if (ui->pid_vel_y->isChecked())
            {
                plot(Graph::PID_VEL_Y, et, msg.vel_error[1]);
                if (auto_y()) check_value_range(msg.vel_error[1]);
            }
            if (ui->pid_vel_z->isChecked())
            {
                plot(Graph::PID_VEL_Z, et, msg.vel_error[2]);
                if (auto_y()) check_value_range(msg.vel_error[2]);
            }
        }

        if (ui->com_button->isChecked())
        {
            if (ui->com_thrust->isChecked())
            {
                plot(Graph::COM_THRUST, et, msg.thrust);
                if (auto_y()) check_value_range(msg.thrust);
            }
            if (ui->com_roll->isChecked())
            {
                plot(Graph::COM_ROLL, et, msg.roll);
                if (auto_y()) check_value_range(msg.roll);
            }
            if (ui->com_pitch->isChecked())
            {
                plot(Graph::COM_PITCH, et, msg.pitch);
                if (auto_y()) check_value_range(msg.pitch);
            }
        }

        pid_error_handler.pop();
    }
}

void MainWindow::plot(const Graph &graph, double key, double value)
{
    ui->customPlot->graph(static_cast<int>(graph))->addData(key, value);
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
    uncheck(ui->imu_mag_button);
    uncheck(ui->global_update_button);
    uncheck(ui->com_button);
    uncheck(ui->pid_pos_button);
    uncheck(ui->pid_vel_button);
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
    check(ui->imu_mag_button);
    check(ui->global_update_button);
    check(ui->com_button);
    check(ui->pid_pos_button);
    check(ui->pid_vel_button);
}
void MainWindow::on_reset_plots_button_clicked()
{
    for (int i = 0; i < NUM_GRAPHS; ++i)
    {
        ui->customPlot->graph(i)->data()->clear();
    }
    time_value_init();
}

void MainWindow::on_log_button_clicked()
{
    using namespace std::experimental;

    std::string logs_path = QCoreApplication::applicationDirPath().toUtf8().constData();
    logs_path = logs_path.erase(logs_path.find("bin"), -1) + std::string("logs/");

    std::ofstream lidar_log(logs_path + std::string("lidar.log"));
    lidar_log << "sec,height\n";
    for (const auto &lidar_height :
        *(ui->customPlot->graph(static_cast<int>(Graph::LIDAR))->data()))
    {
        lidar_log << lidar_height.key << ',' << lidar_height.value << '\n';
    }
    lidar_log.close();

    std::ofstream imu_accel_log(logs_path + std::string("imu_acceleration.log"));
    imu_accel_log << "sec,accel_x,accel_y,accel_z\n";
    auto &imu_x = *(ui->customPlot->graph(static_cast<int>(Graph::IMU_X))->data());
    auto &imu_y = *(ui->customPlot->graph(static_cast<int>(Graph::IMU_Y))->data());
    auto &imu_z = *(ui->customPlot->graph(static_cast<int>(Graph::IMU_Z))->data());
    const QCPGraphData *x = imu_x.begin();
    const QCPGraphData *y = imu_y.begin();
    const QCPGraphData *z = imu_z.begin();
    for (; x != imu_x.end() && y != imu_x.end() && z != imu_z.end(); ++x, ++y, ++z)
    {
        if (x->key != y->key || y->key != z->key || x->key != z->key)
        {
            std::cout << "IMU ACCLERATION VALUES NOT SYNCHRONIZED\n";
        }
        imu_accel_log << x->key << ',' << x->value << ',' << y->value << ',' << z->value << '\n';
    }
    imu_accel_log.close();

    std::ofstream imu_angl_log(logs_path + std::string("imu_angular_rates.log"));
    imu_angl_log << "sec,yaw,pitch,roll\n";
    auto &imu_yaw = *(ui->customPlot->graph(static_cast<int>(Graph::IMU_YAW))->data());
    auto &imu_pitch = *(ui->customPlot->graph(static_cast<int>(Graph::IMU_PITCH))->data());
    auto &imu_roll = *(ui->customPlot->graph(static_cast<int>(Graph::IMU_ROLL))->data());
    const QCPGraphData *yaw = imu_yaw.begin();
    const QCPGraphData *pitch = imu_pitch.begin();
    const QCPGraphData *roll = imu_roll.begin();
    for (; yaw != imu_yaw.end() && pitch != imu_pitch.end() && roll != imu_roll.end();
         ++yaw, ++pitch, ++roll)
    {
        if (yaw->key != pitch->key && yaw->key != roll->key && pitch->key != roll->key)
        {
            std::cout << "IMU ANGLE RATES VALUES NOT SYNCHRONIZED\n";
        }
        imu_angl_log << yaw->key << ',' << yaw->value << ',' << pitch->value << ',' << roll->value
                     << '\n';
    }
    imu_angl_log.close();

    std::ofstream pf_z_log(logs_path + std::string("plane_fit_z.log"));
    pf_z_log << "sec,z,z_dot\n";
    auto &pf_z = *(ui->customPlot->graph(static_cast<int>(Graph::PF_Z))->data());
    auto &pf_z_dot = *(ui->customPlot->graph(static_cast<int>(Graph::PF_Z_DOT))->data());
    const QCPGraphData *pfz = pf_z.begin();
    const QCPGraphData *pfzdot = pf_z_dot.begin();
    for (; pfz != pf_z.end() && pfzdot != pf_z_dot.end(); ++pfz, ++pfzdot)
    {
        if (pfz->key != pfzdot->key) std::cout << "PLANE FIT Z VALUES NOT SYNCHRONIZED\n";
        pf_z_log << pfz->key << ',' << pfz->value << ',' << pfzdot->value << '\n';
    }
    pf_z_log.close();

    std::ofstream pf_ang_log(logs_path + std::string("plane_fit_ang.log"));
    pf_ang_log << "sec,roll,pitch\n";
    auto &pf_roll = *(ui->customPlot->graph(static_cast<int>(Graph::PF_ROLL))->data());
    auto &pf_pitch = *(ui->customPlot->graph(static_cast<int>(Graph::PF_PITCH))->data());
    const QCPGraphData *pfroll = pf_roll.begin();
    const QCPGraphData *pfpitch = pf_pitch.begin();
    for (; pfroll != pf_roll.end() && pfpitch != pf_pitch.end(); ++pfroll, ++pfpitch)
    {
        if (pfroll->key != pfpitch->key) std::cout << "PLANE FIT ANGLE VALUES NOT SYNCHRONIZED\n";
        pf_ang_log << pfroll->value << ',' << pfpitch->value << '\n';
    }
    pf_ang_log.close();

    std::ofstream state_pos_log(logs_path + std::string("state_pos.log"));
    state_pos_log << "sec,x,y,z\n";
    auto &state_x = *(ui->customPlot->graph(static_cast<int>(Graph::STATE_POS_X))->data());
    auto &state_y = *(ui->customPlot->graph(static_cast<int>(Graph::STATE_POS_Y))->data());
    auto &state_z = *(ui->customPlot->graph(static_cast<int>(Graph::STATE_POS_Z))->data());
    const QCPGraphData *sx = state_x.begin();
    const QCPGraphData *sy = state_y.begin();
    const QCPGraphData *sz = state_z.begin();
    for (; sx != state_x.end() && sy != state_x.end() && sz != state_x.end(); ++sx, ++sy, ++sz)
    {
        if (sx->key != sy->key || sy->key != sz->key || sx->key != sz->key)
        {
            std::cout << "STATE POSITION VALUES NOT SYNCHRONIZED\n";
            break;
        }
        state_pos_log << sx->key << ',' << sx->value << ',' << sy->value << ',' << sz->value
                      << '\n';
    }
    state_pos_log.close();

    std::ofstream state_vel_log(logs_path + std::string("state_vel.log"));
    state_vel_log << "sec,x,y,z\n";
    auto &state_vx = *(ui->customPlot->graph(static_cast<int>(Graph::STATE_VEL_X))->data());
    auto &state_vy = *(ui->customPlot->graph(static_cast<int>(Graph::STATE_VEL_Y))->data());
    auto &state_vz = *(ui->customPlot->graph(static_cast<int>(Graph::STATE_VEL_Z))->data());
    const QCPGraphData *svx = state_vx.begin();
    const QCPGraphData *svy = state_vy.begin();
    const QCPGraphData *svz = state_vz.begin();
    for (; svx != state_vx.end() && svy != state_vx.end() && svz != state_vx.end();
         ++svx, ++svy, ++svz)
    {
        if (svx->key != svy->key || svy->key != svz->key || svx->key != svz->key)
        {
            std::cout << "STATE VELOCITY VALUES NOT SYNCHRONIZED\n";
            break;
        }
        state_vel_log << svx->key << ',' << svx->value << ',' << svy->value << ',' << svz->value
                      << '\n';
    }
    state_vel_log.close();

    std::ofstream imu_mag_log(logs_path + std::string("imu_mag.log"));
    imu_mag_log << "sec,x,y,z\n";
    auto &state_mx = *(ui->customPlot->graph(static_cast<int>(Graph::IMU_MAG_X))->data());
    auto &state_my = *(ui->customPlot->graph(static_cast<int>(Graph::IMU_MAG_Y))->data());
    auto &state_mz = *(ui->customPlot->graph(static_cast<int>(Graph::IMU_MAG_Z))->data());
    const QCPGraphData *smx = state_mx.begin();
    const QCPGraphData *smy = state_my.begin();
    const QCPGraphData *smz = state_mz.begin();
    for (; smx != state_mx.end() && smy != state_mx.end() && smz != state_mx.end();
         ++smx, ++smy, ++smz)
    {
        if (smx->key != smy->key || smy->key != smz->key || smx->key != smz->key)
        {
            std::cout << "STATE MAGNET VALUES NOT SYNCHRONIZED\n";
            break;
        }
        imu_mag_log << smx->key << ',' << smx->value << ',' << smy->value << ',' << smz->value
                      << '\n';
    }
    imu_mag_log.close();

    std::ofstream global_pos_log(logs_path + std::string("global_pos.log"));
    global_pos_log << "sec,x,y,z\n";
    auto &global_x = *(ui->customPlot->graph(static_cast<int>(Graph::GLOBAL_UPDATE_X))->data());
    auto &global_y = *(ui->customPlot->graph(static_cast<int>(Graph::GLOBAL_UPDATE_Y))->data());
    auto &global_z = *(ui->customPlot->graph(static_cast<int>(Graph::GLOBAL_UPDATE_Z))->data());
    const QCPGraphData *gx = global_x.begin();
    const QCPGraphData *gy = global_y.begin();
    const QCPGraphData *gz = global_z.begin();
    for (; gx != global_x.end() && gy != global_x.end() && gz != global_x.end(); ++gx, ++gy, ++gz)
    {
        if (gx->key != gy->key || gy->key != gz->key || gx->key != gz->key)
        {
            std::cout << "GLOBAL POSITION VALUES NOT SYNCHRONIZED\n";
            break;
        }
        global_pos_log << gx->key << ',' << gx->value << ',' << gy->value << ',' << gz->value
                       << '\n';
    }
    global_pos_log.close();

    std::ofstream pos_error_log(logs_path + std::string("pid_pos_error.log"));
    pos_error_log << "sec,x,y,z\n";
    auto &pos_error_x = *(ui->customPlot->graph(static_cast<int>(Graph::PID_POS_X))->data());
    auto &pos_error_y = *(ui->customPlot->graph(static_cast<int>(Graph::PID_POS_Y))->data());
    auto &pos_error_z = *(ui->customPlot->graph(static_cast<int>(Graph::PID_POS_Z))->data());
    const QCPGraphData *px = pos_error_x.begin();
    const QCPGraphData *py = pos_error_y.begin();
    const QCPGraphData *pz = pos_error_z.begin();
    for (; px != pos_error_x.end() && py != pos_error_y.end() && pz != pos_error_z.end();
         ++px, ++py, ++pz)
    {
        if (px->key != py->key || py->key != pz->key || px->key != pz->key)
        {
            std::cout << "POS ERROR VALUES NOT SYNCHRONIZED\n";
            break;
        }
        pos_error_log << px->key << ',' << px->value << ',' << py->value << ',' << pz->value
                      << '\n';
    }
    pos_error_log.close();

    std::ofstream vel_error_log(logs_path + std::string("pid_vel_error.log"));
    vel_error_log << "sec,x,y,z\n";
    auto &vel_error_x = *(ui->customPlot->graph(static_cast<int>(Graph::PID_VEL_X))->data());
    auto &vel_error_y = *(ui->customPlot->graph(static_cast<int>(Graph::PID_VEL_Y))->data());
    auto &vel_error_z = *(ui->customPlot->graph(static_cast<int>(Graph::PID_VEL_Z))->data());
    const QCPGraphData *evx = vel_error_x.begin();
    const QCPGraphData *evy = vel_error_y.begin();
    const QCPGraphData *evz = vel_error_z.begin();
    for (; evx != vel_error_x.end() && evy != vel_error_y.end() && evz != vel_error_z.end();
         ++evx, ++evy, ++evz)
    {
        if (evx->key != evy->key || evy->key != evz->key || evx->key != evz->key)
        {
            std::cout << "VEL ERROR VALUES NOT SYNCHRONIZED\n";
            break;
        }
        vel_error_log << evx->key << ',' << evx->value << ',' << evy->value << ',' << evz->value
                      << '\n';
    }
    vel_error_log.close();

    std::ofstream com_val_log(logs_path + std::string("commanded_values.log"));
    com_val_log << "sec,thrust,roll,pitch\n";
    auto &com_thrust = *(ui->customPlot->graph(static_cast<int>(Graph::COM_THRUST))->data());
    auto &com_roll = *(ui->customPlot->graph(static_cast<int>(Graph::COM_ROLL))->data());
    auto &com_pitch = *(ui->customPlot->graph(static_cast<int>(Graph::COM_PITCH))->data());
    const QCPGraphData *ct = com_thrust.begin();
    const QCPGraphData *cr = com_roll.begin();
    const QCPGraphData *cp = com_pitch.begin();
    for (; ct != com_thrust.end() && cr != com_roll.end() && cp != com_pitch.end();
         ++ct, ++cr, ++cp)
    {
        if (ct->key != cr->key || ct->key != cp->key || cr->key != cp->key)
        {
            std::cout << "COMMANDED VALUES NOT SYNCHRONIZED\n";
            break;
        }
        com_val_log << ct->key << ',' << ct->value << ',' << cr->value << ',' << cp->value << '\n';
    }
    com_val_log.close();
}

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