#include <experimental/filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <LinePlotWindow.h>
#include <ui_LinePlotWindow.h>
#include <QtCore/QDebug>
#include <QtCore/QMetaEnum>
#include <QtGui/QScreen>
#include <QtWidgets/QDesktopWidget>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QStatusBar>

#include <common/messages/MsgChannels.hpp>

DerivedQCustomPlot::DerivedQCustomPlot(bool *dragging, QWidget *parent)
    : QCustomPlot(parent), dragging_(dragging)
{
}

void DerivedQCustomPlot::mousePressEvent(QMouseEvent *event)
{
    *dragging_ = true;
    QCustomPlot::mousePressEvent(event);
}

/*
 *
 *        CONSTRUCTOR, DESTRUCTOR, SETUP
 *
 */
void LinePlotWindow::time_value_init()
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

LinePlotWindow::~LinePlotWindow()
{
    delete ui;
}

LinePlotWindow::LinePlotWindow(const YAML::Node &config_in, std::shared_ptr<DataDict> dict,
    ListWindow *const list_window, QListWidget *listWidget, QWidget *parent)
    : QMainWindow(parent),
      ui(new Ui::LinePlotWindow),
      config(config_in),
      zcm(config["zcm-url"].as<std::string>()),
      dragging_(false),
      dict_(dict),
      parent_window_(list_window),
      listWidget_(listWidget)

{
    ui->setupUi(this);

    // Goat rodeo with derived class for mouse clicks
    custom_plot_ = new DerivedQCustomPlot(&dragging_, ui->plot_area);
    custom_plot_->setObjectName(QString::fromUtf8("custom_plot_"));
    ui->verticalLayout->addWidget(custom_plot_);

    time_value_init();

    setup();
    setWindowTitle("Maav Plotter");
    statusBar()->clearMessage();
    custom_plot_->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    custom_plot_->setOpenGl(true);

    custom_plot_->legend->setVisible(true);
    custom_plot_->legend->setBrush(QBrush(QColor(255, 255, 255, 200)));
    custom_plot_->setNoAntialiasingOnDrag(true);

    custom_plot_->replot();

    setAcceptDrops(true);

    zcm.start();
}

void LinePlotWindow::setup()
{
    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%m:%s");
    custom_plot_->xAxis->setTicker(timeTicker);
    custom_plot_->axisRect()->setupFullAxesBox();

    custom_plot_->xAxis->setRange(time_med - time_range, time_med + time_range);
    custom_plot_->yAxis->setRange(value_med - value_range, value_med + value_range);

    connect(custom_plot_->xAxis, SIGNAL(rangeChanged(QCPRange)), custom_plot_->xAxis2,
        SLOT(setRange(QCPRange)));
    connect(custom_plot_->yAxis, SIGNAL(rangeChanged(QCPRange)), custom_plot_->yAxis2,
        SLOT(setRange(QCPRange)));

    connect(&dataTimer, SIGNAL(timeout()), this, SLOT(real_time_data_slot()));
    dataTimer.start(0);

    connect(&drawTimer, SIGNAL(timeout()), this, SLOT(draw_slot()));
    drawTimer.start(1000. / config["refresh-rate"].as<double>());
}

/*
 *
 *      REALTIME PLOTTING FUNCTIONS
 *
 */
void LinePlotWindow::real_time_data_slot()
{
}

void LinePlotWindow::draw_slot()
{
    if (!dragging_)
    {
        int gc = custom_plot_->graphCount();
        if (gc > 0)
        {
            time_med = (custom_plot_->graph(0)->data()->end() - 1)->key;
        }
        update_x_axis();
    }
    custom_plot_->replot(QCustomPlot::rpQueuedReplot);
}

void LinePlotWindow::setDrag()
{
    if (!dragging_) dragging_ = true;
}

/*
 *
 *        HELPERS
 *
 */
double LinePlotWindow::time_manager(uint64_t msg_time)
{
    // Sets min time for elapsed time measurement
    // Sets max time for graph range
    if (msg_time < start_utime) start_utime = msg_time;
    if (msg_time > max_utime) max_utime = msg_time;

    // Return elapsed time
    return static_cast<double>(msg_time - start_utime) / 1e6;
}

double LinePlotWindow::time()
{
    return static_cast<double>(max_utime - start_utime) / 1e6;
}
void LinePlotWindow::check_value_range(double value)
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

void LinePlotWindow::update_x_axis()
{
    custom_plot_->xAxis->setRange(time_med - time_range, time_med + time_range);
}

void LinePlotWindow::update_y_axis()
{
    value_med = (max_value + min_value) / 2;
    value_range = max_value - value_med;
    custom_plot_->yAxis->setRange(min_value, max_value);
}

void LinePlotWindow::closeEvent(QCloseEvent *event)
{
    for (auto ptr : current_graphs_)
    {
        ptr->destroyLinePlots(custom_plot_);
    }
    parent_window_->deleteLinePlotWindow(this);
    event->accept();
}

void LinePlotWindow::keyPressEvent(QKeyEvent *event)
{
    std::cout << "Key press: setting dragging to false\n" << std::endl;
    dragging_ = false;
    QWidget::keyPressEvent(event);
}

void LinePlotWindow::dragEnterEvent(QDragEnterEvent *event)
{
    event->acceptProposedAction();
}

void LinePlotWindow::dropEvent(QDropEvent *event)
{
    std::string mimeData = event->mimeData()->text().toStdString();
    size_t colon_index = mimeData.find(':');
    std::string key = mimeData.substr(0, colon_index);
    std::string type = mimeData.substr(colon_index + 1);
    std::cout << "Key: " << key << std::endl;
    std::cout << "Type: " << type << std::endl;
    if (type == "vector1_t" || type == "vector2_t" || type == "vector3_t" || type == "vector4_t")
    {
        if (dict_->dict[key]->createLinePlots(custom_plot_))
        {
            current_graphs_.push_back(dict_->dict[key]);
        }
    }
    event->acceptProposedAction();
}