#ifndef LISTWINDOW_H
#define LISTWINDOW_H

#include <memory>
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

#include "AbstractData.hpp"
#include "DataDict.hpp"
#include "LinePlotWindow.h"

class LinePlotWindow;

namespace Ui
{
class ListWindow;
}

class ChannelListItem : public QListWidgetItem
{
public:
    ChannelListItem(QString text, QString key, QListWidget* view = 0);

    const QString& getKey() const;

private:
    QString text_;
    QString key_;
    QString type_;
};

class ChannelListWidget : public QListWidget
{
public:
    ChannelListWidget(QWidget* parent = 0);

    void mousePressEvent(QMouseEvent* event);
};

class ListWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit ListWindow(std::shared_ptr<DataDict> dict, YAML::Node config, QWidget* parent = 0);
    ~ListWindow();

    void deleteLinePlotWindow(LinePlotWindow* line_plot_window);

    virtual void closeEvent(QCloseEvent* event);

    void mousePressEvent(QMouseEvent* event);

private:
    void addChannels_generated();

private slots:
    void on_add_clicked();

private:
    Ui::ListWindow* ui_;
    YAML::Node config_;
    std::shared_ptr<DataDict> dict_;
    std::list<std::unique_ptr<LinePlotWindow>> active_windows_;
    ChannelListWidget* list_;
};

#endif  // LISTWINDOW_H
