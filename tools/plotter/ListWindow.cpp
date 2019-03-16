#include <algorithm>
#include <chrono>
#include <experimental/filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <ListWindow.h>
#include <ui_ListWindow.h>
#include <QtCore/QDebug>
#include <QtCore/QMetaEnum>
#include <QtGui/QScreen>
#include <QtWidgets/QDesktopWidget>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QStatusBar>

#include <common/messages/MsgChannels.hpp>

using namespace std::chrono;

ChannelListItem::ChannelListItem(QString text, QString key, QListWidget* view)
    : QListWidgetItem(view), text_(text), key_(key)
{
    QListWidgetItem::setText(text);
}

const QString& ChannelListItem::getKey() const
{
    return key_;
}

ChannelListWidget::ChannelListWidget(QWidget* parent) : QListWidget(parent)
{
}

void ChannelListWidget::mousePressEvent(QMouseEvent* event)
{
    QListWidget::mousePressEvent(event);

    auto position = event->pos();
    if (event->button() == Qt::LeftButton)
    {
        ChannelListItem* item = dynamic_cast<ChannelListItem*>(itemAt(position));

        if (item)
        {
            QDrag* drag = new QDrag(this);
            QMimeData* mimeData = new QMimeData;

            mimeData->setText(item->getKey());
            drag->setMimeData(mimeData);
            // TODO: make look gud
            // drag->setPixmap(iconPixmap);

            drag->exec();
        }
    }
}

ListWindow::ListWindow(std::shared_ptr<DataDict> dict, YAML::Node config, QWidget* parent)
    : QMainWindow(parent), ui_(new Ui::ListWindow), config_(config), dict_(dict)
{
    ui_->setupUi(this);

    list_ = new ChannelListWidget(ui_->centralWidget);
    list_->setObjectName(QString::fromUtf8("listWidget"));
    list_->setGeometry(QRect(10, 70, 256, 651));
    list_->setDragEnabled(true);

    addChannels_generated();
}

ListWindow::~ListWindow()
{
    for (auto& win : active_windows_)
    {
        win->close();
    }
    delete ui_;
}

void ListWindow::closeEvent(QCloseEvent* event)
{
    QApplication::quit();
    event->accept();
}

void ListWindow::on_add_clicked()
{
    active_windows_.emplace_back(new LinePlotWindow(config_, dict_, this, list_));
    active_windows_.back()->show();
}

void ListWindow::deleteLinePlotWindow(LinePlotWindow* line_plot_window)
{
    line_plot_window->close();
    if (active_windows_.size() > 2)
    {
        active_windows_.erase(std::find_if(active_windows_.begin(), active_windows_.end(),
            [line_plot_window](std::unique_ptr<LinePlotWindow>& ptr1) -> bool {
                return ptr1.get() == line_plot_window;
            }));
    }
}

void ListWindow::mousePressEvent(QMouseEvent* event)
{
    auto position = event->pos();
    std::cout << "Click" << std::endl;
    if (event->button() == Qt::LeftButton)
    {
        std::cout << "Left click" << std::endl;
        QListWidgetItem* item = list_->itemAt(position);
        if (item) std::cout << item->text().utf16() << std::endl;
    }
}
