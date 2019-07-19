#include <stdio.h>
#include <cstdlib>
#include <iostream>

#include <QBoxLayout>
#include <QVBoxLayout>
#include <QtCore>
#include <QtGui>

#include <common/messages/MsgChannels.hpp>
#include "ZcmConversion.hpp"

#include "FlightInstrumentsWindow.hpp"

FlightInstrumentsWindow::FlightInstrumentsWindow(YAML::Node config)
    : QWidget(), last_time(0), zcm_{config["zcm-url"].as<std::string>()}
{
    // setup layout
    setupLayout();

    // set default info
    m_infoList->getData()["roll"] = QString("%1").arg(m_ADI->getRoll());
    m_infoList->getData()["pitch"] = QString("%1").arg(m_ADI->getPitch());
    m_infoList->getData()["yaw"] = QString("%1").arg(m_Compass->getYaw());
    m_infoList->getData()["alt"] = QString("%1").arg(m_Compass->getAlt());
    m_infoList->listReload();

    // set window minimum size
    this->setMinimumSize(200, 400);

    // set focus to this window
    this->setFocus();

    // window title
    setWindowTitle("Flight Instruments");

    zcm_.subscribe(maav::STATE_CHANNEL, &FlightInstrumentsWindow::recv, this);
    zcm_.start();
}

FlightInstrumentsWindow::~FlightInstrumentsWindow() {}

int FlightInstrumentsWindow::setupLayout(void)
{
    // left pannel
    QWidget *wLeftPanel = new QWidget(this);
    QVBoxLayout *vl = new QVBoxLayout(wLeftPanel);
    wLeftPanel->setLayout(vl);
    wLeftPanel->setFocusPolicy(Qt::NoFocus);

    m_ADI = new QADI(this);
    m_Compass = new QCompass(this);
    m_infoList = new QKeyValueListView(this);

    vl->addWidget(m_ADI, 0, Qt::AlignTop | Qt::AlignHCenter);
    vl->addWidget(m_Compass, 0, Qt::AlignTop | Qt::AlignHCenter);
    vl->addWidget(m_infoList, 0, 0);
    vl->setMargin(0);
    vl->setSpacing(4);

    // overall layout
    QHBoxLayout *hl = new QHBoxLayout(this);
    this->setLayout(hl);

    hl->addWidget(wLeftPanel, 0);

    return 0;
}

void FlightInstrumentsWindow::mousePressEvent(QMouseEvent *event)
{
    QWidget::mousePressEvent(event);
}

void FlightInstrumentsWindow::recv(
    const zcm::ReceiveBuffer *, const std::string &, const state_t *msg)
{
    uint64_t dt = msg->utime - last_time;
    if (dt > 1000000 / 30)
    {
        last_time = msg->utime;

        Eigen::Quaterniond q{msg->attitude.data[0], msg->attitude.data[1], msg->attitude.data[2],
            msg->attitude.data[3]};
        double roll =
            atan2(2 * q.w() * q.x() + q.y() * q.z(), 1 - 2 * (q.x() * q.x() + 2 * q.y() * q.y()));
        double pitch = asin(2 * (q.w() * q.y() - q.z() * q.x()));
        double heading =
            atan2(2 * q.z() * q.w() - 2 * q.x() * q.y(), 1 - 2 * q.y() * q.y() - 2 * q.z() * q.z());
        m_ADI->setRoll(roll * 180 / M_PI);
        m_ADI->setPitch(pitch * 180 / M_PI);
        m_Compass->setYaw(heading * 180 / M_PI);
        m_Compass->setAlt(-msg->position.data[2]);

        m_infoList->getData()["roll"] = QString("%1").arg(m_ADI->getRoll());
        m_infoList->getData()["pitch"] = QString("%1").arg(m_ADI->getPitch());
        m_infoList->getData()["yaw"] = QString("%1").arg(m_Compass->getYaw());
        m_infoList->getData()["alt"] = QString("%1").arg(m_Compass->getAlt());
        m_infoList->listReload();
    }
}

void FlightInstrumentsWindow::resizeEvent(QResizeEvent *event) { QWidget::resizeEvent(event); }
