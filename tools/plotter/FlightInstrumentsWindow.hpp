#pragma once

#include <QTextEdit>
#include <QtCore>
#include <QtGui>

#include <yaml-cpp/yaml.h>
#include <common/messages/groundtruth_inertial_t.hpp>
#include <common/messages/state_t.hpp>
#include <zcm/zcm-cpp.hpp>

#include "FlightInstruments.hpp"

class FlightInstrumentsWindow : public QWidget
{
public:
    FlightInstrumentsWindow(YAML::Node config);
    virtual ~FlightInstrumentsWindow();

    virtual int setupLayout(void);

protected:
    void mousePressEvent(QMouseEvent *event);
    void resizeEvent(QResizeEvent *event);

    void recv(const zcm::ReceiveBuffer *, const std::string &, const state_t *msg);

protected:
    QADI *m_ADI;
    QCompass *m_Compass;
    QKeyValueListView *m_infoList;

    uint64_t last_time;
    zcm::ZCM zcm_;
};
