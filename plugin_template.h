/**
 * Copyright 2017 by Institute for Infocomm Research, Singapore (I2R). All rights reserved.
 * @author Ng Kam Pheng (ngkp@i2r.a-star.edu.sg)
 */
#ifndef PLUGIN_TEMPLATE_H
#define PLUGIN_TEMPLATE_H
#include <QWidget>
#include <QPushButton>
#include "../common/basewidget.h"
#include "../../common/mission/mission_completed_data.h"
#include "Tools/console.h"
#include "command_processor/commandprocessor.h"
#include "Tools/robotCommunication.h"

#ifdef USING_COMMANDPUB2
#include "../../common/mission/robot_status_data2.h"
#include "../../common/mission/mission_data2.h"
#else
#include "../../common/mission/robot_status_data.h"
#include "../../common/mission/mission_data.h"
#endif

namespace Ui {
class SHARP;
}

namespace gui_plugin
{

/**
 * This is the class template to implement the GUI plugin
 *
 */
class SHARP : public BaseWidget
{
    Q_OBJECT

public:
    explicit SHARP(BaseWidget *parent = 0);
    ~SHARP();

    void onSetRobotName() Q_DECL_OVERRIDE;

public slots:
    void onRobotBinaryReceived(const int &port_type, const QByteArray &message) Q_DECL_OVERRIDE;
    void onRobotTextReceived(const QJsonObject &jobj) Q_DECL_OVERRIDE;
    void onRobotVideoReceived(const QByteArray &message) Q_DECL_OVERRIDE;

private slots:

    void on_config_button_clicked();

    void on_pushButton_Abort_clicked();

    void executeMQTTCommand(QString command);

    void on_pushButton_Dock_clicked();

    void on_pushButton_Undock_clicked();

    void on_pushButton_deliverCommode_clicked();

    void on_pushButton_cleanCommode_clicked();

    void on_pushButton_openDoor_clicked();

    void on_pushButton_closeDoor_clicked();

    void on_pushButton_safetyOn_clicked();

    void on_pushButton_safetyOff_clicked();

    void on_pushButton_parkCommode_clicked();

    void on_pushButton_initChargingState_clicked();

    void on_pushButton_initStandbyState_clicked();

    void on_pushButton_initIdleState_clicked();

    void on_pushButton_gripperExtend_clicked();

    void on_pushButton_grpperRetract_clicked();

    void on_pushButton_gripperClamp_clicked();

    void on_pushButton_gripperRelease_clicked();

    void on_pushButton_gripperExtendedClamp_clicked();

    void on_pushButton_gripperExtendedRelease_clicked();

    void on_pushButton_initDisabledState_clicked();

signals:
    void mqtt_cb(QString msg);

private:
    /// GUI for this widget
    Ui::SHARP *ui;
    QString config_dir = "";
    bool configured = false;

    QPushButton *current_button = NULL;

    #ifdef USING_COMMANDPUB2
    mission::MissionData2 current_mission_data;
    #else
    mission::MissionData current_mission_data;
    #endif
    mission::MissionCompletedData mission_status_data;

    void OnMissionCompleted(const QJsonObject &jobj);
    void OnMissionSequenceCompleted(bool status);
    int sendMission(QByteArray mission_data);
    void command_callback(std::string msg);

    // Console Object
    Console *console;

    // Command Processor
    CommandProcessor *cmd_processor;
    // Communicator
    RobotCommunication *robot_com;
};

} // gui_plugin

#endif // PLUGIN_TEMPLATE_H
