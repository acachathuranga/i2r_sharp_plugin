/**
 * Copyright 2017 by Institute for Infocomm Research, Singapore (I2R). All rights reserved.
 * @author Ng Kam Pheng (ngkp@i2r.a-star.edu.sg)
 */
#include "plugin_template.h"
#include <QDateTime>
#include <QJsonDocument>
#include <QMessageBox>
#include <QFileDialog>
#include <QFile>
#include <QDir>
#include <QColor>
#include "ui_plugin_template.h"
#include "../../common/fsm_defs.h"
#include <QDebug>
#include <yaml-cpp/yaml.h>
#include <fstream>


namespace gui_plugin
{

/**
 * Class constructor
 * @param parent Parent of this object
 */
SHARP::SHARP(BaseWidget *parent) :
    BaseWidget(parent),
    ui(new Ui::SHARP)
{
    ui->setupUi(this);
    ui->config_path_label->setText("No Configuration Path Specified");

    console = new Console(ui->textEdit_status);
    robot_com = new RobotCommunication(boost::bind(&SHARP::command_callback, this, _1), console);
    console->print("## SUTD Commode Delivery System V1.3 ##");

    cmd_processor = new CommandProcessor(boost::bind(&SHARP::sendMission, this, _1), console, robot_com);
    QObject::connect(this, &gui_plugin::SHARP::mqtt_cb, this, &gui_plugin::SHARP::executeMQTTCommand);

    /// Try to fetch default configuration file directory
    QString filename = QCoreApplication::applicationDirPath() + "/../mission_config.yaml";
    QByteArray bjsonstr;
    QFile file(filename);
    if(file.exists())
    {
        console->print("Mission Configuration file found in " + filename.toStdString());
        YAML::Node config = YAML::LoadFile(filename.toStdString());
        if (config["mission_files_dir"])
        {
            std::string dir = config["mission_files_dir"].as<std::string>();
            dir = (dir.at(0) == '/')? dir : QCoreApplication::applicationDirPath().toStdString() + "/../" + dir;
            console->print("Default mission data directory: " + dir);
            QFileInfo config_directory(QString(dir.c_str()));
            if (config_directory.exists())
            {
                console->print("Using default mission data directory");
                config_dir = QString(dir.c_str());
                configured = true;

                // Clear Dummy states
                on_pushButton_Abort_clicked();
                on_pushButton_Abort_clicked();
                on_pushButton_Abort_clicked();
                // Turn off Obstacle safety at startup
                on_pushButton_safetyOff_clicked();
            }
            else
            {
                console->print("Default mission data directory doesn't exist");
            }

        } else
        {
            console->print("'mission_files_dir' parameter not found in config file");
        }
    }
    else
    {
        console->print("Mission Configuration file 'mission_config.yaml' not found in " + filename.toStdString());
    }
}

/**
 * Class destructor
 *
 */
SHARP::~SHARP()
{
    delete cmd_processor;
    delete robot_com;
    delete console;
    delete ui;
}


/**
 * Called when binary data is received from the command centre
 * @param assigned_port_idx Port index where data is received
 * @param message Binary data received
 */
void SHARP::onRobotBinaryReceived(const int &port_type, const QByteArray &message)
{
    Q_UNUSED(port_type);
    Q_UNUSED(message);
}

/**
 * Called when text data is received from the command centre
 * @param jobj Text message from command centre
 */
void SHARP::onRobotTextReceived(const QJsonObject &jobj)
{
    QJsonObject jobj_header = jobj[K_JSONKEY_HEADER].toObject();
    if (jobj_header.contains(K_JSONKEY_STATUS_ID))
    {
        if (jobj_header[K_JSONKEY_STATUS_ID].isDouble())
        {
            switch (jobj_header[K_JSONKEY_STATUS_ID].toInt())
            {
                case StatusType::kStatusCurrentCompletedSubMission:
                {
                    //OnSubMissionCompleted(jobj);
                    break;
                }
                case StatusType::kStatusStatusPub:
                {
                    //OnRobotStatusReceived(jobj);
                    break;
                }
                default: break;
            }
        }
    }
    else if (jobj_header.contains(K_JSONKEY_RESPONSE))
    {
        if (jobj_header[K_JSONKEY_RESPONSE].isDouble())
        {
            switch (jobj_header[K_JSONKEY_RESPONSE].toInt())
            {
                case Command::kCommandMissionRetrieveCurrent:
                {
                    //OnReceiveCurentMission(jobj);
                    break;
                }
                case Command::kCommandMissionExecute:
                case Command::kCommandMissionAbortActive:
                case Command::kCommandMissionAbortAll:
                case Command::kCommandMissionExecuteJSONMission:
                {
                    OnMissionCompleted(jobj);
                    break;
                }
                default: break;
            }
        }
    }
}

/**
 * Signal emitted when robot's video is received
 * @param message video data received
 */
void SHARP::onRobotVideoReceived(const QByteArray &message)
{
    Q_UNUSED(message);
}


void SHARP::OnMissionCompleted(const QJsonObject &jobj)
{
    if (jobj.contains(K_JSONKEY_PAYLOAD))
    {
        QJsonObject jobj_payload = jobj[K_JSONKEY_PAYLOAD].toObject();
        QJsonDocument jdoc(jobj_payload);
        QString qjsonstr = jdoc.toJson(QJsonDocument::Compact);

        mission::MissionCompletedData mission_complete_data;
        if (mission_complete_data.fromJSONString(qjsonstr.toStdString()))
        {
            /*
            // Fetch mission ID and status
            Json::Value jvalue;
            Json::Reader reader;
            reader.parse(qjsonstr.toStdString(), jvalue);
            int subMissionStatus = kErrorUnknown;
            int subMissionID = 0;
            mission_complete_data.isValidInt(qjsonstr.toStdString(), K_JSONKEY_SUBMISSION_STATUS, subMissionStatus);
            mission_complete_data.isValidInt(jvalue, K_JSONKEY_MISSION_ID, subMissionID);
            */
            cmd_processor->subMissionCompletionCallback(mission_complete_data.getMissionID(), mission_complete_data.getMissionStatus());
            qCritical() << "Successfully decoded mission complete data -" << jobj;
        }
        else
        {
            qCritical() << "Failed to decode mission complete data -" << jobj;
            console->print("Failed to decode mission complete data");
        }

    }
    else
    {
        qCritical() << "Missing payload for received robot status";
        console->print("Missing payload for received sub mission status");
    }
}

/**
 * Called when the name of the robot has been set
 *
 */
void SHARP::onSetRobotName()
{
}

}

int gui_plugin::SHARP::sendMission(QByteArray mission_data)
{
    int mission_id = -1;

    // Parse mission file to JSON mission object
    QJsonParseError error;
    QJsonDocument jdoc_mission = QJsonDocument::fromJson(mission_data, &error);
    if (error.error == QJsonParseError::NoError)
    {
        #ifdef USING_COMMANDPUB2
        mission::MissionData2 current_mission_data;
        #else
        mission::MissionData current_mission_data;
        #endif

        if (current_mission_data.fromJSONString(mission_data.toStdString()))
        {
            mission_id = current_mission_data.getUID();
            emit sendCommand(Command::kCommandMissionExecuteJSONMission,
                             SubCommand::kSubCommandUnknown,
                             "Sending normal mission",
                             jdoc_mission.object());
        }
        else
        {
            qCritical() << "Error decoding current mission sent to robot";
        }
    }
    else
    {
        QMessageBox::warning(this, "ERROR", error.errorString());
    }
    return mission_id;
}

void gui_plugin::SHARP::on_config_button_clicked()
{
    config_dir = QFileDialog::getExistingDirectory(this, "Select Configuration Folder");
    if(config_dir != "")
    {
        ui->config_path_label->setText(config_dir);
        configured = true;
    }

}

void gui_plugin::SHARP::on_pushButton_Abort_clicked()
{
    cmd_processor->cancelMission();
    console->print("Mission Aborted by user!");
    emit sendCommand(Command::kCommandMissionAbortActive,
                     SubCommand::kSubCommandUnknown,
                     "Aborting current mission",
                     QJsonObject());
}

void gui_plugin::SHARP::command_callback(std::string msg)
{
    std::cout << msg << std::endl;
    emit mqtt_cb(QString(msg.c_str()));
}

void gui_plugin::SHARP::executeMQTTCommand(QString command)
{
    // Abort all current missions
    on_pushButton_Abort_clicked();

    // Execute received command
    std::string info = "MQTT Command Received: " + command.toStdString();
    console->print(info);
    if(!configured)
    {
        console->print("ERROR: Configuration directory not set");
    }
    else
    {
        cmd_processor->executeMission(command, config_dir);
    }
}

void gui_plugin::SHARP::OnMissionSequenceCompleted(bool status)
{
    console->print("Mission Sequence Completed");
    if (current_button != NULL)
    {
        QString style = (status)? "" : "background-color: rgb(255, 0, 0)";
        current_button->setStyleSheet(style);
        current_button = NULL;
    }
}

void gui_plugin::SHARP::on_pushButton_Dock_clicked()
{
    if (!configured)
    {
        console->print("Warning: Configuration Directory Not Set");
    }
    console->print("Robot Dock Request");
    Json::Value mission;
    Json::FastWriter writer;
    mission["command"] = "dock";
    cmd_processor->executeMission(QString::fromStdString(writer.write(mission)), config_dir);
}

void gui_plugin::SHARP::on_pushButton_Undock_clicked()
{
    if (!configured)
    {
        console->print("Warning: Configuration Directory Not Set");
    }
    console->print("Robot Undock Request");
    Json::Value mission;
    Json::FastWriter writer;
    mission["command"] = "undock";
    cmd_processor->executeMission(QString::fromStdString(writer.write(mission)), config_dir);
}

void gui_plugin::SHARP::on_pushButton_deliverCommode_clicked()
{
    if (!configured)
    {
        console->print("Warning: Configuration Directory Not Set");
    }
    Json::Value mission;
    Json::FastWriter writer;
    mission["command"] = "deliver";
    mission["bed_id"] = ui->lineEdit_deliveryBed->text().toInt();
    console->print("Commode Deliver Request to bed: " + mission["bed_id"].asString());
    cmd_processor->executeMission(QString::fromStdString(writer.write(mission)), config_dir);
}

void gui_plugin::SHARP::on_pushButton_cleanCommode_clicked()
{
    if (!configured)
    {
        console->print("Warning: Configuration Directory Not Set");
    }
    Json::Value mission;
    Json::FastWriter writer;
    mission["command"] = "collect";
    mission["bed_id"] = ui->lineEdit_collectionBed->text().toInt();
    console->print("Commode Collect Request from bed: " + mission["bed_id"].asString());
    cmd_processor->executeMission(QString::fromStdString(writer.write(mission)), config_dir);
}

void gui_plugin::SHARP::on_pushButton_openDoor_clicked()
{
    console->print("Door Open Command");
    Json::Value mission;
    Json::FastWriter writer;
    mission["command"] = "door_open";
    cmd_processor->executeMission(QString::fromStdString(writer.write(mission)), config_dir);
}

void gui_plugin::SHARP::on_pushButton_closeDoor_clicked()
{
    console->print("Door Close Command");
    Json::Value mission;
    Json::FastWriter writer;
    mission["command"] = "door_close";
    cmd_processor->executeMission(QString::fromStdString(writer.write(mission)), config_dir);
}

void gui_plugin::SHARP::on_pushButton_safetyOn_clicked()
{
    if (!configured)
    {
        console->print("Warning: Configuration Directory Not Set");
    }
    console->print("Turning On Collision Safety");
    Json::Value mission;
    Json::FastWriter writer;
    mission["command"] = "safety_on";
    cmd_processor->executeMission(QString::fromStdString(writer.write(mission)), config_dir);
}

void gui_plugin::SHARP::on_pushButton_safetyOff_clicked()
{
    if (!configured)
    {
        console->print("Warning: Configuration Directory Not Set");
    }
    console->print("Turning Off Collision Safety");
    Json::Value mission;
    Json::FastWriter writer;
    mission["command"] = "safety_off";
    cmd_processor->executeMission(QString::fromStdString(writer.write(mission)), config_dir);
}

void gui_plugin::SHARP::on_pushButton_parkCommode_clicked()
{
    if (!configured)
    {
        console->print("Warning: Configuration Directory Not Set");
    }
    console->print("Collecting Commode and Moving to Parking");
    Json::Value mission;
    Json::FastWriter writer;
    mission["command"] = "park";
    cmd_processor->executeMission(QString::fromStdString(writer.write(mission)), config_dir);
}

void gui_plugin::SHARP::on_pushButton_initChargingState_clicked()
{
    cmd_processor->initRobotState(CommandProcessor::RobotState::Charging);
}

void gui_plugin::SHARP::on_pushButton_initStandbyState_clicked()
{
    cmd_processor->initRobotState(CommandProcessor::RobotState::Standby);
}

void gui_plugin::SHARP::on_pushButton_initIdleState_clicked()
{
    cmd_processor->initRobotState(CommandProcessor::RobotState::Idle);
}

void gui_plugin::SHARP::on_pushButton_initDisabledState_clicked()
{
    cmd_processor->initRobotState(CommandProcessor::RobotState::Disabled);
}

void gui_plugin::SHARP::on_pushButton_gripperExtend_clicked()
{
    if (!configured)
    {
        console->print("Warning: Configuration Directory Not Set");
    }
    console->print("Gripper Extending");
    Json::Value mission;
    Json::FastWriter writer;
    mission["command"] = "gripper_extend";
    cmd_processor->executeMission(QString::fromStdString(writer.write(mission)), config_dir);
}

void gui_plugin::SHARP::on_pushButton_grpperRetract_clicked()
{
    if (!configured)
    {
        console->print("Warning: Configuration Directory Not Set");
    }
    console->print("Gripper Retracting");
    Json::Value mission;
    Json::FastWriter writer;
    mission["command"] = "gripper_retract";
    cmd_processor->executeMission(QString::fromStdString(writer.write(mission)), config_dir);
}

void gui_plugin::SHARP::on_pushButton_gripperClamp_clicked()
{
    if (!configured)
    {
        console->print("Warning: Configuration Directory Not Set");
    }
    console->print("Gripper Clamping");
    Json::Value mission;
    Json::FastWriter writer;
    mission["command"] = "gripper_clamp";
    cmd_processor->executeMission(QString::fromStdString(writer.write(mission)), config_dir);
}

void gui_plugin::SHARP::on_pushButton_gripperRelease_clicked()
{
    if (!configured)
    {
        console->print("Warning: Configuration Directory Not Set");
    }
    console->print("Gripper Releasing");
    Json::Value mission;
    Json::FastWriter writer;
    mission["command"] = "gripper_release";
    cmd_processor->executeMission(QString::fromStdString(writer.write(mission)), config_dir);
}

void gui_plugin::SHARP::on_pushButton_gripperExtendedClamp_clicked()
{
    if (!configured)
    {
        console->print("Warning: Configuration Directory Not Set");
    }
    console->print("Gripper Extended Clamping");
    Json::Value mission;
    Json::FastWriter writer;
    mission["command"] = "gripper_extended_clamp";
    cmd_processor->executeMission(QString::fromStdString(writer.write(mission)), config_dir);
}

void gui_plugin::SHARP::on_pushButton_gripperExtendedRelease_clicked()
{
    if (!configured)
    {
        console->print("Warning: Configuration Directory Not Set");
    }
    console->print("Gripper Extended Releasing");
    Json::Value mission;
    Json::FastWriter writer;
    mission["command"] = "gripper_extended_release";
    cmd_processor->executeMission(QString::fromStdString(writer.write(mission)), config_dir);
}
