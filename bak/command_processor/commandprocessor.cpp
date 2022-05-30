#include "commandprocessor.h"

CommandProcessor::CommandProcessor(boost::function<int (QByteArray)> sendMission, Console *console, RobotCommunication *com)
{
    sendMission_ = sendMission;
    console_ = console;
    com_ = com;

    sub_mission_generator = new SubMissionGenerator(boost::bind(&CommandProcessor::sendMission_, this, _1),
                                                    console_,
                                                    boost::bind(&CommandProcessor::publishMsg, this, _1, _2, _3),
                                                    boost::bind(&CommandProcessor::publishBool, this, _1, _2, _3));
}

void CommandProcessor::executeMission(QString mission_cmd, QString data_path, boost::function<void (bool)> completionCallback)
{
    completionCallback_ = completionCallback;
    executeMission(mission_cmd, data_path);
}

void CommandProcessor::executeMission(QString mission_cmd, QString data_path)
{
    mission_file_directory_ = data_path;
    task_manager_command = mission_cmd;

    Json::Value message;
    Json::Reader reader;
    if ( reader.parse( mission_cmd.toStdString(), message ) == false )
    {
        console_->print("Command Processor: Cannot Decode incoming JSON Command: " + mission_cmd.toStdString());
        return;
    }
    else
    {
        if (message["command"] == "cancel")
        {
            revert_changes_ = true;
            cancelMission();
        }
        else if (message["command"] == "stop")
        {
            // Nothing to do
        }
    }
    this->start();
}

void CommandProcessor::run()
{
    taskManager(task_manager_command);
}

bool CommandProcessor::publishMsg (QString topic, QString field, QString msg)
{
    com_->publish(topic.toStdString(), field.toStdString(), msg.toStdString());
}

bool CommandProcessor::publishBool (QString topic, QString field, bool value)
{
    com_->publish(topic.toStdString(), field.toStdString(), value);
}

bool CommandProcessor::sendTask(QString file_name)
{
    bool success = false;
    QString filename = mission_file_directory_ + "/" + file_name + ".txt";
    QByteArray bjsonstr;
    QFile file(filename);
    if(file.exists())
    {
        if (file.open(QIODevice::ReadOnly))
        {
            // Read Mission File
            bjsonstr = file.readAll();
            file.close();
            mission_id_ = sendMission_(bjsonstr);
            response_received_ = false;

            std::unique_lock<std::mutex> lck(mtx_);
            if(!taskCondition.wait_for(lck, std::chrono::minutes(10), [&]{return response_received_;}))
            {
                console_->print("Error: Timeout waiting for mission completion. MissionID: " + std::to_string(mission_id_));
                return false;
            }
            else
            {
                if (mission_response_id_ == mission_id_)
                {
                    // Refer fsm_defs.h (I2R Communication Protocol Constants)
                    if(mission_status_ == kErrorNone)
                    {
                        console_->print("Mission " + filename.toStdString() + " completed successfully");
                        success = true;
                    }
                    else
                    {
                        console_->print("ERROR: Mission " + filename.toStdString() + " failed");
                    }
                }
                else
                {
                    console_->print("Error: Response mismatch. MissionID: " + std::to_string(mission_id_) + "\tResponseID: " + std::to_string(mission_response_id_));
                }
            }
        }
        else
        {
            console_->print("Error Trying to send mission file " + filename.toStdString() + ". Cannot Open File!");
        }

    }
    else
    {
        console_->print("Error Trying to send mission file " + filename.toStdString() + ". File Not Found!");
    }

    return success;
}

void CommandProcessor::subMissionCompletionCallback(int sub_mission_id, int sub_mission_status)
{
    mission_response_id_ = sub_mission_id;
    mission_status_ = sub_mission_status;
    response_received_ = true;
    taskCondition.notify_all();
}

void CommandProcessor::cancelMission()
{
    // Cancel Current Mission (Report Failure)
    mission_response_id_.store(mission_id_.load());
    mission_status_ = kErrorMissionAborted;
    response_received_ = true;
    taskCondition.notify_all();
}

void CommandProcessor::initRobotState(RobotState state)
{
    robotState = state;

    switch (state)
    {
        case RobotState::Charging: com_->publish(SubMissionGenerator::ROBOT_STATUS_TOPIC, SubMissionGenerator::ROBOT_STATUS_FIELD, SubMissionGenerator::ROBOT_STATUS_CHARGING); break;
        case RobotState::Standby:  com_->publish(SubMissionGenerator::ROBOT_STATUS_TOPIC, SubMissionGenerator::ROBOT_STATUS_FIELD, SubMissionGenerator::ROBOT_STATUS_STANDBY); break;
        case RobotState::Idle:  com_->publish(SubMissionGenerator::ROBOT_STATUS_TOPIC, SubMissionGenerator::ROBOT_STATUS_FIELD, SubMissionGenerator::ROBOT_STATUS_IDLE); break;
        case RobotState::Error:  com_->publish(SubMissionGenerator::ROBOT_STATUS_TOPIC, SubMissionGenerator::ROBOT_STATUS_FIELD, SubMissionGenerator::ROBOT_STATUS_ERROR); break;
        default:  com_->publish(SubMissionGenerator::ROBOT_STATUS_TOPIC, SubMissionGenerator::ROBOT_STATUS_FIELD, SubMissionGenerator::ROBOT_STATUS_ERROR);
    }
}

void CommandProcessor::taskManager(QString command)
{
    bool taskSuccess = false;
    Json::Value message;
    Json::Reader reader;

    QQueue<SubMission> mission_list;
    SubMissionGenerator::Status status = sub_mission_generator->generateMission(command, robotState, mission_list);
    if (!status.success)
    {
        com_->publish(SubMissionGenerator::MISSION_STATUS_TOPIC, SubMissionGenerator::MISSION_STATUS_FIELD, SubMissionGenerator::MISSION_FAIL, status.msg);
        return;
    }

    // Starting Robot Mission
    com_->publish(SubMissionGenerator::ROBOT_STATUS_TOPIC, SubMissionGenerator::ROBOT_STATUS_FIELD, SubMissionGenerator::ROBOT_STATUS_BUSY);



    if(message["command"] == "dock")
    {
        taskSuccess = sendTask(SAFETY_OFF);
        if (taskSuccess)  taskSuccess = sendTask(DOCK_TO_CHARGER);
        //if (taskSuccess)  taskSuccess = sendTask(SAFETY_ON);
        if(taskSuccess)
        {
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_SUCCESS);
            com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_CHARGING);
            com_->publish(ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_CHARGER);
        }
        else
        {
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_FAIL);
        }
    }

    else if (message["command"] == "undock")
    {
        //taskSuccess = sendTask(SAFETY_OFF);
        taskSuccess = sendTask(UNDOCK_FROM_CHARGER);
        if (taskSuccess)  taskSuccess = sendTask(SAFETY_ON);
        if(taskSuccess)
        {
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_SUCCESS);
            com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_IDLE);
        }
        else
        {
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_FAIL);
        }
    }

    else if (message["command"] == "deliver")
    {
        /*
         * Exit parking
         * Go to bed
         * Update robot location
         * Release Chair
         * Exit bed
         * Go back to parking
         * Update robot location
         * */
        if (robotState != RobotState::Standby)
        {
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_FAIL, "Invalid Command. Robot not standby at parking");
            return;
        }

        taskSuccess = sendTask(SAFETY_ON);
        if (taskSuccess)  taskSuccess = sendTask(LF_PARKING_EXIT);
        if (taskSuccess)  com_->publish(ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_HALLWAY);
        if (taskSuccess)  taskSuccess = sendTask(LF_HALLWAY_TO_BED_PREFIX + QString(message["bed_id"].asString().c_str()));
        if (taskSuccess)  com_->publish(ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_BED_PREFIX + message["bed_id"].asString());
        if (taskSuccess)  taskSuccess = sendTask(RELEASE_PAYLOAD_TO_BED);

        // Publish Mission Status
        if(taskSuccess)
        {
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_SUCCESS);
        }
        else
        {
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_FAIL);
        }

        if (taskSuccess)  taskSuccess = sendTask(LF_BED_TO_HALLWAY_PREFIX + QString(message["bed_id"].asString().c_str()));
        if (taskSuccess)  com_->publish(ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_HALLWAY);
        if (taskSuccess)  taskSuccess = sendTask(LF_HALLWAY_TO_PARKING);
        if (taskSuccess)  com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_IDLE);
        if (taskSuccess)  com_->publish(ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_PARKING);
        /**
        if (taskSuccess)  com_->publish(DOOR_CONTROL_TOPIC, DOOR_CONTROL_FIELD, DOOR_OPEN);
        if (taskSuccess)  taskSuccess = sendTask(LF_OUTSIDE_TO_CHARGER);
        if (taskSuccess)  com_->publish(DOOR_CONTROL_TOPIC, DOOR_CONTROL_FIELD, DOOR_CLOSE);
        if (taskSuccess)  taskSuccess = sendTask(SAFETY_OFF);
        if (taskSuccess)  taskSuccess = sendTask(DOCK_TO_CHARGER);
        **/

        if(taskSuccess)
        {
            // TODO delete
            last_bed_id = QString(message["bed_id"].asString().c_str());
            // delete end
            robotState = RobotState::Idle;
        }
        else
        {
            robotState = RobotState::Error;
        }
    }

    else if (message["command"] == "collect")
    {
        /*
         * Exit parking
         * Line follow to collection
         * Dock to commode
         * Go back near disposal room
         * Open door
         * Go inside disposal room
         * Close door
         * Safety off
         * Dock to manipulator
         * Line follow to charger
         * Dock to charger
         * Safety on
         */

        QString bed_id = QString(message["bed_id"].asString().c_str());
        if (bed_id == QString(""))
        {
            bed_id = last_bed_id;
        }

        if (robotState != RobotState::Idle)
        {
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_FAIL, "Invalid Command. Robot not idle at parking");
            return;
        }

        taskSuccess = sendTask(SAFETY_ON);
        if (taskSuccess)  taskSuccess = sendTask(LF_PARKING_EXIT);
        if (taskSuccess)  com_->publish(ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_HALLWAY);
        if (taskSuccess)  taskSuccess = sendTask(LF_HALLWAY_TO_BED_PREFIX + bed_id);
        if (taskSuccess)  com_->publish(ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_BED_PREFIX + bed_id.toStdString());

        if (taskSuccess)  taskSuccess = sendTask(COLLECT_PAYLOAD_FROM_BED);

        if (taskSuccess)  taskSuccess = sendTask(LF_BED_EXIT_PREFIX + bed_id);
        if (taskSuccess)  taskSuccess = sendTask(LF_BED_TO_HALLWAY_PREFIX + bed_id);
        if (taskSuccess)  com_->publish(ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_HALLWAY);
        if (taskSuccess)  com_->publish(DOOR_CONTROL_TOPIC, DOOR_CONTROL_FIELD, DOOR_OPEN);

        if (taskSuccess)  taskSuccess = sendTask(LF_HALLWAY_TO_MANIPULATOR);
        if (taskSuccess)  taskSuccess = sendTask(SAFETY_OFF);
        if (taskSuccess)  com_->publish(DOOR_CONTROL_TOPIC, DOOR_CONTROL_FIELD, DOOR_CLOSE);
        if (taskSuccess)  taskSuccess = sendTask(RELEASE_PAYLOAD_TO_MANIPULATOR);
        if (taskSuccess)  com_->publish(ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_MANIPULATOR);

        if (taskSuccess)  taskSuccess = sendTask(SAFETY_OFF);
        if (taskSuccess)  taskSuccess = sendTask(LF_MANIPULATOR_TO_CHARGER);
        if (taskSuccess)  taskSuccess = sendTask(DOCK_TO_CHARGER);
        if (taskSuccess)  com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_CHARGING);
        if (taskSuccess)  com_->publish(ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_CHARGER);

        // Publish Mission Status
        if(taskSuccess)
        {
            robotState = RobotState::Charging;
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_SUCCESS);
        }
        else
        {
            robotState = RobotState::Error;
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_FAIL);
        }
    }

    else if (message["command"] == "park")
    {
        /*
         * Safety Off
         * Undock
         * Line follow to manipulator
         * Collect from manipulator
         * Open door
         * Safety On
         * Go out of door
         * Close door
         * Go to parking
         * */

        if (robotState != RobotState::Charging)
        {
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_FAIL, "Invalid Command. Robot not at Charger");
            return;
        }

        //taskSuccess = sendTask(SAFETY_OFF);
        taskSuccess = sendTask(UNDOCK_FROM_CHARGER);
        if (taskSuccess)  taskSuccess = sendTask(SAFETY_OFF);
        if (taskSuccess)  taskSuccess = sendTask(LF_CHARGER_TO_MANIPULATOR);
        if (taskSuccess)  taskSuccess = sendTask(COLLECT_PAYLOAD_FROM_MANIPULATOR);
        if (taskSuccess)  com_->publish(ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_MANIPULATOR);

        if (taskSuccess)  com_->publish(DOOR_CONTROL_TOPIC, DOOR_CONTROL_FIELD, DOOR_OPEN);
        if (taskSuccess)  taskSuccess = sendTask(SAFETY_OFF);
        if (taskSuccess)  taskSuccess = sendTask(LF_MANIPULATOR_TO_HALLWAY);
        if (taskSuccess)  taskSuccess = sendTask(SAFETY_ON);
        if (taskSuccess)  com_->publish(ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_HALLWAY);
        if (taskSuccess)  com_->publish(DOOR_CONTROL_TOPIC, DOOR_CONTROL_FIELD, DOOR_CLOSE);

        if (taskSuccess)  taskSuccess = sendTask(LF_HALLWAY_TO_PARKING);
        if (taskSuccess)  com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_STANDBY);
        if (taskSuccess)  com_->publish(ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_PARKING);

        // Publish Mission Status
        if(taskSuccess)
        {
            robotState = RobotState::Standby;
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_SUCCESS);
        }
        else
        {
            robotState = RobotState::Error;
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_FAIL);
        }
    }

    else if (message["command"] == "door_open")
    {
        taskSuccess = true;
        com_->publish(DOOR_CONTROL_TOPIC, DOOR_CONTROL_FIELD, DOOR_OPEN);
        com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_IDLE);
    }

    else if (message["command"] == "door_close")
    {
        taskSuccess = true;
        com_->publish(DOOR_CONTROL_TOPIC, DOOR_CONTROL_FIELD, DOOR_CLOSE);
        com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_IDLE);
    }

    else if (message["command"] == "safety_on")
    {
        taskSuccess = sendTask(SAFETY_ON);
        if(taskSuccess)
        {
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_SUCCESS);
        }
        else
        {
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_FAIL);
        }
        com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_IDLE);
    }

    else if (message["command"] == "safety_off")
    {
        taskSuccess = sendTask(SAFETY_OFF);
        if(taskSuccess)
        {
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_SUCCESS);
        }
        else
        {
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_FAIL);
        }
        com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_IDLE);
    }

    else if (message["command"] == "gripper_extend")
    {
        taskSuccess = sendTask(GRIPPER_EXTEND);
        if(taskSuccess)
        {
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_SUCCESS);
        }
        else
        {
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_FAIL);
        }
        com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_IDLE);
    }

    else if (message["command"] == "gripper_retract")
    {
        taskSuccess = sendTask(GRIPPER_RETRACT);
        if(taskSuccess)
        {
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_SUCCESS);
        }
        else
        {
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_FAIL);
        }
        com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_IDLE);
    }

    else if (message["command"] == "gripper_clamp")
    {
        taskSuccess = sendTask(GRIPPER_CLAMP);
        if(taskSuccess)
        {
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_SUCCESS);
        }
        else
        {
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_FAIL);
        }
        com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_IDLE);
    }

    else if (message["command"] == "gripper_release")
    {
        taskSuccess = sendTask(GRIPPER_RELEASE);
        if(taskSuccess)
        {
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_SUCCESS);
        }
        else
        {
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_FAIL);
        }
        com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_IDLE);
    }

    else if (message["command"] == "gripper_extended_clamp")
    {
        taskSuccess = sendTask(GRIPPER_EXTENDED_CLAMP);
        if(taskSuccess)
        {
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_SUCCESS);
        }
        else
        {
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_FAIL);
        }
        com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_IDLE);
    }

    else if (message["command"] == "gripper_extended_release")
    {
        taskSuccess = sendTask(GRIPPER_EXTENDED_RELEASE);
        if(taskSuccess)
        {
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_SUCCESS);
        }
        else
        {
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_FAIL);
        }
        com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_IDLE);
    }

    else
    {
        console_->print("Error: Unknown Command: " + message["command"].asString());
        com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_IDLE);
    }

    // Callback
    if (completionCallback_ != NULL)
    {
        completionCallback_(taskSuccess);
    }
    // Publish Robot Status
    if (taskSuccess)
    {
        console_->print(message["command"].asString() + " : Mission Successfull");
    }
    else
    {
        console_->print(message["command"].asString() + " : Mission Failed");
        com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_ERROR);
        // At least try to turn on safety, in case of error
        sendTask(SAFETY_ON);
    }
    return;
}
