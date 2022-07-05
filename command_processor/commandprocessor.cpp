#include "commandprocessor.h"

CommandProcessor::CommandProcessor(boost::function<int (QByteArray)> sendMission, Console *console, RobotCommunication *com)
{
    sendMission_ = sendMission;
    console_ = console;
    com_ = com;

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
    if (this->isRunning())
    {
        console_->print("Error: Robot is currently running a task. Ignoring new command!");
    }
    this->start();
    //mission_thread_ = new std::thread(&CommandProcessor::taskManager, this, mission_cmd);
}

void CommandProcessor::run()
{
    taskManager(task_manager_command);
}

bool CommandProcessor::sendTask(QString file_name)
{
    std::cout << "sendTask Called" << std::endl;
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

            console_->print("Starting Mission " + filename.toStdString());
            mission_id_ = sendMission_(bjsonstr);
            response_received_ = false;
            robotTask = file_name;

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
        case RobotState::Charging: com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_CHARGING); break;
        case RobotState::Standby:  com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_STANDBY); break;
        case RobotState::Idle:  com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_IDLE); break;
        case RobotState::Error:  com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_ERROR); break;
        case RobotState::Disabled: com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_DISABLED); break;
        default:  com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_ERROR);
    }
}

void CommandProcessor::taskManager(QString command)
{
    bool taskSuccess = false;
    Json::Value message;
    Json::Reader reader;
    if ( reader.parse( command.toStdString(), message ) == false )
    {
        console_->print("Cannot Decode incoming JSON Command: " + command.toStdString());
        if (completionCallback_ != NULL)
        {
            completionCallback_(taskSuccess);
        }
        return;
    }

    if (robotState != RobotState::Disabled)
    {
        // Starting Robot Mission
        com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_BUSY);
    }


    if (message["command"] == "shutdown")
    {
        system("echo NUC717 | sudo -S shutdown now");
    }

    else if (message["command"] == "self_test")
    {
        taskSuccess = sendTask(UNDOCK_FROM_CHARGER);
        if (taskSuccess)  taskSuccess = sendTask(SAFETY_OFF);
        if (taskSuccess)  taskSuccess = sendTask(DOCK_TO_CHARGER);
        if(taskSuccess)
        {
            robotState = RobotState::Charging;
            // Self Test is a previous state resetting event
            previousRobotState = robotState;
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_SUCCESS);
            com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_CHARGING);
        }
        else
        {
            robotState = RobotState::Error;
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_FAIL);
        }
    }

    else if (message["command"] == "enable")
    {
        // If robot is not disabled, use current state. Else, revert to previous state
        RobotState newState = (robotState == RobotState::Disabled)? previousRobotState : robotState;

        taskSuccess = true;
        com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_SUCCESS);
        initRobotState(newState);
    }

    else if (message["command"] == "disable")
    {
        // Record current Robot State and disable robot
        previousRobotState = robotState;
        robotState = RobotState::Disabled;
        taskSuccess = true;
        com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_SUCCESS);
        com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_DISABLED);

    }

    else if(message["command"] == "dock")
    {
        taskSuccess = sendTask(SAFETY_OFF);
        if (taskSuccess)  taskSuccess = sendTask(DOCK_TO_CHARGER);
        //if (taskSuccess)  taskSuccess = sendTask(SAFETY_ON);
        if(taskSuccess)
        {
            robotState = RobotState::Charging;
            // Dock is a previous state resetting event
            previousRobotState = robotState;
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_SUCCESS);
            com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_CHARGING);
            com_->publish(ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_CHARGER);
        }
        else
        {
            robotState = RobotState::Error;
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
            robotState = RobotState::Error;
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_FAIL);
        }
    }

    else if (message["command"] == "deliver")
    {
        /*
         * Exit Charger
         * Go to bed
         * Update robot location
         * Release Chair
         * Go back to Charger
         * Dock to Charger
         * Update robot location
         * */


        if (robotState != RobotState::Standby)
        {
            //com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_FAIL, "Invalid Command. Robot not standby at parking");
            console_->print("Invalid Command. Robot not standby at parking");
            initRobotState(robotState);
            return;
        }
        else
        {
            // Record robot state
            previousRobotState = robotState;
            // TODO delete
            last_bed_id = QString(message["bed_id"].asString().c_str());
            // delete end
        }

        taskSuccess = sendTask(SAFETY_ON);
        if (taskSuccess)  taskSuccess = sendTask(LF_CHARGER_TO_HALLWAY);
        if (taskSuccess)  com_->publish(ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_HALLWAY);
        if (taskSuccess)  taskSuccess = sendTask(LF_HALLWAY_TO_BED_PREFIX + QString(message["bed_id"].asString().c_str()));
        if (taskSuccess)  com_->publish(ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_BED_PREFIX + message["bed_id"].asString());
        if (taskSuccess)  taskSuccess = sendTask(GRIPPER_RELEASE);

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
        if (taskSuccess)  taskSuccess = sendTask(LF_HALLWAY_TO_CHARGER);
        if (taskSuccess)  taskSuccess = sendTask(SAFETY_OFF);
        if (taskSuccess)  taskSuccess = sendTask(DOCK_TO_CHARGER);
        if (taskSuccess)  com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_IDLE);
        if (taskSuccess)  com_->publish(ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_CHARGER);

        if(taskSuccess)
        {
            robotState = RobotState::Idle;
            // Reset the previous state
            previousRobotState = robotState;
        }
        else
        {
            robotState = RobotState::Error;
        }
    }

    else if (message["command"] == "collect")
    {
        /*
         * Exit charger
         * Line follow to collection
         * Dock to commode
         * Go back near disposal room
         * Release commode
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
            //com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_FAIL, "Invalid Command. Robot not idle at parking");
            console_->print("Invalid Command. Robot not idle at parking");
            initRobotState(robotState);
            return;
        }
        else
        {
            // Record robot state
            previousRobotState = robotState;
        }


        taskSuccess = sendTask(SAFETY_ON);
        if (taskSuccess)  taskSuccess = sendTask(LF_CHARGER_TO_HALLWAY);
        if (taskSuccess)  com_->publish(ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_HALLWAY);
        if (taskSuccess)  taskSuccess = sendTask(LF_HALLWAY_TO_BED_PREFIX + bed_id);
        if (taskSuccess)  com_->publish(ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_BED_PREFIX + bed_id.toStdString());

        if (taskSuccess)  taskSuccess = sendTask(COLLECT_PAYLOAD_FROM_BED);

        if (taskSuccess)  taskSuccess = sendTask(LF_BED_TO_HALLWAY_PREFIX + bed_id);
        if (taskSuccess)  com_->publish(ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_HALLWAY);
        if (taskSuccess)  taskSuccess = sendTask(LF_HALLWAY_TO_CLEANING_ROOM);
        if (taskSuccess)  com_->publish(ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_CLEANING_ROOM);

        if (taskSuccess)  taskSuccess = sendTask(GRIPPER_EXTENDED_RELEASE);

        if (taskSuccess)  taskSuccess = sendTask(LF_CLEANING_ROOM_TO_HALLWAY);
        if (taskSuccess)  com_->publish(ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_HALLWAY);

        if (taskSuccess)  taskSuccess = sendTask(LF_HALLWAY_TO_CHARGER);
        if (taskSuccess)  taskSuccess = sendTask(SAFETY_OFF);
        if (taskSuccess)  taskSuccess = sendTask(DOCK_TO_CHARGER);
        if (taskSuccess)  com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_CHARGING);
        if (taskSuccess)  com_->publish(ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_CHARGER);

        // Publish Mission Status
        if(taskSuccess)
        {
            robotState = RobotState::Charging;
            // Reset the previous state
            previousRobotState = robotState;
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
         * Clamp to the commode
         * */

        if (robotState != RobotState::Charging)
        {
            //com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_FAIL, "Invalid Command. Robot not at Charger");
            console_->print("Invalid Command. Robot not at Charger");
            initRobotState(robotState);
            return;
        }
        else
        {
            // Record robot state
            previousRobotState = robotState;
        }


        // Do Nothing here
        /**
        This message will be hijacked by icube-ui program, and it will directly activate robot gripper.
        This message will not be sent through I2R path to avoid robot undocking.
          */

        taskSuccess = true; // No I2R task to run
        com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_STANDBY);

        // Publish Mission Status
        if(taskSuccess)
        {
            robotState = RobotState::Standby;
            // Reset the previous state
            previousRobotState = robotState;
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_SUCCESS);
        }
        else
        {
            robotState = RobotState::Error;
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_FAIL);
        }
    }

    else if (message["command"] == "deliver_wf")
    {
        /*
         * Exit Charger
         * Go to bed
         * Update robot location
         * Release Chair
         * Go back to Charger
         * Dock to Charger
         * Update robot location
         * */


        if (robotState != RobotState::Standby)
        {
            //com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_FAIL, "Invalid Command. Robot not standby at parking");
            console_->print("Invalid Command. Robot not standby at parking");
            initRobotState(robotState);
            return;
        }
        else
        {
            // Record robot state
            previousRobotState = robotState;
            // TODO delete
            last_bed_id = QString(message["bed_id"].asString().c_str());
            // delete end
        }

        taskSuccess = sendTask(SAFETY_ON);
        if (taskSuccess)  taskSuccess = sendTask(LF_CHARGER_TO_HALLWAY);
        if (taskSuccess)  com_->publish(ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_HALLWAY);
        if (taskSuccess)  taskSuccess = sendTask(LF_HALLWAY_TO_WALKER_DROP_PREFIX + QString(message["bed_id"].asString().c_str()));
        if (taskSuccess)  com_->publish(ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_BED_PREFIX + message["bed_id"].asString());
        if (taskSuccess)  taskSuccess = sendTask(GRIPPER_EXTENDED_RELEASE);

        // Publish Mission Status
        if(taskSuccess)
        {
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_SUCCESS);
        }
        else
        {
            com_->publish(MISSION_STATUS_TOPIC, MISSION_STATUS_FIELD, MISSION_FAIL);
        }

        if (taskSuccess)  taskSuccess = sendTask(LF_WALKER_DROP_TO_HALLWAY_PREFIX + QString(message["bed_id"].asString().c_str()));
        if (taskSuccess)  com_->publish(ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_HALLWAY);
        if (taskSuccess)  taskSuccess = sendTask(LF_HALLWAY_TO_CHARGER);
        if (taskSuccess)  taskSuccess = sendTask(SAFETY_OFF);
        if (taskSuccess)  taskSuccess = sendTask(DOCK_TO_CHARGER);
        if (taskSuccess)  com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_IDLE);
        if (taskSuccess)  com_->publish(ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_CHARGER);

        if(taskSuccess)
        {
            robotState = RobotState::Idle;
            // Reset the previous state
            previousRobotState = robotState;
        }
        else
        {
            robotState = RobotState::Error;
        }
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

    else if (message["command"] == "cancel_mission")
    {
        console_->print("Last sub task: " + robotTask.toStdString());
        console_->print("Cancelling mission");
        if (previousRobotState == RobotState::Standby)
        {
            /// Cancelling Delivery mission
            console_->print("Going back to Standby State");

            int rollback_point = -1;
            if (robotTask == LF_HALLWAY_TO_BED_PREFIX + last_bed_id)
            {
                console_->print("go back from bed lf");
                rollback_point = 0;
            }
            else if (robotTask == LF_CHARGER_TO_HALLWAY)
            {
                console_->print("go back from hallway");
                rollback_point = 1;
            }
            else if (robotTask == SAFETY_ON)
            {
                console_->print("go back from undock");
                rollback_point = 2;
            }
            else
            {
                console_->print("Delivery mission beyond cancellation point!");
            }

            switch(rollback_point)
            {
                case 0: taskSuccess = sendTask(SAFETY_ON);
                        if (taskSuccess)  taskSuccess = sendTask(LF_BED_TO_HALLWAY_PREFIX + last_bed_id);
                        if (!taskSuccess) robotTask = LF_HALLWAY_TO_BED_PREFIX + last_bed_id;     // Fallback if cancel error
                        if (!taskSuccess) break;
                        BOOST_FALLTHROUGH;

                case 1: taskSuccess = sendTask(SAFETY_ON);
                        if (taskSuccess)  taskSuccess = sendTask(LF_HALLWAY_TO_CHARGER);
                        if (!taskSuccess) robotTask = LF_CHARGER_TO_HALLWAY;                     // Fallback if cancel error
                        if (!taskSuccess) break;
                        BOOST_FALLTHROUGH;

                case 2: taskSuccess = sendTask(SAFETY_OFF);
                        if (taskSuccess)  taskSuccess = sendTask(DOCK_TO_CHARGER);
                        if (!taskSuccess) robotTask = SAFETY_ON;                                 // Fallback if cancel error
                        if (taskSuccess) break;
                        BOOST_FALLTHROUGH;

                default: console_->print("Error: Mission Cancellation failed!");
            }


            console_->print("Last mission check statement: " + LF_HALLWAY_TO_BED_PREFIX.toStdString() + last_bed_id.toStdString());
            // Publish location
            if (taskSuccess)  com_->publish(ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_CHARGER);
        }

        else if (previousRobotState == RobotState::Idle)
        {

            /// Cancelling Collection mission
            console_->print("Going back to Idle State");

            int rollback_point = -1;
            if (robotTask == LF_HALLWAY_TO_BED_PREFIX + last_bed_id)
            {
                console_->print("go back from bed lf");
                rollback_point = 0;
            }
            else if (robotTask == LF_CHARGER_TO_HALLWAY)
            {
                console_->print("go back from hallway");
                rollback_point = 1;
            }
            else if (robotTask == SAFETY_ON)
            {
                console_->print("go back from undock");
                rollback_point = 2;
            }
            else
            {
                console_->print("Delivery mission beyond cancellation point!");
            }

            switch(rollback_point)
            {
                case 0: taskSuccess = sendTask(SAFETY_ON);
                        if (taskSuccess)  taskSuccess = sendTask(LF_BED_TO_HALLWAY_PREFIX + last_bed_id);
                        if (!taskSuccess) robotTask = LF_HALLWAY_TO_BED_PREFIX + last_bed_id;     // Fallback if cancel error
                        if (!taskSuccess) break;
                        BOOST_FALLTHROUGH;

                case 1: taskSuccess = sendTask(SAFETY_ON);
                        if (taskSuccess)  taskSuccess = sendTask(LF_HALLWAY_TO_CHARGER);
                        if (!taskSuccess) robotTask = LF_CHARGER_TO_HALLWAY;                     // Fallback if cancel error
                        if (!taskSuccess) break;
                        BOOST_FALLTHROUGH;

                case 2: taskSuccess = sendTask(SAFETY_OFF);
                        if (taskSuccess)  taskSuccess = sendTask(DOCK_TO_CHARGER);
                        if (!taskSuccess) robotTask = SAFETY_ON;                                 // Fallback if cancel error
                        if (taskSuccess) break;
                        BOOST_FALLTHROUGH;

                default: console_->print("Error: Mission Cancellation failed!");
            }

            // Publish location
            if (taskSuccess)  com_->publish(ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_CHARGER);

        }

        else if (previousRobotState == RobotState::Charging)
        {
            console_->print("Please release the gripper manually");
        }
        else
        {
            console_->print("Cannot go back since previous state is not initialized");
        }

        // Reinitialize State
        if (taskSuccess) initRobotState(previousRobotState);
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
    else if (message["command"] == "abort")
    {
        console_->print(message["command"].asString() + " : Success");
        com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_IDLE);
    }
    else
    {
        console_->print(message["command"].asString() + " : Mission Failed");
        com_->publish(ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_ERROR);
        // At least try to turn on safety, in case of error
        // sendTask(SAFETY_ON);
    }
    return;
}
