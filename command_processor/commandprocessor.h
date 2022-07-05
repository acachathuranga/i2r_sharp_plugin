#ifndef COMMANDPROCESSOR_H
#define COMMANDPROCESSOR_H

#include "yaml-cpp/yaml.h"
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <QFile>
#include <QByteArray>
#include "Tools/console.h"
#include <QStringList>
#include <QJsonDocument>
#include <QThread>
#include <jsoncpp/json/reader.h>
#include <jsoncpp/json/json.h>
#include "Tools/robotCommunication.h"
#include <mutex>
#include <thread>
#include <atomic>
#include <QThread>

#include "../../common/fsm_defs.h"

class CommandProcessor : public QThread
{
    public:
        enum class RobotState {
            Charging,       // At Robot Charger
            Standby,        // At parking, with commode
            Idle,           // At parking, without commode
            Disabled,       // Disabled by user
            Error           // Error
        };

        /**
         * @brief CommandProcessor
         * @param sendMission       SendMission Function pointer that accepts I2R Mission JSON file, to be sent to robot
         * @param console           Console object pointer that has print and clear functions
         */
        CommandProcessor(boost::function<int (QByteArray)> sendMission, Console *console, RobotCommunication *com);

        void executeMission(QString mission_cmd, QString data_path, boost::function<void (bool)> completionCallback);
        void executeMission(QString mission_cmd, QString data_path);
        void cancelMission();
        void initRobotState(RobotState state);

        void subMissionCompletionCallback(int sub_mission_id, int sub_mission_status);

    private:

        Console *console_;
        QString mission_file_directory_;
        boost::function<int (QByteArray)> sendMission_;
        boost::function<void (bool)> completionCallback_;
        RobotCommunication *com_;

        std::mutex mtx_;
        std::condition_variable taskCondition;
        std::thread *mission_thread_ = NULL;
        bool response_received_;
        std::atomic<int> mission_response_id_;
        std::atomic<int> mission_id_;
        std::atomic<int> mission_status_;
        QString task_manager_command;

        RobotState robotState = RobotState::Charging;
        RobotState previousRobotState = robotState;

        QString robotTask =  "";

        const std::string MISSION_STATUS_TOPIC	{ "robot_command_status" };
        const std::string ROBOT_STATUS_TOPIC    { "robot_status" };
        const std::string ROBOT_LOCATION_TOPIC  { "robot_location" };

        const std::string MISSION_STATUS_FIELD          {"success"};
        const std::string ROBOT_STATUS_FIELD            {"status"};
        const std::string ROBOT_LOCATION_FIELD          {"location"};

        const std::string ROBOT_STATUS_STANDBY          {"standby"};
        const std::string ROBOT_STATUS_IDLE             {"idle"};
        const std::string ROBOT_STATUS_CHARGING         {"charging"};
        const std::string ROBOT_STATUS_BUSY             {"busy"};
        const std::string ROBOT_STATUS_DISABLED         {"disabled"};
        const std::string ROBOT_STATUS_ERROR            {"error"};

        // Robot Locations
        const std::string LOCATION_CHARGER                  {"charger"};
        const std::string LOCATION_HALLWAY                  {"hallway"};
        const std::string LOCATION_CLEANING_ROOM            {"cleaning_room"};
        const std::string LOCATION_BED_PREFIX               {"bed_"};

        const bool MISSION_SUCCESS = true;
        const bool MISSION_FAIL = false;

        void run();
        bool sendTask(QString file_name);
        void taskManager(QString command);

        // TODO Delete
        QString last_bed_id;


        // Task File Name definitions
        const QString GRIPPER_EXTEND = "gripper/Gripper_extend";
        const QString GRIPPER_RETRACT = "gripper/Gripper_retract";
        const QString GRIPPER_CLAMP = "gripper/Gripper_clamp";
        const QString GRIPPER_RELEASE = "gripper/Gripper_release";
        const QString GRIPPER_EXTENDED_CLAMP = "gripper/Gripper_extended_clamp";
        const QString GRIPPER_EXTENDED_RELEASE = "gripper/Gripper_extended_release";

        const QString DOCK_TO_CHARGER = "Dock";
        const QString UNDOCK_FROM_CHARGER = "Undock";

        const QString SAFETY_ON = "SafetyOn";
        const QString SAFETY_OFF = "SafetyOff";

        const QString COLLECT_PAYLOAD_FROM_BED = "Collect_from_bed";

        const QString ROBOT_FOOTPRINT_UPDATE_WITH_PAYLOAD = "robot_footprint_update_with_payload";
        const QString ROBOT_FOOTPRINT_UPDATE_WITHOUT_PAYLOAD = "robot_footprint_update_without_payload";

        const QString LF_CHARGER_TO_HALLWAY = "LF_charger_to_hallway";
        const QString LF_HALLWAY_TO_CHARGER = "LF_hallway_to_charger";

        const QString LF_HALLWAY_TO_CLEANING_ROOM = "LF_hallway_to_cleaning_room";
        const QString LF_CLEANING_ROOM_TO_HALLWAY = "LF_cleaning_room_to_hallway";

        const QString LF_HALLWAY_TO_BED_PREFIX = "toBeds/LF_hallway_to_bed_";
        //const QString LF_HALLWAY_TO_BED_COLLECT_PREFIX = "toBedsCollect/LF_hallway_to_bed_";
        const QString LF_BED_TO_HALLWAY_PREFIX = "fromBeds/LF_bed_to_hallway_";

        const QString LF_HALLWAY_TO_WALKER_DROP_PREFIX = "toWalkerDrop/LF_hallway_to_walker_drop_";
        const QString LF_WALKER_DROP_TO_HALLWAY_PREFIX = "fromWalkerDrop/LF_walker_drop_to_hallway_";
};

#endif // COMMANDPROCESSOR_H
