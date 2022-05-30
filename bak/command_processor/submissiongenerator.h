#ifndef SUBMISSIONGENERATOR_H
#define SUBMISSIONGENERATOR_H

#include <QObject>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include "Tools/console.h"
#include "Tools/robotCommunication.h"
#include "submission.h"
#include <QQueue>
#include <QString>
#include <jsoncpp/json/reader.h>
#include <jsoncpp/json/json.h>

enum class RobotState {
    Charging,       // At Robot Charger
    Standby,        // At parking, with commode
    Idle,           // At parking, without commode
    Error           // Error
};

class SubMissionGenerator
{

public:
    // Definitions
    const QString ROBOT_STATUS_STANDBY          {"standby"};
    const QString ROBOT_STATUS_IDLE             {"idle"};
    const QString ROBOT_STATUS_CHARGING         {"charging"};
    const QString ROBOT_STATUS_BUSY             {"busy"};
    const QString ROBOT_STATUS_ERROR            {"error"};

    const QString MISSION_STATUS_TOPIC	{ "robot_command_status" };
    const QString ROBOT_STATUS_TOPIC    { "robot_status" };
    const QString DOOR_CONTROL_TOPIC    { "door_control" };
    const QString ROBOT_LOCATION_TOPIC  { "robot_location" };

    const QString MISSION_STATUS_FIELD          {"success"};
    const QString ROBOT_STATUS_FIELD            {"status"};
    const QString DOOR_CONTROL_FIELD            {"open"};
    const QString ROBOT_LOCATION_FIELD          {"location"};

    struct Status {
        bool success = false;
        std::string msg = "";
    };

    SubMissionGenerator(boost::function<bool (QString)> sendTask,
                        Console *console,
                        boost::function<bool (QString, QString, QString)> publishMsg,
                        boost::function<bool (QString, QString, bool)> publishBool);
    Status generateMission(QString mission_cmd, RobotState robot_state, QQueue<SubMission>& mission_list);
    RobotState getSuccessState(void);

signals:

public slots:

private:
    Console *console_;
    boost::function<bool (QString)> sendTask_;
    boost::function<bool (QString, QString, QString)> publishMsg_;
    boost::function<bool (QString, QString, bool)> publishBool_;
    QQueue<SubMission> sub_missions_;
    RobotState robot_success_state_;

    // Used to keep last bed_id also. Will be changed in future
    QString bed_id_ = "0";

    void enqueue(SubMission submission);

    // Robot Locations
    const QString LOCATION_CHARGER                  {"charger"};
    const QString LOCATION_MANIPULATOR              {"manipulator"};
    const QString LOCATION_HALLWAY                  {"hallway"};
    const QString LOCATION_PARKING                  {"parking"};
    const QString LOCATION_BED_PREFIX               {"bed_"};

    const bool MISSION_SUCCESS = true;
    const bool MISSION_FAIL = false;
    const bool DOOR_OPEN = true;
    const bool DOOR_CLOSE = false;

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
    const QString COLLECT_PAYLOAD_FROM_MANIPULATOR = "Collect_from_manipulator";
    const QString RELEASE_PAYLOAD_TO_BED = "Release_to_bed";
    const QString RELEASE_PAYLOAD_TO_MANIPULATOR = "Release_to_manipulator";

    const QString ROBOT_FOOTPRINT_UPDATE_WITH_PAYLOAD = "robot_footprint_update_with_payload";
    const QString ROBOT_FOOTPRINT_UPDATE_WITHOUT_PAYLOAD = "robot_footprint_update_without_payload";

    const QString LF_CHARGER_TO_MANIPULATOR = "LF_charger_to_manipulator";
    const QString LF_MANIPULATOR_TO_HALLWAY = "LF_manipulator_to_hallway";
    const QString LF_HALLWAY_TO_PARKING = "LF_hallway_to_parking";

    const QString LF_PARKING_EXIT = "LF_parking_exit";
    const QString LF_HALLWAY_TO_BED_PREFIX = "toBeds/LF_hallway_to_bed_";

    const QString LF_BED_TO_HALLWAY_PREFIX = "fromBeds/LF_bed_to_hallway_";
    const QString LF_BED_EXIT_PREFIX = "exitBeds/LF_bed_exit_";

    const QString LF_HALLWAY_TO_MANIPULATOR = "LF_hallway_to_manipulator";
    const QString LF_MANIPULATOR_TO_CHARGER = "LF_manipulator_to_charger";
};

#endif // SUBMISSIONGENERATOR_H
