#include "submissiongenerator.h"
#include <queue>
#define SUB_CMD boost::bind

SubMissionGenerator::SubMissionGenerator(boost::function<bool (QString)> sendTask,
                                         Console *console,
                                         boost::function<bool (QString, QString, QString)> publishMsg,
                                         boost::function<bool (QString, QString, bool)> publishBool
                                         )
{
    sendTask_ = sendTask;
    console_ = console;
    publishMsg_ = publishMsg;
    publishBool_ = publishBool;
}

void SubMissionGenerator::enqueue(SubMission submission)
{
    sub_missions_.enqueue(submission);
}

RobotState SubMissionGenerator::getSuccessState(void)
{
    return robot_success_state_;
}

SubMissionGenerator::Status SubMissionGenerator::generateMission(QString mission_cmd, RobotState robot_state, QQueue<SubMission>& mission_list)
{
    Status status;
    Json::Value message;
    Json::Reader reader;
    if ( reader.parse( mission_cmd.toStdString(), message ) == false )
    {
        status.success = false;
        status.msg = "SubMissionGenerator: Cannot Decode incoming JSON Command: " + mission_cmd.toStdString();
        console_->print(status.msg);
        return status;
    }

    // Empty submission queue
    sub_missions_.empty();

    // Generating Robot Mission
    if(message["command"] == "dock")
    {
        // Safety Off
        {
            SubMission safety_off(SUB_CMD(sendTask_, SAFETY_OFF));
            safety_off.addFallbackTask(SUB_CMD(sendTask_, SAFETY_ON));
            safety_off.setDescription("Turn OFF Obstacle Guard");
            enqueue(safety_off);
        }

        // Start Docking
        {
            SubMission dock(SUB_CMD(sendTask_, DOCK_TO_CHARGER));
            dock.setDescription("Dock to Robot charger");
            enqueue(dock);
        }

        // Publish Robot Status
        {
            SubMission publish_robot_state(SUB_CMD(publishMsg_, ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_CHARGING));
            publish_robot_state.setDescription("Update Robot State");
            enqueue(publish_robot_state);
        }

        // Publish Robot Location
        {
            SubMission publish_robot_location(SUB_CMD(publishMsg_, ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_CHARGER));
            publish_robot_location.setDescription("Update Robot Location");
            enqueue(publish_robot_location);
        }

        // Set Success State
        robot_success_state_ = RobotState::Charging;
    }

    else if (message["command"] == "undock")
    {
        // Start Undocking
        {
            SubMission undock(SUB_CMD(sendTask_, UNDOCK_FROM_CHARGER));
            undock.addFallbackTask(SUB_CMD(sendTask_, DOCK_TO_CHARGER));
            undock.setDescription("Undock From Charger");
            enqueue(undock);
        }
    }

    else if (message["command"] == "deliver")
    {
       // Check robot state
       if (robot_state != RobotState::Standby)
       {
           status.success = false;
           status.msg = "SubMissionGenerator: Invalid Robot State. Cannot Start delivery";
           console_->print(status.msg);
           return status;
       }

       bed_id_ = QString(message["bed_id"].asString().c_str());

       // Safety On
       {
           SubMission safety_on(SUB_CMD(sendTask_, SAFETY_ON));
           safety_on.addFallbackTask(SUB_CMD(sendTask_, SAFETY_OFF));
           safety_on.setDescription("Turn ON Obstacle Guard");
           enqueue(safety_on);
       }

       // Parking Exit
       {
           SubMission navigate(SUB_CMD(sendTask_, LF_PARKING_EXIT));
           navigate.addFallbackTask(SUB_CMD(sendTask_, LF_HALLWAY_TO_PARKING));
           navigate.setDescription("Exit Parking Location");
           enqueue(navigate);
       }

       // Publish Robot Location
       {
           SubMission publish_robot_location(SUB_CMD(publishMsg_, ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_HALLWAY));
           publish_robot_location.setDescription("Update Robot Location");
           enqueue(publish_robot_location);
       }

       // Go to Bed
       {
           SubMission navigate(SUB_CMD(sendTask_, LF_HALLWAY_TO_BED_PREFIX + bed_id_));
           navigate.addFallbackTask(SUB_CMD(sendTask_, LF_BED_TO_HALLWAY_PREFIX + bed_id_));
           navigate.setDescription("Go to Bed Location");
           enqueue(navigate);
       }

       // Publish Robot Location
       {
           SubMission publish_robot_location(SUB_CMD(publishMsg_, ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_BED_PREFIX + bed_id_));
           publish_robot_location.setDescription("Update Robot Location");
           enqueue(publish_robot_location);
       }

       // Release Payload
       {
           SubMission release_payload(SUB_CMD(sendTask_, RELEASE_PAYLOAD_TO_BED));
           release_payload.fallbackAllowed(false);
           release_payload.setDescription("Release Payload");
           enqueue(release_payload);
       }

       // Go from Bed to Hallway
       {
           SubMission navigate(SUB_CMD(sendTask_, LF_BED_TO_HALLWAY_PREFIX + bed_id_));
           navigate.setDescription("Go back from Bed to Hallway");
           enqueue(navigate);
       }

       // Publish Robot Location
       {
           SubMission publish_robot_location(SUB_CMD(publishMsg_, ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_HALLWAY));
           publish_robot_location.setDescription("Update Robot Location");
           enqueue(publish_robot_location);
       }

       // Enter Parking Location from Hallway
       {
           SubMission navigate(SUB_CMD(sendTask_, LF_HALLWAY_TO_PARKING));
           navigate.setDescription("Enter Parking Location");
           enqueue(navigate);
       }

       // Publish Robot Status
       {
           SubMission publish_robot_state(SUB_CMD(publishMsg_, ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_IDLE));
           publish_robot_state.setDescription("Update Robot State");
           enqueue(publish_robot_state);
       }

       // Publish Robot Location
       {
           SubMission publish_robot_location(SUB_CMD(publishMsg_, ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_PARKING));
           publish_robot_location.setDescription("Update Robot Location");
           enqueue(publish_robot_location);
       }

       // Set Success State
       robot_success_state_ = RobotState::Idle;
    }

    else if (message["command"] == "collect")
    {
       // Check Robot State
        if (robot_state != RobotState::Idle)
        {
            status.success = false;
            status.msg = "SubMissionGenerator: Invalid Robot State. Cannot Start collection";
            console_->print(status.msg);
            return status;
        }

       QString bed_id = QString(message["bed_id"].asString().c_str());
       if (bed_id == QString(""))
       {
           bed_id = bed_id_;
       }

       // Safety On
       {
           SubMission safety_on(SUB_CMD(sendTask_, SAFETY_ON));
           safety_on.setDescription("Turn ON Obstacle Guard");
           enqueue(safety_on);
       }

       // Parking Exit
       {
           SubMission navigate(SUB_CMD(sendTask_, LF_PARKING_EXIT));
           navigate.addFallbackTask(SUB_CMD(sendTask_, LF_HALLWAY_TO_PARKING));
           navigate.setDescription("Exit Parking Location");
           enqueue(navigate);
       }

       // Publish Robot Location
       {
           SubMission publish_robot_location(SUB_CMD(publishMsg_, ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_HALLWAY));
           publish_robot_location.setDescription("Update Robot Location");
           enqueue(publish_robot_location);
       }

       // Go to Bed
       {
           SubMission navigate(SUB_CMD(sendTask_, LF_HALLWAY_TO_BED_PREFIX + bed_id_));
           navigate.addFallbackTask(SUB_CMD(sendTask_, LF_BED_TO_HALLWAY_PREFIX + bed_id_));
           navigate.setDescription("Go to Bed Location");
           enqueue(navigate);
       }

       // Publish Robot Location
       {
           SubMission publish_robot_location(SUB_CMD(publishMsg_, ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_BED_PREFIX + bed_id_));
           publish_robot_location.setDescription("Update Robot Location");
           enqueue(publish_robot_location);
       }

       // Collect Payload at Bed
       {
           SubMission collect_payload(SUB_CMD(sendTask_, COLLECT_PAYLOAD_FROM_BED));
           collect_payload.addFallbackTask(SUB_CMD(sendTask_, RELEASE_PAYLOAD_TO_BED));
           collect_payload.setDescription("Collect Payload");
           enqueue(collect_payload);
       }

       // Clear from Bed area
       {
           SubMission navigate(SUB_CMD(sendTask_, LF_BED_EXIT_PREFIX + bed_id_));
           navigate.setDescription("Clear from Bed location");
           enqueue(navigate);
       }

       // Go from Bed to Hallway
       {
           SubMission navigate(SUB_CMD(sendTask_, LF_BED_TO_HALLWAY_PREFIX + bed_id_));
           navigate.addFallbackTask(SUB_CMD(sendTask_, LF_HALLWAY_TO_BED_PREFIX + bed_id_));
           navigate.setDescription("Go back from Bed to Hallway");
           enqueue(navigate);
       }

       // Publish Robot Location
       {
           SubMission publish_robot_location(SUB_CMD(publishMsg_, ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_HALLWAY));
           publish_robot_location.setDescription("Update Robot Location");
           enqueue(publish_robot_location);
       }

       // Open Door
       {
           SubMission door_open(SUB_CMD(publishBool_, DOOR_CONTROL_TOPIC, DOOR_CONTROL_FIELD, DOOR_OPEN));
           door_open.addFallbackTask(SUB_CMD(publishBool_, DOOR_CONTROL_TOPIC, DOOR_CONTROL_FIELD, DOOR_CLOSE));
       }

       // Go from Hallway to Manipulator
       {
           SubMission navigate(SUB_CMD(sendTask_, LF_HALLWAY_TO_MANIPULATOR));
           navigate.fallbackAllowed(false);
           navigate.setDescription("Go from Hallway to Manipulator");
           enqueue(navigate);
       }

       // Safety Off
       {
           SubMission safety_off(SUB_CMD(sendTask_, SAFETY_OFF));
           safety_off.addFallbackTask(SUB_CMD(sendTask_, SAFETY_ON));
           safety_off.setDescription("Turn OFF Obstacle Guard");
           enqueue(safety_off);
       }

       // Close Door
       {
           SubMission door_open(SUB_CMD(publishBool_, DOOR_CONTROL_TOPIC, DOOR_CONTROL_FIELD, DOOR_CLOSE));
           door_open.addFallbackTask(SUB_CMD(publishBool_, DOOR_CONTROL_TOPIC, DOOR_CONTROL_FIELD, DOOR_OPEN));
       }

       // Handover Payload to Manipulator
       {
           SubMission release_payload(SUB_CMD(sendTask_, RELEASE_PAYLOAD_TO_BED));
           release_payload.setDescription("Handover Payload to Manipulator");
           enqueue(release_payload);
       }

       // Publish Robot Location
       {
           SubMission publish_robot_location(SUB_CMD(publishMsg_, ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_MANIPULATOR));
           publish_robot_location.setDescription("Update Robot Location");
           enqueue(publish_robot_location);
       }

       // Safety Off
       {
           SubMission safety_off(SUB_CMD(sendTask_, SAFETY_OFF));
           safety_off.addFallbackTask(SUB_CMD(sendTask_, SAFETY_ON));
           safety_off.setDescription("Turn OFF Obstacle Guard");
           enqueue(safety_off);
       }

       // Go from Manipulator to Charger
       {
           SubMission navigate(SUB_CMD(sendTask_, LF_MANIPULATOR_TO_CHARGER));
           navigate.setDescription("Go from Manipulator to Charger");
           enqueue(navigate);
       }

       // Start Docking
       {
           SubMission dock(SUB_CMD(sendTask_, DOCK_TO_CHARGER));
           dock.setDescription("Dock to Robot charger");
           enqueue(dock);
       }

       // Publish Robot Status
       {
           SubMission publish_robot_state(SUB_CMD(publishMsg_, ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_CHARGING));
           publish_robot_state.setDescription("Update Robot State");
           enqueue(publish_robot_state);
       }

       // Publish Robot Location
       {
           SubMission publish_robot_location(SUB_CMD(publishMsg_, ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_CHARGER));
           publish_robot_location.setDescription("Update Robot Location");
           enqueue(publish_robot_location);
       }

       // Set Success State
       robot_success_state_ = RobotState::Charging;
    }

    else if (message["command"] == "park")
    {
        // Check Robot State
        if (robot_state != RobotState::Charging)
        {
            status.success = false;
            status.msg = "SubMissionGenerator: Invalid Robot State. Cannot Start Parking";
            console_->print(status.msg);
            return status;
        }

        // Start Undocking
        {
            SubMission undock(SUB_CMD(sendTask_, UNDOCK_FROM_CHARGER));
            undock.addFallbackTask(SUB_CMD(sendTask_, DOCK_TO_CHARGER));
            undock.setDescription("Undock From Charger");
            enqueue(undock);
        }

        // Safety Off
        {
            SubMission safety_off(SUB_CMD(sendTask_, SAFETY_OFF));
            safety_off.addFallbackTask(SUB_CMD(sendTask_, SAFETY_ON));
            safety_off.setDescription("Turn OFF Obstacle Guard");
            enqueue(safety_off);
        }

        // Go from Charger to Manipulator
        {
            SubMission navigate(SUB_CMD(sendTask_, LF_CHARGER_TO_MANIPULATOR));
            navigate.addFallbackTask(SUB_CMD(sendTask_, LF_MANIPULATOR_TO_CHARGER));
            navigate.setDescription("Go from Charger to Manipulator");
            enqueue(navigate);
        }

        // Collect Payload from Manipulator
        {
            SubMission collect_payload(SUB_CMD(sendTask_, COLLECT_PAYLOAD_FROM_MANIPULATOR));
            collect_payload.fallbackAllowed(false);
            collect_payload.setDescription("Collect Payload from Manipulator");
            enqueue(collect_payload);
        }

        // Publish Robot Location
        {
            SubMission publish_robot_location(SUB_CMD(publishMsg_, ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_MANIPULATOR));
            publish_robot_location.setDescription("Update Robot Location");
            enqueue(publish_robot_location);
        }

        // Open Door
        {
            SubMission door_open(SUB_CMD(publishBool_, DOOR_CONTROL_TOPIC, DOOR_CONTROL_FIELD, DOOR_OPEN));
            door_open.addFallbackTask(SUB_CMD(publishBool_, DOOR_CONTROL_TOPIC, DOOR_CONTROL_FIELD, DOOR_CLOSE));
        }

        // Safety Off
        {
            SubMission safety_off(SUB_CMD(sendTask_, SAFETY_OFF));
            safety_off.addFallbackTask(SUB_CMD(sendTask_, SAFETY_ON));
            safety_off.setDescription("Turn OFF Obstacle Guard");
            enqueue(safety_off);
        }

        // Go from Manipulator to Hallway
        {
            SubMission navigate(SUB_CMD(sendTask_, LF_MANIPULATOR_TO_HALLWAY));
            navigate.setDescription("Go from Manipulator to Hallway");
            enqueue(navigate);
        }

        // Safety On
        {
            SubMission safety_on(SUB_CMD(sendTask_, SAFETY_ON));
            safety_on.setDescription("Turn ON Obstacle Guard");
            enqueue(safety_on);
        }

        // Publish Robot Location
        {
            SubMission publish_robot_location(SUB_CMD(publishMsg_, ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_HALLWAY));
            publish_robot_location.setDescription("Update Robot Location");
            enqueue(publish_robot_location);
        }

        // Close Door
        {
            SubMission door_open(SUB_CMD(publishBool_, DOOR_CONTROL_TOPIC, DOOR_CONTROL_FIELD, DOOR_CLOSE));
            door_open.addFallbackTask(SUB_CMD(publishBool_, DOOR_CONTROL_TOPIC, DOOR_CONTROL_FIELD, DOOR_OPEN));
        }

        // Enter Parking Location from Hallway
        {
            SubMission navigate(SUB_CMD(sendTask_, LF_HALLWAY_TO_PARKING));
            navigate.setDescription("Enter Parking Location");
            enqueue(navigate);
        }

        // Publish Robot Status
        {
            SubMission publish_robot_state(SUB_CMD(publishMsg_, ROBOT_STATUS_TOPIC, ROBOT_STATUS_FIELD, ROBOT_STATUS_IDLE));
            publish_robot_state.setDescription("Update Robot State");
            enqueue(publish_robot_state);
        }

        // Publish Robot Location
        {
            SubMission publish_robot_location(SUB_CMD(publishMsg_, ROBOT_LOCATION_TOPIC, ROBOT_LOCATION_FIELD, LOCATION_PARKING));
            publish_robot_location.setDescription("Update Robot Location");
            enqueue(publish_robot_location);
        }

        // Set Success State
        robot_success_state_ = RobotState::Standby;
    }

    else if (message["command"] == "door_open")
    {
        // Open Door
        {
            SubMission door_open(SUB_CMD(publishBool_, DOOR_CONTROL_TOPIC, DOOR_CONTROL_FIELD, DOOR_OPEN));
            door_open.addFallbackTask(SUB_CMD(publishBool_, DOOR_CONTROL_TOPIC, DOOR_CONTROL_FIELD, DOOR_CLOSE));
        }
    }

    else if (message["command"] == "door_close")
    {
        // Close Door
        {
            SubMission door_open(SUB_CMD(publishBool_, DOOR_CONTROL_TOPIC, DOOR_CONTROL_FIELD, DOOR_CLOSE));
            door_open.addFallbackTask(SUB_CMD(publishBool_, DOOR_CONTROL_TOPIC, DOOR_CONTROL_FIELD, DOOR_OPEN));
        }
    }

    else if (message["command"] == "safety_on")
    {
        // Safety On
        {
            SubMission safety_on(SUB_CMD(sendTask_, SAFETY_ON));
            safety_on.setDescription("Turn ON Obstacle Guard");
            enqueue(safety_on);
        }
    }

    else if (message["command"] == "safety_off")
    {
        // Safety Off
        {
            SubMission safety_off(SUB_CMD(sendTask_, SAFETY_OFF));
            safety_off.addFallbackTask(SUB_CMD(sendTask_, SAFETY_ON));
            safety_off.setDescription("Turn OFF Obstacle Guard");
            enqueue(safety_off);
        }
    }

    else if (message["command"] == "gripper_extend")
    {
        // Gripper Extend
        {
            SubMission gripper(SUB_CMD(sendTask_, GRIPPER_EXTEND));
            gripper.addFallbackTask(SUB_CMD(sendTask_, GRIPPER_RETRACT));
            gripper.setDescription("Gripper Extend");
            enqueue(gripper);
        }
    }

    else if (message["command"] == "gripper_retract")
    {
        // Gripper Retract
        {
            SubMission gripper(SUB_CMD(sendTask_, GRIPPER_RETRACT));
            gripper.setDescription("Gripper Retract");
            enqueue(gripper);
        }
    }

    else if (message["command"] == "gripper_clamp")
    {
        // Gripper Clamp
        {
            SubMission gripper(SUB_CMD(sendTask_, GRIPPER_CLAMP));
            gripper.setDescription("Gripper Clamp");
            enqueue(gripper);
        }
    }

    else if (message["command"] == "gripper_release")
    {
        // Gripper Release
        {
            SubMission gripper(SUB_CMD(sendTask_, GRIPPER_RELEASE));
            gripper.setDescription("Gripper Release");
            enqueue(gripper);
        }
    }

    else if (message["command"] == "gripper_extended_clamp")
    {
        // Gripper Extended Clamp
        {
            SubMission gripper(SUB_CMD(sendTask_, GRIPPER_EXTENDED_CLAMP));
            gripper.setDescription("Gripper Extended Clamp");
            enqueue(gripper);
        }
    }

    else if (message["command"] == "gripper_extended_release")
    {
        // Gripper Extended Release
        {
            SubMission gripper(SUB_CMD(sendTask_, GRIPPER_EXTENDED_RELEASE));
            gripper.setDescription("Gripper Extended Release");
            enqueue(gripper);
        }
    }

    else
    {
        status.success = false;
        status.msg = "Error: Unknown Command: " + message["command"].asString();
        console_->print(status.msg);
        return status;
    }

    status.success = true;
    status.msg = "Sub Missions Generated";
    console_->print(status.msg);
    return status;
}
