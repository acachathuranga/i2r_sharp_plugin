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
#include "command_processor/submissiongenerator.h"
#include <mutex>
#include <thread>
#include <atomic>
#include <QQueue>
#include <QThread>

#include "../../common/fsm_defs.h"

class CommandProcessor : public QThread
{
    public:
        enum class RobotState {
            Charging,       // At Robot Charger
            Standby,        // At parking, with commode
            Idle,           // At parking, without commode
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
        std::atomic<bool> mission_status_;
        QString task_manager_command;
        QQueue<SubMission> sub_mission_list;
        QQueue<SubMission> sub_mission_history;
        SubMissionGenerator *sub_mission_generator;
        bool revert_changes_ = false;

        RobotState robotState = RobotState::Charging;


        void run();
        bool sendTask(QString file_name);
        bool publishMsg (QString topic, QString field, QString msg);
        bool publishBool (QString topic, QString field, bool value);

        void taskManager(QString command);
        void fallBackTasks();
        void resumeTasks();

};

#endif // COMMANDPROCESSOR_H
