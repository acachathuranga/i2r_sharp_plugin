#ifndef ROBOTCOMMUNICATION_H
#define ROBOTCOMMUNICATION_H

#include <QObject>
#include <mqtt/async_client.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <Tools/console.h>
#include <jsoncpp/json/json.h>
#include <QTimer>

class RobotCommunication : public QObject
{
    Q_OBJECT

    public:
        RobotCommunication(boost::function<void (std::string)> callback, Console *console);
        ~RobotCommunication();

        void end_communication();
        bool publish(std::string topic, std::string msg);
        bool publish(std::string topic, std::string field, bool value);
        bool publish(std::string topic, std::string field, bool value, std::string msg);
        bool publish(std::string topic, std::string field, std::string value);

    private:
        const std::string SERVER_ADDRESS	{ "tcp://192.168.10.147:1883" };
        const std::string CLIENT_ID         { "robot" };
        const std::string TOPIC 			{ "robot_depart" };
        const int  QOS = 1;
        bool keep_alive_ = true;

        mqtt::connect_options connOpts;
        mqtt::async_client* cli;

        boost::function<void (std::string)> callback_;
        Console *console_;
        QTimer *timer_;

        void connected_cb(const std::string& cause);
        void disconnected_cb(const mqtt::properties &properties, mqtt::ReasonCode cause);
        void connect_client();
        void reconnect_client();
        void check_status(void);
};

#endif // ROBOTCOMMUNICATION_H
