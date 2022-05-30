#include "robotCommunication.h"

RobotCommunication::RobotCommunication(boost::function<void (std::string)> callback, Console *console)
{
    callback_ = callback;
    console_ = console;
    cli = new mqtt::async_client(SERVER_ADDRESS, CLIENT_ID);
    keep_alive_ = true;

    // Connect MQTT Client
    connect_client();

    /// Initialize Connection status checker
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &RobotCommunication::check_status);
    timer_->start(1000);
}

RobotCommunication::~RobotCommunication()
{
    keep_alive_ = false;
    end_communication();
    delete cli;
}

bool RobotCommunication::publish(std::string topic, std::string msg)
{
    auto payload = mqtt::make_message(topic, msg);
    payload->set_qos(QOS);
    try{
        cli->publish(payload);
        return true;
    }
    catch(const mqtt::exception& exc){
        console_->print("Error: Mqtt exception. Attempting reconnection!");
        connect_client();
    }
    return false;
}

bool RobotCommunication::publish(std::string topic, std::string field, bool value)
{
    Json::Value message_json;
    Json::FastWriter writer;
    message_json[field] = value;
    return publish(topic, writer.write(message_json));
}

bool RobotCommunication::publish(std::string topic, std::string field, bool value, std::string msg)
{
    Json::Value message_json;
    Json::FastWriter writer;
    message_json[field] = value;
    message_json["message"] = msg;
    return publish(topic, writer.write(message_json));
}

bool RobotCommunication::publish(std::string topic, std::string field, std::string value)
{
    Json::Value message_json;
    Json::FastWriter writer;
    message_json[field] = value;
    return publish(topic, writer.write(message_json));
}

void RobotCommunication::end_communication()
{
    keep_alive_ = false;
    try {
        // Disconnect MQTT

        // Shutting down and disconnecting from the MQTT server
        cli->unsubscribe(TOPIC);
        cli->stop_consuming();
        cli->disconnect();
    }
    catch (const mqtt::exception& exc) {
        std::cerr << exc.what() << std::endl;
    }
}

void RobotCommunication::connect_client()
{
    connOpts.set_keep_alive_interval(36000);
    connOpts.set_clean_session(true);

    try {
        //cli->set_connected_handler([this](const std::string& cause){connected_cb(cause);});
        cli->connect(connOpts)->wait();
        cli->set_message_callback([this](mqtt::const_message_ptr msg) {callback_(msg->get_payload_str());});
        console_->print("Connected to Mqtt Server: " + SERVER_ADDRESS + ". Client: " + CLIENT_ID);
        cli->subscribe(TOPIC, QOS)->wait();
        console_->print("Subscribed to topic: " + TOPIC);

    }
    catch (const mqtt::exception& exc) {
        std::cerr << exc.what() << std::endl;
    }
}

void RobotCommunication::reconnect_client()
{
    try {
        cli->reconnect();
    }
    catch (const mqtt::exception& exc) {
        std::cerr << exc.what() << std::endl;
    }
}

void RobotCommunication::connected_cb(const std::string& cause)
{
    Q_UNUSED(cause);
//    console_->print("Connected to Mqtt Server: " + SERVER_ADDRESS + ". Client: " + CLIENT_ID);
//    cli->subscribe(TOPIC, QOS);
//    console_->print("Subscribed to topic: " + TOPIC);
}

void RobotCommunication::disconnected_cb(const mqtt::properties &properties, mqtt::ReasonCode cause)
{
    Q_UNUSED(properties);
    Q_UNUSED(cause);

    console_->print("Warning: Mqtt disconnected. Reconnecting now...");
    if (keep_alive_)
    {
        cli->connect(connOpts);
    }
}

void RobotCommunication::check_status(void)
{
    //publish("robot_heartbeat", "alive");
    //std::cout << cli->is_connected() << std::endl;
    if (!cli->is_connected())
    {
        console_->print("Error: WiFi Network Reconnection Detected. Attempting MQTT reconnection!");
        connect_client();
    }
}
