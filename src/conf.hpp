#ifndef CONF_HPP
#define CONF_HPP

#include "../../pwd.hpp"
#include "../../wlan.hpp"

#define WEBNAME "schalter_1"

#define MQTTROOT "/net_dev/"
#define MQTTCMD "/state"

#define MQTTSTATE MQTTROOT WEBNAME MQTTCMD

namespace net_dev {

extern const char* hostname;
const char* hostname = WEBNAME;

namespace update {

    extern const char* web_update_path;
    const char* web_update_path = WEBNAME ".local/firmware";

    extern const unsigned int update_port;
    const unsigned int update_port = 80u;

    extern const char* user;
    const char* user = UPDATE_USERNAME;

    extern const char* pass;
    const char* pass = UPDATE_PASSWORT;
}

namespace mqtt_device {

    extern const char* mqtt_server;
    const char* mqtt_server = MAKELIGHT_MQTT_SERVER;

    extern const int mqtt_server_port;
    const int mqtt_server_port = MAKELIGHT_MQTT_SERVER_PORT;

    extern const char* mqtt_device_topic;
    const char* mqtt_device_topic = MQTTSTATE;

    extern const char* mqtt_device_topic_time;
    const char* mqtt_device_topic_time = MQTTROOT WEBNAME "/time";

    extern const char* mqtt_device_topic_timetick;
    const char* mqtt_device_topic_timetick = MQTTROOT WEBNAME "/timetick";

    extern const char* mqtt_device_topic_date;
    const char* mqtt_device_topic_date = MQTTROOT WEBNAME "/date";
}
}
#endif
