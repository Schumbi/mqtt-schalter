#ifndef SCHALTER_H
#define SCHALTER_H

#include <Arduino.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <PubSubClient.h>
#include <TickerScheduler.h>

#include "conf.hpp"

// net_dev
static const char* hostname = WEBNAME;
// wifi
static const char* myssid = wifi_config::ssid;
static const char* mypass = wifi_config::ssid_password;
// mqtt
static const char* mqtt_server = net_dev::mqtt_device::mqtt_server;
static const int mqtt_server_port = net_dev::mqtt_device::mqtt_server_port;
static const String mqtt_state_string = net_dev::mqtt_device::mqtt_device_topic;
// web update
static const char* update_path = net_dev::update::web_update_path;
static const int update_port = net_dev::update::update_port;
static const char* update_username = net_dev::update::user;
static const char* update_passwort = net_dev::update::pass;

#endif // SCHALTER_H
