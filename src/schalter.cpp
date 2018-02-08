#include "schalter.h"

#include <RTClib.h>
#include <Wire.h>
#include <sntp.h>
#include <time.h>

void setup();
void loop();
void publish_switchState(void*);
void publish_time(void*);
void update_mdns(void*);

void ISR_switch(void*);
void setup_wifi();
void setup_OTA();
void mqtt_subscribe_callback(const MQTT::Publish& msg);

void ntp_loop(void*);
time_t getTimeFromRTC();

static uint16 mqtt_lastId;

// for OTA
static ESP8266WebServer httpServer(update_port);
static ESP8266HTTPUpdateServer httpUpdater;

// create MQTT client
static WiFiClient espClient;
static PubSubClient client(espClient);

static volatile bool switchedOn = false;
static volatile bool transitionOccured = false;

// Getting time from NTP
WiFiUDP udp;
IPAddress timeServerIP;
char* NTPServerName = "0.europe.pool.ntp.org";
const int NTP_PACKET_SIZE = 48;
byte NTPBuffer[NTP_PACKET_SIZE];

time_t intervalNTPMax = 1000 * 60 * 10; // Request NTP time 10 minutes
time_t intervalNTPMin = 1000 * 60 * 1; // Request NTP time 1 minute
time_t intervalNTP = intervalNTPMin;
time_t timeAtLastNTPSync = STARTDATE;
time_t lastTimeUpdate = 0;
time_t globalTime = STARTDATE;

// Ticker to update brightness
static TickerScheduler ticker(3);

RTC_DS1307 rtc;

void ISR_switch()
{
    if (transitionOccured == false) {
        switchedOn = !switchedOn;
        transitionOccured = true;
    }
}

void setup()
{
    pinMode(BUILTIN_LED, OUTPUT); // Initialize the BUILTIN_LED pin as an output

    // Serial Stuff
    Serial.begin(9600);
    Serial.setDebugOutput(false);
    Serial.println("Start...");
    delay(10);

    setup_wifi();
    setup_OTA();

    Wire.begin(SDA, SCL);

    mqtt_lastId = -1;
    client.set_server(mqtt_server, mqtt_server_port);
    client.set_callback(mqtt_subscribe_callback);

    // initialize scheduler
    ticker.add(0, 500, publish_switchState, nullptr, true);
    ticker.add(1, 1000, ntp_loop, nullptr, true);
    ticker.add(2, 1000, publish_time, nullptr, true);
    ticker.enableAll();

    pinMode(D5, INPUT);
    attachInterrupt(digitalPinToInterrupt(D5), ISR_switch, FALLING);

    // Berlin/Europe
    sntp_set_timezone(+1);
    sntp_setservername(0, NTPServerName);

    sntp_init();

    rtc.begin();
    delay(1000 * 3); // wait 3 sceonds for RTC to finish work
    timeAtLastNTPSync = getTimeFromRTC();
    globalTime = timeAtLastNTPSync;
    lastTimeUpdate = millis();
}

void ntp_loop(void*)
{
    time_t time = 0;
    time_t currentMillis = millis();

    if (timeAtLastNTPSync <= STARTDATE || (currentMillis - lastTimeUpdate >= intervalNTP)) {
        // try retrieve time from ntp
        time = sntp_get_current_timestamp();
        Serial.println("");
        // store last ntp time stamp
        timeAtLastNTPSync = time;
        if (time > 0) {
            // time has value, ntp succeded
            rtc.adjust(DateTime(time));
            // store current millis to get time between syncs
            lastTimeUpdate = currentMillis;
            // set interval to max
            intervalNTP = intervalNTPMax;
        } else {
            // error occured, set time to start date 2018-1-1 and add time from start
            timeAtLastNTPSync = STARTDATE + currentMillis / 1000;
            // print error with newline to see error on serial
            Serial.println("!N");
            // set interval to minimum
            intervalNTP = intervalNTPMin;
        }
    }

    if (rtc.isrunning()) {
        // if rtc is running use time from device
        time = getTimeFromRTC();
    } else {
        // RTC not running, print error
        Serial.print("!R");
        // try time from last ntp sync and add millis run
        time = timeAtLastNTPSync + (currentMillis - lastTimeUpdate) / 1000;
        // try to adjust the RTC
        rtc.adjust(DateTime(time));
    }

    // store time
    globalTime = time;

    // example to use the time with std libc
    struct tm* t = localtime(&globalTime);
    int tag = t->tm_mday;
    int monat = t->tm_mon + 1;
    int jahr = t->tm_year + 1900;
    int stunde = t->tm_hour;
    int minute = t->tm_min;
    int sekunde = t->tm_sec;
    Serial.printf("\t%02d.%02d %4d\t%02d:%02d:%02d\r",
        tag, monat, jahr,
        stunde, minute, sekunde);
}

void setup_wifi()
{
    digitalWrite(BUILTIN_LED, LOW);
    delay(10);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(myssid);

    WiFi.hostname(hostname);
    WiFi.begin(myssid, mypass);

    delay(100);
}

void mqtt_subscribe_callback(const MQTT::Publish& msg)
{
    if (msg.topic().equals(mqtt_state_string)) {
        // check if there was a state before we loose connection
        uint16 id = msg.packet_id();
        if (id != mqtt_lastId) {
            mqtt_lastId = id;
            int state = atoi(msg.payload_string().c_str());
            // reset
            bool sw = false;
            uint32_t savedPS = xt_rsil(15);
            switchedOn = state > 0;
            sw = switchedOn;
            xt_wsr_ps(savedPS);
            // set blue led accordingly
            digitalWrite(BUILTIN_LED, sw ? LOW : HIGH);
        }
    }
}

void reconnect_mqtt()
{
    if (client.connected() || WiFi.isConnected() == false)
        return;

    static int ctr = 0;
    // Loop until we're reconnected
    if (ctr == 0) {
        Serial.print("MQTT connection... ");
        // connect
        if (client.connect(hostname)) {
            Serial.println("connected!");
            // ... and resubscribe
            MQTT::Subscribe sub(mqtt_state_string, MQTT::QOS1);
            Serial.println(String("Subscribed to ") + mqtt_state_string);
            client.subscribe(sub);
            digitalWrite(BUILTIN_LED, HIGH);
            ctr = 0;
        } else {
            digitalWrite(BUILTIN_LED, LOW);
            Serial.print("#");
            ctr = 1;
        }
    } else {
        ctr = ctr >= 1000 ? 0 : ctr + 1;
    }
}

void loop()
{
    int ctr = 0;
    if (WiFi.isConnected() == false && ctr < 5) {
        //        setup_wifi();
        while (WiFi.status() != WL_CONNECTED && ctr < 10) {
            digitalWrite(BUILTIN_LED, !digitalRead(LED_BUILTIN));
            delay(100);
            Serial.print(".");
            ctr++;
        }

        if (WiFi.isConnected()) {
            Serial.println("");
            Serial.println("WiFi connected!");
            Serial.print("IP address: ");
            Serial.println(WiFi.localIP());
            digitalWrite(BUILTIN_LED, HIGH);
        }
    }

    if (WiFi.isConnected()) {
        if (!client.connected()) {
            reconnect_mqtt();
        } else {
            client.loop();
        }
    }

    // do what, you need to do
    ticker.update();

    update_mdns(nullptr);
}

void publish_switchState(void*)
{
    if (WiFi.isConnected() == false) {
        return;
    }

    bool transOccured = false;
    bool switched = false;
    // disable interrupts
    uint32_t savedPS = xt_rsil(15);
    transOccured = transitionOccured;
    xt_wsr_ps(savedPS);

    if (transOccured && client.connected()) {
        savedPS = xt_rsil(15);
        switched = switchedOn;
        xt_wsr_ps(savedPS);
        String state = switched ? "1" : "0";
        MQTT::Publish msg(mqtt_state_string, state);
        msg.set_qos(MQTT::QOS1);
        msg.set_retain(true);
        client.publish(msg);

        struct tm* t = localtime(&globalTime);

        int tag = t->tm_mday;
        int monat = t->tm_mon + 1;
        int jahr = t->tm_year + 1900;
        int stunde = t->tm_hour;
        int minute = t->tm_min;
        int sekunde = t->tm_sec;

        Serial.printf("Switched %3s at: %02d:%02d:%02d (%02d.%02d %4d)\n",
            switched ? "ON" : "OFF",
            stunde, minute, sekunde,
            tag, monat, jahr);

        // reset transition state
        savedPS = xt_rsil(15);
        transitionOccured = false;
        xt_wsr_ps(savedPS);
    }
}

void publish_time(void*)
{
    if (WiFi.isConnected() == false) {
        return;
    }

    struct tm* data = localtime(&globalTime);

    char buf[9];
    snprintf(buf, 9, "%02d:%02d:%02d", data->tm_hour, data->tm_min, data->tm_sec);

    String payload = String(buf);
    MQTT::Publish msg(mqtt_time_string, payload);
    msg.set_retain(false);
    msg.set_qos(MQTT::QOS0);
    client.publish(msg);

    payload = String(data->tm_sec % 2);
    MQTT::Publish msg1 = MQTT::Publish(mqtt_time_tick, payload);
    client.publish(msg1);

    char buf2[11];
    snprintf(buf2, 11, "%02d.%02d.%04d", data->tm_mday, data->tm_mon, data->tm_year + 1900);
    payload = String(buf2);
    MQTT::Publish msg2 = MQTT::Publish(mqtt_date_string, payload);
    client.publish(msg2);
}

void update_mdns(void*)
{
    if (WiFi.isConnected() == false)
        return;
    MDNS.update();
}

void setup_OTA()
{
    digitalWrite(BUILTIN_LED, LOW);

    MDNS.begin(hostname);
    Serial.println(String("Set path to: ") + update_path + " User: " + update_username + "Pass: " + update_passwort + " ");
    httpUpdater.setup(&httpServer, update_path, update_username, update_passwort);
    httpServer.begin();
    MDNS.addService("http", "tcp", update_port);
    Serial.println("OTA setup finished!");
}

time_t getTimeFromRTC()
{
    return rtc.now().unixtime();
}
