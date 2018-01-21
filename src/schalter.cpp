#include "schalter.h"

void setup();
void loop();
void publish_switchState(void*);
void update_mdns(void*);

void ISR_switch(void*);
void setup_wifi();
void setup_OTA();
void mqtt_subscribe_callback(const MQTT::Publish& msg);

void ntp_loop(void*);
void startUDP();
uint32_t getTime();
void sendNTPpacket(IPAddress& address);
inline int getSeconds(uint32_t UNIXTime);
inline int getMinutes(uint32_t UNIXTime);
inline int getHours(uint32_t UNIXTime);

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
const char* NTPServerName = "pool.ntp.org";
const int NTP_PACKET_SIZE = 48;
byte NTPBuffer[NTP_PACKET_SIZE];
unsigned long intervalNTP = 120000; // Request NTP time every minute
unsigned long prevNTP = 0;
unsigned long lastNTPResponse = millis();
uint32_t timeUNIX = 0;
unsigned long prevActualTime = 0;
uint8 tmz = 1;

// Ticker to update brightness
static TickerScheduler ticker(2);

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
    Serial.setDebugOutput(true);
    Serial.println("Start...");
    delay(10);

    setup_wifi();
    setup_OTA();

    mqtt_lastId = -1;
    client.set_server(mqtt_server, mqtt_server_port);
    client.set_callback(mqtt_subscribe_callback);

    // initialize scheduler
    ticker.add(0, 500, publish_switchState, nullptr, true);
    ticker.add(1, 500, ntp_loop, nullptr, true);
    ticker.enableAll();

    pinMode(D5, INPUT);
    attachInterrupt(digitalPinToInterrupt(D5), ISR_switch, FALLING);

    startUDP();
    if (!WiFi.hostByName(NTPServerName, timeServerIP)) {
        Serial.println("DNS lookup of time server failed!");
    } else {
        Serial.print("Timer server IP:\t");
        Serial.println(timeServerIP);
        Serial.println("\r\nSending NTP request... ");
        sendNTPpacket(timeServerIP);
    }
}

void ntp_loop(void*)
{
    unsigned long currentMillis = millis();

    if (currentMillis - prevNTP > intervalNTP) { // If a minute has passed since last NTP request
        prevNTP = currentMillis;
        sendNTPpacket(timeServerIP); // Send an NTP request
    }

    uint32_t time = getTime(); // Check if an NTP response has arrived and get the (UNIX) time
    if (time) { // If a new timestamp has been received
        timeUNIX = time;
        lastNTPResponse = currentMillis;
    } else if ((currentMillis - lastNTPResponse) > 3600000) {
        Serial.println("More than 1 hour since last NTP response!");
        Serial.flush();
    }

    uint32_t actualTime = timeUNIX + (currentMillis - lastNTPResponse) / 1000;
    if (actualTime != prevActualTime && timeUNIX != 0) { // If a second has passed since last print
        prevActualTime = actualTime;
        uint32_t localTime = actualTime + tmz * 3600;
        int h = getHours(localTime);
        int m = getMinutes(localTime);
        int s = getSeconds(localTime);
        Serial.printf("%-15s%02d:%02d:%02d", "\rLocal time:", h, m, s);
    }
}

void startUDP()
{
    Serial.println("Starting UDP");
    udp.begin(123);
    Serial.print("Local port:\t");
    Serial.println(udp.localPort());
    Serial.println();
}

uint32_t getTime()
{
    if (udp.parsePacket() == 0) { // If there's no response (yet)
        return 0;
    }
    udp.read(NTPBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
    // Combine the 4 timestamp bytes into one 32-bit number
    uint32_t NTPTime = (NTPBuffer[40] << 24) | (NTPBuffer[41] << 16) | (NTPBuffer[42] << 8) | NTPBuffer[43];
    // Convert NTP time to a UNIX timestamp:
    // Unix time starts on Jan 1 1970. That's 2208988800 seconds in NTP time:
    const uint32_t seventyYears = 2208988800UL;
    // subtract seventy years:
    uint32_t UNIXTime = NTPTime - seventyYears;
    return UNIXTime;
}

void sendNTPpacket(IPAddress& address)
{
    memset(NTPBuffer, 0, NTP_PACKET_SIZE); // set all bytes in the buffer to 0
    // Initialize values needed to form NTP request
    NTPBuffer[0] = 0b11100011; // LI, Version, Mode
    // send a packet requesting a timestamp:
    udp.beginPacket(address, 123); // NTP requests are to port 123
    udp.write(NTPBuffer, NTP_PACKET_SIZE);
    udp.endPacket();
}

inline int getSeconds(uint32_t UNIXTime)
{
    return UNIXTime % 60;
}

inline int getMinutes(uint32_t UNIXTime)
{
    return UNIXTime / 60 % 60;
}

inline int getHours(uint32_t UNIXTime)
{
    return UNIXTime / 3600 % 24;
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

    while (WiFi.status() != WL_CONNECTED) {
        digitalWrite(BUILTIN_LED, LOW);
        delay(500);
        Serial.print(".");
        digitalWrite(BUILTIN_LED, HIGH);
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    digitalWrite(BUILTIN_LED, HIGH);
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
    if (client.connected())
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
            Serial.print("failed!");
            Serial.println(" try again in 2 seconds");
            ctr = 1;
        }
    } else {
        ctr = ctr >= 2000 ? 0 : ctr + 1;
    }
}

void loop()
{
    if (WiFi.isConnected() == false) {
        WiFi.reconnect();
        Serial.print(".");
        delay(1000);
        return;
    }

    if (!client.connected()) {
        reconnect_mqtt();
    } else {
        client.loop();
    }

    // do what, you need to do
    ticker.update();

    update_mdns(nullptr);
}

void publish_switchState(void*)
{
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
        // reset transition state
        savedPS = xt_rsil(15);
        transitionOccured = false;
        xt_wsr_ps(savedPS);
    }
}

void update_mdns(void*)
{
    if (WiFi.isConnected()) {
        MDNS.update();
    }
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
