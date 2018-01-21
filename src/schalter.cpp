#include "schalter.h"

void setup();
void loop();
void publish_switchState(void*);
void update_mdns(void*);

void ISR_switch(void*);
void setup_wifi();
void setup_OTA();
void callback(const MQTT::Publish& msg);

static uint16 mqtt_lastId;

// for OTA
static ESP8266WebServer httpServer(update_port);
static ESP8266HTTPUpdateServer httpUpdater;

// create MQTT client
static WiFiClient espClient;
static PubSubClient client(espClient);

static volatile bool switchedOn = false;
static volatile bool transitionOccured = false;

// Ticker to update brightness
static TickerScheduler ticker(1);

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
    client.set_callback(callback);

    // initialize scheduler
    ticker.add(0, 500, publish_switchState, nullptr, true);
    ticker.enableAll();

    pinMode(D5, INPUT);
    attachInterrupt(digitalPinToInterrupt(D5), ISR_switch, FALLING);
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

void callback(const MQTT::Publish& msg)
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
    httpUpdater.setup(&httpServer, update_path, update_username, update_passwort);
    httpServer.begin();
    MDNS.addService("http", "tcp", update_port);
    Serial.println("OTA setup finished!");
}
