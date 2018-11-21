#include <FS.h>                   //this needs to be first, or it all crashes and burns... (from Adafruit)
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          // https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h>          // https://github.com/bblanchon/ArduinoJson (ver: 5.x)
#include <AsyncMqttClient.h>      // https://github.com/marvinroger/async-mqtt-client + (https://github.com/me-no-dev/ESPAsyncTCP)
#include <SimpleTimer.h>          // https://github.com/schinken/SimpleTimer
#include "config.h"


// Default values can be defined here, if there are different values in config.json, they are overwritten.
char mqtt_server[40];
char mqtt_port[6] = "1883";
char mqtt_login[64];
char mqtt_password[64];
char mqtt_client_id[36];

// Light RGB state by default
int colorRED = 255;
int colorGREEN = 255;
int colorBLUE = 255;
int brightness = 1023;
bool isON = false;

// Flag for saving data
bool shouldSaveConfig = false;

// MQTT client
AsyncMqttClient mqttClient;

SimpleTimer timer;

// Global variables for fade transition
bool startFade = false;
unsigned long lastLoop = 0;
unsigned long transitionTime = 0;
bool inFade = false;
int loopCount = 0;
int stepR, stepG, stepB;
int redVal, grnVal, bluVal;


// Callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}


void readConfigurationFile() {
  //read configuration from FS json
  Serial.println("Mounting FS...");
  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("Reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("JSON config parsed");

          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(mqtt_login, json["mqtt_login"]);
          strcpy(mqtt_password, json["mqtt_password"]);
          strcpy(mqtt_client_id, json["mqtt_client_id"]);

        } else {
          Serial.println("Failed to load json config");
        }
      }
    } else {
      Serial.println("File /config.json not found");
    }
  } else {
    Serial.println("Failed to mount FS");
  }

  Serial.println("Read data from the config file:");
  Serial.printf("  mqtt server: %s\n", mqtt_server);
  Serial.printf("  mqtt port: %s\n", mqtt_port);
  Serial.printf("  mqtt login: %s\n", mqtt_login);
  Serial.printf("  mqtt password: %s\n", mqtt_password);
  Serial.printf("  mqtt client id: %s\n", mqtt_client_id);
}


void writeConfigurationFile() {
  Serial.println("Saving configuration");
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["mqtt_server"] = mqtt_server;
  json["mqtt_port"] = mqtt_port;
  json["mqtt_login"] = mqtt_login;
  json["mqtt_password"] = mqtt_password;
  json["mqtt_client_id"] = mqtt_client_id;

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println("Failed to open config file for writing");
  } else {
    Serial.println("Save data in the config file /config.json");
  }

  json.prettyPrintTo(Serial);
  json.printTo(configFile);
  Serial.println();
  configFile.close();
  shouldSaveConfig = false;
}


void createCustomWiFiManager(bool _resetSettings) {
  // The extra parameters to be configured
  WiFiManagerParameter custom_text("<p>MQTT Server</p>");
  WiFiManagerParameter custom_mqtt_server("mqtt_server", "MQTT server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("mqtt_port", "MQTT port", mqtt_port, 5);
  WiFiManagerParameter custom_mqtt_login("mqtt_login", "MQTT login", mqtt_login, 64);
  WiFiManagerParameter custom_mqtt_password("mqtt_password", "MQTT password", mqtt_password, 64);
  WiFiManagerParameter custom_mqtt_client_id("mqtt_client_id", "MQTT client id", mqtt_client_id, 64);
  

  // WiFiManager
  // Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  // Set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  // Add all custom parameters
  wifiManager.addParameter(&custom_text);
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_login);
  wifiManager.addParameter(&custom_mqtt_password);
  wifiManager.addParameter(&custom_mqtt_client_id);

  // Reset settings if needed
  if (_resetSettings) {
    wifiManager.resetSettings();
  }

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  wifiManager.setMinimumSignalQuality();

  // Sets timeout until configuration portal gets turned off
  // useful to make it all retry or go to sleep in seconds
  wifiManager.setTimeout(120);

  // Fetches ssid and pass and tries to connect
  // if it does not connect it starts an access point with the specified name
  // and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect(WIFI_AP_NAME, WIFI_AP_PASS)) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  // If you get here you have connected to the WiFi
  Serial.println("Connected to WiFi");

  // Read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(mqtt_login, custom_mqtt_login.getValue());
  strcpy(mqtt_password, custom_mqtt_password.getValue());
  strcpy(mqtt_client_id, custom_mqtt_client_id.getValue());
}


void setColor(int red, int green, int blue) {
  analogWrite(PIN_RED, red);
  analogWrite(PIN_GREEN, green);
  analogWrite(PIN_BLUE, blue);
  Serial.printf("Set output color: red=%d green=%d blue=%d\n", red, green, blue);
}


char *uptime(unsigned long milli) {
  static char _return[32];
  unsigned long secs=milli/1000, mins=secs/60;
  unsigned int hours=mins/60, days=hours/24;
  milli-=secs*1000;
  secs-=mins*60;
  mins-=hours*60;
  hours-=days*24;
  sprintf(_return,"%dT%2.2d:%2.2d:%2.2d.%3.3d", (byte)days, (byte)hours, (byte)mins, (byte)secs, (int)milli);
  return _return;
}


void setup() {
  pinMode(CONFIG_TRIGGER_PIN, INPUT);

  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_BLUE, OUTPUT);

  analogWrite(PIN_RED, LOW);
  analogWrite(PIN_GREEN, LOW);
  analogWrite(PIN_BLUE, LOW);

  Serial.begin(115200);
  Serial.println();

  //clean FS, for testing
  //SPIFFS.format();

  readConfigurationFile();

  createCustomWiFiManager(false);

  // Save the custom parameters to FS
  if (shouldSaveConfig) {
    writeConfigurationFile();
  }

  // Configure MQTT
  int p = atoi(mqtt_port);
  mqttClient.setServer(mqtt_server, p);
  mqttClient.setCredentials(mqtt_login, mqtt_password);
  mqttClient.setKeepAlive(30);
  mqttClient.setWill(MQTT_TOPIC_STATUS, 1, true, MQTT_STATUS_PAYLOAD_OFF); //topic, QoS, retain, payload

  String string_client_id(mqtt_client_id);
  string_client_id.trim();
  if (string_client_id != String("")) {
    mqttClient.setClientId(mqtt_client_id);
  }

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onMessage(onMqttMessage);

  Serial.println("MQTT: Connecting to broker...");
  mqttClient.connect();

  // Publish state periodicly
  timer.setInterval(INTERVAL_PUBLISH_STATE, publishState);

  //WiFi.setSleepMode(WIFI_NONE_SLEEP);//TODO: need to try this
}


void onMqttConnect(bool sessionPresent) {
  Serial.println("MQTT: Connected");
  Serial.print("MQTT: Session present: ");
  Serial.println(sessionPresent);

  Serial.print("MQTT: Subscribing at QoS 0, topic: ");
  Serial.println(MQTT_TOPIC_SET);
  mqttClient.subscribe(MQTT_TOPIC_SET, 0);

  Serial.print("MQTT: Publish online status: ");
  Serial.println(MQTT_TOPIC_STATUS);
  mqttClient.publish(MQTT_TOPIC_STATUS, 1, true, MQTT_STATUS_PAYLOAD_ON);

  publishState();
}


void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println();
  Serial.print("MQTT: Disconnected: ");
  if (reason == AsyncMqttClientDisconnectReason::TCP_DISCONNECTED) {
    Serial.println("TCP disconnected");
  } else if (reason == AsyncMqttClientDisconnectReason::MQTT_UNACCEPTABLE_PROTOCOL_VERSION) {
    Serial.println("Unacceptable protocol version");
  } else if (reason == AsyncMqttClientDisconnectReason::MQTT_IDENTIFIER_REJECTED) {
    Serial.println("Indentifier rejected");
  } else if (reason == AsyncMqttClientDisconnectReason::MQTT_SERVER_UNAVAILABLE) {
    Serial.println("Server unavailable");
  } else if (reason == AsyncMqttClientDisconnectReason::MQTT_MALFORMED_CREDENTIALS) {
    Serial.println("Malformed credentials");
  } else if (reason == AsyncMqttClientDisconnectReason::MQTT_NOT_AUTHORIZED) {
    Serial.println("Not authorized");
  } else {
    Serial.println("Unknown reason");
  }

  delay(3000);
  Serial.println("MQTT: Reconnecting to broker...");
  if (WiFi.isConnected()) {
    mqttClient.connect();
  }
}


void publishState() {
  const int BUFFER_SIZE = JSON_OBJECT_SIZE(20);
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;

  // Light state: state, color, brightness
  JsonObject& root = jsonBuffer.createObject();
  root["state"] = (isON) ? "ON" : "OFF";
  JsonObject& color = root.createNestedObject("color");
  color["r"] = colorRED;
  color["g"] = colorGREEN;
  color["b"] = colorBLUE;
  root["brightness"] = brightness;

  // Additional parameters: ip-address, mac-address, RSSI, uptime
  // IP-address
  char ip[16];
  memset(ip, 0, 18);
  sprintf(ip, "%s", WiFi.localIP().toString().c_str());
  root["ip"] = ip;

  // MAC address
  uint8_t macAddr[6];
  WiFi.macAddress(macAddr);
  char mac[18];
  memset(mac, 0, 18);
  sprintf(mac, "%02X:%02X:%02X:%02X:%02X:%02X", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
  root["mac"] = mac;

  //RSSI
  char rssi[8];
  sprintf(rssi, "%d", WiFi.RSSI());
  root["rssi"] = rssi;

  //  Uptime
  root["uptime"] = uptime( millis() );
  
  char buffer[root.measureLength() + 1];
  root.printTo(buffer, sizeof(buffer));

  Serial.printf("\nMQTT: Publish state: %s\n", buffer);

  mqttClient.publish(MQTT_TOPIC_STATE, 0, true, buffer);
}


void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println();
  Serial.println("MQTT: Message received.");
  Serial.printf("  topic: %s\n", topic);
  Serial.printf("  qos: %d\n", properties.qos);
  Serial.printf("  dup: %d\n", properties.dup);
  Serial.printf("  retain: %d\n", properties.retain);

  Serial.print("  payload: ");
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.parseObject(payload);
  json.printTo(Serial);
  Serial.println();

  if (json.success()) {
    if (json.containsKey("state")) {
      const char* state = json["state"];
      String stringState(state);
      String stringOn("ON");
      Serial.printf("  state: %s\n", state);
      if (stringState == stringOn) {
        isON = true;
        
        if (json.containsKey("brightness")) {
          brightness = json["brightness"];
          Serial.printf("  brightness: %d\n", brightness);
        }

        if (json.containsKey("color")) {
          colorRED = json["color"]["r"];
          colorGREEN = json["color"]["g"];
          colorBLUE = json["color"]["b"];
        }

        if (json.containsKey("transition")) {
          transitionTime = json["transition"];
          Serial.printf("  transition: %d\n", transitionTime);
          brightness = 1023; //NOTE: With a transition brightness goes from min to max
          startFade = true;
        } else {
          inFade = false;
          int realRed = map(colorRED, 0, 255, 0, brightness);
          int realGreen = map(colorGREEN, 0, 255, 0, brightness);
          int realBlue = map(colorBLUE, 0, 255, 0, brightness);
          setColor(realRed, realGreen, realBlue);
        }

      } else {
        inFade = false;
        if (isON) {
          setColor(0, 0, 0);
          isON = false;
        }
      }
      publishState();
    }

  } else {
    Serial.println("parseObject() failed");
  }
}


void loop() {
  
  if ( digitalRead(CONFIG_TRIGGER_PIN) == LOW ) {
    //TODO: this part has to be fixed
    mqttClient.disconnect(true);
    createCustomWiFiManager(true);
    mqttClient.connect();
  }

  timer.run();

  //-------------------------
  if (startFade) {
    Serial.println("Start the fade effect");

    // If we don't want to fade, skip it.
    if (transitionTime == 0) {
      int realRed = map(colorRED, 0, 255, 0, brightness);
      int realGreen = map(colorGREEN, 0, 255, 0, brightness);
      int realBlue = map(colorBLUE, 0, 255, 0, brightness);
      setColor(realRed, realGreen, realBlue);

      redVal = colorRED;
      grnVal = colorGREEN;
      bluVal = colorBLUE;

      startFade = false;
    } else {
      loopCount = 0;

      stepR = calculateStep(redVal, colorRED);
      stepG = calculateStep(grnVal, colorGREEN);
      stepB = calculateStep(bluVal, colorBLUE);

      inFade = true;
    }
  }

  if (inFade) {
    startFade = false;
    unsigned long now = millis();
    if (now - lastLoop > transitionTime) {
      if (loopCount <= 1020) {
        lastLoop = now;

        redVal = calculateVal(stepR, redVal, loopCount);
        grnVal = calculateVal(stepG, grnVal, loopCount);
        bluVal = calculateVal(stepB, bluVal, loopCount);

        int realRed = map(redVal, 0, 255, 0, brightness);
        int realGreen = map(grnVal, 0, 255, 0, brightness);
        int realBlue = map(bluVal, 0, 255, 0, brightness);
        setColor(realRed, realGreen, realBlue);

        Serial.printf("Loop count: %d\n", loopCount);
        loopCount++;
      }
      else {
        inFade = false;
      }
    }
  }
}



// From https://www.arduino.cc/en/Tutorial/ColorCrossfader
/* BELOW THIS LINE IS THE MATH -- YOU SHOULDN'T NEED TO CHANGE THIS FOR THE BASICS

  The program works like this:
  Imagine a crossfade that moves the red LED from 0-10,
    the green from 0-5, and the blue from 10 to 7, in
    ten steps.
    We'd want to count the 10 steps and increase or
    decrease color values in evenly stepped increments.
    Imagine a + indicates raising a value by 1, and a -
    equals lowering it. Our 10 step fade would look like:

    1 2 3 4 5 6 7 8 9 10
  R + + + + + + + + + +
  G   +   +   +   +   +
  B     -     -     -

  The red rises from 0 to 10 in ten steps, the green from
  0-5 in 5 steps, and the blue falls from 10 to 7 in three steps.

  In the real program, the color percentages are converted to
  0-255 values, and there are 1020 steps (255*4).

  To figure out how big a step there should be between one up- or
  down-tick of one of the LED values, we call calculateStep(),
  which calculates the absolute gap between the start and end values,
  and then divides that gap by 1020 to determine the size of the step
  between adjustments in the value.
*/
int calculateStep(int prevValue, int endValue) {
  int step = endValue - prevValue; // What's the overall gap?
  if (step) {                      // If its non-zero,
    step = 1020 / step;          //   divide by 1020
  }

  return step;
}

/* The next function is calculateVal. When the loop value, i,
   reaches the step size appropriate for one of the
   colors, it increases or decreases the value of that color by 1.
   (R, G, and B are each calculated separately.)
*/
int calculateVal(int step, int val, int i) {
  if ((step) && i % step == 0) { // If step is non-zero and its time to change a value,
    if (step > 0) {              //   increment the value if step is positive...
      val += 1;
    }
    else if (step < 0) {         //   ...or decrement it if step is negative
      val -= 1;
    }
  }

  // Defensive driving: make sure val stays in the range 0-255
  if (val > 255) {
    val = 255;
  }
  else if (val < 0) {
    val = 0;
  }

  return val;
}
