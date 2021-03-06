#ifndef MAGIC_HOME_CONFIG_H
#define MAGIC_HOME_CONFIG_H

#define CONFIG_TRIGGER_PIN 0

#define MQTT_TOPIC_STATE "light/bed"
#define MQTT_TOPIC_SET "light/bed/set"

#define MQTT_TOPIC_STATUS "light/bed/status"
#define MQTT_STATUS_PAYLOAD_ON "Online"
#define MQTT_STATUS_PAYLOAD_OFF "Offline"

#define PIN_RED 12
#define PIN_GREEN 13
#define PIN_BLUE 14

#define WIFI_AP_NAME "LED_bed"
#define WIFI_AP_PASS "123456"

#define INTERVAL_PUBLISH_STATE 600000 // Interval to send statistics to the mqtt broker

#endif
