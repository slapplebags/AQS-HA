/*
  ESP32 + (SEN66 OR BMV080) + MQTT + Home Assistant Discovery
  with WiFiManager captive-portal config

  Portal lets you enter Wi-Fi + MQTT + HA prefix + friendly name + sensor type (sen66|bmv080).
*/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>

// Increase MQTT packet size for HA discovery payloads
#define MQTT_MAX_PACKET_SIZE 1024

#include <PubSubClient.h>
#include <Preferences.h>
#include <WiFiManager.h>

// SEN66
#include <SensirionI2cSen66.h>

// BMV080
#include "SparkFun_BMV080_Arduino_Library.h"

// ---------------- Settings & constants ----------------
#ifndef NO_ERROR
#define NO_ERROR 0
#endif

#define CONFIG_TRIGGER_PIN 0

// Publish interval (MQTT + HA entities update cadence)
const uint32_t READ_INTERVAL_MS = 10000;

// BMV080 wants frequent servicing; this keeps it happy even when publishing slower
const uint32_t BMV_SERVICE_MS = 50;

// BMV080 default I2C addr on SparkFun breakout
#define BMV080_ADDR 0x57

// ---------------- Globals ----------------
WiFiClient espClient;
PubSubClient mqtt(espClient);
Preferences prefs;

// SEN66 instance
SensirionI2cSen66 sen66;

// BMV080 instance
SparkFunBMV080 bmv080;

static char errorMessage[64];
static int16_t error;

// Runtime IDs/topics
String device_id;
String mqtt_client_id;
String base_topic;
String avail_topic;

// ---------------- Config ----------------
enum SensorType : uint8_t { SENSOR_SEN66 = 0, SENSOR_BMV080 = 1 };

struct AppConfig {
  char mqtt_host[64]      = "10.18.14.36";
  char mqtt_port[8]       = "1883";
  char mqtt_user[64]      = "front_door";
  char mqtt_pass[64]      = "!90c6965dD";
  char ha_prefix[32]      = "homeassistant";
  char friendly_name[32]  = "Air Sensor";
  char sensor_type[16]    = "sen66"; // "sen66" or "bmv080"
} cfg;

SensorType activeSensor = SENSOR_SEN66;

bool shouldSaveConfig = false;
bool discovery_published = false;

unsigned long last_publish_ms = 0;
unsigned long last_bmv_service_ms = 0;

// BMV080 cached values (updated frequently)
float bmv_pm1 = NAN, bmv_pm25 = NAN, bmv_pm10 = NAN;
bool  bmv_obstructed = false;

// ---------------- Forward declarations ----------------
void loadConfig();
void saveConfig();
void ensureMqtt();

void publishDiscovery();
void publishAvailability(const char* state);
void publishOne(const String& suffix, const String& value);

void publishConfigSensor(const String& object_id, const String& name, const String& unit,
                         const String& device_class, const String& state_class, const String& icon,
                         const String& state_topic_suffix);

void publishConfigBinarySensor(const String& object_id, const String& name,
                               const String& device_class, const String& icon,
                               const String& state_topic_suffix,
                               const String& payload_on, const String& payload_off);

void publishSen66State(float pm1, float pm25, float pm4, float pm10,
                       float rh, float tempC, float vocIndex, float noxIndex, uint16_t co2);

void publishBmv080State(float pm1, float pm25, float pm10, bool obstructed);

String chipId();
void safeTopicify(String &s);

bool initSen66();
bool initBmv080();

// ---------------- WiFiManager save callback ----------------
void saveConfigCallback() { shouldSaveConfig = true; }

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(CONFIG_TRIGGER_PIN, INPUT_PULLUP);

  Wire.begin();

  // Load stored config first (so portal defaults are correct)
  loadConfig();

  // Decide active sensor from cfg
  if (String(cfg.sensor_type).equalsIgnoreCase("bmv080")) {
    activeSensor = SENSOR_BMV080;
  } else {
    activeSensor = SENSOR_SEN66;
  }

  // Create a device_id that is stable and unique per board.
  // (If you want SEN66 serial-based IDs only, keep that inside initSen66().)
  device_id = chipId();

  // WiFiManager
  bool forcePortal = (digitalRead(CONFIG_TRIGGER_PIN) == LOW);

  WiFi.mode(WIFI_STA);
  WiFiManager wm;
  wm.setDebugOutput(true);
  wm.setSaveConfigCallback(saveConfigCallback);

  String apName = "AIR-" + chipId();

  // Custom params for MQTT & HA & Sensor Type
  WiFiManagerParameter p_mqtt_host("mqtt_host", "MQTT Host", cfg.mqtt_host, sizeof(cfg.mqtt_host));
  WiFiManagerParameter p_mqtt_port("mqtt_port", "MQTT Port", cfg.mqtt_port, sizeof(cfg.mqtt_port));
  WiFiManagerParameter p_mqtt_user("mqtt_user", "MQTT User", cfg.mqtt_user, sizeof(cfg.mqtt_user));
  WiFiManagerParameter p_mqtt_pass("mqtt_pass", "MQTT Password", cfg.mqtt_pass, sizeof(cfg.mqtt_pass));
  WiFiManagerParameter p_ha_prefix("ha_prefix", "HA Discovery Prefix", cfg.ha_prefix, sizeof(cfg.ha_prefix));
  WiFiManagerParameter p_friendly("friendly_name", "Device Friendly Name", cfg.friendly_name, sizeof(cfg.friendly_name));
  WiFiManagerParameter p_sensor_type("sensor_type", "Sensor Type (sen66|bmv080)", cfg.sensor_type, sizeof(cfg.sensor_type));

  wm.addParameter(&p_mqtt_host);
  wm.addParameter(&p_mqtt_port);
  wm.addParameter(&p_mqtt_user);
  wm.addParameter(&p_mqtt_pass);
  wm.addParameter(&p_ha_prefix);
  wm.addParameter(&p_friendly);
  wm.addParameter(&p_sensor_type);

  bool wifi_ok = false;
  if (forcePortal) {
    Serial.println("Config trigger active — starting captive portal...");
    wifi_ok = wm.startConfigPortal(apName.c_str());
  } else {
    Serial.println("Attempting Wi-Fi autoconnect...");
    wifi_ok = wm.autoConnect(apName.c_str());
  }

  if (!wifi_ok) {
    Serial.println("Wi-Fi not configured/connected. Rebooting in 5s...");
    delay(5000);
    ESP.restart();
  }

  // Copy values back from portal
  strncpy(cfg.mqtt_host, p_mqtt_host.getValue(), sizeof(cfg.mqtt_host));
  strncpy(cfg.mqtt_port, p_mqtt_port.getValue(), sizeof(cfg.mqtt_port));
  strncpy(cfg.mqtt_user, p_mqtt_user.getValue(), sizeof(cfg.mqtt_user));
  strncpy(cfg.mqtt_pass, p_mqtt_pass.getValue(), sizeof(cfg.mqtt_pass));
  strncpy(cfg.ha_prefix, p_ha_prefix.getValue(), sizeof(cfg.ha_prefix));
  strncpy(cfg.friendly_name, p_friendly.getValue(), sizeof(cfg.friendly_name));
  strncpy(cfg.sensor_type, p_sensor_type.getValue(), sizeof(cfg.sensor_type));

  // Normalize sensor type
  String st = String(cfg.sensor_type);
  st.trim();
  st.toLowerCase();
  if (st != "bmv080") st = "sen66";
  strncpy(cfg.sensor_type, st.c_str(), sizeof(cfg.sensor_type));

  if (strlen(cfg.ha_prefix) == 0) {
    strncpy(cfg.ha_prefix, "homeassistant", sizeof(cfg.ha_prefix));
  }

  if (shouldSaveConfig) {
    saveConfig();
    Serial.println("Config saved.");
  }

  // Determine active sensor now that config is final
  activeSensor = (String(cfg.sensor_type) == "bmv080") ? SENSOR_BMV080 : SENSOR_SEN66;

  // Init sensor and compute device_id (SEN66 prefers serial-based id)
  bool sensor_ok = false;
  if (activeSensor == SENSOR_SEN66) {
    sensor_ok = initSen66();
  } else {
    sensor_ok = initBmv080();
  }

  if (!sensor_ok) {
    Serial.println("Sensor init failed. Rebooting in 5s...");
    delay(5000);
    ESP.restart();
  }

  safeTopicify(device_id);
  mqtt_client_id = "esp32_" + device_id;

  // Topics depend on sensor type
  String prefix = (activeSensor == SENSOR_SEN66) ? "sen66/" : "bmv080/";
  base_topic  = prefix + device_id;
  avail_topic = base_topic + "/status";

  Serial.println("Base topic: " + base_topic);
  Serial.println("Availability topic: " + avail_topic);

  // MQTT setup
  mqtt.setServer(cfg.mqtt_host, atoi(cfg.mqtt_port));
  mqtt.setBufferSize(768);

  Serial.print("Wi-Fi connected, IP: ");
  Serial.println(WiFi.localIP());
}

// ---------------- Loop ----------------
void loop() {
  ensureMqtt();

  // Keep BMV080 serviced frequently even if we publish slowly
  if (activeSensor == SENSOR_BMV080) {
    unsigned long now = millis();
    if (now - last_bmv_service_ms >= BMV_SERVICE_MS) {
      last_bmv_service_ms = now;

      if (bmv080.readSensor()) {
        bmv_pm10 = bmv080.PM10();
        bmv_pm25 = bmv080.PM25();
        bmv_pm1  = bmv080.PM1();
        bmv_obstructed = bmv080.isObstructed();
      }
    }
  }

  // Publish cadence
  unsigned long now = millis();
  if (now - last_publish_ms >= READ_INTERVAL_MS) {
    last_publish_ms = now;

    if (activeSensor == SENSOR_SEN66) {
      float pm1=0, pm25=0, pm4=0, pm10=0;
      float rh=0, tempC=0, voc=0, nox=0;
      uint16_t co2=0;

      error = sen66.readMeasuredValues(pm1, pm25, pm4, pm10, rh, tempC, voc, nox, co2);
      if (error != NO_ERROR) {
        errorToString(error, errorMessage, sizeof errorMessage);
        Serial.printf("SEN66 readMeasuredValues() error: %s\n", errorMessage);
      } else {
        Serial.printf("SEN66 PM1=%.1f PM2.5=%.1f PM4=%.1f PM10=%.1f RH=%.1f T=%.2f VOC=%.1f NOx=%.1f CO2=%u\n",
                      pm1, pm25, pm4, pm10, rh, tempC, voc, nox, co2);
        publishSen66State(pm1, pm25, pm4, pm10, rh, tempC, voc, nox, co2);
      }
    } else {
      if (!isnan(bmv_pm1) && !isnan(bmv_pm25) && !isnan(bmv_pm10)) {
        Serial.printf("BMV080 PM1=%.2f PM2.5=%.2f PM10=%.2f Obstructed=%s\n",
                      bmv_pm1, bmv_pm25, bmv_pm10, bmv_obstructed ? "true" : "false");
        publishBmv080State(bmv_pm1, bmv_pm25, bmv_pm10, bmv_obstructed);
      } else {
        Serial.println("BMV080: no readings yet (still warming/servicing).");
      }
    }
  }

  mqtt.loop();
}

// --------------- Sensor init ---------------
bool initSen66() {
  sen66.begin(Wire, SEN66_I2C_ADDR_6B);

  error = sen66.deviceReset();
  if (error != NO_ERROR) {
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.printf("SEN66 deviceReset() error: %s\n", errorMessage);
  }
  delay(1200);

  // Prefer SEN66 serial number for device_id
  int8_t serialNumber[32] = {0};
  error = sen66.getSerialNumber(serialNumber, 32);
  if (error == NO_ERROR) {
    device_id = "sen66_" + String((const char*)serialNumber);
  } else {
    device_id = "sen66_" + chipId();
  }

  error = sen66.startContinuousMeasurement();
  if (error != NO_ERROR) {
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.printf("SEN66 startContinuousMeasurement() error: %s\n", errorMessage);
    return false;
  }

  return true;
}

bool initBmv080() {
  // BMV080 uses SparkFun library init sequence
  if (!bmv080.begin(BMV080_ADDR, Wire)) {
    Serial.println("BMV080 not detected at 0x57. Check wiring/jumper.");
    return false;
  }
  Serial.println("BMV080 found!");

  bmv080.init();

  if (!bmv080.setMode(SF_BMV080_MODE_CONTINUOUS)) {
    Serial.println("BMV080 setMode(CONTINUOUS) failed.");
    return false;
  }

  // device_id for BMV080: stable per board
  device_id = "bmv080_" + chipId();
  return true;
}

// --------------- Config (NVS) ---------------
void loadConfig() {
  prefs.begin("aircfg", true);
  String host = prefs.getString("mqtt_host", cfg.mqtt_host);
  String port = prefs.getString("mqtt_port", cfg.mqtt_port);
  String user = prefs.getString("mqtt_user", cfg.mqtt_user);
  String pass = prefs.getString("mqtt_pass", cfg.mqtt_pass);
  String pref = prefs.getString("ha_prefix", cfg.ha_prefix);
  String name = prefs.getString("friendly", cfg.friendly_name);
  String styp = prefs.getString("sensor_type", cfg.sensor_type);
  prefs.end();

  strncpy(cfg.mqtt_host, host.c_str(), sizeof(cfg.mqtt_host));
  strncpy(cfg.mqtt_port, port.c_str(), sizeof(cfg.mqtt_port));
  strncpy(cfg.mqtt_user, user.c_str(), sizeof(cfg.mqtt_user));
  strncpy(cfg.mqtt_pass, pass.c_str(), sizeof(cfg.mqtt_pass));
  strncpy(cfg.ha_prefix, pref.c_str(), sizeof(cfg.ha_prefix));
  strncpy(cfg.friendly_name, name.c_str(), sizeof(cfg.friendly_name));
  strncpy(cfg.sensor_type, styp.c_str(), sizeof(cfg.sensor_type));

  if (strlen(cfg.ha_prefix) == 0) strncpy(cfg.ha_prefix, "homeassistant", sizeof(cfg.ha_prefix));
  if (strlen(cfg.sensor_type) == 0) strncpy(cfg.sensor_type, "sen66", sizeof(cfg.sensor_type));

  Serial.printf("Loaded config: MQTT %s:%s, HA prefix='%s', name='%s', sensor_type='%s'\n",
                cfg.mqtt_host, cfg.mqtt_port, cfg.ha_prefix, cfg.friendly_name, cfg.sensor_type);
}

void saveConfig() {
  prefs.begin("aircfg", false);
  prefs.putString("mqtt_host", cfg.mqtt_host);
  prefs.putString("mqtt_port", cfg.mqtt_port);
  prefs.putString("mqtt_user", cfg.mqtt_user);
  prefs.putString("mqtt_pass", cfg.mqtt_pass);
  prefs.putString("ha_prefix", cfg.ha_prefix);
  prefs.putString("friendly",  cfg.friendly_name);
  prefs.putString("sensor_type", cfg.sensor_type);
  prefs.end();
}

// --------------- MQTT helpers ---------------
void ensureMqtt() {
  if (mqtt.connected()) return;

  while (!mqtt.connected()) {
    Serial.printf("MQTT connecting to %s:%s ...\n", cfg.mqtt_host, cfg.mqtt_port);

    bool ok = false;
    if (strlen(cfg.mqtt_user)) {
      ok = mqtt.connect(
        mqtt_client_id.c_str(),
        cfg.mqtt_user, cfg.mqtt_pass,
        avail_topic.c_str(), 1, true, "offline"
      );
    } else {
      ok = mqtt.connect(
        mqtt_client_id.c_str(),
        avail_topic.c_str(), 1, true, "offline"
      );
    }

    if (!ok) {
      Serial.printf("MQTT connect failed (state %d). Retrying...\n", mqtt.state());
      delay(2000);
      continue;
    }

    Serial.println("MQTT connected.");
    publishAvailability("online");
    publishDiscovery();
  }
}

void publishAvailability(const char* state) {
  mqtt.publish(avail_topic.c_str(), state, true);
}

void publishOne(const String& suffix, const String& value) {
  String topic = base_topic + "/" + suffix;
  mqtt.publish(topic.c_str(), value.c_str(), false);
}

// --------------- Home Assistant Discovery ---------------
void publishDiscovery() {
  if (discovery_published) {
    Serial.println("Discovery already published, skipping.");
    return;
  }

  Serial.printf("Publishing HA discovery prefix='%s' device_id='%s' sensor='%s'\n",
                cfg.ha_prefix, device_id.c_str(), (activeSensor == SENSOR_SEN66) ? "sen66" : "bmv080");

  if (activeSensor == SENSOR_SEN66) {
    publishConfigSensor("pm1",        "PM1.0",       "µg/m³", "pm1",            "measurement", "mdi:blur",            "pm1");
    publishConfigSensor("pm25",       "PM2.5",       "µg/m³", "pm25",           "measurement", "",                    "pm25");
    publishConfigSensor("pm4",        "PM4.0",       "µg/m³", "",               "measurement", "mdi:blur",            "pm4");
    publishConfigSensor("pm10",       "PM10",        "µg/m³", "pm10",           "measurement", "",                    "pm10");
    publishConfigSensor("humidity",   "Humidity",    "%",     "humidity",       "measurement", "",                    "humidity");
    publishConfigSensor("temperature","Temperature", "°C",    "temperature",    "measurement", "",                    "temperature");
    publishConfigSensor("voc_index",  "VOC Index",   "",      "",               "measurement", "mdi:chemical-weapon", "voc_index");
    publishConfigSensor("nox_index",  "NOx Index",   "",      "",               "measurement", "mdi:chemical-weapon", "nox_index");
    publishConfigSensor("co2eq",      "CO2 (eq)",    "ppm",   "carbon_dioxide", "measurement", "",                    "co2eq");
  } else {
    publishConfigSensor("pm1",   "PM1.0",  "µg/m³", "pm1",  "measurement", "mdi:blur", "pm1");
    publishConfigSensor("pm25",  "PM2.5",  "µg/m³", "pm25", "measurement", "",         "pm25");
    publishConfigSensor("pm10",  "PM10",   "µg/m³", "pm10", "measurement", "",         "pm10");

    // Binary sensor for obstruction
    publishConfigBinarySensor("obstructed", "Optics Obstructed",
                              "problem", "mdi:alert-circle",
                              "obstructed",
                              "ON", "OFF");
  }

  discovery_published = true;
  Serial.println("Discovery publish complete.");
}

void publishConfigSensor(
  const String& object_id,
  const String& name,
  const String& unit,
  const String& device_class,
  const String& state_class,
  const String& icon,
  const String& state_topic_suffix
) {
  String topic = String(cfg.ha_prefix) + "/sensor/" + device_id + "/" + object_id + "/config";

  String payload = "{";
  payload += "\"name\":\"" + name + "\",";
  payload += "\"unique_id\":\"" + device_id + "_" + object_id + "\",";
  payload += "\"state_topic\":\"" + base_topic + "/" + state_topic_suffix + "\",";
  payload += "\"availability_topic\":\"" + avail_topic + "\",";

  if (unit.length())         payload += "\"unit_of_measurement\":\"" + unit + "\",";
  if (device_class.length()) payload += "\"device_class\":\"" + device_class + "\",";
  if (state_class.length())  payload += "\"state_class\":\"" + state_class + "\",";
  if (icon.length())         payload += "\"icon\":\"" + icon + "\",";

  payload += "\"device\":{";
  payload +=   "\"identifiers\":[\"" + device_id + "\"],";
  payload +=   "\"manufacturer\":\"" + String((activeSensor == SENSOR_SEN66) ? "Sensirion" : "Bosch") + "\",";
  payload +=   "\"model\":\"" + String((activeSensor == SENSOR_SEN66) ? "SEN66" : "BMV080") + "\",";
  payload +=   "\"name\":\"" + String(cfg.friendly_name) + "\"";
  payload += "}";
  payload += "}";

  bool ok = mqtt.publish(topic.c_str(), payload.c_str(), true);
  Serial.printf("HA sensor config publish '%s': %s (len=%d)\n",
                object_id.c_str(), ok ? "OK" : "FAILED", payload.length());
}

void publishConfigBinarySensor(
  const String& object_id,
  const String& name,
  const String& device_class,
  const String& icon,
  const String& state_topic_suffix,
  const String& payload_on,
  const String& payload_off
) {
  String topic = String(cfg.ha_prefix) + "/binary_sensor/" + device_id + "/" + object_id + "/config";

  String payload = "{";
  payload += "\"name\":\"" + name + "\",";
  payload += "\"unique_id\":\"" + device_id + "_" + object_id + "\",";
  payload += "\"state_topic\":\"" + base_topic + "/" + state_topic_suffix + "\",";
  payload += "\"availability_topic\":\"" + avail_topic + "\",";
  payload += "\"payload_on\":\"" + payload_on + "\",";
  payload += "\"payload_off\":\"" + payload_off + "\",";

  if (device_class.length()) payload += "\"device_class\":\"" + device_class + "\",";
  if (icon.length())         payload += "\"icon\":\"" + icon + "\",";

  payload += "\"device\":{";
  payload +=   "\"identifiers\":[\"" + device_id + "\"],";
  payload +=   "\"manufacturer\":\"Bosch\",";
  payload +=   "\"model\":\"BMV080\",";
  payload +=   "\"name\":\"" + String(cfg.friendly_name) + "\"";
  payload += "}";
  payload += "}";

  bool ok = mqtt.publish(topic.c_str(), payload.c_str(), true);
  Serial.printf("HA binary_sensor config publish '%s': %s (len=%d)\n",
                object_id.c_str(), ok ? "OK" : "FAILED", payload.length());
}

// --------------- State publishers ---------------
void publishSen66State(float pm1, float pm25, float pm4, float pm10,
                       float rh, float tempC, float vocIndex, float noxIndex, uint16_t co2) {
  if (!discovery_published && mqtt.connected()) publishDiscovery();

  publishOne("pm1",         String(pm1, 1));
  publishOne("pm25",        String(pm25, 1));
  publishOne("pm4",         String(pm4, 1));
  publishOne("pm10",        String(pm10, 1));
  publishOne("humidity",    String(rh, 1));
  publishOne("temperature", String(tempC, 2));
  publishOne("voc_index",   String(vocIndex, 1));
  publishOne("nox_index",   String(noxIndex, 1));
  publishOne("co2eq",       String(co2));
}

void publishBmv080State(float pm1, float pm25, float pm10, bool obstructed) {
  if (!discovery_published && mqtt.connected()) publishDiscovery();

  publishOne("pm1",  String(pm1, 2));
  publishOne("pm25", String(pm25, 2));
  publishOne("pm10", String(pm10, 2));

  // For binary_sensor payloads
  publishOne("obstructed", obstructed ? "ON" : "OFF");
}

// --------------- Utilities ---------------
String chipId() {
  uint64_t mac = ESP.getEfuseMac();
  uint32_t low = (uint32_t)(mac & 0xFFFFFF);
  char buf[9];
  snprintf(buf, sizeof(buf), "%06X", low);
  return String(buf);
}

void safeTopicify(String &s) {
  s.replace(" ", "_");
  s.replace("/", "_");
  s.replace("\\", "_");
  s.replace("+", "_");
  s.replace("#", "_");
}
