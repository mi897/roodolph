/*
  ESP32 Sensor Node
  - DHT11 (humidity + temp)
  - BMP280 (temp + pressure)
  - Weighted temperature: configurable weights (w_dht, w_bmp)
  - Publish JSON over MQTT
  - FreeRTOS tasks + message queue
  - Persist configuration in Preferences

  Dependencies (install via Library Manager):
    - DHT sensor library (Adafruit or similar) -> DHT.h
    - Adafruit_BMP280 (Adafruit BMP280 Library)
    - PubSubClient (for MQTT)
    - ArduinoJson (for parsing config JSON)
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <Adafruit_BMP280.h>
#include <Preferences.h>
#include <ArduinoJson.h>

// ---------- USER CONFIG ----------
#ifndef WIFI_SSID
#define WIFI_SSID ""
#endif
#ifndef WIFI_PASS
#define WIFI_PASS ""
#endif
#ifndef MQTT_SERVER
#define MQTT_SERVER ""
#endif
#ifndef MQTT_PORT
#define MQTT_PORT 1883
#endif
#ifndef MQTT_USER
#define MQTT_USER ""
#endif
#ifndef MQTT_PASSWD
#define MQTT_PASSWD ""
#endif

const char* MQTT_BASE_TOPIC = "sensornode/esp32_01"; // base topic for this node
// publishes telemetry to: sensornode/esp32_01/telemetry
// subscribes to config topic: sensornode/esp32_01/config

// I2C pins for BMP280 (ESP32 default commonly)
// Change if you wired differently
const int I2C_SDA = 21;
const int I2C_SCL = 22;

const int DHT_PIN = 4;     // digital pin connected to DHT11 data pin
#define DHTTYPE DHT11      // DHT 11

// Defaults (persisted in Preferences)
const float DEFAULT_W_DHT = 0.5;   // weight for DHT temperature
const float DEFAULT_W_BMP = 0.5;   // weight for BMP temperature
const uint32_t DEFAULT_INTERVAL_S = 60; // publish every 60 seconds

// Queue length for messages to publish
const int MSG_QUEUE_LEN = 10;
const int MSG_MAX_LEN = 512; // max JSON length

// ---------- Globals ----------
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
DHT dht(DHT_PIN, DHTTYPE);
Adafruit_BMP280 bmp; // I2C
Preferences prefs;

// Config values (persisted)
float w_dht = DEFAULT_W_DHT;
float w_bmp = DEFAULT_W_BMP;
uint32_t interval_s = DEFAULT_INTERVAL_S;

// FreeRTOS queue for outgoing messages (char arrays)
QueueHandle_t msgQueue = NULL;

// forward declarations
void sensorTask(void* pvParameters);
void mqttTask(void* pvParameters);
void handleMqttConfig(const char* payload, size_t length);
void publishConfig(); // publish current config as retained
String makeTelemetryJson(float humidity, float temp_dht, float temp_bmp, float temp_weighted, float pressure, uint32_t ts);

// ---------- Helper: persistent config ----------
void loadConfigFromPrefs() {
  prefs.begin("sensornode", true); // read-only open first to check
  if (prefs.isKey("w_dht") && prefs.isKey("w_bmp") && prefs.isKey("interval")) {
    prefs.end();
    prefs.begin("sensornode", false);
    w_dht = prefs.getFloat("w_dht", DEFAULT_W_DHT);
    w_bmp = prefs.getFloat("w_bmp", DEFAULT_W_BMP);
    interval_s = prefs.getUInt("interval", DEFAULT_INTERVAL_S);
    prefs.end();
  } else {
    // No stored config — write defaults
    prefs.begin("sensornode", false);
    prefs.putFloat("w_dht", DEFAULT_W_DHT);
    prefs.putFloat("w_bmp", DEFAULT_W_BMP);
    prefs.putUInt("interval", DEFAULT_INTERVAL_S);
    prefs.end();
    w_dht = DEFAULT_W_DHT;
    w_bmp = DEFAULT_W_BMP;
    interval_s = DEFAULT_INTERVAL_S;
  }
}

void saveConfigToPrefs() {
  prefs.begin("sensornode", false);
  prefs.putFloat("w_dht", w_dht);
  prefs.putFloat("w_bmp", w_bmp);
  prefs.putUInt("interval", interval_s);
  prefs.end();
}

// ---------- MQTT callbacks ----------
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Only config topic is subscribed to
  String t = String(topic);
  String configTopic = String(MQTT_BASE_TOPIC) + "/config";
  if (t == configTopic) {
    handleMqttConfig((const char*)payload, length);
  }
}

// ---------- Config handler (via MQTT JSON) ----------
/*
  Expected JSON:
  { "w_dht": 0.6, "w_bmp": 0.4, "interval": 30 }

  Any field omitted will remain unchanged.
*/
void handleMqttConfig(const char* payload, size_t length) {
  // parse with ArduinoJson (use small dynamic doc)
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    Serial.print("Config JSON parse error: ");
    Serial.println(err.c_str());
    return;
  }

  bool changed = false;
  if (doc.containsKey("w_dht")) {
    float new_w_dht = doc["w_dht"].as<float>();
    if (new_w_dht >= 0.0 && new_w_dht <= 1.0) {
      w_dht = new_w_dht;
      changed = true;
    }
  }
  if (doc.containsKey("w_bmp")) {
    float new_w_bmp = doc["w_bmp"].as<float>();
    if (new_w_bmp >= 0.0 && new_w_bmp <= 1.0) {
      w_bmp = new_w_bmp;
      changed = true;
    }
  }
  if (doc.containsKey("interval")) {
    uint32_t new_interval = doc["interval"].as<uint32_t>();
    if (new_interval >= 5 && new_interval <= 3600) {
      interval_s = new_interval;
      changed = true;
    }
  }

  // enforce weights sum rule: if both provided, keep as-is; else normalize to 1 if needed
  if (w_dht + w_bmp != 0.0f) {
    float sum = w_dht + w_bmp;
    if (fabs(sum - 1.0f) > 1e-6) {
      // normalize
      w_dht = w_dht / sum;
      w_bmp = w_bmp / sum;
    }
  } else {
    // fallback to defaults
    w_dht = DEFAULT_W_DHT;
    w_bmp = DEFAULT_W_BMP;
  }

  if (changed) {
    saveConfigToPrefs();
    // publish current config (retained) so others know
    publishConfig();
    Serial.printf("Config updated: w_dht=%.3f w_bmp=%.3f interval=%u\n", w_dht, w_bmp, interval_s);
  }
}

// ---------- Produce telemetry JSON ----------
String makeTelemetryJson(float humidity, float temp_dht, float temp_bmp, float temp_weighted, float pressure, uint32_t ts) {
  StaticJsonDocument<256> doc;
  doc["node"] = MQTT_BASE_TOPIC;
  doc["timestamp"] = ts; // epoch seconds (millis()/1000)
  JsonObject s = doc.createNestedObject("sensors");
  s["humidity"] = humidity;
  JsonObject t = s.createNestedObject("temperature");
  t["dht"] = temp_dht;
  t["bmp"] = temp_bmp;
  t["weighted"] = temp_weighted;
  s["pressure_hpa"] = pressure; // hPa
  doc["weights"]["w_dht"] = w_dht;
  doc["weights"]["w_bmp"] = w_bmp;
  char buf[384];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  return String(buf, n);
}

// ---------- Publish current config (retained) ----------
void publishConfig() {
  StaticJsonDocument<192> doc;
  doc["w_dht"] = w_dht;
  doc["w_bmp"] = w_bmp;
  doc["interval"] = interval_s;
  uint8_t buf[128];
  unsigned int n = serializeJson(doc, buf, sizeof(buf));
  String topic = String(MQTT_BASE_TOPIC) + "/config_status";
  // publish retained so clients can read current config
  mqttClient.publish(topic.c_str(), buf, n, true);
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32 Sensor Node starting...");

  // load persisted config
  loadConfigFromPrefs();
  Serial.printf("Loaded config: w_dht=%.3f w_bmp=%.3f interval=%u\n", w_dht, w_bmp, interval_s);

  // init I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  // init BMP280
  if (!bmp.begin(0x76)) { // try 0x76 then 0x77 if needed
    Serial.println("BMP280 not found at 0x76, trying 0x77...");
    if (!bmp.begin(0x77)) {
      Serial.println("Could not find BMP280 sensor! Check wiring.");
    } else {
      Serial.println("BMP280 found at 0x77");
    }
  } else {
    Serial.println("BMP280 found at 0x76");
  }
  // Set BMP280 sampling (optional)
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,   // temperature
                  Adafruit_BMP280::SAMPLING_X16,  // pressure
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  // init DHT
  dht.begin();

  // init WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Connecting WiFi to %s ...\n", WIFI_SSID);

  // init MQTT client
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  // create message queue
  msgQueue = xQueueCreate(MSG_QUEUE_LEN, MSG_MAX_LEN);
  if (msgQueue == NULL) {
    Serial.println("Failed to create message queue!");
    while (1) { delay(1000); } // fatal
  }

  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(mqttTask, "MQTTTask", 8192, NULL, 1, NULL, 1);

  // Nothing in loop()
}

// ---------- FreeRTOS sensor task ----------
void sensorTask(void* pvParameters) {
  (void) pvParameters;
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    // wait for interval
    uint32_t wait_ms = interval_s * 1000UL;
    // we'll use vTaskDelayUntil to maintain steady interval
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(wait_ms));

    // read humidity from DHT
    float humidity = NAN;
    float temp_dht = NAN;
    float temp_bmp = NAN;
    float pressure_hpa = NAN;

    // DHT reading - try a few times if NaN
    const int DHT_TRIES = 2;
    for (int i = 0; i < DHT_TRIES; ++i) {
      float h = dht.readHumidity();
      float t = dht.readTemperature();
      if (!isnan(h)) humidity = h;
      if (!isnan(t)) temp_dht = t;
      if (!isnan(h) && !isnan(t)) break;
      vTaskDelay(pdMS_TO_TICKS(250));
    }

    // BMP reading
    if (bmp.getTemperatureSensor()) { // not a robust check, but attempt
      temp_bmp = bmp.readTemperature();
      float p = bmp.readPressure(); // Pa
      if (!isnan(p)) {
        pressure_hpa = p / 100.0f; // convert to hPa
      }
    } else {
      // try reading once
      temp_bmp = bmp.readTemperature();
      float p = bmp.readPressure();
      if (!isnan(p)) pressure_hpa = p / 100.0f;
    }

    // If both temps not available, skip sending
    bool have_temp = (!isnan(temp_dht) && !isnan(temp_bmp)) || (!isnan(temp_dht) || !isnan(temp_bmp));
    if (!have_temp && isnan(humidity) && isnan(pressure_hpa)) {
      Serial.println("Sensor read failed (all NaN). Skipping this cycle.");
      continue;
    }

    // compute weighted temp:
    float temp_weighted = NAN;
    if (!isnan(temp_dht) && !isnan(temp_bmp)) {
      // apply weights
      temp_weighted = w_dht * temp_dht + w_bmp * temp_bmp;
    } else if (!isnan(temp_dht)) {
      temp_weighted = temp_dht;
    } else if (!isnan(temp_bmp)) {
      temp_weighted = temp_bmp;
    }

    uint32_t ts = (uint32_t)(millis() / 1000UL);

    String payload = makeTelemetryJson(
      isnan(humidity) ? -1.0f : humidity,
      isnan(temp_dht) ? -999.0f : temp_dht,
      isnan(temp_bmp) ? -999.0f : temp_bmp,
      isnan(temp_weighted) ? -999.0f : temp_weighted,
      isnan(pressure_hpa) ? -1.0f : pressure_hpa,
      ts
    );

    // Enqueue message (non-blocking, drop if full)
    char msgbuf[MSG_MAX_LEN];
    size_t len = payload.length();
    if (len >= MSG_MAX_LEN) {
      Serial.println("Payload too big, skipping");
      continue;
    }
    memcpy(msgbuf, payload.c_str(), len);
    msgbuf[len] = '\0';
    if (xQueueSend(msgQueue, msgbuf, pdMS_TO_TICKS(100)) != pdTRUE) {
      Serial.println("Message queue full, dropping telemetry");
    } else {
      Serial.println("Telemetry queued:");
      Serial.println(payload);
    }
  }
}

// ---------- MQTT task ----------
void mqttTask(void* pvParameters) {
  (void) pvParameters;

  // Wait for WiFi connection
  Serial.println("MQTTTask: waiting for WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  Serial.println("\nWiFi connected. IP: ");
  Serial.println(WiFi.localIP());

  // connect to broker
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT broker ");
    Serial.print(MQTT_SERVER);
    Serial.print(":");
    Serial.print(MQTT_PORT);
    Serial.print(" ... ");
    if (mqttClient.connect(MQTT_BASE_TOPIC, MQTT_USER, MQTT_PASSWD)) {
      Serial.println("connected");
      // subscribe to config topic
      String configTopic = String(MQTT_BASE_TOPIC) + "/config";
      mqttClient.subscribe(configTopic.c_str());
      // publish retained config status
      publishConfig();
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 5s");
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
  }

  // main loop: service client and publish queued messages
  char outMsg[MSG_MAX_LEN];
  String telemetryTopic = String(MQTT_BASE_TOPIC) + "/telemetry";

  for (;;) {
    if (!mqttClient.connected()) {
      // try reconnect
      Serial.println("MQTT disconnected. Reconnecting...");
      while (!mqttClient.connected()) {
        if (mqttClient.connect(MQTT_BASE_TOPIC, MQTT_USER, MQTT_PASSWD)) {
          Serial.println("MQTT reconnect succeeded");
          String configTopic = String(MQTT_BASE_TOPIC) + "/config";
          mqttClient.subscribe(configTopic.c_str());
          publishConfig();
        } else {
          Serial.print("MQTT reconnect failed, rc=");
          Serial.print(mqttClient.state());
          Serial.println(", retrying in 3s");
          vTaskDelay(pdMS_TO_TICKS(3000));
        }
      }
    }

    // handle incoming messages / keepalive
    mqttClient.loop();

    // check queue for outgoing telemetry; block a little to wait for a message
    if (xQueueReceive(msgQueue, outMsg, pdMS_TO_TICKS(1000)) == pdTRUE) {
      // publish
      bool ok = mqttClient.publish(telemetryTopic.c_str(), outMsg, false);
      if (!ok) {
        Serial.println("Publish failed; re-queueing once");
        // attempt one requeue (non-blocking)
        xQueueSendToFront(msgQueue, outMsg, pdMS_TO_TICKS(100));
      } else {
        Serial.println("Published telemetry to topic:");
        Serial.println(telemetryTopic);
      }
    }

    // small delay; mqttClient.loop() needs to be called frequently
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ---------- loop left empty (we're using FreeRTOS tasks) ----------
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
