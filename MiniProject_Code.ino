#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "esp_task_wdt.h"

// ================== WIFI CONFIG ===================
const char* ssid = "Unifi@5GHz";
const char* password = "Ansontay7";

String GOOGLE_SCRIPT_URL =
"https://script.google.com/macros/s/AKfycbyrg3smCTpl7qGHyqbH3ZJZ7lNGvEeUkIrYfzZKPb4sD-oi5LXJfoLE0t2SswhLaoaaIg/exec";

// ================== HARDWARE ======================
#define TRIG_PIN 5
#define ECHO_PIN 18
#define STATUS_LED 2

Adafruit_BME280 bme;

// ================== SAMPLING ======================
const unsigned long SAMPLE_INTERVAL = 60000; // 1 minute
unsigned long lastSample = 0;

float tempBuf[5], humBuf[5], presBuf[5], distBuf[5];
int sampleCount = 0;

// ================== DATA STRUCT ===================
struct SensorData {
  float temperature;
  float humidity;
  float pressure;
  float distance;
  bool valid;
};

// ==================================================

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(STATUS_LED, OUTPUT);

  digitalWrite(STATUS_LED, LOW);

  if (!bme.begin(0x76)) {
    Serial.println("BME280 not detected!");
    while (1);
  }

  connectWiFi();

  // Watchdog: reset if system freezes >10s
  esp_task_wdt_config_t wdt_config = {
  .timeout_ms = 10000,   // 10 seconds
  .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
  .trigger_panic = true
};

esp_task_wdt_init(&wdt_config);
esp_task_wdt_add(NULL);
}

// ================= WIFI ===========================
void connectWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi");

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    Serial.print(".");
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected");
    digitalWrite(STATUS_LED, HIGH);
  } else {
    Serial.println("\nWiFi Failed");
    digitalWrite(STATUS_LED, LOW);
  }
}

// ================= SENSOR READ ====================
float readUltrasonicCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return NAN;

  return duration * 0.034 / 2;
}

bool checkValid(float v, float minV, float maxV) {
  return !(isnan(v) || v < minV || v > maxV);
}

SensorData readAllSensors() {
  SensorData data;

  data.temperature = bme.readTemperature();
  data.humidity    = bme.readHumidity();
  data.pressure    = bme.readPressure() / 100000.0F;
  data.distance    = readUltrasonicCM();

  data.valid =
    checkValid(data.temperature, -40, 80) &&
    checkValid(data.humidity, 0, 100) &&
    checkValid(data.pressure, 0.5, 1.5);

  if (isnan(data.distance)) data.distance = -1;

  return data;
}

// ================= DIAGNOSTICS ====================
void printSystemStatus() {
  Serial.println("----- SYSTEM STATUS -----");
  Serial.print("Free Heap: ");
  Serial.println(ESP.getFreeHeap());
  Serial.print("Uptime (s): ");
  Serial.println(millis() / 1000);
  Serial.println("-------------------------");
}

// ================= DATA SEND ======================
void sendToGoogle(float t, float h, float p, float d) {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
    if (WiFi.status() != WL_CONNECTED) return;
  }

  digitalWrite(STATUS_LED, LOW);

  HTTPClient http;
  String url = GOOGLE_SCRIPT_URL +
               "?temperature=" + String(t, 2) +
               "&humidity=" + String(h, 2) +
               "&pressure=" + String(p, 2) +
               "&distance=" + String(d, 2);

  http.begin(url);
  int httpCode = http.GET();

  Serial.print("HTTP Code: ");
  Serial.println(httpCode);

  http.end();
  digitalWrite(STATUS_LED, HIGH);
}

// ================= MAIN LOOP ======================
void loop() {
  esp_task_wdt_reset();

  if (millis() - lastSample < SAMPLE_INTERVAL) return;
  lastSample = millis();

  SensorData data = readAllSensors();
  if (!data.valid) return;

  tempBuf[sampleCount] = data.temperature;
  humBuf[sampleCount]  = data.humidity;
  presBuf[sampleCount] = data.pressure;
  distBuf[sampleCount] = data.distance;
  sampleCount++;

  Serial.printf("Buffered Sample %d/5\n", sampleCount);

  if (sampleCount >= 5) {
    float avgT = 0, avgH = 0, avgP = 0, avgD = 0;

    for (int i = 0; i < 5; i++) {
      avgT += tempBuf[i];
      avgH += humBuf[i];
      avgP += presBuf[i];
      avgD += distBuf[i];
    }

    avgT /= 5; avgH /= 5; avgP /= 5; avgD /= 5;

    sendToGoogle(avgT, avgH, avgP, avgD);
    printSystemStatus();

    sampleCount = 0;
  }
}