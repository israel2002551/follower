#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP32-RTSPServer.h>
#include "esp_camera.h"
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// --- ESP32-CAM Dedicated Safe Sensor Pins ---
#define I2C_SDA_PIN   14
#define I2C_SCL_PIN   15
#define SONIC_TRIG    13
#define SONIC_ECHO    12

const unsigned long sonicTimeoutUs = 25000;
const long sonicMinDistanceMm = 20;
const long sonicMaxDistanceMm = 4000;
const int sonicSampleCount = 3;

// --- Camera Pin Definition Array ---
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

const char* ssid     = "Galaxy A12580F";
const char* password = "isreal13";

const char* mqtt_broker = "broker.emqx.io";
const int mqtt_port     = 1883;
const char* tele_topic  = "nodes/sentinel_alpha_99x2/telemetry";

int quality = 12;
bool laserReady = false;
WiFiClient espClient;
PubSubClient mqtt_client(espClient);
RTSPServer rtspServer;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

TaskHandle_t videoTaskHandle = NULL;

bool setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;  
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 10;
  config.fb_count = 2;

  if (psramFound()) {
    config.jpeg_quality = 12;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) return false;

  sensor_t *s = esp_camera_sensor_get();
  if (config.pixel_format == PIXFORMAT_JPEG && s != NULL) {
    s->set_framesize(s, FRAMESIZE_VGA); 
  }
  return true;
}

long getUltrasonicDistanceMM() {
  long samples[sonicSampleCount];
  int validSamples = 0;

  for (int sampleIndex = 0; sampleIndex < sonicSampleCount; sampleIndex++) {
    digitalWrite(SONIC_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(SONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(SONIC_TRIG, LOW);

    unsigned long duration = pulseIn(SONIC_ECHO, HIGH, sonicTimeoutUs);
    long distanceMm = (duration * 343L) / 2000L;
    if (duration > 0 && distanceMm >= sonicMinDistanceMm && distanceMm <= sonicMaxDistanceMm) {
      samples[validSamples++] = distanceMm;
    }

    if (sampleIndex < sonicSampleCount - 1) {
      delay(30);
    }
  }

  if (validSamples < 2) {
    return -1;
  }

  for (int index = 1; index < validSamples; index++) {
    long value = samples[index];
    int sortedIndex = index - 1;
    while (sortedIndex >= 0 && samples[sortedIndex] > value) {
      samples[sortedIndex + 1] = samples[sortedIndex];
      sortedIndex--;
    }
    samples[sortedIndex + 1] = value;
  }

  if (validSamples == 2) {
    return (samples[0] + samples[1]) / 2;
  }
  return samples[validSamples / 2];
}

void reconnectMQTT() {
  while (!mqtt_client.connected()) {
    String clientId = "ESP32CAM-SensorNode-" + String(random(0xffff), HEX);
    if (mqtt_client.connect(clientId.c_str())) {
      Serial.println("MQTT connected for telemetry publishing.");
    } else {
      delay(2000);
    }
  }
}

void sendVideo(void* pvParameters) { 
  while (true) { 
    if(rtspServer.readyToSendFrame()) {
      camera_fb_t* fb = esp_camera_fb_get();
      if (fb != NULL) {
        rtspServer.sendRTSPFrame(fb->buf, fb->len, quality, fb->width, fb->height);
        esp_camera_fb_return(fb);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(16)); 
  }
}

void networkTask(void* pvParameters) {
  TickType_t lastTelemetryTime = xTaskGetTickCount();
  
  while (true) {
    if (!mqtt_client.connected()) reconnectMQTT();
    mqtt_client.loop();

    if ((xTaskGetTickCount() - lastTelemetryTime) >= pdMS_TO_TICKS(200)) {
      lastTelemetryTime = xTaskGetTickCount();
      
      long laser_dist = -1;
      if (laserReady) {
        VL53L0X_RangingMeasurementData_t measure;
        lox.getRangingMeasurement(&measure, false);
        laser_dist = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : -1;
      }
      long sonic_dist = getUltrasonicDistanceMM();
      
      JsonDocument teleDoc;
      teleDoc["laser_mm"] = laser_dist;
      teleDoc["sonic_mm"] = sonic_dist;
      teleDoc["wifi_rssi"] = WiFi.RSSI();
      
      char buffer[128];
      serializeJson(teleDoc, buffer);
      mqtt_client.publish(tele_topic, buffer);
    }
    vTaskDelay(pdMS_TO_TICKS(10)); 
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize Ultrasonic Pins for Two-Pin Operation
  pinMode(SONIC_TRIG, OUTPUT);
  pinMode(SONIC_ECHO, INPUT);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
  Serial.print("WiFi Connected. IP: ");
  Serial.println(WiFi.localIP());

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!lox.begin(0x29, false, &Wire)) {
    Serial.println("VL53L0X Laser not found on I2C bus!");
    laserReady = false;
  } else {
    laserReady = true;
  }

  setupCamera();
  sensor_t * s = esp_camera_sensor_get(); 
  if (s != NULL) quality = s->status.quality; 
  
  rtspServer.init();
  mqtt_client.setServer(mqtt_broker, mqtt_port);

  xTaskCreate(sendVideo, "VideoTask", 8192, NULL, 9, &videoTaskHandle);
  xTaskCreate(networkTask, "NetworkTask", 4096, NULL, 5, NULL);
}

void loop() {
  vTaskDelete(NULL); 
}





//20d84caac4cb36d7af1e5722
