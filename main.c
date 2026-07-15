#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP32-RTSPServer.h>
#include "esp_camera.h"
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <esp_now.h>

// --- ESP32-CAM Dedicated Safe Sensor Pins ---
#define I2C_SDA_PIN   14
#define I2C_SCL_PIN   15
#define SONIC_TRIG    13
#define SONIC_ECHO    12

// --- ESP32-C3 Receiver MAC Address ---
// REPLACE THIS with your physical ESP32-C3's MAC Address
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

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

const char* mqtt_broker = "broker.hivemq.com";
const int mqtt_port     = 1883;
const char* mqtt_topic  = "nodes/sentinel_alpha_99x2/hardware_control";
const char* tele_topic  = "nodes/sentinel_alpha_99x2/telemetry";

int quality = 12;
WiFiClient espClient;
PubSubClient mqtt_client(espClient);
RTSPServer rtspServer;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

TaskHandle_t videoTaskHandle = NULL;

// Simplified ESP-NOW Data Structure (No Servo variables)
typedef struct struct_message {
    char drive[12];
} struct_message;
struct_message controlData;

esp_now_peer_info_t peerInfo;

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

// Two-Pin Ultrasonic Distance Calculation
long getUltrasonicDistanceMM() {
  digitalWrite(SONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(SONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONIC_TRIG, LOW);
  
  long duration = pulseIn(SONIC_ECHO, HIGH, 30000); // 30ms timeout limit
  if (duration == 0) return 9999;
  return (duration * 0.343) / 2; // Returns distance in millimeters
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  JsonDocument doc; 
  DeserializationError error = deserializeJson(doc, payload, length);
  if (!error) {
    const char* drive = doc["drive"];
    if (drive) {
      strcpy(controlData.drive, drive);
      esp_now_send(broadcastAddress, (uint8_t *) &controlData, sizeof(controlData));
    }
  }
}

void reconnectMQTT() {
  while (!mqtt_client.connected()) {
    String clientId = "ESP32CAM-SensorNode-" + String(random(0xffff), HEX);
    if (mqtt_client.connect(clientId.c_str())) {
      mqtt_client.subscribe(mqtt_topic);
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
      
      VL53L0X_RangingMeasurementData_t measure;
      lox.getRangingMeasurement(&measure, false);
      
      long laser_dist = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : -1;
      long sonic_dist = getUltrasonicDistanceMM();
      
      JsonDocument teleDoc;
      teleDoc["laser_mm"] = laser_dist;
      teleDoc["sonic_mm"] = sonic_dist;
      
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

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add ESP-NOW Peer");
    return;
  }

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!lox.begin(0x29, false, &Wire)) {
    Serial.println("VL53L0X Laser not found on I2C bus!");
    while(1);
  }

  setupCamera();
  sensor_t * s = esp_camera_sensor_get(); 
  if (s != NULL) quality = s->status.quality; 
  
  rtspServer.init();
  mqtt_client.setServer(mqtt_broker, mqtt_port);
  mqtt_client.setCallback(mqttCallback);

  xTaskCreate(sendVideo, "VideoTask", 8192, NULL, 9, &videoTaskHandle);
  xTaskCreate(networkTask, "NetworkTask", 4096, NULL, 5, NULL);
}

void loop() {
  vTaskDelete(NULL); 
}
