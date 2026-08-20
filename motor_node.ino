#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const char* wifiSsid = "REPLACE_WITH_WIFI_SSID";
const char* wifiPassword = "REPLACE_WITH_WIFI_PASSWORD";
const char* mqttBroker = "broker.emqx.io";
const int mqttPort = 1883;
const char* controlTopic = "nodes/sentinel_alpha_99x2/hardware_control";

// --- BTS7960 Driver Pins on ESP32-C3 ---
#define MOTOR_L_RPWM  4
#define MOTOR_L_LPWM  5
#define MOTOR_R_RPWM  6
#define MOTOR_R_LPWM  7

// --- PWM Configurations ---
const int motorFreq = 20000; 
const int motorRes  = 8;     
const int defaultDriveSpeed = 180;
const unsigned long commandTimeoutMs = 700;

unsigned long lastCommandAt = 0;
unsigned long lastMqttAttemptAt = 0;
char lastAction[12] = "STOP";
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void processSpatialAction(const char* action, int requestedSpeed = -1) {
  int driveSpeed = requestedSpeed >= 0 ? constrain(requestedSpeed, 0, 255) : defaultDriveSpeed;
  strncpy(lastAction, action, sizeof(lastAction) - 1);
  lastAction[sizeof(lastAction) - 1] = '\0';
  lastCommandAt = millis();

  if (strcmp(action, "FORWARD") == 0) {
    ledcWrite(MOTOR_L_RPWM, driveSpeed); ledcWrite(MOTOR_L_LPWM, 0);
    ledcWrite(MOTOR_R_RPWM, driveSpeed); ledcWrite(MOTOR_R_LPWM, 0);
  } else if (strcmp(action, "BACKWARD") == 0) {
    ledcWrite(MOTOR_L_RPWM, 0); ledcWrite(MOTOR_L_LPWM, driveSpeed);
    ledcWrite(MOTOR_R_RPWM, 0); ledcWrite(MOTOR_R_LPWM, driveSpeed);
  } else if (strcmp(action, "LEFT") == 0) {
    ledcWrite(MOTOR_L_RPWM, 0); ledcWrite(MOTOR_L_LPWM, driveSpeed);
    ledcWrite(MOTOR_R_RPWM, driveSpeed); ledcWrite(MOTOR_R_LPWM, 0);
  } else if (strcmp(action, "RIGHT") == 0) {
    ledcWrite(MOTOR_L_RPWM, driveSpeed); ledcWrite(MOTOR_L_LPWM, 0);
    ledcWrite(MOTOR_R_RPWM, 0); ledcWrite(MOTOR_R_LPWM, driveSpeed);
  } else { 
    // CUT MOTOR CURRENT
    ledcWrite(MOTOR_L_RPWM, 0); ledcWrite(MOTOR_L_LPWM, 0);
    ledcWrite(MOTOR_R_RPWM, 0); ledcWrite(MOTOR_R_LPWM, 0);
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, controlTopic) != 0) {
    return;
  }

  JsonDocument doc;
  if (deserializeJson(doc, payload, length)) {
    processSpatialAction("STOP");
    return;
  }

  const char* drive = doc["drive"];
  int speed = doc["speed"] | defaultDriveSpeed;
  if (drive == NULL) {
    processSpatialAction("STOP");
    return;
  }

  processSpatialAction(drive, speed);
}

void maintainMqttConnection() {
  if (mqttClient.connected()) {
    return;
  }

  if (millis() - lastMqttAttemptAt < 2000) {
    return;
  }

  lastMqttAttemptAt = millis();
  processSpatialAction("STOP");
  String clientId = "ESP32C3-MotorNode-" + WiFi.macAddress();
  clientId.replace(":", "");
  if (mqttClient.connect(clientId.c_str())) {
    mqttClient.subscribe(controlTopic);
    Serial.println("MQTT control connection established.");
  } else {
    Serial.println("MQTT connection failed; motors stopped.");
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize BTS7960 Output Channels
  ledcAttach(MOTOR_L_RPWM, motorFreq, motorRes);
  ledcAttach(MOTOR_L_LPWM, motorFreq, motorRes);
  ledcAttach(MOTOR_R_RPWM, motorFreq, motorRes);
  ledcAttach(MOTOR_R_LPWM, motorFreq, motorRes);

  processSpatialAction("STOP");

  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSsid, wifiPassword);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  Serial.print("WiFi connected. ESP32-C3 MAC Address: ");
  Serial.println(WiFi.macAddress());

  mqttClient.setServer(mqttBroker, mqttPort);
  mqttClient.setCallback(mqttCallback);
  maintainMqttConnection();
}

void loop() {
  maintainMqttConnection();
  mqttClient.loop();

  if (strcmp(lastAction, "STOP") != 0 && millis() - lastCommandAt > commandTimeoutMs) {
    processSpatialAction("STOP");
  }
  delay(100);
}
