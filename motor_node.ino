#include <WiFi.h>
#include <esp_now.h>

// --- BTS7960 Driver Pins on ESP32-C3 ---
#define MOTOR_L_RPWM  4
#define MOTOR_L_LPWM  5
#define MOTOR_R_RPWM  6
#define MOTOR_R_LPWM  7

// --- PWM Configurations ---
const int motorFreq = 20000; 
const int motorRes  = 8;     
const int driveSpeed = 180;  

// Simplified Payload Structure (No Servo variables)
typedef struct struct_message {
    char drive[12];
} struct_message;
struct_message incomingData;

void processSpatialAction(const char* action) {
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

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingDataRaw, int len) {
  memcpy(&incomingData, incomingDataRaw, sizeof(incomingData));
  processSpatialAction(incomingData.drive);
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
  Serial.print("ESP32-C3 MAC Address: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop() {
  delay(100);
}
