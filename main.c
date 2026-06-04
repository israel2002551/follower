#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "esp_peer.h"
#include "mqtt_client.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "cJSON.h"

static const char *TAG = "ESP32_AI_ROBOT";

// --- Hardware GPIO Configuration ---
#define MOTOR_IN1  12
#define MOTOR_IN2  13
#define MOTOR_IN3  14
#define MOTOR_IN4  15
#define SERVO_PWM  16

// --- HiveMQ Public Broker Isolation Routing ---
// Matches the exact ROBOT_ID string inside your updated server.py
#define MQTT_CONTROL_TOPIC "nodes/sentinel_alpha_99x2/hardware_control"

#define SERVO_MIN_DEGREE  0
#define SERVO_MAX_DEGREE  180
static int current_servo_angle = 90;

static esp_mqtt_client_handle_t mqtt_client;
static esp_peer_handle_t webrtc_peer;

// --- AI-Thinker Camera Pin Matrix ---
static camera_config_t camera_config = {
    .pin_pwdn = 32, .pin_reset = -1, .pin_xclk = 0,
    .pin_sscb_sda = 26, .pin_sscb_scl = 27,
    .pin_d7 = 35, .pin_d6 = 34, .pin_d5 = 39, .pin_d4 = 36,
    .pin_d3 = 21, .pin_d2 = 19, .pin_d1 = 18, .pin_d0 = 5,
    .pin_vsync = 25, .pin_href = 23, .pin_pclk = 22,
    .xclk_freq_hz = 20000000, .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0, .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_VGA, .jpeg_quality = 12, .fb_count = 2
};

void init_hardware_peripherals() {
    // 1. Initialize GPIO Pins for L298N H-Bridge
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = ((1ULL<<MOTOR_IN1) | (1ULL<<MOTOR_IN2) | (1ULL<<MOTOR_IN3) | (1ULL<<MOTOR_IN4)),
        .pull_down_en = 0, .pull_up_en = 0
    };
    gpio_config(&io_conf);

    // 2. Configure Hardware PWM Timer for Camera Tracking Servo
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_1,
        .duty_resolution  = LEDC_TIMER_13_BIT, // 8192 duty units max
        .freq_hz          = 50,                // 50Hz Standard Servo frequency
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_1,
        .timer_sel      = LEDC_TIMER_1_BIT,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = SERVO_PWM,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
}

void set_servo_angle(int angle) {
    if (angle < SERVO_MIN_DEGREE) angle = SERVO_MIN_DEGREE;
    if (angle > SERVO_MAX_DEGREE) angle = SERVO_MAX_DEGREE;
    current_servo_angle = angle;
    
    // Mapping angle parameter to corresponding duty cycle value (0.5ms - 2.5ms pulses)
    int duty = (int)((((angle / 180.0) * 2.0) + 0.5) / 20.0 * 8192.0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

void process_spatial_action(const char* action, int pan_delta) {
    // Step-update camera tilt tracking pan state 
    if(pan_delta != 0) {
        set_servo_angle(current_servo_angle + pan_delta);
    }

    // Process steering and drive actions, including the new Emergency Safety Halt
    if (strcmp(action, "FORWARD") == 0) {
        gpio_set_level(MOTOR_IN1, 1); gpio_set_level(MOTOR_IN2, 0);
        gpio_set_level(MOTOR_IN3, 1); gpio_set_level(MOTOR_IN4, 0);
    } else if (strcmp(action, "BACKWARD") == 0) {
        gpio_set_level(MOTOR_IN1, 0); gpio_set_level(MOTOR_IN2, 1);
        gpio_set_level(MOTOR_IN3, 0); gpio_set_level(MOTOR_IN4, 1);
    } else if (strcmp(action, "LEFT") == 0) {
        gpio_set_level(MOTOR_IN1, 0); gpio_set_level(MOTOR_IN2, 1);
        gpio_set_level(MOTOR_IN3, 1); gpio_set_level(MOTOR_IN4, 0);
    } else if (strcmp(action, "RIGHT") == 0) {
        gpio_set_level(MOTOR_IN1, 1); gpio_set_level(MOTOR_IN2, 0);
        gpio_set_level(MOTOR_IN3, 0); gpio_set_level(MOTOR_IN4, 1);
    } else { 
        // Explicit "STOP" or fallback execution state cuts power to L298N bridges immediately
        gpio_set_level(MOTOR_IN1, 0); gpio_set_level(MOTOR_IN2, 0);
        gpio_set_level(MOTOR_IN3, 0); gpio_set_level(MOTOR_IN4, 0);
    }
}

// --- HiveMQ Input Routing Callbacks ---
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    if (event->event_id == MQTT_EVENT_DATA) {
        if (strncmp(event->topic, MQTT_CONTROL_TOPIC, event->topic_len) == 0) {
            cJSON *root = cJSON_ParseWithLength(event->data, event->data_len);
            if (root) {
                cJSON *drive = cJSON_GetObjectItem(root, "drive");
                cJSON *pan = cJSON_GetObjectItem(root, "pan");
                
                if (drive && pan) {
                    process_spatial_action(drive->valuestring, pan->valueint);
                }
                cJSON_Delete(root);
            }
        }
    }
}

void app_main(void) {
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Hardware Initialisation Failure");
        return;
    }

    init_hardware_peripherals();
    set_servo_angle(90); // Default servo orientation straight ahead

    // Initialize public cloud messaging layers
    esp_mqtt_client_config_t mqtt_cfg = { 
        .broker.address.uri = "mqtt://broker.hivemq.com" 
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
    esp_mqtt_client_subscribe(mqtt_client, MQTT_CONTROL_TOPIC, 1);

    // Bootstrap Secure WebRTC Stream Engine
    esp_peer_cfg_t peer_cfg = {
        .enable_data_channel = false,
        .manual_ch_create = false
    };
    esp_peer_open(&peer_cfg, esp_peer_get_default_impl(), &webrtc_peer);

    ESP_LOGI(TAG, "System connected to HiveMQ. Awaiting target tracking profile commands...");
    
    // Core Media Capture & Encryption Execution Loop
    while(1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            esp_peer_video_frame_t frame = {
                .data = fb->buf,
                .size = fb->len,
                .pts = esp_timer_get_time()
            };
            // Streams securely over WebRTC peer context
            esp_peer_send_video_data(webrtc_peer, &frame);
            esp_camera_fb_return(fb);
        }
        vTaskDelay(pdMS_TO_TICKS(33)); // Sync execution bounds (~30 FPS limit)
    }
}
