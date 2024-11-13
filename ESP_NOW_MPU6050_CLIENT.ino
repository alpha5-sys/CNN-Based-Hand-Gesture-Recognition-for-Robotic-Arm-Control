#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_Sensor_Calibration.h>
#include <esp_now.h>
#include <WiFi.h>
#include <math.h>

// Pin definitions for the flex sensors
#define FLEX_1_PIN 33
#define FLEX_2_PIN 35
#define FLEX_3_PIN 36
#define FLEX_4_PIN 39
#define FLEX_5_PIN 34

// MPU6050 object
Adafruit_MPU6050 mpu;
Adafruit_Madgwick filter;

// Constants for the filter
#define FILTER_UPDATE_RATE_HZ 10  // 100 Hz filter update rate

// Structure to hold sensor data including quaternion
typedef struct SensorData {
    int flex_1;
    int flex_2;
    int flex_3;
    int flex_4;
    int flex_5;
    float gyr_x;
    float gyr_y;
    float gyr_z;
    float acc_x;
    float acc_y;
    float acc_z;
    float body_acc_x;
    float body_acc_y;
    float body_acc_z;
    float quat_w;  // Quaternion w
    float quat_x;  // Quaternion x
    float quat_y;  // Quaternion y
    float quat_z;  // Quaternion z
} SensorData;

// Create a SensorData object to hold the data
SensorData sensorData;

// Peer info for ESP-NOW
esp_now_peer_info_t peerInfo;

// Function to handle data sent callback
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
    Serial.begin(115200);

    // Initialize I2C communication for MPU6050
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            delay(10);
        }
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

    // Initialize the sensor fusion filter
    filter.begin(FILTER_UPDATE_RATE_HZ);

    // Initialize flex sensor pins
    pinMode(FLEX_1_PIN, INPUT);
    pinMode(FLEX_2_PIN, INPUT);
    pinMode(FLEX_3_PIN, INPUT);
    pinMode(FLEX_4_PIN, INPUT);
    pinMode(FLEX_5_PIN, INPUT);

    // Read sensor values before initializing Wi-Fi
    readSensors();

    // Set up ESP-NOW
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register callbacks
    esp_now_register_send_cb(OnDataSent);

    // Add peer (receiver)
    memset(&peerInfo, 0, sizeof(peerInfo));
    uint8_t broadcastAddress[] = {0xC0, 0x49, 0xEF, 0xCC, 0x19, 0xE0}; // Change to your receiver's MAC address
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0; // Use the default channel
    peerInfo.encrypt = false; // No encryption
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
}

void readSensors() {
    // Read flex sensor values
    int raw_flex_1 = analogRead(FLEX_1_PIN);
    int raw_flex_2 = analogRead(FLEX_2_PIN);
    int raw_flex_3 = analogRead(FLEX_3_PIN);
    int raw_flex_4 = analogRead(FLEX_4_PIN);
    int raw_flex_5 = analogRead(FLEX_5_PIN);

    // Map raw flex sensor values from 0-4095 to 0-100
    sensorData.flex_1 = map(raw_flex_1, 0, 4095, 0, 500);
    sensorData.flex_2 = map(raw_flex_2, 0, 4095, 0, 500);
    sensorData.flex_3 = map(raw_flex_3, 0, 4095, 0, 500);
    sensorData.flex_4 = map(raw_flex_4, 0, 4095, 0, 500);
    sensorData.flex_5 = map(raw_flex_5, 0, 4095, 0, 500);

    // Read MPU6050 data
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Feed accelerometer and gyroscope data to the sensor fusion filter
    filter.update(g.gyro.x, g.gyro.y, g.gyro.z, a.acceleration.x, a.acceleration.y, a.acceleration.z, 0.0f, 0.0f, 0.0f);

  //   // Get quaternion values from the sensor fusion filter
  float qw, qx, qy, qz;
  filter.getQuaternion(&qw, &qx, &qy, &qz);

    // Update sensor data with quaternion values
    sensorData.quat_w = qw;
    sensorData.quat_x = qx;
    sensorData.quat_y = qy;
    sensorData.quat_z = qz;

    // Get body acceleration by subtracting gravity
    float grav[3];
    filter.getGravityVector(&grav[0], &grav[1], &grav[2]);
    sensorData.body_acc_x = a.acceleration.x - grav[0];
    sensorData.body_acc_y = a.acceleration.y - grav[1];
    sensorData.body_acc_z = a.acceleration.z - grav[2];

    // Get the raw accelerometer values
    sensorData.acc_x = a.acceleration.x;
    sensorData.acc_y = a.acceleration.y;
    sensorData.acc_z = a.acceleration.z;

    // Get gyroscope data
    sensorData.gyr_x = g.gyro.x;
    sensorData.gyr_y = g.gyro.y;
    sensorData.gyr_z = g.gyro.z;
}


void loop() {
    // Read the sensors before sending data
    readSensors();

    // Send data over ESP-NOW
    esp_err_t result = esp_now_send(peerInfo.peer_addr, (uint8_t *)&sensorData, sizeof(sensorData));

    if (result == ESP_OK) {
        Serial.println("Data sent successfully");
    } else {
        Serial.println("Error sending the data");
    }

    // Display sensor data for debugging
    Serial.print("Flex: ");
    Serial.print(sensorData.flex_1); Serial.print(",");
    Serial.print(sensorData.flex_2); Serial.print(",");
    Serial.print(sensorData.flex_3); Serial.print(",");
    Serial.print(sensorData.flex_4); Serial.print(",");
    Serial.print(sensorData.flex_5); Serial.print(",");

    Serial.print("Gyro: ");
    Serial.print(sensorData.gyr_x); Serial.print(",");
    Serial.print(sensorData.gyr_y); Serial.print(",");
    Serial.print(sensorData.gyr_z); Serial.print(",");

    Serial.print("Acc: ");
    Serial.print(sensorData.acc_x); Serial.print(",");
    Serial.print(sensorData.acc_y); Serial.print(",");
    Serial.print(sensorData.acc_z);

    Serial.print("Body Acc: ");
    Serial.print(sensorData.body_acc_x); Serial.print(",");
    Serial.print(sensorData.body_acc_y); Serial.print(",");
    Serial.print(sensorData.body_acc_z); Serial.print(",");

    // Display quaternion values
    Serial.print("Quat: ");
    Serial.print(sensorData.quat_w); Serial.print(",");
    Serial.print(sensorData.quat_x); Serial.print(",");
    Serial.print(sensorData.quat_y); Serial.print(",");
    Serial.print(sensorData.quat_z);

    Serial.println();  // End of line

    delay(10);  // Run at roughly 100 Hz to match filter update rate
}
