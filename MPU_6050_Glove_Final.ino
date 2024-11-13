#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AHRS.h>  // Include AHRS for sensor fusion algorithms
#include <Adafruit_Sensor_Calibration.h>
#include <math.h>

// Pin definitions for the flex sensors
#define FLEX_1_PIN 33
#define FLEX_2_PIN 35
#define FLEX_3_PIN 36
#define FLEX_4_PIN 39
#define FLEX_5_PIN 34

// MPU6050 object
Adafruit_MPU6050 mpu;
Adafruit_Madgwick filter;  // Using the Madgwick sensor fusion algorithm (faster, suitable for most cases)

// Constants for the filter
#define FILTER_UPDATE_RATE_HZ 10  // 100 Hz filter update rate

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
}

void loop() {
  // Read flex sensor values
  int raw_flex_1 = analogRead(FLEX_1_PIN);
  int raw_flex_2 = analogRead(FLEX_2_PIN);
  int raw_flex_3 = analogRead(FLEX_3_PIN);
  int raw_flex_4 = analogRead(FLEX_4_PIN);
  int raw_flex_5 = analogRead(FLEX_5_PIN);

  // Map raw flex sensor values from 0-1023 to 0-100
  int flex_1 = map(raw_flex_1, 0, 4095, 0, 500);
  int flex_2 = map(raw_flex_2, 0, 4095, 0, 500);
  int flex_3 = map(raw_flex_3, 0, 4095, 0, 500);
  int flex_4 = map(raw_flex_4, 0, 4095, 0, 500);
  int flex_5 = map(raw_flex_5, 0, 4095, 0, 500);

  // Read MPU6050 data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Feed accelerometer and gyroscope data to the sensor fusion filter
  filter.update(g.gyro.x, g.gyro.y, g.gyro.z, a.acceleration.x, a.acceleration.y, a.acceleration.z, 0.0f, 0.0f, 0.0f);


  // Get quaternion data using the getQuaternion method
  float qw, qx, qy, qz;
  filter.getQuaternion(&qw, &qx, &qy, &qz);

  // Compute the gravity vector based on the quaternion
  float grav[3];
  filter.getGravityVector(&grav[0], &grav[1], &grav[2]);

  // Compute body acceleration by subtracting gravity
  float body_acc_x = a.acceleration.x - grav[0];
  float body_acc_y = a.acceleration.y - grav[1];
  float body_acc_z = a.acceleration.z - grav[2];

  // Compute world acceleration (just the raw accelerometer data)
  float acc_world_x = (1 - 2 * (qy * qy + qz * qz)) * a.acceleration.x + (2 * (qx * qy - qz * qw)) * a.acceleration.y + (2 * (qx * qz + qy * qw)) *a.acceleration.z;
  float acc_world_y = (2 * (qx * qy + qz * qw)) * a.acceleration.x + (1 - 2 * (qx * qx + qz * qz)) * a.acceleration.y + (2 * (qy * qz - qx * qw)) *a.acceleration.z;
  float acc_world_z = (2 * (qx * qz - qy * qw)) * a.acceleration.x + (2 * (qy * qz + qx * qw)) * a.acceleration.y+ (1 - 2 * (qx * qx + qy * qy)) * a.acceleration.z;

  // // Display sensor data
  Serial.print(flex_1); Serial.print(",");
  Serial.print(flex_2); Serial.print(",");
  Serial.print(flex_3); Serial.print(",");
  Serial.print(flex_4); Serial.print(",");
  Serial.print(flex_5); Serial.print(",");

  Serial.print(qw); Serial.print(",");
  Serial.print(qx); Serial.print(",");
  Serial.print(qy); Serial.print(",");
  Serial.print(qz); Serial.print(",");

  Serial.print(g.gyro.x); Serial.print(",");
  Serial.print(g.gyro.y); Serial.print(",");
  Serial.print(g.gyro.z); Serial.print(",");

  Serial.print(a.acceleration.x); Serial.print(",");
  Serial.print(a.acceleration.y); Serial.print(",");
  Serial.print(a.acceleration.z); Serial.print(",");

  Serial.print(body_acc_x); Serial.print(",");
  Serial.print(body_acc_y); Serial.print(",");
  Serial.print(body_acc_z); Serial.print(",");

  Serial.print(acc_world_x); Serial.print(",");
  Serial.print(acc_world_y); Serial.print(",");
  Serial.print(acc_world_z);

  Serial.println();  // End of line

  delay(10);  // Run at roughly 100 Hz to match filter update rate
} 