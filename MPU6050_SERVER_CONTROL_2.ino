#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>  // Include the servo library for ESP32

// Structure to hold sensor data
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
} SensorData;

// Define servo objects
Servo baseServo;
Servo shoulderServo;
Servo elbowServo;
Servo wristRotationServo;
Servo wristTiltServo;
Servo gripperServo;

// Define pin numbers for each servo
const int baseServoPin = 32;
const int shoulderServoPin = 33;
const int elbowServoPin = 25;
const int wristRotationServoPin = 26;
const int wristTiltServoPin = 27;
const int gripperServoPin = 14;

// Control flags and states
bool movementComplete = false;   // Flag to track if the movement of a servo is complete
int currentServo = 0;            // State to track the current servo being controlled
unsigned long lastFlexTime = 0;  // Timestamp of the last flex_3 detection
const unsigned long debounceDelay = 1000;  // 1 second debounce delay

// Function prototypes for robot controls
void controlBase(float acc_x);
void controlShoulder(float acc_y);
void controlElbow(float acc_y);
void controlWristRotation(float acc_x);  // Use flex for wrist rotation
void controlWristTilt(int acc_y);       // Use gyro for wrist tilt
void controlGripper(int flex_1);

// Callback function when data is received
void OnDataReceive(const esp_now_recv_info *recv_info, const uint8_t *data, int len) {
  SensorData receivedData;
  memcpy(&receivedData, data, sizeof(receivedData));  // Copy data into receivedData structure

  // Print received data to Serial Monitor
  Serial.print("  Flex 1: "); Serial.print(receivedData.flex_1);
  Serial.print(", Flex 2: "); Serial.print(receivedData.flex_2);
  Serial.print(", Flex 3: "); Serial.print(receivedData.flex_3);
  Serial.print(", Flex 4: "); Serial.print(receivedData.flex_4);
  Serial.print(", Flex 5: "); Serial.print(receivedData.flex_5);
  Serial.print(", Gyro X: "); Serial.print(receivedData.gyr_x);
  Serial.print(", Gyro Y: "); Serial.print(receivedData.gyr_y);
  Serial.print(", Gyro Z: "); Serial.print(receivedData.gyr_z);
  Serial.print(", Acc X: "); Serial.print(receivedData.acc_x);
  Serial.print(", Acc Y: "); Serial.print(receivedData.acc_y);
  Serial.print(", Acc Z: "); Serial.print(receivedData.acc_z);

  // Get the current time
  unsigned long currentTime = millis();

  // Check for completion using flex_3 sensor (used to signal readiness to move to the next servo)
  if (receivedData.flex_3 < 15 && movementComplete && (currentTime - lastFlexTime > debounceDelay)) {
    currentServo++;  // Move to the next servo
    if (currentServo > 5) {
      currentServo = 0;  // Reset to the base servo after gripper is complete
      Serial.println("Cycle complete. Resetting to base servo.");
    }
    movementComplete = false;  // Reset the flag
    lastFlexTime = currentTime;  // Update the debounce timestamp
    Serial.println("Flex_3 detected, moving to next servo.");
  }

  // Call the control functions based on the current servo
  switch (currentServo) {
    case 0:  // Control base
      controlBase(receivedData.acc_x);
      break;
    case 1:  // Control shoulder
      controlShoulder(receivedData.acc_y);
      break;
    case 2:  // Control elbow
      controlElbow(receivedData.acc_y);
      break;
    case 3:  // Control wrist rotation
      controlWristRotation(receivedData.acc_x);
      break;
    case 4:  // Control wrist tilt
      controlWristTilt(receivedData.acc_y);
      break;
    case 5:  // Control gripper
      controlGripper(receivedData.flex_1);
      break;
  }
}

void setup() {
  Serial.begin(115200);
  
  // Set device as a Wi-Fi station
  WiFi.mode(WIFI_STA);
  
  // Set up ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the receive callback function
  esp_now_register_recv_cb(OnDataReceive);
  
  // Attach servos to their respective pins
  baseServo.attach(baseServoPin);
  shoulderServo.attach(shoulderServoPin);
  elbowServo.attach(elbowServoPin);
  wristRotationServo.attach(wristRotationServoPin);
  wristTiltServo.attach(wristTiltServoPin);
  gripperServo.attach(gripperServoPin);
}

void loop() {
  // Nothing to do here, everything is handled in the callback function
}

// Functions to control each part of the robot

void controlBase(float acc_x) {
  int baseAngle = map(acc_x, 0, -10, 0, 180);  // Example mapping
  baseServo.write(baseAngle);  // Set servo position
  // delay(1000);  // Simulate time for smooth transition
  movementComplete = true;  // Mark movement as complete
  Serial.print("Base control: "); Serial.println(baseAngle);
}

void controlShoulder(float acc_y) {
  int shoulderAngle = map(acc_y, 0, -10, 0, 180);  // Example mapping
  shoulderServo.write(shoulderAngle);  // Set servo position
  // delay(1000);  // Simulate time for smooth transition
  movementComplete = true;  // Mark movement as complete
  Serial.print("Shoulder control: "); Serial.println(shoulderAngle);
}

void controlElbow(float acc_y) {
  int elbowAngle = map(acc_y, 0, 10, 0, 180);  // Example mapping
  elbowServo.write(elbowAngle);  // Set servo position
  // delay(1000);  // Simulate time for smooth transition
  movementComplete = true;  // Mark movement as complete
  Serial.print("Elbow control: "); Serial.println(elbowAngle);
}

void controlWristRotation(float acc_x) {
  int wristRotationAngle = map(acc_x, 0, -10, 0, 180);  // Example mapping
  wristRotationServo.write(wristRotationAngle);  // Set servo position
  // delay(1000);  // Simulate time for smooth transition
  movementComplete = true;  // Mark movement as complete
  Serial.print("Wrist rotation control: "); Serial.println(wristRotationAngle);
}

void controlWristTilt(int acc_y) {
  int wristTiltAngle = map(acc_y, 0, 10, 0, 90);  // Example mapping
  wristTiltServo.write(wristTiltAngle);  // Set servo position
  // delay(1000);  // Simulate time for smooth transition
  movementComplete = true;  // Mark movement as complete
  Serial.print("Wrist tilt control: "); Serial.println(wristTiltAngle);
}

void controlGripper(int flex_1) {
  int gripperPosition = map(flex_1, 0, 24, 180, 0);  // Example mapping
  gripperServo.write(gripperPosition);  // Set gripper servo position
  // delay(1000);  // Simulate time for smooth transition
  movementComplete = true;  // Mark movement as complete
  Serial.print("Gripper control: "); Serial.println(gripperPosition);
}
