#include <esp_now.h>
#include <WiFi.h>

// MAC address of the slave board
uint8_t slaveAddress[] = { 0xF8, 0xB3, 0xB7, 0x2B, 0x0C, 0x64 };

// Structure to hold angle data
typedef struct __attribute__((packed)) {
  float angle1;
  float angle2;
} MotorData;

MotorData motorData;

#include <SimpleFOC.h>

// SPI pins
#define SCK 19
#define MISO 26
#define MOSI 22
#define CS1 17
#define CS2 18

// Motor/sensor definitions
#define MOTOR1_A 25
#define MOTOR1_B 13
#define MOTOR1_C 27
#define MOTOR1_EN 12

#define MOTOR2_A 16
#define MOTOR2_B 5
#define MOTOR2_C 23
#define MOTOR2_EN 14

MagneticSensorSPI sensor1 = MagneticSensorSPI(AS5048_SPI, CS1);
MagneticSensorSPI sensor2 = MagneticSensorSPI(AS5048_SPI, CS2);

BLDCMotor motor1 = BLDCMotor(11);
BLDCMotor motor2 = BLDCMotor(11);

BLDCDriver3PWM driver1 = BLDCDriver3PWM(MOTOR1_A, MOTOR1_B, MOTOR1_C, MOTOR1_EN);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(MOTOR2_A, MOTOR2_B, MOTOR2_C, MOTOR2_EN);

void setup() {
  Serial.begin(115200);

  // Init ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, slaveAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
  }

  // Init motors and sensors
  SPI.begin(SCK, MISO, MOSI);
  sensor1.init();
  sensor2.init();

  motor1.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);

  driver1.voltage_power_supply = 12;
  driver2.voltage_power_supply = 12;
  driver1.init();
  driver2.init();

  motor1.linkDriver(&driver1);
  motor2.linkDriver(&driver2);

  motor1.controller = MotionControlType::torque;
  motor2.controller = MotionControlType::torque;

  motor1.voltage_limit = 7;
  motor2.voltage_limit = 7;

  motor1.init();
  motor2.init();
  motor1.initFOC();
  motor2.initFOC();

  Serial.println("Master ready, sending angles...");
}

void loop() {
  motor1.loopFOC();
  motor2.loopFOC();

  motorData.angle1 = motor1.shaft_angle;
  motorData.angle2 = motor2.shaft_angle;

  esp_now_send(slaveAddress, (uint8_t*)&motorData, sizeof(motorData));

  motor1.move(0);
  motor2.move(0);

  delay(10);  // ~100Hz update rate
}
