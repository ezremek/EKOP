#include <esp_now.h>
#include <WiFi.h>
#include <SimpleFOC.h>

// SPI pins
#define SCK 19
#define MISO 26
#define MOSI 22
#define CS1 17  // Sensor 1
#define CS2 18  // Sensor 2

// Motor 1 setup (First SimpleFOC Shield)
#define MOTOR1_A 25
#define MOTOR1_B 13
#define MOTOR1_C 27
#define MOTOR1_EN 12

// Motor 2 setup (Second SimpleFOC Shield)
#define MOTOR2_A 16
#define MOTOR2_B 5
#define MOTOR2_C 23
#define MOTOR2_EN 14

// MAC address of the peer board (set the other board's MAC here)
//uint8_t peerAddress[] = { 0xE8, 0x31, 0xCD, 0xAD, 0x91, 0xD0 }; // example: master MAC
//uint8_t peerAddress[] = { 0xf8, 0xb3, 0xb7, 0x50, 0xbd, 0x38 }; // example: master MAC
uint8_t peerAddress[] = { 0xcc, 0x8d, 0xa2, 0xed, 0x33, 0xd4 }; // example: master MAC

// Sensor and motor definitions
MagneticSensorSPI sensor1 = MagneticSensorSPI(AS5048_SPI, CS1);
MagneticSensorSPI sensor2 = MagneticSensorSPI(AS5048_SPI, CS2);
BLDCMotor motor1 = BLDCMotor(11);
BLDCMotor motor2 = BLDCMotor(11);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(MOTOR1_A, MOTOR1_B, MOTOR1_C, MOTOR1_EN);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(MOTOR2_A, MOTOR2_B, MOTOR2_C, MOTOR2_EN);

// Data structure for sending angles
typedef struct {
  float angle1;
  float angle2;
} MotorData;

MotorData outgoingData;
MotorData incomingData;
bool dataReceived = false;
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 10;  // 100 Hz

float gain = 2.0;    // Reduced gain to minimize oscillation
float alpha = 0.1;   // Low-pass filter coefficient
float deadband = 0.01;
float smoothed_angle1 = 0;
float smoothed_angle2 = 0;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void OnDataRecv(const esp_now_recv_info *recvInfo, const uint8_t *incomingDataRaw, int len) {
  if (len == sizeof(MotorData)) {
    memcpy(&incomingData, incomingDataRaw, sizeof(MotorData));
    dataReceived = true;
  }
}

void setup() {
  Serial.begin(115200);

  // WiFi/ESP-NOW setup
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  Serial.print("My MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // SPI
  SPI.begin(SCK, MISO, MOSI);
  sensor1.init();
  sensor2.init();

  // Motor 1
  motor1.linkSensor(&sensor1);
  driver1.init();
  motor1.linkDriver(&driver1);
  motor1.controller = MotionControlType::torque;
  motor1.foc_modulation = FOCModulationType::SinePWM;
  motor1.init();
  motor1.initFOC();

  // Motor 2
  motor2.linkSensor(&sensor2);
  driver2.init();
  motor2.linkDriver(&driver2);
  motor2.controller = MotionControlType::torque;
  motor2.foc_modulation = FOCModulationType::SinePWM;
  motor2.init();
  motor2.initFOC();

  Serial.println("Dual motor ESP-NOW setup ready.");
}

void loop() {
  // FOC loops
  motor1.loopFOC();
  motor2.loopFOC();

  // Read local angles
  outgoingData.angle1 = motor1.shaft_angle;
  outgoingData.angle2 = motor2.shaft_angle;

  // Send angles every 10 ms (100 Hz)
  if (millis() - lastSendTime >= sendInterval) {
    esp_now_send(peerAddress, (uint8_t*)&outgoingData, sizeof(MotorData));
    lastSendTime = millis();
  }

  // Apply mirrored spring-like control if data received
  if (dataReceived) {
    smoothed_angle1 = (1.0 - alpha) * smoothed_angle1 + alpha * incomingData.angle1;
    smoothed_angle2 = (1.0 - alpha) * smoothed_angle2 + alpha * incomingData.angle2;

    float diff1 = smoothed_angle1 - motor1.shaft_angle;
    float diff2 = smoothed_angle2 - motor2.shaft_angle;

    motor1.move(fabs(diff1) > deadband ? gain * diff1 : 0);
    motor2.move(fabs(diff2) > deadband ? gain * diff2 : 0);

    dataReceived = false;
  }
}
