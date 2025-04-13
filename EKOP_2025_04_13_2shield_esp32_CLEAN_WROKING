#include <SimpleFOC.h>
//#include <PciManager.h>
//#include <PciListenerImp.h>

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

// Magnetic Sensors
MagneticSensorSPI sensor1 = MagneticSensorSPI(AS5048_SPI, CS1);
MagneticSensorSPI sensor2 = MagneticSensorSPI(AS5048_SPI, CS2);

// BLDC Motors
BLDCMotor motor1 = BLDCMotor(11);
BLDCMotor motor2 = BLDCMotor(11);

// Drivers
BLDCDriver3PWM driver1 = BLDCDriver3PWM(MOTOR1_A, MOTOR1_B, MOTOR1_C, MOTOR1_EN);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(MOTOR2_A, MOTOR2_B, MOTOR2_C, MOTOR2_EN);


void setup() {
  Serial.begin(115200);

  // SPI sensor setup
  SPI.begin(SCK, MISO, MOSI);
  sensor1.init();
  sensor2.init();

  // Motor 1 setup
  motor1.linkSensor(&sensor1);
  //  driver1.voltage_power_supply = 12;
  motor1.linkDriver(&driver1);
  motor1.controller = MotionControlType::torque;
  driver1.init();
  //  motor1.voltage_limit = 7;
  motor1.foc_modulation = FOCModulationType::SinePWM;
  motor1.init();
  motor1.initFOC();

  // Motor 2 setup
  motor2.linkSensor(&sensor2);
  //   driver2.voltage_power_supply = 12;
  motor2.linkDriver(&driver2);
  motor2.controller = MotionControlType::torque;
  driver2.init();
  //  motor2.voltage_limit = 7;
  motor2.foc_modulation = FOCModulationType::SinePWM;
  motor2.init();
  motor2.initFOC();

  Serial.println("Spring-linked motors with spike filtering ready!");
  _delay(1000);
}

void loop() {
  motor1.loopFOC();
  motor2.loopFOC();
  motor1.move(5 * (motor2.shaft_angle - motor1.shaft_angle));
  motor2.move(5 * (motor1.shaft_angle - motor2.shaft_angle));
}
