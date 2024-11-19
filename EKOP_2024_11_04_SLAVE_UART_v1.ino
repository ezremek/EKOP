#include <SimpleFOC.h>
#include <HardwareSerial.h>

HardwareSerial SerialUART(1);  // UART port for communication

// Setup for slave motor
MagneticSensorSPI sensor2 = MagneticSensorSPI(AS5048_SPI, 14);  // CS pin
BLDCDriver3PWM driver2 = BLDCDriver3PWM(13, 25, 27, 12);        // Motor driver pins
BLDCMotor motor2 = BLDCMotor(11);                               // 11 pole pairs

float master_angle = 0;  // Variable to store received master angle

void setup() {
  Serial.begin(115200);
  SerialUART.begin(115200, SERIAL_8N1, 16, 17);  // TX=16, RX=17

  sensor2.init();
  motor2.linkSensor(&sensor2);
  driver2.voltage_power_supply = 12;
  driver2.init();
  motor2.linkDriver(&driver2);

  motor2.controller = MotionControlType::torque;
  motor2.init();
  motor2.initFOC();

  Serial.println("Slave ready");
}

void loop() {
  motor2.loopFOC();

  // Send motor2 shaft angle to the master
  float angle_to_send = motor2.shaft_angle;
  SerialUART.print(angle_to_send);
  SerialUART.print('\n');

  // Receive master angle
  if (SerialUART.available()) {
    master_angle = SerialUART.parseFloat();
  }

  // Virtual link control
  motor2.move(5 * (master_angle - motor2.shaft_angle));
}
