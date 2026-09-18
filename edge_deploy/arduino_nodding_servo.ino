/*
 * Arduino code for RPLidar nodding mechanism - NIDAR AirMouse
 * Hardware: Arduino Nano + MG996R servo + RPLidar A2/A3
 * Servo mounted to nod RPLidar from -45° to +45° pitch
 * 
 * ROS2 side: /servo_angle_cmd (Float64 rad) -> serial "A90\n" -> servo 0-180°
 * 
 * Wiring:
 * Arduino D9 -> Servo PWM (orange)
 * Arduino 5V -> Servo VCC (red) via external 5V 2A BEC (not Arduino 5V!)
 * Arduino GND -> Servo GND (brown) + Jetson GND
 * Arduino USB -> Jetson /dev/ttyUSB1
 * 
 * Servo range: 0-180°, we use 45° to 135° for -45° to +45° nodding
 * Center 90° = 0° pitch (horizontal)
 */

#include <Servo.h>

Servo noddingServo;
const int servoPin = 9;
int currentAngle = 90;  // center
int targetAngle = 90;

void setup() {
  Serial.begin(115200);
  noddingServo.attach(servoPin);
  noddingServo.write(currentAngle);
  Serial.println("Nodding Lidar ready");
}

void loop() {
  // Check serial for angle command: "A90\n" or "A45\n"
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 1 && cmd[0] == 'A') {
      int angle = cmd.substring(1).toInt();
      if (angle >= 0 && angle <= 180) {
        targetAngle = angle;
      }
    }
  }

  // Smooth servo movement (avoid jerk)
  if (currentAngle != targetAngle) {
    if (currentAngle < targetAngle) currentAngle++;
    else currentAngle--;
    noddingServo.write(currentAngle);
    delay(10);  // 10ms per degree = ~0.1s for 10°
  }

  // Publish current angle back (optional)
  // Serial.print("P"); Serial.println(currentAngle);

  delay(5);
}
