#include <Servo.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 sensor;
Servo barServo;

// Ultrasonic sensor pins
#define TRIG 7
#define ECHO 8

// MPU6050 variables
int ax, ay, az;
float accel_ang_x = 0;
float filteredAngX = 0;
const float alpha = 0.9;  // smoothing factor (0.9 = more stable, 0.5 = faster)

// Servo and step input
int baseAngle = 80;      // horizontal position (0°)
int stepAngle = 18;      // step equivalent to ~5° real movement
unsigned long t0;
bool stepApplied = false;
bool stabilized = false;

// Variables for stability detection
float lastAng = 0, lastDist = 0;
int stableCounter = 0;
bool returnedToZero = false;
unsigned long stableTime = 0;

// -----------------------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();
  sensor.initialize();
  barServo.attach(9);
  
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  // MPU6050 calibration (adjust according to your sensor)
  sensor.setXAccelOffset(-734);
  sensor.setYAccelOffset(1267);
  sensor.setZAccelOffset(1152);
  sensor.setXGyroOffset(87);
  sensor.setYGyroOffset(-1);
  sensor.setZGyroOffset(3);

  barServo.write(baseAngle);
  delay(2000);

  Serial.println("time_ms,angle_mpu,distance_cm");

  t0 = millis();
}
// -----------------------------------
void loop() {

  if (stabilized && !returnedToZero && millis() - stableTime >= 2000) {
    barServo.write(baseAngle); 
    returnedToZero = true;
}

  if (stabilized) return; // if already stabilized, stop execution

  unsigned long t = millis() - t0;

  // --- MPU6050 reading ---
  sensor.getAcceleration(&ax, &ay, &az);
  accel_ang_x = atan(ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / 3.1416);

  // Exponential filter (reduces noise)
  filteredAngX = alpha * filteredAngX + (1 - alpha) * accel_ang_x;

  // --- Ultrasonic sensor reading ---
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  float duration = pulseIn(ECHO, HIGH, 30000); // 30 ms timeout
  float distance = duration * 0.0343 / 2;      // in cm

  // --- Apply step input after 3 seconds ---
  if (!stepApplied && t >= 3000) {
    barServo.write(stepAngle);
    stepApplied = true;
  }

  // --- Send data to Serial (for CSV logging) ---
  Serial.print(t);
  Serial.print(",");
  Serial.print(filteredAngX, 2);
  Serial.print(",");
  Serial.println(distance, 2);

  // --- Detect stabilization ---
  if (abs(filteredAngX - lastAng) < 1.0 && abs(distance - lastDist) < 1.0) {
    stableCounter++;
    if (stableCounter > 20 && t > 4000 && !stabilized) {
    stabilized = true;
    stableTime = millis();   
    }
  } else {
    stableCounter = 0;
  }
  lastAng = filteredAngX;
  lastDist = distance;

  delay(50);  // 20 Hz sampling rate
}