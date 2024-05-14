[12:18 PM] Veniot,Nicholas
#include <AFMotor.h>
#include <NewPing.h>
 
#define TRIGGER_PIN 12
#define ECHO_PIN 11
#define MAX_DISTANCE 200
#define SETPOINT 9
 
float distance = 0;
float error = 0;
float prevError = 0;
int highSteps = 0;
int steps = 0;
int lowSteps = -50;
unsigned long previousMillis = 0;
const int STARTING_POINT = 48;
const long interval = 100;  // Interval in milliseconds
 
//pid values
float kp = 1.8;
float ki = 0;
float kd = 0;
 
float pid_p;
float pid_i;
float pid_d;
float pid_total;
 
// Create an Ultrasonic sensor object
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
 
// Connect a stepper motor to motor port #2 (M3 and M4)
AF_Stepper motor(200, 2);
 
void setup() {
  Serial.begin(115200);
  Serial.println("Stepper test!");
  motor.setSpeed(50);
  motor.step(STARTING_POINT, BACKWARD);
}
 
void loop() {
  unsigned long currentMillis = millis();
 
  // Call the function to measure distance every 500 milliseconds
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    measureDistance();
 
 
    error = distance - SETPOINT;
    pid_p = error * kp;
    float distanceDifference = error - prevError;
    pid_d = kd * ((error - prevError) / interval);
 
    //if centered dont add i or it will drift
    if (error > -3 && error < 3) {
      pid_i += (ki * error);
    } else {
      pid_i = 0;
    }
 
    pid_total = pid_p + pid_i + pid_d;
    prevError = error;
 
    int mode = distance < SETPOINT ? FORWARD : BACKWARD;
    pid_total = pid_total < 0 ? -pid_total : pid_total;
 
    // Serial.println(mode);
    // motor.step(1, mode);
 
    if (true) {
      motor.step(pid_total, distance < SETPOINT ? FORWARD : BACKWARD);
      Serial.print("Direction: ");
      Serial.print(mode);
      Serial.print(" Speed: ");
      Serial.print(pid_total);
    }
 
    steps += mode == FORWARD ? pid_total : -pid_total;
    steps = constrain(steps, lowSteps, highSteps);
 
 
 
    Serial.print(" Steps: ");
    Serial.println(steps);
  }
 
  // Serial.print("Steps: ");
  // Serial.println(steps);
  // Serial.print("PID: ");
  // Serial.println(pid_total);
}
 
void measureDistance() {
  // Measure the distance using the ultrasonic sensor
  distance = sonar.ping_cm();
  // Serial.print("Distance: ");
  // Serial.print(distance);
  // Serial.println(" cm");
}