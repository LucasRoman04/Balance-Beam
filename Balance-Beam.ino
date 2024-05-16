#include <AFMotor.h>
#include <NewPing.h>
#include <Encoder.h>
 
#define TRIGGER_PIN 12
#define ECHO_PIN 11
#define MAX_DISTANCE 200
#define SETPOINT 9 //9
#define MARGIN 3
#define ENCODER_PIN1 7
#define ENCODER_PIN2 8

// int SETPOINT = 9;
int encoderReading;
float distance = 0;
float error = 0;
float prevError = 0;
unsigned long previousMillis = 0;
const int STARTING_POINT = 35;
const long interval = 50;  // Interval in milliseconds
int mode;
float distanceDifference;
 
// PID values
float kp = 1.002;  //1.7
float ki = 0;
float kd = 300;  //300
 
float pid_p;
float pid_i;
float pid_d;
float pid_total;
 
// Create an Ultrasonic sensor object
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
 
// Connect a stepper motor to motor port #2 (M3 and M4)
AF_Stepper motor(200, 2);

// Create an Encoder object(CAUSING ISSUES)
// Encoder encoder(ENCODER_PIN1, ENCODER_PIN2);
 
void setup() {
  Serial.begin(115200);
  Serial.println("Stepper test!");
  motor.setSpeed(50);
}
 
void loop() {
  unsigned long currentMillis = millis();

 
  // Call every 50 milliseconds
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    distance = sonar.ping_cm();
 
    // PID calculations
    error = distance - SETPOINT;
    pid_p = error * kp;
    distanceDifference = error - prevError;
    pid_d = kd * ((error - prevError) / interval);
    pid_i = pid_i + (ki * error);
 
    // Calculate total PID output
    pid_total = pid_p + pid_i + pid_d;
 
    // Save previous error
    prevError = error;
 
    // Get motor direction (mode)
    mode = distance < SETPOINT ? FORWARD : BACKWARD;
 
    // Dont let PID output go below 0
    pid_total = pid_total < 0 ? -pid_total : pid_total;
 
    // If cart is at setpoint, stop moving
    pid_total = distance < (SETPOINT + MARGIN) && distance > (SETPOINT - MARGIN) ? 0 : pid_total;
 
    // Move motor in relation with cart distance
    motor.step(pid_total, distance < SETPOINT ? FORWARD : BACKWARD);

  }
 
  // Data prints
  // Serial.print("Steps: ");
  // Serial.println(steps);
  // Serial.print("PID Total: ");
  // Serial.println(pid_total);
  // Serial.print("Error: ");
  // Serial.println(error);
  // Serial.print("PID I: ");
  // Serial.println(pid_i);
  // Serial.print("Direction: ");
  // Serial.println(mode);
  // Serial.println(encoderReading);
}
 
void measureDistance() {
  // Measure the distance using the ultrasonic sensor
  distance = sonar.ping_cm();
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}