#include <AFMotor.h>
#include <NewPing.h>
 
#define TRIGGER_PIN 12
#define ECHO_PIN 11
#define MAX_DISTANCE 200
#define SETPOINT 11
 
int distance = 0;
unsigned long previousMillis = 0;
const long interval = 500;  // Interval in milliseconds
 
// Define PID parameters
double Kp = 1.0;  // Proportional gain
double Ki = 0.0;  // Integral gain
double Kd = 0.0;  // Derivative gain
 
double integral = 0.0;
double prev_error = 0.0;
 
// Create an Ultrasonic sensor object
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
 
// Connect a stepper motor with 100 steps per revolution
// to motor port #2 (M3 and M4)
AF_Stepper motor(200, 2);
 
void setup() {
  Serial.begin(9600);    
  Serial.println("Stepper test!");
 
  motor.setSpeed(25);  
}
 
void loop() {
  unsigned long currentMillis = millis();
 
  // Call the function to measure distance every 500 milliseconds
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
 
    measureDistance();
  }
 
  // Compute error
  double error = SETPOINT - distance;
 
  // Compute integral term
  integral += error;
 
  // Compute derivative term
  double derivative = error - prev_error;
 
  // Compute control signal
  double output = Kp * error + Ki * integral + Kd * derivative;
 
  // Update previous error
  prev_error = error;
 
  // Adjust motor position based on control signal
  // motor.step(output, FORWARD); // Adjust motor position based on control signal
}
 
void measureDistance() {
  // Measure the distance using the ultrasonic sensor
  distance = sonar.ping_cm();
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}