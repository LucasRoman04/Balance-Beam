#include <AFMotor.h>
#include <NewPing.h>

#define TRIGGER_PIN 12
#define ECHO_PIN 11
#define MAX_DISTANCE 200
#define SETPOINT 10

float distance = 0;
float error = 0;
int maxSteps = 50;
int steps = 50;
unsigned long previousMillis = 0;
const long interval = 100;  // Interval in milliseconds

// Create an Ultrasonic sensor object
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Connect a stepper motor to motor port #2 (M3 and M4)
AF_Stepper motor(200, 2);

void setup() {
  Serial.begin(9600);
  Serial.println("Stepper test!");

  motor.setSpeed(50);
}

void loop() {
  unsigned long currentMillis = millis();

  // Call the function to measure distance every 500 milliseconds
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    measureDistance();
  }

  error = distance - SETPOINT;

  if (distance < SETPOINT && steps < maxSteps) {       
    motor.step(1, FORWARD);       
    steps++;     
  } else if (distance > SETPOINT && steps > -maxSteps) {       
    motor.step(1, BACKWARD);       
    steps--;     
   } else {
  }

  Serial.println(steps);

}

void measureDistance() {
  // Measure the distance using the ultrasonic sensor
  distance = sonar.ping_cm();
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}