#include <AFMotor.h>
#include <NewPing.h>

#define TRIGGER_PIN 12
#define ECHO_PIN 11
#define MAX_DISTANCE 200
#define SETPOINT 9

float distance = 0;
float error = 0;
float prevError = 0;
int highSteps = 100;
int steps = 0;
int lowSteps = 0;
unsigned long previousMillis = 0;
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
    // if (error > -3 && error < 3) {
    //   pid_i = pid_i + (ki * error);
    // } else {
    //   pid_i = 0;
    // }

    pid_total = pid_p + pid_i + pid_d;
    prevError = error;

    if (distance < SETPOINT && steps > lowSteps) {
      steps += pid_total;
      // motor.step(abs(pid_total), FORWARD);      
    } else if (distance > SETPOINT && steps < highSteps) {
      steps += pid_total;
      // motor.step(abs(pid_total), BACKWARD);
    } else if (distance < 10.5 && distance > 7.5) {
      // motor.release();
      // delay(800);
    }

    steps = constrain(steps, lowSteps, highSteps);
  }

  Serial.print("Steps: ");
  Serial.println(steps);
  // Serial.print("PID: ");
  // Serial.println(pid_total);
}

void measureDistance() {
  // Measure the distance using the ultrasonic sensor
  distance = sonar.ping_cm();
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}
