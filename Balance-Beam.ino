#include <AFMotor.h>
#include <NewPing.h>

#define TRIGGER_PIN 12
#define ECHO_PIN 11
#define MAX_DISTANCE 200
#define SETPOINT 9

float distance = 0;
float error = 0;
float prevError = 0;
int highSteps = 100;  // Max amount of steps it should be able to take
int steps = 0;        // Amount of steps it has taken
int lowSteps = 0;     // Steps should not go below 0
unsigned long previousMillis = 0;
const int STARTING_POINT = 48;
const long interval = 100;  // Interval in milliseconds

//pid values
float kp = 1.4;  //1.7
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

void setup() {
  Serial.begin(115200);
  Serial.println("Stepper test!");
  motor.setSpeed(50);
  // motor.step(STARTING_POINT, BACKWARD);  // Set bar to balance point off start
}

void loop() {
  unsigned long currentMillis = millis();

  // Call every 100 milliseconds
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    measureDistance();

    // PID calculations
    error = distance - SETPOINT;
    pid_p = error * kp;
    float distanceDifference = error - prevError;
    pid_d = kd * ((error - prevError) / interval);
    pid_i = pid_i + (ki * error);

    Serial.print("pid_i: ");
    Serial.print(pid_i);

    // // If centered don't add i or it will drift
    // if (error > -3 && error < 3) {
    //   pid_i = pid_i + (ki * error);
    // } else {
    //   pid_i = 0;
    // }

    // Calculate total PID output
    pid_total = pid_p + pid_i + pid_d;

    Serial.print(" Pid total: ");
    Serial.println(pid_total);

    // Save previous error
    prevError = error;

    // Get motor direction
    int mode = distance < SETPOINT ? FORWARD : BACKWARD;

    // Dont let PID output go below 0
    pid_total = pid_total < 0 ? -pid_total : pid_total;

    // Motor reacting to cart distance
    // Serial.println(mode);
    // motor.step(1, mode);
    if (distance < 10.5 && distance > 7.5) { pid_total = 0; }

    motor.step(pid_total, distance < SETPOINT ? FORWARD : BACKWARD);



    // Adjust steps and contrain them between min and max
    steps += mode == FORWARD ? pid_total : -pid_total;
    steps = constrain(steps, lowSteps, highSteps);

    // Serial.print("    Direction: ");
    // Serial.println(mode);
  }

  // Data prints
  // Serial.print("Steps: ");
  // Serial.println(steps);
  // Serial.print("PID: ");
  // Serial.println(pid_total);
  // Serial.print("Error: ");
  // Serial.println(error);
}

void measureDistance() {
  // Measure the distance using the ultrasonic sensor
  distance = sonar.ping_cm();
  // Serial.print("Distance: ");
  // Serial.print(distance);
  // Serial.print(" cm");
}