#include <AFMotor.h>
#include <NewPing.h>

#define TRIGGER_PIN 12
#define ECHO_PIN 11
#define MAX_DISTANCE 200
#define SETPOINT 9

//motor variables
AF_Stepper motor(200, 2);
int mode;
int highSteps = 0;
int steps = 0;
int lowSteps = -50;
const int STARTING_POINT = 48;

//error variables
float distance = 0;
float error = 0;
float prevError = 0;

//sample time variables
unsigned long previousMillis = 0;
const long interval = 100;  // Interval in milliseconds

//pid values
float kp = 1.2;
float ki = 0;
float kd = 0;
float pid_p;
float pid_i;
float pid_d;
float pid_total;

// Create an Ultrasonic sensor object
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);



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

    //measure distance from sensor, find how off it is from setpoint, apply PID
    distance = sonar.ping_cm();
    error = distance - SETPOINT;
    pid_p = error * kp;
    float distanceDifference = error - prevError;
    pid_d = kd * ((error - prevError) / interval);

    //if near the center add I, else dont
    if (error < 3) {
      pid_i += (ki * error);
    } else {
      pid_i = 0;
    }
    //sum pid values, save error
    pid_total = pid_p + pid_i + pid_d;
    prevError = error;

    //set motor mode based on where the cart is on the rail
    mode = distance < SETPOINT ? FORWARD : BACKWARD;
    pid_total = abs(pid_total);

    //apply pid output to motor
    motor.step(pid_total, distance < SETPOINT ? FORWARD : BACKWARD);
    Serial.print("distance: ");
    Serial.print(distance);
    Serial.print(" Speed: ");
    Serial.println(pid_total);


    //step code
    // steps += mode == FORWARD ? pid_total : -pid_total;
    // steps = constrain(steps, lowSteps, highSteps)
    // Serial.print(" Steps: ");
    // Serial.println(steps);
  }
}
