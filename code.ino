#include <Servo.h>

// Define pins for the ultrasonic sensor
const int trigPin = 12;    // TRIG pin for the ultrasonic sensor
const int echoPin = 13;    // ECHO pin for the ultrasonic sensor
Servo motor;               // Servo motor object
const int motorPin = 5;    // Pin for the servo motor

// Control parameters
float setPoint = 17.0;     // Target distance (center of the bar)
float Kp = 1;              // Proportional constant (P) - slight increase
float Ki = 0.0;            // Integral constant (I) - small value to prevent excessive error accumulation
float Kd = 0;              // Derivative constant (D) - slight increase to reduce overshoot

// PID variables
float error = 0, lastError = 0;
float integral = 0;
float derivative = 0;
float output = 0;

// Function prototypes
float readDistance();      // Reads distance from the ultrasonic sensor
void pidControl(float distance); // Controls the motor using the PID algorithm

void setup() {
  pinMode(trigPin, OUTPUT); // Set TRIG pin as output
  pinMode(echoPin, INPUT);  // Set ECHO pin as input

  motor.attach(motorPin);  // Attach the servo motor
  motor.write(90);         // Start at horizontal position

  Serial.begin(9600);      // Begin serial communication for monitoring
}

void loop() {
  // Read distance from the ultrasonic sensor
  float distance = readDistance();

  // If the reading is valid
  if (distance >= 0 && distance <= 30) {
    Serial.print("Distance: ");
    Serial.println(distance);

    // Control the motor using PID
    pidControl(distance);
  } else {
    motor.write(90); // Default horizontal position
  }

  delay(10); // Small delay for faster response
}

// Function to read distance using the ultrasonic sensor
float readDistance() {
  // Send a short TRIG pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); // Slightly longer pulse width
  digitalWrite(trigPin, LOW);

  // Measure the duration of the reflected pulse with a short timeout
  unsigned long duration = pulseIn(echoPin, HIGH, 20000); // 20ms timeout
  if (duration == 0) {
    return -1; // No signal detected
  }

  // Calculate and return the distance
  float distance = (duration / 2.0) * 0.0343;
  return distance;
}

// PID control algorithm
void pidControl(float distance) {
  // Calculate the error
  error = setPoint - distance;

  // Calculate integral and derivative terms
  integral += error * 0.01; // Improve integral with a small value to avoid excessive accumulation
  derivative = (error - lastError) / 0.01; // Calculate derivative over a fixed time interval

  // Calculate output using PID formula
  output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // Adjust motor angle (limit between 0 and 180 degrees)
  int angle = 90 + output;  // 90 is the horizontal position
  angle = constrain(angle, 70, 110);

  motor.write(angle);       // Move the motor to the calculated position
  lastError = error;        // Update the last error value
}
