#include <Servo.h>

#define PWM 9
#define OPEN_GATE 90
#define CLOSE_GATE 0

// Global Variables
Servo motor;

void setup() {
  Serial.begin(9600);
  motor.attach(PWM);
  Serial.println("Ready.");
}

void loop() {
  // Flags
  static bool openGate = false;
  
  // Read user input
  if (Serial.available() > 0) {
    String buff = Serial.readString();
    if (buff == "open\n")
      openGate = true;
    else if (buff == "close\n")
      openGate = false;
    else
      Serial.print("Invalid command: " + buff);
  }

  // Update motor position
  if (openGate)
    motor.write(OPEN_GATE);
  else
    motor.write(CLOSE_GATE);

  // Wait for motor position to update
  delay(15);
}
