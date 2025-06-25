#include <Braccio.h>
#include <Servo.h>

// Declare servos used by Braccio
Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;

// Function declarations
void homePosition();
void pickPosition();
void pickObject();
void sortRed();
void sortGreen();

// Home position
#define HOME_BASE       90
#define HOME_SHOULDER   90
#define HOME_ELBOW      90
#define HOME_WRIST_VER  80
#define HOME_WRIST_ROT  90
#define HOME_GRIPPER    10

// Red Bin Position
#define RED_BASE        150
#define RED_SHOULDER    25
#define RED_ELBOW       85
#define RED_WRIST_VER   30
#define RED_WRIST_ROT   60
#define RED_GRIPPER     80

// Green Bin Position
#define GREEN_BASE     180
#define GREEN_SHOULDER 25
#define GREEN_ELBOW    85
#define GREEN_WRIST_VER 30
#define GREEN_WRIST_ROT 60
#define GREEN_GRIPPER   80

// Pickup
#define PICKUP_BASE      90
#define PICKUP_SHOULDER  25
#define PICKUP_ELBOW     85
#define PICKUP_WRIST_VER 30
#define PICKUP_WRIST_ROT 60
#define PICKUP_GRIPPER   80

void setup() {
  Serial.begin(115200);
  Braccio.begin();  // Uses the globally defined servos
  homePosition();
  Serial.println("READY");
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "RED") {
      pickObject();
      delay(500);
      sortRed();
      Serial.println("DONE");
    } 
    else if (command == "GREEN") {
      pickObject();
      delay(500);
      sortGreen();
      Serial.println("DONE");
    }
  }
}

void homePosition() {
  Braccio.ServoMovement(20, HOME_BASE, HOME_SHOULDER, HOME_ELBOW,
                        HOME_WRIST_VER, HOME_WRIST_ROT, HOME_GRIPPER);
  delay(500);
}

void pickPosition() {
  Braccio.ServoMovement(20, PICKUP_BASE, PICKUP_SHOULDER, PICKUP_ELBOW,
                        PICKUP_WRIST_VER, PICKUP_WRIST_ROT, PICKUP_GRIPPER);
  delay(500);
}

void pickObject() {
  Braccio.ServoMovement(20, PICKUP_BASE, PICKUP_SHOULDER, PICKUP_ELBOW,
                        PICKUP_WRIST_VER, PICKUP_WRIST_ROT, 10); // Open gripper
  delay(500);

  Braccio.ServoMovement(20, PICKUP_BASE, PICKUP_SHOULDER, PICKUP_ELBOW,
                        PICKUP_WRIST_VER, PICKUP_WRIST_ROT, 80); // Close gripper
  delay(500);

  Braccio.ServoMovement(20, HOME_BASE, HOME_SHOULDER, HOME_ELBOW,
                        HOME_WRIST_VER, HOME_WRIST_ROT, 80); // Lift
}

void sortRed() {
  Braccio.ServoMovement(20, RED_BASE, HOME_SHOULDER, HOME_ELBOW,
                        HOME_WRIST_VER, HOME_WRIST_ROT, RED_GRIPPER);
  delay(500);
  Braccio.ServoMovement(20, RED_BASE, RED_SHOULDER, RED_ELBOW,
                        RED_WRIST_VER, RED_WRIST_ROT, RED_GRIPPER);
  delay(500);
  Braccio.ServoMovement(20, RED_BASE, RED_SHOULDER, RED_ELBOW,
                        RED_WRIST_VER, RED_WRIST_ROT, 10); // Release
  delay(500);
  homePosition();
}

void sortGreen() {
  Braccio.ServoMovement(20, GREEN_BASE, HOME_SHOULDER, HOME_ELBOW,
                        HOME_WRIST_VER, HOME_WRIST_ROT, GREEN_GRIPPER);
  delay(500);
  Braccio.ServoMovement(20, GREEN_BASE, GREEN_SHOULDER, GREEN_ELBOW,
                        GREEN_WRIST_VER, GREEN_WRIST_ROT, GREEN_GRIPPER);
  delay(500);
  Braccio.ServoMovement(20, GREEN_BASE, GREEN_SHOULDER, GREEN_ELBOW,
                        GREEN_WRIST_VER, GREEN_WRIST_ROT, 10); // Release
  delay(500);
  homePosition();
}
