#include <Braccio.h>
#include <Servo.h>

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;

#define HOME_BASE       90    
#define HOME_SHOULDER   90    
#define HOME_ELBOW      90    
#define HOME_WRIST_VER  90 
#define HOME_WRIST_ROT  90  
#define HOME_GRIPPER    73    


#define RED_BASE        60    
#define RED_SHOULDER    45   
#define RED_ELBOW       90   
#define RED_WRIST_VER   0    
#define RED_WRIST_ROT   0    
#define RED_GRIPPER     73  


#define YELLOW_BASE      90    
#define YELLOW_SHOULDER  30    
#define YELLOW_ELBOW     120   
#define YELLOW_WRIST_VER 180  
#define YELLOW_WRIST_ROT 180  
#define YELLOW_GRIPPER   73    


#define PICKUP_BASE     90    
#define PICKUP_SHOULDER 60    
#define PICKUP_ELBOW    120   
#define PICKUP_WRIST_VER 30   
#define PICKUP_WRIST_ROT 30   
#define PICKUP_GRIPPER  10   

void setup() {
  Serial.begin(9600);
  Braccio.begin();
  homePosition();
  Serial.println("READY");
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "RED") {
      pickObject();
      sortRed();
      Serial.println("DONE_RED");
    }
    else if (command == "YELLOW") {
      pickObject();
      sortYellow();
      Serial.println("DONE_YELLOW");
    }

  }
}


void homePosition() {
  Braccio.ServoMovement(30, HOME_BASE, HOME_SHOULDER, HOME_ELBOW, 
                        HOME_WRIST_VER, HOME_WRIST_ROT, HOME_GRIPPER);
}

void pickObject() {
 
  Braccio.ServoMovement(30, PICKUP_BASE, PICKUP_SHOULDER, PICKUP_ELBOW,
                        PICKUP_WRIST_VER, PICKUP_WRIST_ROT, 73);
  delay(500);
  
 
  Braccio.ServoMovement(30, PICKUP_BASE, PICKUP_SHOULDER, PICKUP_ELBOW,
                        PICKUP_WRIST_VER, PICKUP_WRIST_ROT, 10);
  delay(1000);
  
  Braccio.ServoMovement(30, PICKUP_BASE, PICKUP_SHOULDER, PICKUP_ELBOW,
                        PICKUP_WRIST_VER, PICKUP_WRIST_ROT, 73);
  delay(500);
}

void sortRed() {
  Braccio.ServoMovement(30, RED_BASE, RED_SHOULDER, RED_ELBOW, 
                        RED_WRIST_VER, RED_WRIST_ROT, RED_GRIPPER);
  delay(500);
  
  Braccio.ServoMovement(30, RED_BASE, RED_SHOULDER, RED_ELBOW, 
                        RED_WRIST_VER, RED_WRIST_ROT, 10);
  delay(500);
  
  homePosition(); 
}

void sortYellow() {
  Braccio.ServoMovement(30, YELLOW_BASE, YELLOW_SHOULDER, YELLOW_ELBOW, 
                        YELLOW_WRIST_VER, YELLOW_WRIST_ROT, YELLOW_GRIPPER);
  delay(500);
  
  Braccio.ServoMovement(30, YELLOW_BASE, YELLOW_SHOULDER, YELLOW_ELBOW, 
                        YELLOW_WRIST_VER, YELLOW_WRIST_ROT, 10);
  delay(500);
  
  homePosition();
}
