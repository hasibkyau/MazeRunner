#include "MazeRunner.h"
#include <HCSR04.h>

//Declaring Sonar sensor variable
int S1, S2, S3;
//Declaring digital pin for IR sensor
int IRA = 32, IRB = 33, IRC = 25, IRD = 34, IRE = 19; //IRD = Right side IR
//Declaring variable for IR
int valA = 0, valB = 0, valC = 0, valD = 0, valE = 0;

//HCSR04 sonarA(22, 23); //Front Sonor - initialisation class HCSR04 (trig pin , echo pin)
//HCSR04 sonarB(12, 21); //Right Sonor - initialisation class HCSR04 (trig pin , echo pin)
//HCSR04 sonarC(2, 15); //Left Sonor - initialisation class HCSR04 (trig pin , echo pin)

//Using class "Motor" {methods = Forward, Backward, Stop, Speed, Status}
Motor motorA(15, 4, 5);  // Right Motor - (in1, in2, en)
Motor motorB(13, 12, 14);  // Left Motor - (in1, in2, en)

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  //ActivateSonar(); // reading sonar 
  motorA.Speed(255);
  motorA.Forward();
  motorB.Speed(255);
  motorB.Forward();
  delay(500);
}

// Reading all Sonar sensor and passing data to another function for processing
void ActivateSonar() {
  // S1 = sonarA.dist();
  // Serial.print("Sonor 1: ");
  // Serial.println(S1);
  // S2 = sonarB.dist();
  // Serial.print("Sonor 2: ");
  // Serial.println(S2);
  // S3 = sonarC.dist();
  // Serial.print("Sonor 3: ");
  // Serial.println(S3);

  // PutSonorData(S1, S2, S3); // sending data for processing
}

