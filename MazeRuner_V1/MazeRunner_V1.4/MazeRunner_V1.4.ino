/*
 * Program: MazeRunner_V1.2.ino
 * 13/08/23 Hasibur Rahman
 * Hardware: ESP32 + L298N + OLED + 2 DC Motors
 */

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h> //OLED Display
#include <L298N.h> // Motor Driver
#include <NewPing.h> //Sonar Sensor

#define BLOCK 25 //Min distance required before taking a turn
#define SPEED 90 //Motor Speed

#define SONAR_NUM 3       // Number of sensors.
#define MAX_DISTANCE 400  // Maximum distance (in cm) to ping.

int front, left, right;



int enL = 13, in1 = 14, in2 = 15; // Left Motor connections
int enR = 25, in3 = 26, in4 = 27; // Right Motor connections
L298N LeftMotor(enL, in1, in2);
L298N RightMotor(enR, in3, in4);

// Initialize OLED display (128x64 pixels) using I2C communication.
Adafruit_SSD1306 display(128, 64, &Wire, -1);

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(15, 4, MAX_DISTANCE),  // Front Sensor: Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(19, 23, MAX_DISTANCE), // Left Sensor: Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(5, 18, MAX_DISTANCE)   // Right Sensor: Each sensor's trigger pin, echo pin, and max distance to ping.
};

// Function to initialize the OLED display
void displaySetup() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  display.setTextColor(WHITE);
}


void Intro() {
}


//Function for reding data from Sonar sensors
void readSonar() {
  front = sonar[0].ping_cm();  // Measure front distance
  left = sonar[1].ping_cm();   // Measure left distance
  right = sonar[2].ping_cm();  // Measure right distance
}

// Function to display text on the OLED screen at a specific position and size
void displayText(int x, int y, int textSize, const char *displayText) {
}


//Pinsetup
void pinSetup() {
  pinMode(enL, OUTPUT);
  pinMode(enR, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}



int TrackSize=10;
int LeftBlock = 30, RightBlock = 30, FrontBlock = 5, Balanced = 5, UBlock = 20;
bool F = true, L = false, R = false;
//Setup periferal devices
void setup() {
  Serial.begin(115200);  // Initialize the serial communication for debugging
  displaySetup();        // Initialize the OLED display
  pinSetup(); // Set all the motor control pins to outputs
  readSonar();
  TrackSize = left+right;
  LeftBlock=TrackSize/2;
  RightBlock=LeftBlock;
  delay(2000);
}



//Main Code
void loop() {
  readSonar();
  DisplayStatus();
  //Right Turn
  if (right >= 15) {
    F = false, L = false, R = true;
    goRight();

  }
  else if(front>15){
    F = true, L = false, R = false;
    goForward();
  }
  else if(left>=15){
    F = false, L = true, R = false;
    goLeft();
  }
  else{
    F = false, L = false, R = false;
    uTurn();
  }
}


//Function for Going Forward
void goForward() {
  RightMotor.forward();
  LeftMotor.forward();
  // RightMotor.setSpeed(SPEED);
  // LeftMotor.setSpeed(SPEED);  
    if(right > RightBlock){
      LeftMotor.setSpeed(SPEED+TrackSize);
      RightMotor.setSpeed(SPEED-TrackSize);
    }else if(right == RightBlock){
      LeftMotor.setSpeed(SPEED);
      RightMotor.setSpeed(SPEED);
    }else{
      LeftMotor.setSpeed(SPEED-TrackSize);
      RightMotor.setSpeed(SPEED+TrackSize);  
    }
}


void goRight(){
    int tFront = front;
    Straight();
    while(front>tFront-2){
        readSonar();
        DisplayStatus();
    }
    
    stop(500);

    RightMotor.stop();
    LeftMotor.setSpeed(SPEED-20);
    delay(500);

    stop(500);

    Straight();
    delay(500);
}

void goLeft(){
    int tFront = front;
    Straight();
    while(front>15){
        readSonar();
        DisplayStatus();
    }
    
    stop(500);

    LeftMotor.backward();
    RightMotor.forward();
    LeftMotor.setSpeed(SPEED-20);
    RightMotor.setSpeed(SPEED-20);
    delay(500);

    stop(500);

    Straight();
    delay(500);
}

void uTurn(){
    LeftMotor.forward();
    RightMotor.backward();
    LeftMotor.setSpeed(SPEED-30);
    RightMotor.setSpeed(SPEED-30);
}

void Straight(){
  LeftMotor.forward();
  RightMotor.forward();
  LeftMotor.setSpeed(SPEED);
  RightMotor.setSpeed(SPEED);
}

void stop(int t){
  LeftMotor.stop();
  RightMotor.stop();
  delay(t);
  LeftMotor.forward();
  RightMotor.forward();
}


//Function for showing current status
void DisplayStatus() {
  display.clearDisplay();
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.print("F:");
  display.print(front);

  display.setCursor(42, 0);
  display.print("L:");
  display.print(left);

  display.setCursor(84, 0);
  display.print("R:");
  display.print(right);

  display.setCursor(0, 10);
  if (F) display.print("Go Forward");
  else if (R) display.print("Go Right");
  else if (L) display.print("Go Left");
  else {
    display.print("U Turn");
  }
  display.display();

  Serial.print("Front:");
  Serial.println(front);
  Serial.print(" Left:");
  Serial.println(left);
  Serial.print(" Right:");
  Serial.println(right);
  Serial.println(" ");
}



//Print some informations in Serial Monitor
void printSomeInfo() {
  Serial.print("Motor is moving = ");
  Serial.print(LeftMotor.isMoving());
  Serial.print(" at setSpeed = ");
  Serial.println(LeftMotor.getSpeed());
}