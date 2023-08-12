#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// For two motors instance at once
#include <L298N.h>

#include <NewPing.h>

#define BLOCK 20
#define SPEED 80

#define SONAR_NUM 3       // Number of sensors.
#define MAX_DISTANCE 400  // Maximum distance (in cm) to ping.


// Initialize OLED display (128x64 pixels) using I2C communication.
Adafruit_SSD1306 display(128, 64, &Wire, -1);

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(15, 4, MAX_DISTANCE),  // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(19, 23, MAX_DISTANCE),
  NewPing(5, 18, MAX_DISTANCE)

};
// Left Motor connections
int enL = 13, in1 = 12, in2 = 14;

// Right Motor connections
int enR = 25, in3 = 27, in4 = 26;


L298N LeftMotor(enL, in1, in2);
L298N RightMotor(enR, in3, in4);


void setup() {
  Serial.begin(115200);  // Initialize the serial communication for debugging
  displaySetup();        // Initialize the OLED display
  // Set all the motor control pins to outputs
  pinSetup();
  // Set initial speed
  LeftMotor.setSpeed(SPEED);
  RightMotor.setSpeed(SPEED);
}

// Function to display text on the OLED screen at a specific position and size
void displayText(int x, int y, int textSize, const char *displayText) {
}

int front, left, right;
int LeftBlock = 30, RightBlock = 30, FrontBlock = 5, Balanced = 5, UBlock = 20;
bool F = true, L = false, R = false;
void readSonar() {
  front = sonar[0].ping_cm();  // Measure front distance
  left = sonar[1].ping_cm();   // Measure left distance
  right = sonar[2].ping_cm();  // Measure right distance
}

void loop() {

  readSonar();
  DisplayStatus();

  if (right < 20 && left < 20 && front > 10) {
    F = true, L = false, R = false;
    goForward();
  }

  else if (right > 25) {
    F = false, L = false, R = true;
    goRight();
  }

  else if (left>25 && right<25 && front<10) {
    F = false, L = true, R = false;
    goLeft();
    }

  else if (front<10 && left<30 && right<30) {
    F = false, L = false, R = false;
    uTurn();
  }


}



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

void goForward() {
  RightMotor.forward();
  LeftMotor.forward();

  if(left<=3){
    RightMotor.setSpeed(0);
    LeftMotor.setSpeed(SPEED-20);
  }else if(right<=3){
    RightMotor.setSpeed(SPEED-20);
    LeftMotor.setSpeed(0);    
  }else{
    LeftMotor.setSpeed(SPEED+right);
    RightMotor.setSpeed(SPEED+left);
  }
}

void goLeft() {
    LeftMotor.setSpeed(0);
    RightMotor.setSpeed(SPEED);
}


void goRight() {
    
    LeftMotor.setSpeed(SPEED);
    RightMotor.setSpeed(SPEED);
    delay(200);

    LeftMotor.forward();
    RightMotor.backward();
    LeftMotor.setSpeed(SPEED-10);
    RightMotor.setSpeed(SPEED-10);
    
    if(front<25){ 
        while(front<25){
            readSonar();
            DisplayStatus();
        } 
    }

    else{     
    
        while(left<25){
          readSonar();
          DisplayStatus();
        }

    }
  
}


void uTurn() {
  RightMotor.stop();
  LeftMotor.stop();
  delay(500);

  LeftMotor.setSpeed(SPEED - 10);
  RightMotor.setSpeed(SPEED - 10);

  if (right > left) {
    RightMotor.forward();
    LeftMotor.backward();
  } else if (left > right) {
    RightMotor.backward();
    LeftMotor.forward();
  }

  while (front < 20) {
    readSonar();
    DisplayStatus();
    printSomeInfo();
  }

  RightMotor.stop();
  LeftMotor.stop();
  delay(500);

  RightMotor.forward();
  LeftMotor.forward();

  LeftMotor.setSpeed(SPEED);
  RightMotor.setSpeed(SPEED);
}

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

/*
Print some informations in Serial Monitor
*/
void printSomeInfo() {
  Serial.print("Motor is moving = ");
  Serial.print(LeftMotor.isMoving());
  Serial.print(" at speed = ");
  Serial.println(LeftMotor.getSpeed());
}

// Function to initialize the OLED display
void displaySetup() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)

  display.setTextColor(WHITE);
}


void Intro() {
}