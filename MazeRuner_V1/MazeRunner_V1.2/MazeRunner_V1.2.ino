#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// For two motors instance at once
#include <L298N.h>

#include <NewPing.h>

#define BLOCK 20
#define SPEED 90

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


  if (right >= 20) {
    F = false, L = false, R = true;
    //goRight();
    if(front>=20){
      GO(10);
    }
    else{
      GO(5);
    }
    STOP(500);
    Right90();
    STOP(300);
    GO(10);
  }

  
  else if (front >= 20) {
    F = true, L = false, R = false;
    goForward();
  }

  else if (left>=20) {
    F = false, L = true, R = false;
    if(front>20){
      GO(10);
    }
    else{
      GO(5);
    }
    STOP(500);
    Left90();
    STOP(500);
    GO(10);
    }

  else{
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

  //#Method 1: Robot will follow only the right wall
  // if(right<=3){
  //   RightMotor.setSpeed(SPEED);
  //   LeftMotor.setSpeed(40);
  // }else if(right>=5){
  //   RightMotor.setSpeed(40);
  //   LeftMotor.setSpeed(SPEED);    
  // }
  // else{
  //   RightMotor.setSpeed(SPEED+10);
  //   LeftMotor.setSpeed(SPEED+10);   
  // }

  //# Method 2: Robot will take decision from both left and right wall distance
  // if(left<10){
  //   if(left<=3){
  //     RightMotor.setSpeed(SPEED-20);
  //     LeftMotor.setSpeed(SPEED+20);
  //   }else if(right<=3){
  //     RightMotor.setSpeed(SPEED+20);
  //     LeftMotor.setSpeed(SPEED-20);  
  //   }
  //   else{
  //     LeftMotor.setSpeed(SPEED);
  //     RightMotor.setSpeed(SPEED);
  //   }
  // }

 // else{
    if(right<=3){
      RightMotor.setSpeed(SPEED+30);
      LeftMotor.setSpeed(SPEED-10);
    }else if(right>3 && right<=4){
      RightMotor.setSpeed(SPEED+20);
      LeftMotor.setSpeed(SPEED-10);
    }else if(right>=5 && right<=7){
      RightMotor.setSpeed(SPEED);
      LeftMotor.setSpeed(SPEED);  
    }else if(right>7 && right<=10){
      RightMotor.setSpeed(SPEED-10);
      LeftMotor.setSpeed(SPEED+20);  
    }else if(right>10){
      RightMotor.setSpeed(SPEED-10);
      LeftMotor.setSpeed(SPEED+30);  
    }else{
      LeftMotor.setSpeed(SPEED);
      RightMotor.setSpeed(SPEED);
    }
  //}
  // else if(left<=5){
  //   RightMotor.setSpeed(SPEED-20);
  //   LeftMotor.setSpeed(SPEED+20);
  // }else if(right<=5){
  //   RightMotor.setSpeed(SPEED+20);
  //   LeftMotor.setSpeed(SPEED-20);    
  // }

}

void goLeft() {
    // LeftMotor.setSpeed(SPEED);
    // RightMotor.setSpeed(SPEED);
    // delay(300);


    LeftMotor.backward();
    RightMotor.forward();
    RightMotor.setSpeed(SPEED);
    LeftMotor.setSpeed(SPEED);
    while(front<20){
            readSonar();
            DisplayStatus();
    }
    LeftMotor.forward();
}


void goRight() {
    //   LeftMotor.setSpeed(SPEED);
    // RightMotor.setSpeed(SPEED);
    // delay(400);

    LeftMotor.forward();
    RightMotor.stop();
    LeftMotor.setSpeed(SPEED);
    //RightMotor.setSpeed(SPEED);
    
    if(front<20){ 
        while(front<15){
            readSonar();
            DisplayStatus();
        } 
    }

    else{     
    
        while(left<15){
          readSonar();
          DisplayStatus();
        }

    }
    RightMotor.forward();
    delay(400);

  
}


void uTurn() {
  RightMotor.stop();
  LeftMotor.stop();
  delay(500);

  LeftMotor.setSpeed(SPEED - 10);
  RightMotor.setSpeed(SPEED - 10);

  if (right > left) {
    RightMotor.backward();
    LeftMotor.forward();
  } else if (left > right) {
    RightMotor.forward();
    LeftMotor.backward();
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

void GO(int s){
  RightMotor.setSpeed(SPEED);
  LeftMotor.setSpeed(SPEED);
  RightMotor.forward();
  LeftMotor.forward();
  int s1 = front;
  int destination=s1-s;
  while(front>destination){
    readSonar();
    DisplayStatus();
  }
}

void Right90(){
  LeftMotor.forward();
  RightMotor.backward();
  RightMotor.setSpeed(SPEED+10);
  LeftMotor.setSpeed(SPEED);
  while(front<20){
    readSonar();
    DisplayStatus();
  }
  RightMotor.forward();
}

void Left90(){
  LeftMotor.backward();
  RightMotor.forward();
  RightMotor.setSpeed(SPEED+10);
  LeftMotor.setSpeed(SPEED);
  while(front<20){
    readSonar();
    DisplayStatus();
  }
  RightMotor.forward();
}

void STOP(int t){
  RightMotor.stop();
  LeftMotor.stop();
  delay(t);
}