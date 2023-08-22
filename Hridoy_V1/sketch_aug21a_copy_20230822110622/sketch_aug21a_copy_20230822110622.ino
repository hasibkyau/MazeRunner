// Include necessary libraries
#include <SPI.h>              // For Serial Peripheral Interface communication
#include <Wire.h>             // For I2C communication
#include <Adafruit_GFX.h>     // Adafruit's graphics library for displays
#include <Adafruit_SSD1306.h> // Adafruit's SSD1306 OLED display library
#include <NewPing.h>          // Library for using ultrasonic distance sensor
#include <L298NX2.h>          // Library for controlling motors using L298N module
  
#define SONAR_NUM 3      // Number of sensors.
#define MAX_DISTANCE 400 // Maximum distance (in cm) to ping.

int block = 10;
// int motorDelay = 5000;
int front, left, right;                 // Declare variables to store distances from sensors
bool F = true, L = false, R = false;    // Define boolean variables to represent sensor directions


NewPing sonar[SONAR_NUM] = {      // Sensor object array.
    NewPing(15, 4, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
    NewPing(18, 5, MAX_DISTANCE),
    NewPing(19, 23, MAX_DISTANCE)};

// Initialize OLED display (128x64 pixels) using I2C communication.
Adafruit_SSD1306 display(128, 64, &Wire, -1);



// Left Motor connections
int enL = 13, in1 = 14, in2 = 12;
// Right Motor connections
int enR = 25, in3 = 26, in4 = 27;

int LeftSpeed = 100;
int RightSpeed = 120;
int wait = 500;


L298NX2 motors(enL, in1, in2, enR, in3, in4);   // Create an instance of the L298NX2 motor controller with the specified pins


void setup(){
    Serial.begin(115200); // Initialize the serial communication for debugging
    displaySetup();       // Initialize the OLED display

    // Turn off motors - Initial state
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);

    delay(2000);
}



void readsonar(){
    front = sonar[0].ping_cm(); // Measure front distance
    left = sonar[1].ping_cm();  // Measure left distance
    right = sonar[2].ping_cm(); // Measure right distance
} 



void goForward() {  // Function to make the robot go forward
    // Set initial speed for both motors
    motors.setSpeedA(LeftSpeed);
    motors.setSpeedB(RightSpeed);
    motors.forward();       // Move both motors forward
    readsonar();
    if (right < 6) {
        motors.setSpeedA(LeftSpeed-40);
        motors.setSpeedB(RightSpeed);
        motors.forward(); 
    }
    else if (left < 6) {
        motors.setSpeedA(LeftSpeed);
        motors.setSpeedB(RightSpeed - 40);
        motors.forward(); 
    }

    else{   
        F = true, L = false, R = false;
        motors.setSpeedA(LeftSpeed);
        motors.setSpeedB(RightSpeed);
        motors.forward(); 
    } 
    printSomeInfo();        // Print motor information
}
void goLeft() {     // Function to make the robot turn left
    // Set initial speed for both motors


    motors.stopA();
    motors.forwardB();
    motors.setSpeedB(RightSpeed);
    
    // delay(wait);
    while (front <30){
        readsonar();
        DisplayStatus(); 
        printSomeInfo();
    }

    motors.setSpeedA(LeftSpeed);
    motors.forwardA(); 
}

void goRight() {    // Function to make the robot turn right
    // Set initial speed for both motors
    motors.setSpeedA(LeftSpeed);
    motors.setSpeedB(0);
    motors.forward();

    // delay(wait);
    while (front <30){
        readsonar();
        DisplayStatus(); 
        printSomeInfo();
    }


    motors.setSpeedB(RightSpeed);
    motors.forward();
}



void uTurn() {  // Function to make the robot perform a U-turn
    readsonar();
    // Set initial speed for both motors
    motors.setSpeedA(LeftSpeed);
    motors.setSpeedB(RightSpeed);
    // Stop motors, then move one motor backward and the other forward
    motors.stop();
    motors.backwardA();
    motors.forwardB();
    while (front <25){
        readsonar();
        DisplayStatus();
        printSomeInfo();
    }
    readsonar();
    motors.setSpeedA(LeftSpeed);
    motors.setSpeedB(RightSpeed);
    motors.forwardA();
    motors.forwardB();
}

void loop(){
    MazeSolve();
    // motors.forwardA();
    // motors.forwardB();
    // motors.setSpeedA(LeftSpeed);
    // motors.setSpeedB(RightSpeed);
    // readsonar();
    // DisplayStatus();
}

void MazeSolve(){
    readsonar();
    display.clearDisplay(); // Clear the display buffer
    DisplayStatus();   // Display the current status on the OLED display
    printSomeInfo();
    
    // Check distance readings and decide robot's movement
    if (right > 25) {
        F = false, L = false, R = true;
        goRight();
    }

    else if (left > 25){
        F = false, L = true, R = false;
        goLeft();
    }
    
    else if (front > 15){   
        F = true, L = false, R = false;
        goForward();
    }

    else{
        F = false, L = false, R = false;
        uTurn();
    }

    display.display(); // Display the buffer on the OLED screen
    
}




// Function to display the status information on the OLED display
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



// Function to initialize the OLED display
void displaySetup(){
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize with the I2C addr 0x3C (for the 128x32)
    display.display();                         // showing the Adafruit logo
    // do not forget display.display(); otherwise the picture will not be visible
    display.setTextColor(WHITE);
}




// Function to display text on the OLED display with the specified position, size, text, and value
void displayText(int x, int y, int textSize, const char *displayText, int value) {
    display.setCursor(x, y);         // Set the cursor position on the display
    display.setTextSize(textSize);   // Set the text size
    display.print(displayText);      // Print the provided text
    display.println(value);          // Print the provifront, left, rightded value
}



// Function to print information about motor movement and speed to the Serial Monitor
void printSomeInfo() {
    Serial.print("Motor A is moving = ");
    Serial.print(motors.isMovingA() ? "YES" : "NO");
    Serial.print(" at speed = ");
    Serial.println(motors.getSpeedA());
    Serial.print("Motor B is moving = ");
    Serial.print(motors.isMovingB() ? "YES" : "NO");
    Serial.print(" at speed = ");
    Serial.println(motors.getSpeedB());
}


