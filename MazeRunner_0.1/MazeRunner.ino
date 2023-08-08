#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HCSR04.h> // sonar Library
// For two motors instance at once
#include <L298N.h>


// Initialize OLED display (128x64 pixels) using I2C communication.
Adafruit_SSD1306 display(128, 64, &Wire, -1);


HCSR04 hc(5, new int[3]{2, 12, 35}, 3); //initialisation class HCSR04 (trig pin , echo pin, number of sensor)
//front = 2, left = 12, right = 35


// Left Motor connections
int enL = 25, in1 = 26, in2 = 27;
// Right Motor connections
int enR = 17, in3 = 18, in4 = 19;

L298N LeftMotor(enL, in1, in2);
L298N RightMotor(enR, in1, in2);

/*
Print some informations in Serial Monitor
*/
void printSomeInfo()
{
  Serial.print("Motor is moving = ");
  Serial.print(LeftMotor.isMoving());
  Serial.print(" at speed = ");
  Serial.println(LeftMotor.getSpeed());
}

// Function to initialize the OLED display
void displaySetup(){
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize with the I2C addr 0x3C (for the 128x32)

    display.setTextColor(WHITE);
    // Set initial speed
    LeftMotor.setSpeed(70);
    RightMotor.setSpeed(70);
}


void setup(){
    Serial.begin(115200); // Initialize the serial communication for debugging
    displaySetup(); // Initialize the OLED display
    // Set all the motor control pins to outputs
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

// Function to display text on the OLED screen at a specific position and size
void displayText(int x, int y, int textSize, const char *displayText){

}


int block = 10;
int front, left, right;

void loop(){
    //int front = measureDistance(15, 2);   // Measure front distance
    //int left = measureDistance(13, 12);  // Measure left distance
    //int right = measureDistance(32, 35); // Measure right distance


    front = hc.dist(0);   // Measure front distance
    left = hc.dist(1);  // Measure left distance
    right = hc.dist(2); // Measure right distance

    DisplayStatus();
    
    if(right>10){
        goRight();
    }
    else if(front>10){
        goForward();
    }
    else if(left>10){
        goLeft();
    }
    else{
        uTurn();
    }
}

void goForward(){
    RightMotor.forward();
    LeftMotor.forward();
    printSomeInfo();

}

void goLeft(){
    RightMotor.stop();
    LeftMotor.stop();

    RightMotor.backward();
    LeftMotor.forward();
    
    printSomeInfo();
}


void goRight(){
    RightMotor.stop();
    LeftMotor.stop();

    RightMotor.forward();
    LeftMotor.backward();
    
    printSomeInfo();
    
}


void uTurn(){
    RightMotor.stop();
    LeftMotor.stop();

    RightMotor.forward();
    LeftMotor.backward();
    
    printSomeInfo();
}

void DisplayStatus(){
    display.clearDisplay();
    display.setTextSize(1);
    
    display.setCursor(0, 0);
    display.print("Front: ");
    display.println(front);

    display.setCursor(0, 10);
    display.print("Left: ");
    display.println(left);

    display.setCursor(0, 20);
    display.print("Right: ");
    display.println(right);
    display.display();
    
    Serial.print("Front:");
    Serial.println(front);
    Serial.print(" Left:");
    Serial.println(left);
    Serial.print(" Right:");
    Serial.println(right);
    Serial.println(" ");
}

void Intro(){

}