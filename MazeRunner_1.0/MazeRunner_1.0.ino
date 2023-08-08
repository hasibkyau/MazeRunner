#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// For two motors instance at once
#include <L298N.h>

#include <NewPing.h>

#define SONAR_NUM 3      // Number of sensors.
#define MAX_DISTANCE 300 // Maximum distance (in cm) to ping.


// Initialize OLED display (128x64 pixels) using I2C communication.
Adafruit_SSD1306 display(128, 64, &Wire, -1);

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(2, 15, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(18, 19, MAX_DISTANCE),
  NewPing(5, 4, MAX_DISTANCE)
};
// Left Motor connections
int enL = 25, in1 = 26, in2 = 27;
// Right Motor connections
int enR = 13, in3 = 12, in4 = 14;

L298N LeftMotor(enL, in1, in2);
L298N RightMotor(enR, in3, in4);

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
    LeftMotor.setSpeed(100);
    RightMotor.setSpeed(100);
}


void setup(){
    Serial.begin(115200); // Initialize the serial communication for debugging
    displaySetup(); // Initialize the OLED display
    // Set all the motor control pins to outputs
    pinSetup();
}

// Function to display text on the OLED screen at a specific position and size
void displayText(int x, int y, int textSize, const char *displayText){

}


int block = 10;
int front, left, right;
bool F=true, L=false, R=false;

void loop(){

    front = sonar[0].ping_cm();  // Measure front distance
    left =  sonar[1].ping_cm();  // Measure left distance
    right = sonar[2].ping_cm(); // Measure right distance

    DisplayStatus();
    
    if(right>10){
        F=false, L=false, R=true;
        goRight();
    }
    else if(front>10){
        F=true, L=false, R=false;
        goForward();
    }
    else if(left>10){
        F=false, L=true, R=false;
        goLeft();
    }
    else{
        F=false, L=true, R=false;
        uTurn();
    }
}

void pinSetup(){
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
    display.print("F:");
    display.print(front);

    display.setCursor(42, 0);
    display.print("L:");
    display.print(left);

    display.setCursor(84, 0);
    display.print("R:");
    display.print(right);
    
    display.setCursor(0, 10);
    if(F)display.print("Go Forward");
    else if(R)display.print("Go Right");
    else if(L)display.print("Go Left");
    else{
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

void Intro(){

}