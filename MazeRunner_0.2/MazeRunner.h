class Motor {
   private:
    int in1;
    int in2;
    int enable1Pin;

  public:
    Motor(int en, int in1, int in2) {
      this -> enable1Pin = en;
			this -> in1 = in1;
      this -> in2 = in2;
      
			pinMode(in1, OUTPUT);
      pinMode(in2, OUTPUT);
      pinMode(enable1Pin, OUTPUT);

      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);

			analogWrite(enable1Pin, 200);

    };

		void Forward(int spd) {
				digitalWrite(in1, LOW);
				digitalWrite(in2, HIGH);
        analogWrite(enable1Pin, spd);
		};

		void Backward(int spd) {
				//Serial.println("Backward");
				digitalWrite(in1, HIGH);
				digitalWrite(in2, LOW);
        analogWrite(enable1Pin, spd);
		};

		void Stop() {
				//Serial.println("Stop");
				digitalWrite(in1, LOW);
				digitalWrite(in2, LOW);
        analogWrite(enable1Pin, 0);
		};
};
//motorB=left
Motor motorA(13, 12, 14);  // Right Motor - (inputpin1, inputpin2, enablepin, pwmChannel[0-18])
Motor motorB(27, 26, 25);  // Left Motor - (in1, in2, en)



#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


Adafruit_SSD1306 display(128, 64, &Wire, -1);


void setup() {
  Serial.begin(115200);

  SonarSetup(15,4);
  SonarSetup(5,18);
  SonarSetup(19,23);

  displaySetup();
}

int space=5;

void loop() {
  int front = Sonar(15,4);
  int left = Sonar(19,23);
  int right = Sonar(5,18);

  ObstacleStatus(front, left, right);

  if(front>=10){
      motorA.Forward(150);
	    motorB.Forward(150);

      if(left>10){
        motorB.Forward(50);
        delay(1000);
        motorB.Forward(150);
        delay(1000);
      }
  }
  else if(left>=10){
        motorB.Forward(50);
        delay(1000);
        motorB.Forward(150);
        delay(1000);
  }
  else if(right>=10){
        motorA.Forward(50);
        delay(1000);
        motorA.Forward(150);
        delay(1000);
  }
  else{
      motorA.Forward(120);
      motorB.Backward(120);
      delay(1000); 
  }


  delay(200);  
}



void displaySetup(){
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  display.display();  //showing the Adafruit logo
  //do not forget display.display(); otherwise the picture will not be visible
  delay(200); //waiting 10ms
  display.clearDisplay();
  //loading screen
  display.setTextColor(WHITE); 
}

void ObstacleStatus(int front, int left, int right){
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
}


void SonarSetup(int trigPin, int echoPin){
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

int Sonar(int trigPin, int echoPin){
  long duration;
  int distance;
   // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  Serial.println(distance);

  return distance;
}
