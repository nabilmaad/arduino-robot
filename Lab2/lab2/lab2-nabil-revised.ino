#include <SoftwareSerial.h>
#include <Servo.h>

// Pin number constants
#define LEFT_MOTOR 45
#define LEFT_SENSOR 48
#define RIGHT_MOTOR 8
#define RIGHT_SENSOR 49
#define LED 13
#define LCD_PIN 18
#define LCD_DELAY 10

// Motor: direction constants
#define STOP 0
#define LEFT_BACKWARD 0
#define LEFT_FORWARD 180
#define RIGHT_BACKWARD 180
#define RIGHT_FORWARD 0

// Robot and lab dimensions (in cm)
#define WHEEL_DIAMETER 5.5
#define TILE_LENGTH 30.5
#define WHEEL_DISTANCE 20.0

// Optical Incremental Encoder transition needed
// to traverse 1 tile (HIGH to LOW or inverse)
int transitionsForOneTile;

// Optical Incremental Encoder transition needed
// for the wheels to traverse 1/4th of the circle
// whose diameter is D2, making the robot rotate
// 90 degrees if the wheels rotate in opposite directions
int transitionsForRotate90;

SoftwareSerial LCD(0, LCD_PIN);
Servo left;
Servo right;

void setup() {
  delay(5000);
  calculateTransitionsNeededForOneTile();
  calculateTransitionsNeededForRotate90();
  
  // Initialize all pins.
  left.attach(LEFT_MOTOR);
  right.attach(RIGHT_MOTOR);
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(LCD_PIN, OUTPUT);
  LCD.begin(9600);
  Serial.begin(9600);
}

void loop() {  
  // Move forward for 2 tiles
  forward(2);
  
  // Rotate 90 degrees right
  rotateRight90();
  
  // Move forward for 2 tiles
  forward(2);
  
  // Rotate 90 degrees right
  rotateRight90();
  
  // Move forward for 2 tiles
  forward(3);
  
  // Rotate 90 degrees right
  rotateRight90();
  
  // Move forward for 2 tiles
  forward(3);
  
  // Rotate 90 degrees right
  rotateRight90();
  
  // Move forward for 2 tiles
  forward(2);
  
  // Rotate 45 degrees left
  rotateLeft45();
  
  // Move forward for 1 diagonal tile
  forward(sqrt(2));
  
  // Rotate 135 degrees left
  rotateLeft135();
  
  // Move forward for 4 tiles
  forward(4);
  
  // Rotate 90 degrees left
  rotateLeft90();
  
  // Move forward for 2 tiles
  forward(2);
  
  // Rotate 90 degrees left
  rotateLeft90();
  
  // Move forward for 2 tiles
  forward(2);
    
  left.detach();
  right.detach();
  delay(60000);
}

void calculateTransitionsNeededForOneTile() {
  // Calculate number of transitions to traverse one tile
  double wheelPerimeter = PI * WHEEL_DIAMETER;
  double distanceTraversedForEachTransition = wheelPerimeter/64;
  transitionsForOneTile = TILE_LENGTH / distanceTraversedForEachTransition;
}

void calculateTransitionsNeededForRotate90() {
  // Calculate number of transitions to rotate 90 degrees
  double wheelPerimeter = PI * WHEEL_DIAMETER;
  double distanceTraversedForEachTransition = wheelPerimeter/64;
  
  double robotCirclePerimeter = PI * WHEEL_DISTANCE;
  transitionsForRotate90 = (robotCirclePerimeter / distanceTraversedForEachTransition) / 4;
}

void forward(int tiles) {
  // Start moving forward
  left.write(LEFT_FORWARD);
  right.write(RIGHT_FORWARD);
  
  int currentLeftPosition = digitalRead(LEFT_SENSOR);
  int currentRightPosition = digitalRead(RIGHT_SENSOR);
  int totalLeftTransitions = tiles * transitionsForOneTile;
  int totalRightTransitions = totalLeftTransitions;
  while(totalLeftTransitions != 0 && totalRightTransitions != 0) {
    if(digitalRead(LEFT_SENSOR) != currentLeftPosition && digitalRead(RIGHT_SENSOR) != currentRightPosition) {
      totalLeftTransitions--;
      totalRightTransitions--;
      currentLeftPosition = digitalRead(LEFT_SENSOR);
      currentRightPosition = digitalRead(RIGHT_SENSOR);
    }
  }
}

void rotateRight90() {
  left.write(LEFT_FORWARD);
  right.write(RIGHT_BACKWARD);
  
  int totalLeftTransitions = transitionsForRotate90;
  int totalRightTransitions = totalLeftTransitions;
  int currentLeftPosition = digitalRead(LEFT_SENSOR);
  int currentRightPosition = digitalRead(RIGHT_SENSOR);
  while(totalLeftTransitions != 0 && totalRightTransitions != 0) {
    if(digitalRead(LEFT_SENSOR) != currentLeftPosition && digitalRead(RIGHT_SENSOR) != currentRightPosition) {
      totalLeftTransitions--;
      totalRightTransitions--;
      currentLeftPosition = digitalRead(LEFT_SENSOR);
      currentRightPosition = digitalRead(RIGHT_SENSOR);
    }
  }
}

void rotateLeft90() {
  left.write(LEFT_BACKWARD);
  right.write(RIGHT_FORWARD);
  
  int currentLeftPosition = digitalRead(LEFT_SENSOR);
  int currentRightPosition = digitalRead(RIGHT_SENSOR);
  int totalLeftTransitions = transitionsForRotate90;
  int totalRightTransitions = totalLeftTransitions;
  while(totalLeftTransitions != 0 && totalRightTransitions != 0) {
    if(digitalRead(LEFT_SENSOR) != currentLeftPosition && digitalRead(RIGHT_SENSOR) != currentRightPosition) {
      totalLeftTransitions--;
      totalRightTransitions--;
      currentLeftPosition = digitalRead(LEFT_SENSOR);
      currentRightPosition = digitalRead(RIGHT_SENSOR);
    }
  }
}

void rotateLeft45() {
  left.write(LEFT_BACKWARD);
  right.write(RIGHT_FORWARD);
  
  int currentLeftPosition = digitalRead(LEFT_SENSOR);
  int currentRightPosition = digitalRead(RIGHT_SENSOR);
  int totalLeftTransitions = transitionsForRotate90 / 2;
  int totalRightTransitions = totalLeftTransitions;
  while(totalLeftTransitions != 0 && totalRightTransitions != 0) {
    if(digitalRead(LEFT_SENSOR) != currentLeftPosition && digitalRead(RIGHT_SENSOR) != currentRightPosition) {
      totalLeftTransitions--;
      totalRightTransitions--;
      currentLeftPosition = digitalRead(LEFT_SENSOR);
      currentRightPosition = digitalRead(RIGHT_SENSOR);
    }
  }
}

void rotateLeft135() {
  left.write(LEFT_BACKWARD);
  right.write(RIGHT_FORWARD);
  
  int currentLeftPosition = digitalRead(LEFT_SENSOR);
  int currentRightPosition = digitalRead(RIGHT_SENSOR);
  int totalLeftTransitions = 3 * transitionsForRotate90 / 2;
  int totalRightTransitions = totalLeftTransitions;
  while(totalLeftTransitions != 0 && totalRightTransitions != 0) {
    if(digitalRead(LEFT_SENSOR) != currentLeftPosition && digitalRead(RIGHT_SENSOR) != currentRightPosition) {
      totalLeftTransitions--;
      totalRightTransitions--;
      currentLeftPosition = digitalRead(LEFT_SENSOR);
      currentRightPosition = digitalRead(RIGHT_SENSOR);
    }
  }
}

//void stop_all() {
//  write(LEFT_MOTOR, STOP);
//  write(RIGHT_MOTOR, STOP);
//}

void lcdDisplay(String topLine, String bottomLine){
  // Calculation of the line offset for both line of the LCD screen
//  int topLineOffset = int((16-topLine.length())/2);
//  int bottomLineOffset = int((16-topLine.length())/2);
//
//  // Clear the LCD screen
//  backlightOn();
//  //displayOn();
//  lcdClear();
//  
//  // Print the top line text
//  if(topLine.length() > 0){
//    lcdPosition(0, topLineOffset);
//    LCD.print(topLine);
//  }
//  
//  // Print the bottom line text
//  if(bottomLine.length() > 0){
//    lcdPosition(1, bottomLineOffset);
//    LCD.print(bottomLine);
//  }
}

// Helper function for the LCD curser position
void lcd_position(int row, int col) {
  LCD.write(0xFE);   //command flag
  LCD.write((col + row*64 + 128));    //position 
  delay(LCD_DELAY);
}

void clear_LCD(){
  LCD.write(0xFE);   //command flag
  LCD.write(0x01);   //clear command.
  delay(LCD_DELAY);
}
