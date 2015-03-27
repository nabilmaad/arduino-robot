/**************************************************************** *
* Names:        Nabil Maadarani 6134578
*               Franck Mamboue  6175122
*               Christian Kabuya 6177914
* Course Code:  SEG4145
* Lab Number:   3
* File name:    lab3.ino
* Date:         Mars 12, 2015
*
* Description
* *************
* This script makes the robot display our student numbers for 5 seconds
* while flickering the LEDs, then travels forward in a straight line one
* tile at a time stopping for one second after one tile has been traversed.
* If an object has been detected approximately 10 cm away or closer the
* following collision avoidance algorithm must be performed: 
* o The robot must come to an immediate stop 
* o The ambient temperature must be measured by the robot and displayed on 
* the LCD screen for 5 seconds with the following message: 
*  - Temperature (top line) 
*  - x degrees (bottom line) 
* o The robot must move backwards one tile. 
* o The robot must rotate clockwise approximately 90 degrees 
* o The robot must move forward two tiles 
* o the robot must rotate counter clockwise approximately 90 degrees 
* o The robot continues moving forward until another object is encountered
* 
* The robot must display its current movement (i.e. moving forward).
* *************************************************************** */

#include <SoftwareSerial.h>
#include <Servo.h>
#include <Wire.h> 

// Pin number constants
#define LEFT_MOTOR 45
#define LEFT_SENSOR 48
#define RIGHT_MOTOR 8
#define RIGHT_SENSOR 49
#define LED 13
#define LCD_PIN 18
#define LCD_DELAY 10
#define SONAR 22

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

// Thermal sensor: address
#define TEMPSENSOR 0x68

// Variable for reading the temperature
int reg = 0x01;

// Optical Incremental Encoder transition needed
// to traverse 1 tile (HIGH to LOW or inverse)
double transitionsForOneTile;

// Optical Incremental Encoder transition needed
// for the wheels to traverse 1/4th of the circle
// whose diameter is D2, making the robot rotate
// 90 degrees if the wheels rotate in opposite directions
double transitionsForRotate90;

SoftwareSerial LCD(0, LCD_PIN);
Servo left;
Servo right;

/****************************************************************************** *
* Name **************
* setup()
*
* Description
* *************
* This function first calculates the number of transitions needed for traversing
* one tile. Then, it calculates the number of transitions needed for rotating
* for 90 degrees. Finally, it sets up the wheels, LED, and LCD of the robot by indicating
* their pin numbers on the board. Additionally, it initialized the LCD instance.
*
* Parameters
* *************
* None
*
* Returns
* *************
* void
*
* ****************************************************************************** */
void setup() {
  delay(5000);
  calculateTransitionsNeededForOneTile();
  calculateTransitionsNeededForRotate90();
  
  // Initialize all pins.
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(LCD_PIN, OUTPUT);
  LCD.begin(9600);
  Serial.begin(9600);
  
  // Thermal Sensor: join I2C bus
  Wire.begin();
  
  // Display student numbers
  clear_LCD();
  lcd_position(0,5);
  LCD.print("6134578");
  lcd_position(1,5);
  LCD.print("6175122");

  flickerLed();
}

/****************************************************************************** *
* Name **************
* loop()
*
* Description
* *************
* This is the program's main loop. In this case, it performs the steps necessary
* for path described at the top of this document. It also adds a 60 second
* delay at the end, allowing the experiment to repeat if the robot is not
* switched off a minute later.
*
* Parameters
* *************
* None
*
* Returns
* *************
* void
*
* ****************************************************************************** */
void loop() {
  // Move forward
  forward(1.0, false);
  delay(1000);
}

/****************************************************************************** *
* Name **************
* calculateTransitionsNeededForOneTile()
*
* Description
* *************
* This function calculates the number of transitions needed to traverse one tile.
*
* Parameters
* *************
* None
*
* Returns
* *************
* void
*
* ****************************************************************************** */
void calculateTransitionsNeededForOneTile() {
  // Calculate number of transitions to traverse one tile
  double wheelPerimeter = PI * WHEEL_DIAMETER;
  double distanceTraversedForEachTransition = wheelPerimeter/64;
  transitionsForOneTile = TILE_LENGTH / distanceTraversedForEachTransition;
}

/****************************************************************************** *
* Name **************
* calculateTransitionsNeededForRotate90()
*
* Description
* *************
* This function calculates the number of transitions needed for a 90 degrees rotation.
*
* Parameters
* *************
* None
*
* Returns
* *************
* void
*
* ****************************************************************************** */
void calculateTransitionsNeededForRotate90() {
  // Calculate number of transitions to rotate 90 degrees
  double wheelPerimeter = PI * WHEEL_DIAMETER;
  double distanceTraversedForEachTransition = wheelPerimeter/64;
  
  double robotCirclePerimeter = PI * WHEEL_DISTANCE;
  transitionsForRotate90 = (robotCirclePerimeter / distanceTraversedForEachTransition) / 4;
}

/****************************************************************************** *
* Name **************
* printTemperature()
*
* Description
* *************
* This function prints the ambient temperature on the robot's LCD screen
*
* Parameters
* *************
* None
*
* Returns
* *************
* void
*
* ****************************************************************************** */
void printTemperature() {
  Wire.beginTransmission(TEMPSENSOR); 
  Wire.write(reg); // Indicate temperature value to read 
  Wire.endTransmission(); 
  Wire.requestFrom(TEMPSENSOR, 1); // Request data 
  while(Wire.available() < 1); // Wait for data 
  byte temperatureData = Wire.read(); // Temp. value 
 
  clear_LCD();
  lcd_position(0, 2);
  LCD.print("Temperature");
  lcd_position(1, 3);
  LCD.print(temperatureData);
  lcd_position(1, 6);
  LCD.print("degrees");
 
  delay(2000); // Delay 2s
}

/****************************************************************************** *
* Name **************
* flickerLed()
*
* Description
* *************
* This function flickers the LED light of the robot for 5 seconds
*
* Parameters
* *************
* None
*
* Returns
* *************
* void
*
* ****************************************************************************** */
void flickerLed() {
  // Flickering LED
  for(int i=0; i<5; i++) {
    // Flash LED light
    digitalWrite(LED, HIGH);
    // wait for half a second
    delay(500);
    // turn the LED off by making the voltage LOW
    digitalWrite(LED, LOW);
    // wait for half a second
    delay(500);
  }
}

/****************************************************************************** *
* Name **************
* distanceToClosestObject()
*
* Description
* *************
* This function calculates the distance from the robot to the closest object in cm
*
* Parameters
* *************
* None
*
* Returns
* *************
* distance    long    distance to the closest object in cm
*
* ****************************************************************************** */
long distanceToClosestObject() {
  pinMode(SONAR, OUTPUT);
  digitalWrite(SONAR, LOW);
  delay(0.002);
  digitalWrite(SONAR, HIGH);
  delay(0.005);
  pinMode(SONAR, INPUT);
  
  long time = pulseIn(SONAR, HIGH);
  return (time / (29.0 * 2.0));
}

/****************************************************************************** *
* Name **************
* forward(double tiles)
*
* Description
* *************
* This function makes the robot  travel forward in a straight line
*
* Parameters
* *************
* number of tiles
* doingAvoidanceAlgorithm. A boolean indicating if the robot is currently
* doing the avoidance algorithm.
*
* Returns
* *************
* void
*
* ****************************************************************************** */
void forward(double tiles, boolean doingAvoidanceAlgorithm) {
  left.attach(LEFT_MOTOR);
  right.attach(RIGHT_MOTOR);
  
  // Display "Moving Forward"
  clear_LCD();
  lcd_position(0,5);
  LCD.print("Moving");
  lcd_position(1,5);
  LCD.print("Forward");

  // Start moving forward
  left.write(LEFT_FORWARD);
  right.write(RIGHT_FORWARD);
  
  double totalTransitions = tiles * transitionsForOneTile;
  rotateWheels(totalTransitions, doingAvoidanceAlgorithm);
}

/****************************************************************************** *
* Name **************
* backward(double tiles)
*
* Description
* *************
* This function makes the robot  travel backward in a straight line 
*
* Parameters
* *************
* number of tiles
* doingAvoidanceAlgorithm. A boolean indicating if the robot is currently
* doing the avoidance algorithm.
*
* Returns
* *************
* void
*
* ****************************************************************************** */
void backward(double tiles, boolean doingAvoidanceAlgorithm) {
  left.attach(LEFT_MOTOR);
  right.attach(RIGHT_MOTOR);
  
  // Display "Moving Forward"
  clear_LCD();
  lcd_position(0,5);
  LCD.print("Moving");
  lcd_position(1,4);
  LCD.print("Backward");

  // Start moving forward
  left.write(LEFT_BACKWARD);
  right.write(RIGHT_BACKWARD);
  
  double totalTransitions = tiles * transitionsForOneTile;
  rotateWheels(totalTransitions, doingAvoidanceAlgorithm);
}

/****************************************************************************** *
* Name **************
* rotateRight90()
*
* Description
* *************
* This function makes the robot rotate right for 90 degrees.
*
* Parameters
* *************
* doingAvoidanceAlgorithm. A boolean indicating if the robot is currently
* doing the avoidance algorithm.
*
* Returns
* *************
* void
*
* ****************************************************************************** */
void rotateRight90(boolean doingAvoidanceAlgorithm) {
  left.attach(LEFT_MOTOR);
  right.attach(RIGHT_MOTOR);
  
  // Display "Rotating Right"
  clear_LCD();
  lcd_position(0,4);
  LCD.print("Rotating");
  lcd_position(1,6);
  LCD.print("Right");

  // Start rotating right
  left.write(LEFT_FORWARD);
  right.write(RIGHT_BACKWARD);
  
  double totalTransitions = transitionsForRotate90;
  rotateWheels(totalTransitions, doingAvoidanceAlgorithm);
}

/****************************************************************************** *
* Name **************
* rotateLeft90()
*
* Description
* *************
* This function makes the robot rotate left for 90 degrees.
*
* Parameters
* *************
* doingAvoidanceAlgorithm. A boolean indicating if the robot is currently
* doing the avoidance algorithm.
*
* Returns
* *************
* void
*
* ****************************************************************************** */
void rotateLeft90(boolean doingAvoidanceAlgorithm) {
  left.attach(LEFT_MOTOR);
  right.attach(RIGHT_MOTOR);
  
  // Display "Rotating Left"
  clear_LCD();
  lcd_position(0,4);
  LCD.print("Rotating");
  lcd_position(1,6);
  LCD.print("Left");

  // Start rotating left
  left.write(LEFT_BACKWARD);
  right.write(RIGHT_FORWARD);
  
  double totalTransitions = transitionsForRotate90;
  rotateWheels(totalTransitions, doingAvoidanceAlgorithm);
}

/****************************************************************************** *
* Name **************
* rotateWheels(double totalTransitions)
*
* Description
* *************
* This function lets the wheels rotate for the number of transitions passed
* in the parameters.
*
* Parameters
* *************
* totalTransitions. The number of transitions that need to occur before
* wheel rotations are stopped.
* doingAvoidanceAlgorithm. A boolean indicating if the robot is currently
* doing the avoidance algorithm.
*
* Returns
* *************
* void
*
* ****************************************************************************** */
void rotateWheels(double totalTransitions, boolean doingAvoidanceAlgorithm) {
  
    int currentLeftPosition = digitalRead(LEFT_SENSOR);
    int currentRightPosition = digitalRead(RIGHT_SENSOR);
    
    int totalLeftTransitions = totalTransitions;
    int totalRightTransitions = totalTransitions; 
    
    if(doingAvoidanceAlgorithm) {
      while(totalLeftTransitions != 0 && totalRightTransitions != 0) {
          if(digitalRead(LEFT_SENSOR) != currentLeftPosition) {
              currentLeftPosition = digitalRead(LEFT_SENSOR);
              totalLeftTransitions--;
          }
          if(digitalRead(RIGHT_SENSOR) != currentRightPosition) {
              currentRightPosition = digitalRead(RIGHT_SENSOR);
              totalRightTransitions--;
          }
      }
      stop_all();
    } else {
      boolean detectedAnObstacle = false;
      while(totalLeftTransitions != 0 && totalRightTransitions != 0) {
          if(detect_obstacle()) {
            detectedAnObstacle = true;
            break;
          }
          if(digitalRead(LEFT_SENSOR) != currentLeftPosition) {
              currentLeftPosition = digitalRead(LEFT_SENSOR);
              totalLeftTransitions--;
          }
          if(digitalRead(RIGHT_SENSOR) != currentRightPosition) {
              currentRightPosition = digitalRead(RIGHT_SENSOR);
              totalRightTransitions--;
          }
      }
      stop_all();
      if(detectedAnObstacle)
        detection_algorithm();  
    }
}

/****************************************************************************** *
* Name **************
* detect_obstacle()
*
* Description
* *************
* This function returns true whenever the robot finds an obstacle and false otherwise.
*
* Parameters
* *************
* None
*
* Returns
* *************
* boolean
*
* ****************************************************************************** */
boolean detect_obstacle() {
  if(distanceToClosestObject() <= 10.0 && distanceToClosestObject() != 1.12) {
     return true;
 }
 return false;
}


/****************************************************************************** *
* Name **************
* detection_algorithm()
*
* Description
* *************
* This function performs the detection algorithm.
*
* Parameters
* *************
* None
*
* Returns
* *************
* void
*
* ****************************************************************************** */
void detection_algorithm() {
    printTemperature();
    backward(1.0, true);
    rotateRight90(true);
    forward(2.0, true);
    rotateLeft90(true);
}

/****************************************************************************** *
* Name **************
* stop_all()
*
* Description
* *************
* This function makes both wheels stop moving, by detaching left and right motors
* from the Servo library.
*
* Parameters
* *************
* None
*
* Returns
* *************
* void
*
* ****************************************************************************** */
void stop_all() {
  left.detach();
  right.detach();
}

/****************************************************************************** *
* Name **************
* lcd_position(int row, int col)
*
* Description
* *************
* This function positions the cursor of the LCD screen, so we can print at the
* appropriate place.
*
* Parameters
* *************
* Name      Type      In/Out      Description
*-------    ------    --------    -----------
* row       int       In          Row where cursor is to be placed (0 or 1)
* col       int       In          Column where cursor is to be placed (0 to 15)
*
* Returns
* *************
* void
*
* ****************************************************************************** */
void lcd_position(int row, int col) {
LCD.write(0xFE);   //command flag
LCD.write((col + row*64 + 128));    //position
delay(LCD_DELAY);
}

/****************************************************************************** *
* Name **************
* clear_LCD()
*
* Description
* *************
* This function clears the robot's LCD screen, by sending it a clear command.
*
* Parameters
* *************
* None
*
* Returns
* *************
* void
*
* ****************************************************************************** */
void clear_LCD(){
LCD.write(0xFE);   //command flag
LCD.write(0x01);   //clear command.
delay(LCD_DELAY);
}
