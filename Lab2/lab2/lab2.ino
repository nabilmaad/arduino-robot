/**************************************************************** *
* Names:        Nabil Maadarani 6134578
*               Franck Mamboue  6175122
* Course Code:  SEG4145
* Lab Number:   2
* File name:    lab2.ino
* Date:         February 26, 2015
*
* Description
* *************
* This script makes the robot display our student numbers for 5 seconds
* while flickering the LEDs, then perform path described in the lab
* manual while displaying its current movement (i.e. moving forward).
* *************************************************************** */

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
#define WHEEL_DISTANCE 18.5

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
  // Display student numbers
  clear_LCD();
  lcd_position(0,5);
  LCD.print("6134578");
  lcd_position(1,5);
  LCD.print("6175122");

  // Flickering LED
  for(int i=0; i<5; i++){
  // Flash LED light
  digitalWrite(LED, HIGH);
  // wait for half a second
  delay(500);
  // turn the LED off by making the voltage LOW
  digitalWrite(LED, LOW);
  // wait for half a second
  delay(500);
  }
  
  // Move forward for 2 tiles
  forward(2.0);
  
  // Rotate 90 degrees right
  rotateRight90();
  
  // Move forward for 2 tiles
  forward(2.0);
  
  // Rotate 90 degrees right
  rotateRight90();
  
  // Move forward for 3 tiles
  forward(3.0);
  
  // Rotate 90 degrees right
  rotateRight90();
  
  // Move forward for 3 tiles
  forward(3.0);
  
  // Rotate 90 degrees right
  rotateRight90();
  
  // Move forward for 2 tiles
  forward(2.0);
  
  // Rotate 45 degrees left
  rotateLeft45();
  
  // Move forward for 1 diagonal tile
  forward(sqrt(2));
  
  // Rotate 135 degrees left
  rotateLeft135();
  
  // Move forward for 4 tiles
  forward(4.0);
  
  // Rotate 90 degrees left
  rotateLeft90();
  
  // Move forward for 2 tiles
  forward(2.0);
  
  // Rotate 90 degrees left
  rotateLeft90();
  
  // Move forward for 2 tiles
  forward(2.0);
    
  stop_all();
  delay(60000);
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
  double wheelPerimeter = PI * WHEEL_DIAMETER;
  double distanceTraversedForEachTransition = wheelPerimeter/64;
  
  double robotCirclePerimeter = PI * WHEEL_DISTANCE;
  transitionsForRotate90 = (robotCirclePerimeter / distanceTraversedForEachTransition) / 4;
}

/****************************************************************************** *
* Name **************
* forward(double tiles)
*
* Description
* *************
* This function makes the robot move forward for a distance defined by the
* number if tiles passed as a parameter.
*
* Parameters
* *************
* tiles. The number of tiles to traverse.
*
* Returns
* *************
* void
*
* ****************************************************************************** */
void forward(double tiles) {
  // Attach wheels
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
  
  // Rotate wheels for the right number of transitions
  double totalTransitions = tiles * transitionsForOneTile;
  rotateWheels(totalTransitions);
  
  // Stop wheels
  stop_all();
  delay(10);
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
* None
*
* Returns
* *************
* void
*
* ****************************************************************************** */
void rotateRight90() {
  // Attach wheels
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
  
  // Rotate wheels for the right number of transitions
  double totalTransitions = transitionsForRotate90;
  rotateWheels(totalTransitions);

  // Stop wheels
  stop_all();
  delay(10);
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
* None
*
* Returns
* *************
* void
*
* ****************************************************************************** */
void rotateLeft90() {
  // Attach wheels
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
  
  // Rotate wheels for the right number of transitions
  double totalTransitions = transitionsForRotate90;
  rotateWheels(totalTransitions);

  // Stop wheels
  stop_all();
  delay(10);
}

/****************************************************************************** *
* Name **************
* rotateLeft45()
*
* Description
* *************
* This function makes the robot rotate left for 45 degrees.
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
void rotateLeft45() {
  // Attach wheels
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

  // Rotate wheels for the right number of transitions
  double totalTransitions = transitionsForRotate90 / 2;
  rotateWheels(totalTransitions);

  // Stop wheels
  stop_all();
  delay(10);
}

/****************************************************************************** *
* Name **************
* rotateLeft135()
*
* Description
* *************
* This function makes the robot rotate left for 135 degrees.
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
void rotateLeft135() {
  // Attach wheels
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

  // Rotate wheels for the right number of transitions
  double totalTransitions = 3 * transitionsForRotate90 / 2;
  rotateWheels(totalTransitions);

  // Stop wheels
  stop_all();
  delay(10);
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
*
* Returns
* *************
* void
*
* ****************************************************************************** */
void rotateWheels(double totalTransitions) {
    // Get the current value from the sensors
    int currentLeftPosition = digitalRead(LEFT_SENSOR);
    int currentRightPosition = digitalRead(RIGHT_SENSOR);
    
    // Total number of transitions needed for each wheels
    int totalLeftTransitions = totalTransitions;
    int totalRightTransitions = totalTransitions; 
    
    while(totalLeftTransitions != 0 && totalRightTransitions != 0) {
        // Left counter
        if(digitalRead(LEFT_SENSOR) != currentLeftPosition) {
            currentLeftPosition = digitalRead(LEFT_SENSOR);
            totalLeftTransitions--;
        }
        // Right counter
        if(digitalRead(RIGHT_SENSOR) != currentRightPosition) {
            currentRightPosition = digitalRead(RIGHT_SENSOR);
            totalRightTransitions--;
        }
    }
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
