/**************************************************************** *
* Names:        Nabil Maadarani 6134578
                Franck Mamboue  6175122
* Course Code:  SEG4145
* Lab Number:   1
* File name:    Path2.ino
* Date:         January 22, 2015
*
* Description
* *************
* This script makes the robot display our student numbers for 5 seconds
* while flickering the LED, display "Path 2" for 3 seconds, then perform
* path 2 described in the lab manual while displaying its current
* movement (i.e. moving forward).
* *************************************************************** */

#include <SoftwareSerial.h>

// Pin number constants
#define LEFT_MOTOR 2
#define RIGHT_MOTOR 4
#define LED 13
#define LCD_PIN 18
#define LCD_DELAY 10

// Motor: direction constants
#define STOP 0
#define LEFT_BACKWARD 10
#define LEFT_FORWARD 189
#define RIGHT_BACKWARD 191.5
#define RIGHT_FORWARD 10

// LCD instance
SoftwareSerial LCD(0, LCD_PIN);

/****************************************************************************** *
* Name **************
* setup()
*
* Description
* *************
* This function simply sets up the wheels, LED, and LCD of the robot by indicating
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
  // Initialize all pins.
  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(LCD_PIN, OUTPUT);
  LCD.begin(9600);
}

/****************************************************************************** *
* Name **************
* loop()
*
* Description
* *************
* This is the program's main loop. In this case, it performs the steps necessary
* for path 1 described at the top of this document. It also adds a 60 second
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
  // wait for a second
  delay(500);
  // turn the LED off by making the voltage LOW
  digitalWrite(LED, LOW);
  // wait for a second
  delay(500);
  }

  // Diplay path name
  clear_LCD();
  lcd_position(0,5);
  LCD.print("Path 2");
  delay(3000);
  
  // Move forward for 5 seconds
  clear_LCD();
  lcd_position(0,5);
  LCD.print("Moving");
  lcd_position(1,5);
  LCD.print("Forward");
  left_forward();
  right_forward();
  delay(5000);
 
  // Turn 90 degrees to the right
  clear_LCD();
  lcd_position(0,4);
  LCD.print("Rotating");
  lcd_position(1,6);
  LCD.print("Right");
  left_forward();
  right_backward();
  delay(1125);
  
  // Move forward for 5 seconds
  clear_LCD();
  lcd_position(0,5);
  LCD.print("Moving");
  lcd_position(1,5);
  LCD.print("Forward");
  left_forward();
  right_forward();
  delay(5000);
  
  // Turn 135 degrees to the left
  clear_LCD();
  lcd_position(0,4);
  LCD.print("Rotating");
  lcd_position(1,6);
  LCD.print("Left");
  left_backward();
  right_forward();
  delay(1687.5);
  
  // Move forward for 5 seconds
  clear_LCD();
  lcd_position(0,5);
  LCD.print("Moving");
  lcd_position(1,5);
  LCD.print("Forward");
  left_forward();
  right_forward();
  delay(5000);
  
//  // Turn 135 degrees to the right
  clear_LCD();
  lcd_position(0,4);
  LCD.print("Rotating");
  lcd_position(1,6);
  LCD.print("Right");
  left_forward();
  right_backward();
  delay(1687.5);
  
  // Move forward for 5 seconds
  clear_LCD();
  lcd_position(0,5);
  LCD.print("Moving");
  lcd_position(1,5);
  LCD.print("Forward");
  left_forward();
  right_forward();
  delay(5000);
  
  // Turn 90 degrees to the left
  clear_LCD();
  lcd_position(0,4);
  LCD.print("Rotating");
  lcd_position(1,6);
  LCD.print("Left");
  left_backward();
  right_forward();
  delay(1125);
  
  // Move forward for 5 seconds
  clear_LCD();
  lcd_position(0,5);
  LCD.print("Moving");
  lcd_position(1,5);
  LCD.print("Forward");
  left_forward();
  right_forward();
  delay(5000);
  
  // Stop
  clear_LCD();
  lcd_position(0,5);
  LCD.print("Stopped");
  stop_all();
  
  delay(60000);
}

/****************************************************************************** *
* Name **************
* left_forward()
*
* Description
* *************
* This function makes the left wheel rotate forwards, by calling analogWrite on
* it with a constant of 189.
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
void left_forward() {
  analogWrite (LEFT_MOTOR, LEFT_FORWARD);
}

/****************************************************************************** *
* Name **************
* left_backward()
*
* Description
* *************
* This function makes the left wheel rotate backwards, by calling analogWrite on
* it with a constant of 10.
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
void left_backward() {
  analogWrite (LEFT_MOTOR, LEFT_BACKWARD);
}

/****************************************************************************** *
* Name **************
* right_forward()
*
* Description
* *************
* This function makes the right wheel rotate forwards, by calling analogWrite on
* it with a constant of 10.
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
void right_forward() {
  analogWrite (RIGHT_MOTOR, RIGHT_FORWARD);
}

/****************************************************************************** *
* Name **************
* right_backward()
*
* Description
* *************
* This function makes the right wheel rotate backwards, by calling analogWrite on
* it with a constant of 191.5.
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
void right_backward() {
  analogWrite (RIGHT_MOTOR, RIGHT_BACKWARD);
}

/****************************************************************************** *
* Name **************
* stop_all()
*
* Description
* *************
* This function makes both wheels stop moving, by calling analogWrite on both 
* with a constant of 0.
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
  analogWrite (LEFT_MOTOR, STOP);
  analogWrite (RIGHT_MOTOR, STOP);
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