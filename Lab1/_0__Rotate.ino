/**************************************************************** *
* Names:		Nabil Maadarani 6134578
				Franck Mamboue  6175122
* Course Code:	SEG4145
* Lab Number:	1
* File name:	Rotate.ino
* Date:			January 22, 2015
*
* Description
* *************
* This script makes the robot rotate constantly in the same place
* *************************************************************** */

// Pin number constants
#define LEFT_MOTOR 2
#define RIGHT_MOTOR 4
#define LED 13
#define LCD 18

// Motor: direction constants
#define STOP 0
#define LEFT_BACKWARD 10
#define LEFT_FORWARD 191.5
#define RIGHT_BACKWARD 191.5
#define RIGHT_FORWARD 10

/****************************************************************************** *
* Name **************
* setup()
*
* Description
* *************
* This function simply sets up the wheels of the robot by indicating
* their pin numbers on the board.
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
}

/****************************************************************************** *
* Name **************
* loop()
*
* Description
* *************
* This is the program's main loop. In this case, it makes the left wheel rotate
* backwards, and the front wheel rotate forwards, causing the robot to rotate
* in the same place.
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
  // move left wheel backward
  left_backward();
  // move right wheel forward
  right_forward();
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
 analogWrite(LEFT_MOTOR, LEFT_BACKWARD);
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
  analogWrite(RIGHT_MOTOR, RIGHT_FORWARD);
}