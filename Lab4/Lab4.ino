/**************************************************************** *
* Names:        Nabil Maadarani 6134578
*               Franck Mamboue  6175122
*               Christian Kabuya 6177914
* Course Code:  SEG4145
* Lab Number:   4
* File name:    lab4.ino
* Date:         March 26, 2015
*
* Description
* *************
* This script establishes a WiFi connection to an existing WiFi network (in the lab) before data can be
* read and processed. It uses the User Datagram Protocol.
*
* *************************************************************** */

#include <Wirefree.h>
#include <WifiClient.h>
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

WIFI_PROFILE wireless_prof = {
                        /* SSID */ "Robolab",
         /* WPA/WPA2 passphrase */ "w1r3l3ss!",
                  /* IP address */ "10.136.160.28",
                 /* subnet mask */ "255.255.255.0",
                  /* Gateway IP */ "10.136.160.1", };

String remote_server = "137.122.47.201"; // peer device IP address
String remote_port = "9876";

// Initialize client with IP address and port number
WifiClient client(remote_server, remote_port, PROTO_UDP);

/****************************************************************************** *
* Name **************
* setup()
*
* Description
* *************
* This function tries to get a WIFI connection. If it fails, it prints a failure
* message. Otherwise, it prints a success message and sends a message over UDP
* socket to the peer device configured earlier.
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
void setup()
{
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
  
  // connect to AP
  Wireless.begin(&wireless_prof);
  
  // if you get a connection, report back via serial:
  if (client.connect()) {
    Serial.println("connected");
    
    // Send message over UDP socket to peer device
    client.println("Hello server!");
  } 
  else {
    // if connection setup failed:
    Serial.println("failed");
  }
}

/****************************************************************************** *
* Name **************
* loop()
*
* Description
* *************
* This is the program's main loop. In this case, it reads incoming data from the
* peer device configured earlier, and processes it.
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
void loop()
{
  // if there are incoming bytes available 
  // from the peer device, read them and print them:
  while (client.available()) {
    int in;

    while ((in = client.read()) == -1);
    
    // Send appropriate command to robot
    if((char)in == '1') {
        char distance[3];
        int i = 0;
        while (client.available()) {
          int param;
          while ((param = client.read()) == -1);
          if((char)param == ']') {
            distance[i] = '\0';
            forward((int) strtol(distance, NULL, 10));
            client.println("Done");
            break;
          } else {
            distance[i] = (char)param;
            i++;
          }
        }
    } else if((char)in == '2') {
        char distance[3];
        int i = 0;
        while (client.available()) {
          int param;
          while ((param = client.read()) == -1);
          if((char)param == ']') {
            distance[i] = '\0';
            backward((int) strtol(distance, NULL, 10));
            client.println("Done");
            break;
          } else {
            distance[i] = (char)param;
            i++;
          }
        }
    } else if((char)in == '3') {
        char angle[4];
        int i = 0;
        while (client.available()) {
          int param;
          while ((param = client.read()) == -1);
          if((char)param == ']') {
            angle[i] = '\0';
            rotateRight((int) strtol(angle, NULL, 10));
            client.println("Done");
            break;
          } else {
            angle[i] = (char)param;
            i++;
          }
        }
    } else if((char)in == '4') {
        char angle[4];
        int i = 0;
        while (client.available()) {
          int param;
          while ((param = client.read()) == -1);
          if((char)param == ']') {
            angle[i] = '\0';
            rotateLeft((int) strtol(angle, NULL, 10));
            client.println("Done");
            break;
          } else {
            angle[i] = (char)param;
            i++;
          }
        }
    }  else if((char)in == '5') {
        printDistanceToClosestObject();
        client.println("Done");
    } else if((char)in == '6') {
        char temperature[5];
        int i = 0;
        while (client.available()) {
          int param;
          while ((param = client.read()) == -1);
          Serial.println(i);
          if(i == 3) {
            temperature[i] = param;
            temperature[i+1] = '\0';
            printTemperature((int)temperature);
            client.println("Done");
            break;
          } else {
            temperature[i] = (char)param;
            i++;
          }
        }
      }
  }

  delay(1);
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
* printTemperature(int tempValue)
*
* Description
* *************
* This function prints the temperature on the robot's LCD screen depending on the
* value specified in the parameters.
*
* Parameters
* *************
* tempValue. An integer indicating the robot which temperature value to read
*
* Returns
* *************
* void
*
* ****************************************************************************** */
void printTemperature(int tempValue) {
  Wire.beginTransmission(TEMPSENSOR); 
  Wire.write(tempValue); // Indicate temperature value to read 
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
* void
*
* ****************************************************************************** */
void printDistanceToClosestObject() {
  pinMode(SONAR, OUTPUT);
  digitalWrite(SONAR, LOW);
  delay(0.002);
  digitalWrite(SONAR, HIGH);
  delay(0.005);
  pinMode(SONAR, INPUT);
  
  long time = pulseIn(SONAR, HIGH);
  long distance = (time / (29.0 * 2.0));
  
  char sentence[6];
  char distanceStr[3];
  sprintf(distanceStr, "%ld", distance);
  strcpy(sentence, distanceStr);
  strcat(sentence, " cm");
  
  // Display distance
  clear_LCD();
  lcd_position(0,1);
  LCD.print("Closest object");
  lcd_position(1,5);
  LCD.print(sentence);
}

/****************************************************************************** *
* Name **************
* forward(int distance)
*
* Description
* *************
* This function makes the robot travel forward for the amount of centimeters specified
* in the distance parameter.
*
* Parameters
* *************
* distance. An integer indicating the amount of centimeters the robot has to travel.
*
* Returns
* *************
* void
*
* ****************************************************************************** */
void forward(int distance) {
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
  
  double totalTransitions = (distance * transitionsForOneTile) / TILE_LENGTH;
  rotateWheels(totalTransitions);
  stop_all();
}

/****************************************************************************** *
* Name **************
* backward(int distance)
*
* Description
* *************
* This function makes the robot travel backward for the amount of centimeters specified
* in the distance parameter.
*
* Parameters
* *************
* distance. An integer indicating the amount of centimeters the robot has to travel.
*
* Returns
* *************
* void
*
* ****************************************************************************** */
void backward(int distance) {
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
  
  double totalTransitions = (distance * transitionsForOneTile) / TILE_LENGTH;
  rotateWheels(totalTransitions);
  stop_all();
}

/****************************************************************************** *
* Name **************
* rotateRight(int angle)
*
* Description
* *************
* This function makes the robot rotate right (clockwise) for the amount of degrees specified in the parameters.
*
* Parameters
* *************
* angle. An integer indicating the number of degrees the robot has to rotate.
*
* Returns
* *************
* void
*
* ****************************************************************************** */
void rotateRight(int angle) {
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
  
  double totalTransitions = angle * (transitionsForRotate90 / 90);
  rotateWheels(totalTransitions);
  stop_all();
}

/****************************************************************************** *
* Name **************
* rotateLeft(int angle)
*
* Description
* *************
* This function makes the robot rotate left (counter clockwise) for the amount of degrees specified in the parameters.
*
* Parameters
* *************
* angle. An integer indicating the number of degrees the robot has to rotate.
*
* Returns
* *************
* void
*
* ****************************************************************************** */
void rotateLeft(int angle) {
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
  
  double totalTransitions = angle * (transitionsForRotate90 / 90);
  rotateWheels(totalTransitions);
  stop_all();
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
  
  // Display "Stopped"
  clear_LCD();
  lcd_position(0,4);
  LCD.print("Stopped");
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
