package test;

import java.io.*;
import java.net.*;

/**************************************************************** *
* Names:        Nabil Maadarani 6134578
*               Franck Mamboue  6175122
*               Christian Kabuya 6177914
* Course Code:  SEG4145
* Lab Number:   4
* File name:    UDPServer.java
* Date:         March 26, 2015
*
* Description
* *************
 * The main program automatically establishes a connection with the robot and performs
 * the operations described in the following menu options: 
	
 *	Enter the correct number to select an operation: 
 *	1 – Move the robot forward. 
 *	2 – Move the robot backward. 
 *	3 – Rotate the robot clockwise. 
 *	4 – Rotate the robot counter clockwise. 
 *	5 – Read the distance to the nearest object. 
 *	6 – Read temperature values. 
 *	7 – Quit. 
 
 *	An error message should be displayed if a connection cannot be established with the 
 *	robot and the program should immediately terminate. Selecting option 1 or 2 must 
 *	prompt the user to enter the distance for traversal in centimeters from 0 and 20 
 *	centimeters inclusive. Selecting options 3 or 4 must prompt the user to enter the rotation 
 *	value in degrees from 0 to 359 inclusive. Selecting option 6 must prompt the user to 
 *	return all temperature values that can be generated from the robot.
* *************************************************************** */
class UDPServer {
   public static void main(String args[]) {
       try {
    	   	DatagramSocket serverSocket = new DatagramSocket(9876);
            byte[] receiveData = new byte[10000];
            byte[] sendData = new byte[1024];
            byte[] paramData = new byte[1024];

            while(true) {
            	  // Receive client packet
				  DatagramPacket receivePacket = new DatagramPacket(receiveData, receiveData.length);
				  serverSocket.receive(receivePacket);
				  // Print menu
				  printMenu();
				  // Read user menu input (must be 1 to 7)
				  BufferedReader bufferReader = new BufferedReader(new InputStreamReader(System.in));
		    	  // Read requested menu option
		          String request = bufferReader.readLine();
		          // Send menu option to client
				  InetAddress IPAddress = receivePacket.getAddress();
				  int port = receivePacket.getPort();
				  sendData = request.getBytes();
				  DatagramPacket commandPacket = new DatagramPacket(sendData, sendData.length, IPAddress, port);
				  serverSocket.send(commandPacket);
				  // Read a function parameter if there is one
				  String param = null;
				  switch (request) {
					   case "1":
						   System.out.println("Enter the distance for traversal in centimeters from 0 and 20 centimeters inclusive:\n");
					       String distanceForward = bufferReader.readLine();
					       param = distanceForward + "]";
					       break;
					   case "2":
						   System.out.println("Enter the distance for traversal in centimeters from 0 and 20 centimeters inclusive:\n");
					       String distanceBackwards = bufferReader.readLine();
					       param = distanceBackwards + "]";
					       break;
					   case "3":
						   System.out.println("Enter the rotation value in degrees from 0 to 359 inclusive:\n");
					       String rotateClockwise = bufferReader.readLine();
					       param = rotateClockwise + "]";
					       break;
					   case "4":
						   System.out.println("Enter the rotation value in degrees from 0 to 359 inclusive:\n");
					       String rotateCounterClockwise = bufferReader.readLine();
					       param = rotateCounterClockwise + "]";
					       break;
					   case "6":
						   System.out.println("Enter the hex value of the temperature you are looking for (0x01 to 0x09):\n");
					       String temperatures = bufferReader.readLine();
					       param = temperatures;
					       break;
					   case "7":
						   System.out.println("Bye bye!");
						   System.exit(0);
				  }
				  // Send function parameter to client if there is one
				  if (param != null) {
					  paramData = param.getBytes();
					  DatagramPacket paramPacket = new DatagramPacket(paramData, paramData.length, IPAddress, port);
					  serverSocket.send(paramPacket);
				  }
            }
		} catch (IOException e) {
			// TODO Auto-generated catch block
			System.out.println("connection cannot be established with the robot.");
			return;
		}
	}
   
   public static void printMenu() {
	   System.out.println("Enter the correct number to select an operation: \n"
	   		+ "1 – Move the robot forward. \n"
			+ "2 – Move the robot backward. \n"
	   		+ "3 – Rotate the robot clockwise. \n"
	   		+ "4 – Rotate the robot counter clockwise. \n"
	   		+ "5 – Read the distance to the nearest object. \n"
	   		+ "6 – Read temperature values. \n"
	   		+ "7 – Quit.\n");
   }
   
}