// Lab2.java
package ca.mcgill.ecse211.navigation;


import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class lab3 {
public static boolean obstacle= false; //initializing the boolean reflecting if the user is running with an obstacle or not
  // initializing mototrs and dimensions
  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  public static final double WHEEL_RAD = 2.13;
  public static final double TRACK = 12;

  public static void main(String[] args) throws OdometerExceptions {
 
    int buttonChoice; //initialize button variable.

    // initialize odometer objects 
    Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); 
    //ObstacleAvoidance obstacleAvoidance = new ObstacleAvoidance(); 
    Display odometryDisplay = new Display(lcd); 
    navigation navigation = new navigation(leftMotor,rightMotor,TRACK,WHEEL_RAD);
   
    do {
        
    

     
          lcd.clear();

          // obstacle or no obstacle 
          lcd.drawString("< Left  | Right >", 0, 0);
          lcd.drawString("  No    | with   ", 0, 1);
          lcd.drawString("  obs - | obs-   ", 0, 2);
          lcd.drawString("  tacle | tacle  ", 0, 3);
          lcd.drawString("        |        ", 0, 4);
          
          buttonChoice = Button.waitForAnyPress();
    }while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
       // Start threads
          Thread odoThread = new Thread(odometer);
          odoThread.start();
          Thread odoDisplayThread = new Thread(odometryDisplay);
          odoDisplayThread.start();
          
          if(buttonChoice == Button.ID_LEFT){
        	 navigation.run();
          }
          
          if(buttonChoice == Button.ID_RIGHT){
        	  lab3.obstacle=true;
        	 navigation.run();
      
          }
        

        while (Button.waitForAnyPress() != Button.ID_ESCAPE);
        System.exit(0);
      }
    }