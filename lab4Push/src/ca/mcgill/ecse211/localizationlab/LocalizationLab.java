// Lab4.java

package ca.mcgill.ecse211.localizationlab;


import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class LocalizationLab {

  // Motor and sensor instantiation
  private static final Port usPort = LocalEV3.get().getPort("S1");
  private static final EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

  // Constants which are used in this lab
  
  public static final double WHEEL_RADIUS = 2.07;
  public static final double TRACK = 11.3;
  public static final double TAU = 360;

  public static void main(String[] args) {
    int buttonChoice;

    @SuppressWarnings("resource")
    SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
    SampleProvider usDistance =
        usSensor.getMode("Distance"); // usDistance provides samples from this instance
    float[] usData =
        new float[usDistance.sampleSize()]; // usData is the buffer in which data are returned

  

    // creates a buffer for the value returned from the sensor
    float[] colorData = new float[colorSensor.getRedMode().sampleSize()];

    TextLCD t = LocalEV3.get().getTextLCD();

    // drawing the options of edge detection
    do {
      // clear the display
      t.clear();

      // ask the user whether the motors should drive in a square or float
      t.drawString("< Left | Right >", 0, 0);
      t.drawString("       |        ", 0, 1);
      t.drawString("Rising | Falling", 0, 2);
      t.drawString(" edge  |   edge ", 0, 3);
      t.drawString("       |        ", 0, 4);

      buttonChoice = Button.waitForAnyPress();
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

    // clearing the display and initializing the classes needed
    t.clear();
    Odometer odometer;
	
		odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RADIUS);

    try {
		OdometryDisplay odometryDisplay = new OdometryDisplay(t);
	} catch (OdometerExceptions e) {
		// TODO Auto-generated catch block
		e.printStackTrace();
	}
	Thread odoThread = new Thread(odometer);
    odoThread.start();
    Navigation navigation =
        new Navigation(
            odometer, leftMotor, rightMotor,  LocalizationLab.TRACK,LocalizationLab.WHEEL_RADIUS);

    // initialize usl depending on which edge chosen
    UltrasonicLocalizer usl;
    if (buttonChoice == Button.ID_LEFT) {
    	//Thread odoThread = new Thread(odometer);
        //odoThread.start();
      usl =
          new UltrasonicLocalizer(
              odometer,
              usSensor,
              usData,
              UltrasonicLocalizer.LocalizationType.RISING_EDGE,
              navigation,
              leftMotor,
              rightMotor, t);
    } else {
    	//Thread odoThread = new Thread(odometer);
        //odoThread.start();
      usl =
          new UltrasonicLocalizer(
              odometer,
              usSensor,
              usData,
              UltrasonicLocalizer.LocalizationType.FALLING_EDGE,
              navigation,
              leftMotor,
              rightMotor, t);
    }
    usl.localize();

    // waiting for a button press before doing the light localization
    while (Button.waitForAnyPress() != Button.ID_ENTER) ;

    // initializing the light localizer
    LightLocalizer ll =
        new LightLocalizer(odometer, colorSensor, colorData, navigation, leftMotor, rightMotor);
    ll.localize();

    // waiting for a button press before leaving to check the odometer values
    while (Button.waitForAnyPress() != Button.ID_ENTER) ;
    System.exit(0);
  }
}
