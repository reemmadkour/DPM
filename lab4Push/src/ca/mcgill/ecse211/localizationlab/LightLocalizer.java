package ca.mcgill.ecse211.localizationlab;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;

import java.util.ArrayList;

public class LightLocalizer {
  // constant needed for the lab
  private static final int ROTATE_SPEED = 100;
  private static final long LOOP_TIME = 10;
  private static final double D = 4;
  // variables needed for the lab
  ArrayList<Double> lastNValue = new ArrayList<>();
  ArrayList<Double> xyPoints = new ArrayList<>();
  private Odometer odometer;
  private EV3ColorSensor colorSensor;
  private float[] colorData;
  private Navigation navigation;
  private EV3LargeRegulatedMotor leftMotor, rightMotor;
  private long correctionStart, correctionEnd, startTime;
  private double currentColor, lastColor, dCdt;
  private double yp, yn, xp, xn = 0;
  private boolean firstpass = true;

  public LightLocalizer(
      Odometer odometer,
      EV3ColorSensor colorSensor,
      float[] colorData,
      Navigation navigation,
      EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor) {
    this.odometer = odometer;
    this.colorSensor = colorSensor;
    this.colorData = colorData;
    this.navigation = navigation;
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    resetMotor();
  }

  /** main method which turns to catch all the lines to determine offset from origin */
  public void localize() {
    // method to get closer to origin to catch all lines75
    gotoEstimatedOrigin();
    startTime = System.currentTimeMillis();
    navigation.turnCW(375);
    while (odometer.getXYT()[2] < 375) {
      currentColor = getFilteredData();
      if (firstpass) {
        lastColor = currentColor;
        firstpass = false;
      }
      // rather than check the value of light captured, we are checking the instaneous
      // differentiation
      dCdt = currentColor - lastColor;
      lastColor = currentColor;

      // add the differentiation of the color to the array
      lastNValueAdd(dCdt);
      if (pastline()) {
        xyPoints.add(odometer.getXYT()[2]);
        if (xyPoints.size() == 4) {
          break;
        }
      }
    }

    // get the angles and set them to get theta x and theta y
    yp = xyPoints.get(0);
    xp = xyPoints.get(1);
    yn = xyPoints.get(2);
    xn = xyPoints.get(3);

    double xOffset = -D * Math.cos((yn - yp) / 2);
    double yOffset = -D * Math.abs(Math.cos((xn - xp) / 2));

    // corrrect the odometer
    odometer.setX(xOffset);
    odometer.setY(yOffset);

    // this makes sure it travels to the true origin
   navigation.turnTo(0);
    navigation.travelTo(0, 0,false);
   // navigation.turnTo(0 /*1.44 * LocalizationLab.TAU / 360*/);
  }

  /** method to get the light value from the light sensor */
  private float getFilteredData() {
    correctionStart = System.currentTimeMillis();

    colorSensor.getRedMode().fetchSample(colorData, 0);
    float colorValue = colorData[0] * 100;

    // the correctionstart and corredtionend are to make sure that a value is taken once every
    // LOOP_TIME
    correctionEnd = System.currentTimeMillis();
    if (correctionEnd - correctionStart < LOOP_TIME) {
      try {
        Thread.sleep(LOOP_TIME - (correctionEnd - correctionStart));
      } catch (InterruptedException e) {
      }
    }

    return colorValue;
  }

  /** this method indicates whether a line has actually been passed over */
  public boolean pastline() {
    double biggest = -100;
    double smallest = 100;

    // since crossing a line causes a drop and a rise in the derivative, the filter
    // only considers a line crossed if the biggest value is higher than some threshold
    for (int i = 0; i < this.lastNValue.size(); i++) {
      // and the reverse for the lowest value, thus creating one beep per line
      if (this.lastNValue.get(i) > biggest) {
        biggest = this.lastNValue.get(i);
      }
      if (this.lastNValue.get(i) < smallest) {
        smallest = this.lastNValue.get(i);
      }
    }

    // if a sample is considered to be a line, the array is cleared as to not retrigger an other
    // time
    if (biggest > 5 && smallest < -3) {
      this.lastNValue.clear();
      Sound.setVolume(30);
      Sound.beep();
      return true;
    } else {
      return false;
    }
  }

  /** method adds in values to the array with logic */
  public void lastNValueAdd(double value) {
    // the array doesnt take chucks of the values and risk to miss a big difference.
    // Instead we slide along
    if (this.lastNValue.size() > 40) {
      // the oldest value is removed to make space

      this.lastNValue.remove(0);
      this.lastNValue.add(value);
    } else {
      // if the array happens to be less that, simply add them

      this.lastNValue.add(value);
    }
  }

  /**
   * method orientes towards the 45, advances a bit, turn back to the 0 degree to have a better
   * chance of catching all the lines
   */
  public void gotoEstimatedOrigin() {
    navigation.turnTo(LocalizationLab.TAU / 8);
    navigation.advance(17);
    navigation.turnTo(0);
  }

  /** method resets the motors */
  private void resetMotor() {
    leftMotor.stop(true);
    rightMotor.stop(true);
    // leftMotor.rotate(0);
    // rightMotor.rotate(0);
    for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
      motor.setAcceleration(3000);
      motor.setSpeed(ROTATE_SPEED);
    }
  }
}
