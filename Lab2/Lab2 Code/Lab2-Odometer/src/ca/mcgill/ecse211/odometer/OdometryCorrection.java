/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;
import lejos.hardware.sensor.*;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.lcd.LCD;
import lejos.robotics.SampleProvider;

public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private Odometer odometer;
  private OdometerData odoData;
	private static final Port csPort = LocalEV3.get().getPort("S1");
	private float[] csData;
	private SampleProvider ColorID;
	private int countx=0;
	private int county=0;
	private static final double TILE = 30.48;
	
	
  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection() throws OdometerExceptions {

    this.odometer = Odometer.getOdometer();
    this.odoData = OdometerData.getOdometerData();
	
	//2.Sensor instance
	SensorModes ColorSensor = new EV3ColorSensor(csPort);
	
	
	ColorID = ColorSensor.getMode("Red");
	
	
	this.csData = new float[ColorID.sampleSize()];

  }

  /**
   * Here is where the odometer correction code should be run.
   * 
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;
    int totalLines = 0, yLines = 0, xLines = 0;

	// Store previously stored values
	double origX = -TILE / 2, origY = -TILE / 2, origTh = 0;
    while (true) {
    	correctionStart = System.currentTimeMillis();
		ColorID.fetchSample(csData, 0);
		float intensity = csData[0]; //last value sent by sensor
      if (intensity<0.3) {
    	  double measurements[] = null;
			try {
				measurements = Odometer.getOdometer().getXYT();
			} catch (OdometerExceptions e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}

			// Beep when black line passed
			Sound.beep();

			// TODO Calculate new (accurate) robot position
			// Check if robot has started moving (to avoid false color sensor positives)

			// Increment when line passed
			totalLines++;
			String print = "Lines passed: " + totalLines;
			LCD.drawString(print, 0, 4);
		

			// If moving up Y axis
			if (measurements[2] > 345 || measurements[2] < 15) {
				// Increment number of lines perpendicular to Y-axis (lines crossed when going across Y-axis)
				yLines++;
				origTh = 0;
				// If first line, set Y to 0
				if (totalLines == 1) {
					origY = 0;
				// Else increment tile size
				} else {
					origY += TILE;
				}
				// update only y on odometer
				odometer.setXYT(measurements[0], origY, origTh);
			
			// If moving up X axis
			} else if (measurements[2] > 75 && measurements[2] < 105) {
				// Increment nummber of lines perpendicular to X-axis (lines crossed when going across X-axis)
				xLines++;
				origTh = 90;
				// If first line of X-axis passed set X to 0
				if (measurements[0] < 15) {
					origX = 0;
				// Otherwise increment tile size
				} else {
					origX += TILE;
				}
				// Update odometer for just X
				odometer.setXYT(origX, measurements[1], origTh);

				
				
				
				
			// If moving down Y axis
			} else if (measurements[2] > 165 && measurements[2] < 195) {
				origTh = 180;
				// If first line down Y-axis, do nothing (set the origY as the Y coordinate)
				if (measurements[1] > origY) {

				// If not first line, decrement tile size
				} else if (yLines > 1) {
					origY -= TILE;
				
				} else {
				// If last line, set to zero
					origY = 0;
				}
				yLines--;
				odometer.setXYT(measurements[0], origY, origTh);
			
			// If moving down X axis
			} else if (measurements[2] > 255 && measurements[2] < 285) {
				origTh = 270;
				
				// If first line down X-axis, do nothing (set the origX as the X-coordinate)
				if (measurements[0] >= origX) {
					//xLines--;
				// If not first line, decrement tile size
				} else if (xLines > 1) {
					origX -= TILE;
				// If last line, set to zero
				} else {
					origX = 0;
				}
				xLines--;
				odometer.setXYT(origX, measurements[1], origTh);
			}
		

      }

      // TODO Trigger correction (When do I have information to correct?)
      // TODO Calculate new (accurate) robot position

      // TODO Update odometer with new calculated (and more accurate) vales


      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
  }
}
