package ca.mcgill.ecse211.localizationlab;
import lejos.hardware.sensor.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;


import lejos.hardware.Button;

/**creates a class to start the navigation
 * @author reem madkour and usaid barlas
 
 */


public class Navigation {

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	
	private final double TRACK;
	private final double WHEEL_RAD;
	public static final double TILE_SIZE=30.48;;
	public static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;

	double dx, dy, dt;

	int i = 0;
	public Odometer odometer;
	//private OdometerData odoData;

	
	/** creates an instance of the navigation class
	 * @param leftmotor, rightmotor: motors passed from the lab2 class
	 * @param TRACK is the distance between the wheels
	 * @param WHEEL_RAD is the radius of the wheel
	 *
	 */
		public Navigation(Odometer odometer,EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
				final double TRACK, final double WHEEL_RAD) { // constructor
			this.odometer = odometer;
			this.leftMotor = leftMotor;
			this.rightMotor = rightMotor;
			//odoData = OdometerData.getOdometerData();
			//odometer.setXYT(0 , 0 , 0);
			this.TRACK = TRACK;
			this.WHEEL_RAD = WHEEL_RAD;
			
		}


		private static double odoAngle = 0;
		/**
		 * @param x :  waypoint x coordinate
		 * @param y:   waypoint y coordinate
		 * @param obs:boolean for i we are avoiding an obstacle or not
		 */
		void travelTo(double x, double y,boolean obs) {
			double calcTheta = 0, len = 0, deltaX = 0, deltaY = 0;

	
			odoAngle = odometer.getXYT()[2];

			deltaX = x*TILE_SIZE- odometer.getXYT()[0];;
			deltaY = y*TILE_SIZE - odometer.getXYT()[1];
		
			len = Math.hypot(Math.abs(deltaX), Math.abs(deltaY));

			//get angle up to 180
			calcTheta = Math.toDegrees(Math.atan2(deltaX, deltaY));

			//if result is negative subtract it from 360 to get the positive
			if (calcTheta < 0)
				calcTheta = 360 - Math.abs(calcTheta);

			// turn to the found angle
			//turnTo(calcTheta);
		

			// go
			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);
			leftMotor.rotate(convertDistance(WHEEL_RAD, len), true);
			rightMotor.rotate(convertDistance(WHEEL_RAD, len), true);

			
		}
		
		/** changes heading of the robot to the given angle
		 * @param theta : angle needed to turn to
		 */
		void turnTo(double theta) {
			boolean turnLeft = false; //to do the minimal turn
			double deltaAngle = 0;
			// get the delta nagle
			odoAngle=odometer.getXYT()[2];
			deltaAngle = theta - odoAngle;

			// if the delta angle is negative find the equivalent positive
			if (deltaAngle < 0) {
				deltaAngle = 360 - Math.abs(deltaAngle);
			}

			// Check if angle is the minimal or not
			if (deltaAngle > 180) {
				turnLeft = true;
				deltaAngle = 360 - Math.abs(deltaAngle);
			} else {
				turnLeft = false;
			}

			// set to rotation speed
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);

			//turn robot to direction we chose
			if (turnLeft) {
				leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, deltaAngle), true);
				rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, deltaAngle), false);
			} else {
				leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, deltaAngle), true);
				rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, deltaAngle), false);
			}

		}

		boolean isNavigating() {
			if((leftMotor.isMoving() || rightMotor.isMoving()))
				return true;
			else 
				return false;

		}
		
		/**gives the angle you need  the wheels to roate by to cover a certain arc length (distance)
		 * @param radius: radius of wheel
		 * @param distance: distance you want the robot to travel 
		 *@return: angle for rotate
		 */
		public static int convertDistance(double radius, double distance) {
		    return (int) ((180.0 * distance) / (Math.PI * radius));
		  }

		  public static int convertAngle(double radius, double width, double angle) {
		    return convertDistance(radius, Math.PI * width * angle / 360.0);
		  }




  public void turnCW(long degree) {
    leftMotor.rotate(
        convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, degree), true);
    rightMotor.rotate(
        -convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, degree), true);
  }

  public void turnCCW(long degree) {
    leftMotor.rotate(
        -convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, degree), true);
    rightMotor.rotate(
        convertAngle(LocalizationLab.WHEEL_RADIUS, LocalizationLab.TRACK, degree), true);
  }

  public void advance(long distance) {
    leftMotor.rotate(convertDistance(LocalizationLab.WHEEL_RADIUS, distance), true);
    rightMotor.rotate(convertDistance(LocalizationLab.WHEEL_RADIUS, distance), false);
  }
}
