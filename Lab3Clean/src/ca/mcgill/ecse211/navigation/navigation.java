package ca.mcgill.ecse211.navigation;
import lejos.hardware.sensor.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.*;

import lejos.hardware.Button;

public class navigation implements Runnable {

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private float[] usData;
	private SampleProvider usDistance ;
	private final double TRACK;
	private final double WHEEL_RAD;
	public static final double TILE_SIZE=30.48;;
	public static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	double currentT, currentY, currentX;
	double dx, dy, dt;
	double distanceToTravel;
	int i = 0;
	private Odometer odometer;
	private OdometerData odoData;

		//array list for points
		public navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
				final double TRACK, final double WHEEL_RAD) throws OdometerExceptions { // constructor
			this.odometer = Odometer.getOdometer();
			this.leftMotor = leftMotor;
			this.rightMotor = rightMotor;
			odoData = OdometerData.getOdometerData();
			odoData.setXYT(0 , 0 , 0);
			this.TRACK = TRACK;
			this.WHEEL_RAD = WHEEL_RAD;
			SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
			usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
			// this instance
			this.usData = new float[usDistance.sampleSize()]; // usData is the buffer in which data are
			// returned
		}

		// run method (required for Thread)
		public void run() {
			for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
				motor.stop();
				motor.setAcceleration(300);  // reduced the acceleration to make it smooth
			}
			// wait 5 seconds
			try {
				Thread.sleep(2000);
			} catch (InterruptedException e) {
				// there is nothing to be done here because it is not expected that
				// the odometer will be interrupted by another thread
			}
			// implemented this for loop so that navigation will work for any number of points
			
			int waypoints[][] = new int[][] { { 0, 2 }, { 1, 1 }, { 2, 2 }, { 2, 1 }, { 1, 0 } };
			int waypoints2[][] = new int[][] { { 1, 1 }, { 0, 2 }, { 2, 2 }, { 2, 1 }, { 1, 0 } };
			int waypoints3[][] = new int[][] { { 1, 0 }, { 2, 1 }, { 2, 2 }, { 0, 2 }, { 1, 1 } };
			int waypoints4[][] = new int[][] { { 0, 1 }, { 1, 2 }, { 1, 0 }, { 2, 1 }, { 2, 2 } };
			while(i<5) {
				travelTo(waypoints2[i][0], waypoints2[i][1],lab3.obstacle);
				i++;
			}
		}
		private static double prevAngle = 0;
		void travelTo(double x, double y,boolean obs) {
			double absAngle = 0, len = 0, deltaX = 0, deltaY = 0;

	
			prevAngle = odometer.getXYT()[2];

			deltaX = x*TILE_SIZE- odometer.getXYT()[0];;
			deltaY = y*TILE_SIZE - odometer.getXYT()[1];
			distanceToTravel = Math.sqrt(dx*dx+dy*dy);len = Math.hypot(Math.abs(deltaX), Math.abs(deltaY));

			// Get absolute angle the robot must be facing
			absAngle = Math.toDegrees(Math.atan2(deltaX, deltaY));

			// If the value of absolute angle is negative, loop it back
			if (absAngle < 0)
				absAngle = 360 - Math.abs(absAngle);

			// Make robot turn to the absolute angle
			turnTo(absAngle);
		

			// drive forward required distance
			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);
			leftMotor.rotate(convertDistance(WHEEL_RAD, len), true);
			rightMotor.rotate(convertDistance(WHEEL_RAD, len), true);

			while(isNavigating()) { //avoiding the obstacles
				usDistance.fetchSample(usData,0);
				float distance = usData[0]*100;
				if(distance<= 15) {
					
				
					if(odometer.getXYT()[0]<2.4*30.48&&odometer.getXYT()[0]>1.3*30.48&&odometer.getXYT()[1]<2.5*30.48&&odometer.getXYT()[1]>1.6*30.48){
						leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), true);  // turn when facing obstacle and travel a certain distance and then turn again 
						rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), false);// then travel a certain distance
						leftMotor.rotate(convertDistance(WHEEL_RAD, 40), true);
						rightMotor.rotate(convertDistance(WHEEL_RAD, 40), false);
						leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), true);
						rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), false);
						leftMotor.rotate(convertDistance(WHEEL_RAD, 45), true);
						rightMotor.rotate(convertDistance(WHEEL_RAD, 45), false);
					}
					else {
					leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), true);  // turn when facing obstacle and travel a certain distance and then turn again 
					rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), false);// then travel a certain distance
					leftMotor.rotate(convertDistance(WHEEL_RAD, 40), true);
					rightMotor.rotate(convertDistance(WHEEL_RAD, 40), false);
					leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), true);
					rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), false);
					leftMotor.rotate(convertDistance(WHEEL_RAD, 45), true);
					rightMotor.rotate(convertDistance(WHEEL_RAD, 45), false);
					}
					i--;
				}
			}
		}
		void turnTo(double theta) {
			boolean turnLeft = false;
			double deltaAngle = 0;
			// Get change in angle we want
			deltaAngle = theta - prevAngle;

			// If deltaAngle is negative, loop it back
			if (deltaAngle < 0) {
				deltaAngle = 360 - Math.abs(deltaAngle);
			}

			// Check if we want to move left or right
			if (deltaAngle > 180) {
				turnLeft = true;
				deltaAngle = 360 - Math.abs(deltaAngle);
			} else {
				turnLeft = false;
			}

			// Set slower rotate speed
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);

			// Turn motors according to which direction we want to turn in
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

		private static int convertDistance(double radius, double distance) {
			return (int) ((180.0 * distance) / (Math.PI * radius));
		}
		private static int convertAngle(double radius, double width, double angle) {
			return convertDistance(radius, Math.PI * width * angle / 360.0);
		}
}

