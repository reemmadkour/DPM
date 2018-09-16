package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 100;
  private static final int FILTER_OUT = 20;


  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl;

  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.filterControl = 0;

    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED+100);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {

    // rudimentary filter - toss out invalid samples corresponding to null
    // signal.
    // (n.b. this was not included in the Bang-bang controller, but easily
    // could have).
    //
	  //if (distance>2000) {}
	   if ((distance >= 60 ) && filterControl < FILTER_OUT) {
		   if (filterControl>7){
		   WallFollowingLab.leftMotor.setSpeed(40);// Start moving forward 
			WallFollowingLab.rightMotor.setSpeed(65); 
			WallFollowingLab.leftMotor.backward(); 
			WallFollowingLab.rightMotor.backward();}
		
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
    } 
     else {
    	 if((distance<=60)) { filterControl = 0;}
      // distance went below 255: reset filter and leave
      // distance alone.
     
      this.distance = distance;
    
	   //this.distance = distance;
	    int distError=bandCenter-distance;// Compute error 
	    /**if (distance>2000) {
	    WallFollowingLab.leftMotor.setSpeed(40);// Start moving forward 
		WallFollowingLab.rightMotor.setSpeed(75); 
		WallFollowingLab.leftMotor.backward(); 
		WallFollowingLab.rightMotor.backward();}**/
	    
	       
	     if (Math.abs(distError) <= bandWidth ) {// Within limits, same speed 
	    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);// Start moving forward 
	    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED+100); 
	    	WallFollowingLab.leftMotor.forward(); 
	    	WallFollowingLab.rightMotor.forward();}
	    else if (distError > 0) {// Too close to the wall 
	    	/**if (distance<9) {WallFollowingLab.leftMotor.setSpeed(30);// Start moving forward 
			WallFollowingLab.rightMotor.setSpeed(55); 
			WallFollowingLab.leftMotor.backward(); 
			WallFollowingLab.rightMotor.backward();}**/
	    	if (distError>15) {distError=15;}
	    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED+100-50); 
	    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED+7*Math.abs((distError))); 
	    	WallFollowingLab.rightMotor.forward(); 
	    	WallFollowingLab.leftMotor.forward();}
	    else if (distError < 0) { //too far
	    	if (Math.abs(distError)>10) {distError=10;}
	    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED+100+30); 
	    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED-5*(Math.abs(distError))); 
	    	WallFollowingLab.rightMotor.forward(); 
	    	WallFollowingLab.leftMotor.forward();} 
	    //Thread.sleep(50);//  Allow other threads to get CPU
	    }}    // TODO: process a movement based on the us distance passed in (P style)
  


  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
