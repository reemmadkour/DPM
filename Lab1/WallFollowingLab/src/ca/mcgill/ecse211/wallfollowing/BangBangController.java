package ca.mcgill.ecse211.wallfollowing;

public class BangBangController implements UltrasonicController {

  private final int bandCenter; 
  private final int bandWidth;
  private final int motorLow;
  private final int motorHigh;
  private int distance;

  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(motorHigh);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  public void processUSData(int distance) {
	  
	  
    this.distance = distance;
    int distError=bandCenter-distance;// Compute error 
    //if (distance>2000){}
    if (distance>2000) {
    WallFollowingLab.leftMotor.setSpeed(40);// Start moving forward 
	WallFollowingLab.rightMotor.setSpeed(75); 
	WallFollowingLab.leftMotor.backward(); 
	WallFollowingLab.rightMotor.backward();}
    
       
    else if (Math.abs(distError) <= bandWidth ) {// Within limits, same speed 
    	WallFollowingLab.leftMotor.setSpeed(motorHigh);// Start moving forward 
    	WallFollowingLab.rightMotor.setSpeed(motorHigh+100); 
    	WallFollowingLab.leftMotor.forward(); 
    	WallFollowingLab.rightMotor.forward();}
    else if (distError > 0) {// Too close to the wall 
    		/**if (distance<5) {WallFollowingLab.leftMotor.setSpeed(40);// Start moving forward 
    		WallFollowingLab.rightMotor.setSpeed(75); 
    		WallFollowingLab.leftMotor.backward(); 
    		WallFollowingLab.rightMotor.backward();}**/
    	WallFollowingLab.rightMotor.setSpeed(motorHigh+100-50); 
    	WallFollowingLab.leftMotor.setSpeed(motorHigh+motorLow+90+15); 
    	WallFollowingLab.rightMotor.forward(); 
    	WallFollowingLab.leftMotor.forward();}
    else if (distError < 0) { //too far
    	
    	WallFollowingLab.rightMotor.setSpeed(motorHigh+100+20); 
    	WallFollowingLab.leftMotor.setSpeed(motorHigh-motorLow); 
    	WallFollowingLab.rightMotor.forward(); 
    	WallFollowingLab.leftMotor.forward();} 
    //Thread.sleep(50);//  Allow other threads to get CPU
    }
    // TODO: process a movement based on the us distance passed in (BANG-BANG style)
  

  @Override
  public int readUSDistance() {
    return this.distance;
  }
}
