package modeling;

/**
 * Class to support the performance characteristics of the car, including acceleration/deceleration
 * maximum speed, and rate of turning. 
 * @author Robert Lee/hh940
 */
public class CarPerformance
{
	// The current limits of speed, etc for the car.
	private double currentMaxSpeed;
	private double currentMaxAcceleration;
	private double currentMaxDeceleration;
	private double currentMaxTurning;
	
	/*
	 * Constructor to set up all the class members.
	 * @param maxCarSpeed (double - maximum speed achievable by vehicle)
	 * @param maxCarAcceleration (double - maximum increase in speed in one simulation step)
	 * @param maxCarDeceleration (double - maximum decrease in speed  in one simulation step)
	 * @param maxCarTurning (double - maximum rate of turning in one simulation step)
	 */
	public CarPerformance(double maxCarSpeed, double maxCarAcceleration, double maxCarDeceleration, double maxCarTurning)
	{
		currentMaxSpeed = maxCarSpeed;
		currentMaxAcceleration = maxCarAcceleration;
		currentMaxDeceleration = maxCarDeceleration;
		currentMaxTurning = maxCarTurning;
	}
	
	// Methods to set the statistics of the car
	public void setCurrentMaxSpeed(double speed) {currentMaxSpeed = speed;}
	public void setCurrentMaxAcceleration(double accel) {currentMaxAcceleration = accel;}
	public void setCurrentMaxDeceleration(double decel) {currentMaxDeceleration = decel;}
	public void setCurrentMaxTurning(double turning) {currentMaxTurning = turning;}
	
	// Accessor methods for the statistics of the car
	public double getCurrentMaxSpeed() {return currentMaxSpeed;}
	public double getCurrentMaxAccel() {return currentMaxAcceleration;}
	public double getCurrentMaxDecel() {return currentMaxDeceleration;}
	public double getCurrentMaxTurning() {return currentMaxTurning;}
		
}
