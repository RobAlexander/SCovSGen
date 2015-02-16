package modeling;

/**
 *
 * @author Robert Lee
 */
public class CarPerformance
{
	//the maximum possible values of the cars statistics
	private double maxSpeed;
    private double maxAcceleration;
	private double maxDeceleration;
	private double maxTurning;
	
	//the current limits of speed, etc for the car.
	private double currentMaxSpeed;
	private double currentMaxAcceleration;
	private double currentMaxDeceleration;
	private double currentMaxTurning;
	
	public CarPerformance(double maxCarSpeed, double maxCarAcceleration, double maxCarDeceleration, double maxCarTurning)
	{
		maxSpeed = maxCarSpeed;
		maxAcceleration = maxCarAcceleration;
		maxDeceleration = maxCarDeceleration;
		maxTurning = maxCarTurning;
	}
	
	public double getMaxSpeed() {
		return maxSpeed;
	}

	public void setMaxSpeed(double maxSpeed) {
		this.maxSpeed = maxSpeed;
	}

	public double getMaxAcceleration() {
		return maxAcceleration;
	}

	public void setMaxAcceleration(double maxAcceleration) {
		this.maxAcceleration = maxAcceleration;
	}

	public double getMaxDeceleration() {
		return maxDeceleration;
	}

	public void setMaxDeceleration(double maxDeceleration) {
		this.maxDeceleration = maxDeceleration;
	}

	public double getMaxTurning() {
		return maxTurning;
	}

	public void setMaxTurning(double maxTurning) {
		this.maxTurning = maxTurning;
	}
	
	//Methods to set the statistics of the car
	public void setCurrentMaxSpeed(double speed) {currentMaxSpeed = speed;}
	public void setCurrentMaxAcceleration(double accel) {currentMaxAcceleration = accel;}
	public void setCurrentMaxDeceleration(double decel) {currentMaxDeceleration = decel;}
	public void setCurrentMaxTurning(double turning) {currentMaxTurning = turning;}
	
	//Accessor methods for the statistics of the car
	public double getCurrentMaxSpeed() {return currentMaxSpeed;}
	public double getCurrentMaxAccel() {return currentMaxAcceleration;}
	public double getCurrentMaxDecel() {return currentMaxDeceleration;}
	public double getCurrentMaxTurning() {return currentMaxTurning;}
	
	/**
	 * A method which sets the current statistics of the car to be the original
	 * values they were set as.
	 */
	public void reset()
	{
		currentMaxSpeed = maxSpeed;
		currentMaxAcceleration = maxAcceleration;
		currentMaxDeceleration = maxDeceleration;
		currentMaxTurning = maxTurning;
	}
	

}
