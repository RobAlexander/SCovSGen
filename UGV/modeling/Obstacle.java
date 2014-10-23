package modeling;
import sim.portrayal.Oriented2D;
import sim.util.*;

/**
 *
 * @author Robert Lee
 * 
 * HH 2.10.14 This class represents STATIC obstacles.  These are initialised when the map is generated
 * and they do not change.
 */
public abstract class Obstacle extends Entity implements Oriented2D
{
	protected double direction; // HH 2.10.14 - We'll need this if the roads aren't grid-aligned
	
	public Obstacle(int idNo, int typeNo, double inDirection)
	{
		super(idNo, typeNo);
		direction = inDirection;
	}
	
	public double getDirection()
	{
		return direction;
	}
	
	/**
	 * method which returns true or false if a provided coordinate is in the shape
	 * would have to be overwritten when implemented
	 */
	public abstract boolean inShape(Double2D coord);	
	
	/**
	 * Returns the distance from the closest part of the obstacle to the coord provided.
	 * 
	 * @param coord the coordinate the distance to be checked for
	 */
	public double obstacleToPoint(Double2D coord)
	{
		return Double.MAX_VALUE;
	}
	
	@Override
	public double orientation2D() {
		
		// HH 2.10.14 - For some reason, the orientation of the vehicle seems to be displayed 
		// relative to zero degrees along the increasing x axis.  As a result, we need to flip 
		// the compass over the 45/225 degree bisection to convert between the two systems.
		return Math.toRadians(Car.correctAngle(90-direction));
	}	
}
