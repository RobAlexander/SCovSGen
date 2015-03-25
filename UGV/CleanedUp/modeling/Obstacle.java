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
	private static final long serialVersionUID = 1L;
	
	protected double direction; // HH 2.10.14 - We'll need this if the roads aren't grid-aligned
	
	/**
	 * Constructor.
	 * @param idNo (int - )
	 * @param typeNo (int - )
	 * @param inDirection (double - )
	 */
	public Obstacle(int idNo, int typeNo, double inDirection)
	{
		super(idNo, typeNo);
		direction = inDirection;
	}
	
	/**
	 * This method returns the orientation of the vehicle as either 0 or 90 degrees
	 * @return double (0 if the obstacle is oriented N/S or 90 if it is oriented E/W)
	 */
	public double getDirection()
	{
		return direction;
	}
	
	/**
	 * method which returns true or false if a provided coordinate is in the shape
	 * would have to be overwritten when implemented
	 * @param coord (Double2D - )
	 * @return boolean ()
	 */
	public abstract boolean inShape(Double2D coord);	
	
	/**
	 * Returns the distance from the closest part of the obstacle to the coord provided.
	 * @param coord (Double2D - the coordinate the distance to be checked for)
	 * @return boolean ()
	 */
	public double obstacleToPoint(Double2D coord)
	{
		return Double.MAX_VALUE;
	}
	
	/**
	 * This method...
	 * @return double ()
	 */
	@Override
	public double orientation2D() {
		
		// HH 2.10.14 - For some reason, the orientation of the vehicle seems to be displayed 
		// relative to zero degrees along the increasing x axis.  As a result, we need to flip 
		// the compass over the 45/225 degree bisection to convert between the two systems.
		return Math.toRadians(Utility.correctAngle(90-direction));
	}	
}
