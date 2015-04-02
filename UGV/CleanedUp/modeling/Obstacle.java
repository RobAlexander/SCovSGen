package modeling;
import sim.portrayal.Oriented2D;
import sim.util.*;

/**
 * @author Robert Lee/hh940
 * 
 * This class represents STATIC obstacles.  These are initialised when the map is generated
 * and they do not change.
 */
public abstract class Obstacle extends Entity implements Oriented2D
{
	private static final long serialVersionUID = 1L;
	
	protected double direction; // We'll need this if the roads aren't grid-aligned
	
	/**
	 * Constructor, create a new obstacle by initialising Entity and setting direction
	 * @param idNo (int - unique id for obstacle)
	 * @param typeNo (int - obstacle type identifier)
	 * @param inDirection (double - direction in which obstacle is oriented, 0 if the obstacle is oriented N/S or 90 if it is oriented E/W)
	 */
	public Obstacle(int idNo, int typeNo, double inDirection)
	{
		super(idNo, typeNo);
		direction = inDirection; // TODO - Consider some range checking to make sure this is only 0 or 90
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
	 * method which returns true or false if a provided coordinate is in the shape:
	 * would have to be overwritten when implemented
	 * @param coord (Double2D - coordinates of location we want to test for intersection with the obstacle shape)
	 * @return boolean (true if the coord intersects with the obstacle shape)
	 */
	public abstract boolean inShape(Double2D coord);	
		
	/**
	 * This method provides the orientation of the vehicle in radians and relative to a scale/axis
	 * appropriate for orientation2D.
	 * @return double (return the orientation of the vehicle in radians as required for orientation2D)
	 */
	@Override
	public double orientation2D() {
		
		// For some reason, the orientation of the vehicle seems to be displayed 
		// relative to zero degrees along the increasing x axis.  As a result, we need to flip 
		// the compass over the 45/225 degree bisection to convert between the two systems.
		return Math.toRadians(Utility.correctAngle(90-direction));
	}	
}
