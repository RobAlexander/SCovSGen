package modeling;

/**
 *
 * @author rl576
 * 
 * HH - 10.7.14 - modified to provide an additional constructor to allow a U-turn waypoint
 */
public class Waypoint extends Entity
{
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	
	private int nextPoint; //the id of the point to go to after this waypoint
	
	/** Constructor for Waypoint
	 * 
	 * @param ID the id of the waypoint
	 * @param next the id of the point that should be travelled to after this waypoint is reached
	 */
	public Waypoint(int ID, int next)
	{
		super(ID, TWAYPOINT);
		nextPoint = next;
	}
	
	/** HH Additional Constructor for Waypoint used to enforce a U-turn
	 * 
	 * @param ID the id of the waypoint
	 * @param next the id of the point that should be travelled to after this waypoint is reached
	 * @param type - the integer type 
	 */
	public Waypoint(int ID, int next, int type)
	{
		super(ID, type);
		nextPoint = next;
	}
	
	/**
	 * A method which returns the id of the point to go to after this waypoint has
	 * been reached
	 * 
	 * @return the id of the next point for the car to travel to
	 */
	public int getNextPoint() {return nextPoint;}
}
