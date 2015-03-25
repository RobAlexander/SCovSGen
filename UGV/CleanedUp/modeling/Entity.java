package modeling;
import sim.engine.*;
import sim.util.*;

/**
 * The class from which every object used in simulations is created, contains only
 * the required methods for identifying more about the object.
 * 
 * @author Robert Lee
 */
public class Entity implements Steppable, Constants
{
	private static final long serialVersionUID = 1L;
	
	protected final int ID;
	protected final int type;	
	protected Double2D location;
	protected boolean isSchedulable;
	
	/**
	 * Constructor.
	 * @param idNo (int - )
	 * @param typeNo (int - )
	 */
	public Entity(int idNo, int typeNo)
	{
		ID = idNo;
		type = typeNo;
		
	}
	
	/**
	 * This method...
	 * @param state (SimState - )
	 */
	public void step(SimState state)
	{
		//do nothing - this will be overwritten in the child classes - if required
	}
	
	/**
	 * A method which returns the stored location of the object
	 * @return Double2D ()
	 */
	public Double2D getLocation() {
		return location;
	}

	/**
	 * A meethod which sets the stored location to the supplied coordinates
	 * @param location (Double2D - )
	 */
	public void setLocation(Double2D location) {
		this.location = location;
	}

	/**
	 * This method...
	 * @return boolean ()
	 */
	public boolean isSchedulable() {
		return isSchedulable;
	}
	
	/**
	 * This method...
	 * @return int ()
	 */
	public int getType() {return type;}

	/**
	 * This method...
	 * @return int ()
	 */
	public int getID() {return ID;}
}
