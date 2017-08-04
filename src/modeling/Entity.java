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
	 * @param location (Double2D - new location for the Entity)
	 */
	public void setLocation(Double2D location) {
		this.location = location;
	}

	/**
	 * This method reports whether the Entity is schedulable.
	 * @return boolean (returns true if the Entity is schedulable)
	 */
	public boolean isSchedulable() {
		return isSchedulable;
	}
	
	/**
	 * This method returns the type of the entity e.g. TCAR
	 * @return int (the index of the enumerated type which represents this entity)
	 */
	public int getType() {return type;}

	/**
	 * This method returns the unique identifier assigned to the Entity.
	 * @return int (unique ID of the Entity)
	 */
	public int getID() {return ID;}
}
