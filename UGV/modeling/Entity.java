package modeling;
import sim.engine.*;
import sim.field.continuous.*;
import sim.util.*;

/**
 * The class from which every object used in simulations is created, contains only
 * the required methods for identifying more about the object.
 * 
 * @author Robert Lee
 */
public class Entity implements Steppable, Constants
{
	protected final int ID;
	protected final int type;	
	protected Double2D location;
	protected boolean isSchedulable;
	
	
	public Entity(int idNo, int typeNo)
	{
		ID = idNo;
		type = typeNo;
		
	}
	
	
	public void step(SimState state)
	{
		//do nothing - this will be overwritten in the child classes - if required
	}
	
	/**
	 * A method which sets the stored location of the object to a provided coordinate
	 * 
	 * @param location the provided location to be stored in the Entity
	 */

	public Double2D getLocation() {
		return location;
	}


	public void setLocation(Double2D location) {
		this.location = location;
	}


	public boolean isSchedulable() {
		return isSchedulable;
	}
	
	public int getType() {return type;}
	public int getID() {return ID;}
}
