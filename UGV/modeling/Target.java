package modeling;
import sim.util.*;

/**
 *
 * @author Robert Lee
 */
public class Target extends Entity
{	
	public Target(int idNo)
	{
		super(idNo, TTARGET);
	}
	
	/* this method will command the target to move to another location
	 * 
	 * @param simulation this is required for the random number generator
	 */
//	public void move(COModel simulation)
//	{
//		Double2D newLocation = new Double2D(simulation.random.nextDouble() * 100, simulation.random.nextDouble() * 100);
//		myLocation = newLocation;
//		simulation.environment.setObjectLocation(this, newLocation);
//	}
}
