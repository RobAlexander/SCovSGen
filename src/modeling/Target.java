package modeling;

/**
 * A class to represent the Target which the UGV is trying to reach.
 * @author Robert Lee
 */
public class Target extends Entity
{	
	private static final long serialVersionUID = 1L;

	/**
	 * Constructor.  Initialise the Entity superclass with the unique ID and the 
	 * type identifier TTARGET.
	 * @param idNo (int - unique identifier for Target object)
	 */
	public Target(int idNo)
	{
		super(idNo, TTARGET);
	}
}
