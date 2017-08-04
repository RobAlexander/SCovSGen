/**
 * 
 */
package modeling;

/**
 * This class allows us to create Crash Objects which can be visualised in the UI.
 * 
 * @author hh940 
 */
public class Crash extends Entity {

	private static final long serialVersionUID = 1L;

	/**
	 * Constructor
	 * @param idNo (int - unique identifier)
	 * @param typeNo (int - integer identifier for type of crash e.g. TFAILURE)
	 */
	public Crash(int idNo, int typeNo) {
		super(idNo, typeNo);
	}

}

