/**
 * 
 */
package modeling;

/**
 * This class allows us to create Failure Objects which can be visualised in the UI.
 * 
 * @author hh940 
 */
public class Failure extends Entity {

	private static final long serialVersionUID = 1L;

	/**
	 * Constructor.
	 * @param idNo (int - unique identifier)
	 * @param typeNo (int - integer identifier for type of crash e.g. TFAILURE)
	 */
	public Failure(int idNo, int typeNo) {
		super(idNo, typeNo);
	}

}
