/**
 * 
 */
package simcontroller;

import modeling.COModelWithUI;
import sim.display.Console;

/**
 * @author xueyi/hh940
 *
 */
public class SimulationWithUI {

	/**
	 * This method allows you to run simulations through the User Interface, and can be configured to run
	 * with faults active, or no faults active.  A fixed percentage of faults can be inserted at random, or
	 * a specific fault can be instantiated.
	 * @param String[] args ()
	 */
    public static void main(String[] args)
    {
    	System.out.println("Before we do anything: Free =" + Runtime.getRuntime().freeMemory());
    	System.out.println("Before we do anything: Max =" + Runtime.getRuntime().maxMemory());
    	System.out.println("Before we do anything: Total =" + Runtime.getRuntime().totalMemory());
    	
    	// Choose to run the simulation without any faults, with a supplied fault, or with a fixed percentage of faults
    	// inserted at random
    	COModelWithUI vid = new COModelWithUI(0, true); // Change to (<f>, False) if want to run a specific seeded fault, use fault index for <f>
    	//COModelWithUI vid = new COModelWithUI(16, false); 
    	
    	Console c = new Console(vid);
		c.setVisible(true);
    }
}
