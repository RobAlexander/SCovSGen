/**
 * 
 */
package dominant;

import modeling.COModelWithUI;
import sim.display.Console;

/**
 * @author xueyi
 *
 */
public class SimulationWithUI {

	/**
	 * @param args
	 */
	
    public static void main(String[] args)
    {
    	System.out.println("Before we do anything: Free =" + Runtime.getRuntime().freeMemory());
    	System.out.println("Before we do anything: Max =" + Runtime.getRuntime().maxMemory());
    	System.out.println("Before we do anything: Total =" + Runtime.getRuntime().totalMemory());
    	
    	//COModelWithUI vid = new COModelWithUI(0, true); // HH 29.1.15 Change to <f>, False if want to run a specific seeded fault, use fault index for <f>
    	COModelWithUI vid = new COModelWithUI(16, false); 
    	Console c = new Console(vid);
		c.setVisible(true);

    }

}
