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
    	
    	COModelWithUI vid = new COModelWithUI();
    	Console c = new Console(vid);
		c.setVisible(true);

    }

}
