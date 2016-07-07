/**
 * 
 */
package dominant;

import java.security.SecureRandom;

import modeling.COModelWithoutUI;
import modeling.Constants;


/**
 * @author HH940 
 */
public class RandomSimulationWithoutUI {

	/**
	 * This method will run a batch of simulations and generate outputs to file.  It can be configured to run
	 * multiple iterations (m) of the following: for each iteration of (j) we will run the simulation at
	 * a series of different fault levels (defined by i).  For each simulation that we create, we run 5
	 * different maps (k), each of which is run for a batch of 3 runtime configurations.  Each output file 
	 * will contain the results of 15 runs.  There will be results for 4 different fault levels, and these
	 * will be repeated for a loop of 20.  
	 * @param args (String[] - not used)
	 */
    public static void main(String[] args)
    {
    	// Allow to run with different % faults
    	double percentageFaults;
    	
    	long ExternalSeed; // Declare outside loop
    	
    	// Additional Loop to uncomment if we want e.g. 10 sets of 'background' failure data
    	// NOTE: this may now be unnecessary due to the number of other loops below.
    	//for (int m=4; m < 10; m++)
    	//{
    		// Add outer loop to set how many times we want to run the experiment with each
    	    // level of percentage faults.
    		for (int j = 1; j <= Constants.NO_RANDOM_RUNS; j++) {    	

    			// NOTE - differences in percentages of faults must be > 1% or files will be overwritten
    			// e.g. percentageFaults = 0.01, 0.02, 0.03 ... is okay, (NOT e.g. 0.001, 0.002)
    			// ALSO: Consider whether resolution is meaningful compared to number of faults e.g. if there are only 29 faults,
    			// there will be very little difference between 1%, 2%, 3% etc.
    			
    			for (int i=5; i < 21; i=i+5) // Start with 5% faults as already have a lot of data on 0% faults
    			{
    				percentageFaults = (double)i/100;

    				// Different methods to initialise the model depending upon whether outer loop is being used or not 
    				// - this ensures that the output files are given unique names so they will not be overwritten
    				//COModelWithoutUI mod = new COModelWithoutUI(percentageFaults, (m*100 + j), true); // Use this if we are using the m outer loop
    				COModelWithoutUI mod = new COModelWithoutUI(percentageFaults, j, true); // Use this if we are not using the m outer loop

    				// Run 5 different initial configurations at each fault level; run each of these
    				// with 3 different sets of run-time behaviour
    				for (int k=0; k < 5; k++)
    				{
    					ExternalSeed = Math.round(new SecureRandom().nextInt()); // Get a new Map/Initial Configuration
    					mod.runBatch(3, ExternalSeed); // Run a batch of 3 for each initial configuration
    				}
    			}
    		}
    	//}
    }    
}
