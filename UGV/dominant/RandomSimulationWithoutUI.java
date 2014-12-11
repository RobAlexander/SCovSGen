/**
 * 
 */
package dominant;

import java.security.SecureRandom;

import modeling.COModelWithoutUI;
import modeling.Constants;


/**
 * @author HH 31/7/14
 */
public class RandomSimulationWithoutUI {

	/**
	 * @param args
	 */
	
    public static void main(String[] args)
    {
    	// HH 27.8.14 - Run with different % faults
    	double percentageFaults;
    	
    	long ExternalSeed; // HH 21.10.14 Declare outside loop
    	
    	// HH 1.9.14 Add outer loop to set the External Random Seed (that will be used to generate the network map)
    	// and which we may want to remain constant over a set of different percentage faults (and for each of the 10 random
    	// runs associated with each of the percentage faults).
    	for (int j = 1; j <= Constants.NO_RANDOM_RUNS; j++) {    	
    	
    		// HH 21.10.14 - Moved to inside the loop
    		// HH 1.9.14 - Generate the random seed to use on this run
    		//long ExternalSeed = Math.round(new SecureRandom().nextInt());
    		
    		// HH 28.8.14 : NOTE - differences in percentages of faults must be > 1% or files will be overwritten
    		// e.g. percentageFaults = 0.01, 0.02, 0.03 ... is okay, (NOT e.g. 0.001, 0.002)
    		// ALSO: Consider whether resolution is meaningful compared to number of faults e.g. if there are only 29 faults,
    		// there will be very little difference between 1%, 2%, 3% etc.
    		// HH 4.12.14 Updated to a change of 5% on each loop due to #faults = 19 to ensure that we get different
    		// numbers of faults at the different levels
    		for (int i=0; i < 21; i=i+5)
    		{
    			percentageFaults = (double)i/100;

    			COModelWithoutUI mod = new COModelWithoutUI(percentageFaults, j);
    			
    			// HH 21.10.14 - Run 5 different initial configurations at each fault level; run each of these
    			// with 3 different sets of run-time behaviour
    			for (int k=0; k < 5; k++)
    			{
    				ExternalSeed = Math.round(new SecureRandom().nextInt()); // Get a new Map/Initial Configuration
    				mod.runBatch(3, ExternalSeed, j); // Run a batch of 3 for each initial configuration
    			}
    		}
    	}
    }
   
    
}
