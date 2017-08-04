/**
 * 
 */
package simcontroller;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import modeling.COModelWithoutUI;
import modeling.Constants;

/**
 * @author HH940
 * 
 * This class has been created to allow us to supply a set of existing experiment runs (i.e. the Random Seeds 
 * that define the runs) as a text file, and use the external random seed that defines each run to re-generate 
 * the network map and run the model.  This is a variation of the RunSpecificBatchFromFile, which (despite its 
 * name) does not actually run the model.  You can supply an array of specific faults to instantiate, one at a
 * time, or set a percentage level of faults to inject into each run.  You can choose to run a single iteration
 * of each experiment, or a batch of e.g. 3 runs per External Seed/Fault combination.
 */
public class ActuallyRunSpecificBatchFromFile {

	/**
	 * This method reads External Random Seeds from a file and uses them to construct a simulation map.
	 * The simulation can include seeded faults which can be specified in an array (to be instantiated
	 * one at a time), or can be determined at random at a supplied percentage level.  The simulation
	 * is run for the specified number of times.  This loop repeats for all seeds in the file, and then
	 * for all faults specified in the fault array.
	 * @param iterationLimit (int - identifier used to reconstruct input filename)
	 * @param wantRandomFaults (boolean - true if you want random faults)
	 * @param inSetFaultArray (int[] - for non-random faults, supply an array of faults to instantiate)
	 */
    public static void runBatch(int iterationLimit, boolean wantRandomFaults, int[] inSetFaultArray)
    {
    	// Parameters that we need
    	double percentageFaults = (double)5/100; // Set the percentage of faults that we want to insert
    	Long tempLong;
    	long ExternalSeed; 
    	COModelWithoutUI mod;
    	
    	FileReader UGVInputFile = null;
    	String inputString = "";
		
		// Set up the loop stuff
		int faultLoops = 1; // Default if we are going to set the faults randomly
		if (wantRandomFaults == false)
		{
			faultLoops = inSetFaultArray.length;
		}
		
		// Run the loop
		for (int i=0; i<faultLoops; i++)
		{
			// We're going to have to open the file and close it on each loop so that we get the full set of external
			// seeds to run against each fault configuration.  
			try {
				UGVInputFile = new FileReader(Constants.outFilePath + "selectedExternalSeeds_" + iterationLimit + ".txt");
			} catch (FileNotFoundException e2) {
				e2.printStackTrace();
			}
	    	
	    	// Read the input file and initialise the modelling parameters
	    	final BufferedReader UGVInputFileReader = new BufferedReader(UGVInputFile);
	    	
	    	inputString = "";
	    	
			try {
				inputString = UGVInputFileReader.readLine();
			} catch (IOException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}
			
			// Deal with the runs where we want to specify the fault index
			if (wantRandomFaults == true)
			{
				// Set up the simulation outside the loop so that we only need one file
				mod = new COModelWithoutUI(percentageFaults, iterationLimit, wantRandomFaults); // Use iterationLimit to identify the files
			} else {
				// Set up the simulation outside the loop so that we only need one file
				mod = new COModelWithoutUI(inSetFaultArray[i], iterationLimit, wantRandomFaults); // Use iterationLimit to identify the files
			}
			
			while (inputString != null)
			{
				// Read the External Seed from file
				tempLong = new Long(inputString);
				ExternalSeed = tempLong.longValue();

				// Run map generation and analysis to calculate the values for our coverage criteria -
				// they will be categorised by a static method defined within this class so the category 
				// boundaries can easily be found and updated if necessary
				//mod.runBatch(3, ExternalSeed); // Run a batch of 3 for each initial configuration
				mod.runBatch(1, ExternalSeed); // Run a batch of 1 for each initial configuration (to speed things up)

				// Get the next line
				try {
					inputString = UGVInputFileReader.readLine();
				} catch (IOException e1) {
					e1.printStackTrace();
				}
			} 
		
			try {
				UGVInputFileReader.close();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} // Close the file   
		}
    }     
}
