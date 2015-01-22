/**
 * 
 */
package dominant;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import modeling.COModelWithoutUI;
import modeling.Constants;



/**
 * @author HH 15.1.15
 * 
 * This method has been created to allow us to supply a set of existing experiment runs as a text file, and
 * use the external random seed that defines each run to re-generate the network map and run the model.  This 
 * is a variation of the RunSpecificBatchFromFile, which (despite its name) does not actually run the model. 
 */
public class ActuallyRunSpecificBatchFromFile {

	/**
	 * @param args
	 */
	
    public static void runBatch(int iterationLimit)
    {
    	// HH Parameters that we need to read from file
    	double percentageFaults = (double)5/100; // HH 20.1.15 We need to insert some faults
    	Long tempLong;
    	long ExternalSeed; 
    	COModelWithoutUI mod;
    	
    	FileReader UGVInputFile = null;
		try {
			UGVInputFile = new FileReader(Constants.outFilePath + "selectedExternalSeeds_" + iterationLimit + ".txt");
		} catch (FileNotFoundException e2) {
			e2.printStackTrace();
		}
    	
    	// HH Read the input file and initialise the modelling parameters
    	final BufferedReader UGVInputFileReader= new BufferedReader(UGVInputFile);
    	
    	String inputString = "";
    	
		try {
			inputString = UGVInputFileReader.readLine();
		} catch (IOException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		
		// Set up the simulation outside the loop so that we only need one file
		mod = new COModelWithoutUI(percentageFaults, iterationLimit); // Use iterationLimit to identify the files
		
    	while (inputString != null)
    	{
    		// Split the line up to access the Internal Seed, External Seed and PercentageFaults
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
