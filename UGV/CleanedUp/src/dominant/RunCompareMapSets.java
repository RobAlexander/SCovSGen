/**
 * 
 */
package dominant;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintStream;
import modeling.Constants;
import modeling.Utility;

/**
 * @author hh940
 * 
 * This Class is a wrapper for running a whole experiment to compare the situation coverage and 
 * fault-finding performance of two methods of situation generation: random, and search-based.
 */
public class RunCompareMapSets {

	/**
	 * Evaluate the run time for and run (generating outputs) the Search Based and
	 * Random map sets. A summary file is produced with information about how long 
	 * the run process took for each of the map sets. 
	 * NOTE: The files are only identified based on the first parameter supplied to
	 * runBatch (below), so this method can be used to compare any two map sets, 
	 * although the commenting to the output file should be changed if this is the 
	 * case.  Depending upon the naming of the map set files, the code will need to 
	 * be changed below in order to read the right files.
	 * @param args (String[] - not used)
	 */
	
    public static void main(String[] args)
    {
    	// Open an overall log file so that we can capture the start and end times of the various loops
    	File infoLog = new File(Constants.outFilePath + "RunCompareMapSetsLog.txt");
    	PrintStream ps;
    	
    	try{
    		ps = new PrintStream(new FileOutputStream(infoLog));
    	}
    	catch(FileNotFoundException e)
    	{
    		System.out.print("RunCompareMapSetsLog log file not found!");
    		return;
    	}
    	
    	// Define the array of random seeds that we want to loop through
    	int[] setFaultArray = {1}; // Make this an empty array if you want to run random faults
    		
    	// Log the start time
    	long startTime = java.lang.System.currentTimeMillis();
    	ps.println("Search Based Map Evaluation starting at time: " + startTime + ", " + Utility.timeToString() + ".");

    	// Run the model with the Search-Based seed set
    	ActuallyRunSpecificBatchFromFile.runBatch(150, false, setFaultArray); // FALSE = We're going to supply the Fault # in runBatch

    	// Log the finish time
    	long endTime = java.lang.System.currentTimeMillis();
    	ps.println("Search Based Map Evaluation ending at time: " + endTime + ", " + Utility.timeToString() + ".");
    	
    	startTime = java.lang.System.currentTimeMillis();
    	ps.println("Random Map Evaluation starting at time: " + startTime + ", " + Utility.timeToString() + ".");
    	
    	// Run the model with the Search-Based seed set
    	ActuallyRunSpecificBatchFromFile.runBatch(1150, false, setFaultArray); // FALSE = We're going to supply the Fault # in runBatch

    	endTime = java.lang.System.currentTimeMillis();
    	ps.println("Random Map Evaluation ending at time: " + endTime + ", " + Utility.timeToString() + ".");
    	
    	ps.close();
    }

}
