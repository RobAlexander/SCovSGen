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
 * @author hh940 HH 15.1.15
 * 
 * This Class is a wrapper for running a whole experiment to compare the situation coverage and 
 * fault-finding performance of two methods of situation generation: random, and search-based.
 */
public class RunCompareMapSets {

	/**
	 * @param args
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
    	
    	// HH 22.1.15 Define the array of random seeds that we want to loop through
    	int[] setFaultArray = {1}; // Make this an empty array if you want to run random faults
    		
    	// Log the start time
    	long startTime = java.lang.System.currentTimeMillis();
    	ps.println("Search Based Map Evaluation starting at time: " + startTime + ", " + Utility.timeToString() + ".");

    	// Run the model with the Search-Based seed set
    	ActuallyRunSpecificBatchFromFile.runBatch(150, false, setFaultArray); // FALSE = We're going to supply the Fault # in runBatch

    	// Log the finish time
    	long endTime = java.lang.System.currentTimeMillis();
    	ps.println("Search Based Map Generation ending at time: " + endTime + ", " + Utility.timeToString() + ".");
    	
    	startTime = java.lang.System.currentTimeMillis();
    	ps.println("Random Map Evaluation starting at time: " + startTime + ", " + Utility.timeToString() + ".");
    	
    	// Run the model with the Search-Based seed set
    	ActuallyRunSpecificBatchFromFile.runBatch(1150, false, setFaultArray); // FALSE = We're going to supply the Fault # in runBatch

    	endTime = java.lang.System.currentTimeMillis();
    	ps.println("Random Map Generation ending at time: " + endTime + ", " + Utility.timeToString() + ".");
    	
    	ps.close();
    }

}
