/**
 * 
 */
package dominant;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.security.SecureRandom;

import modeling.COModelWithoutUI;
import modeling.Constants;

/**
 * @author hh940 HH 15.1.15
 * 
 * This Class is a wrapper for running a whole experiment to compare the situation coverage and 
 * fault-finding performance of two methods of situation generation: random, and search-based.
 */
public class RunComparison {

	/**
	 * This method...
	 * @param String[] args ()
	 */
    public static void main(String[] args)
    {
    	// Open an overall log file so that we can capture the start and end times of the various loops
    	File infoLog = new File(Constants.outFilePath + "RunComparisonLog.txt");
    	PrintStream ps;
    	
    	try{
    		ps = new PrintStream(new FileOutputStream(infoLog));
    	}
    	catch(FileNotFoundException e)
    	{
    		System.out.print("RunComparison log file not found!");
    		return;
    	}
    	
    	// HH Outer loop enables us to run a number of different levels of search effort (which
    	// should also provide different levels of situation coverage and will therefore help
    	// us to explore the relationship between situation coverage and #accidents/faults found).
    	int iterationLimit = 20000; // Use this as a start value if we are looping, or the #iterations if we are doing a single run 
    	double tempPercentCov;
    	
    	// HH 22.1.15 Define the array of random seeds that we want to loop through
    	int[] setFaultArray = {1, 3, 7, 9, 11, 16, 17}; // Make this an empty array if you want to run random faults
    	//int[] setFaultArray = {6, 7, 11, 14, 15, 16, 19}; // Make this an empty array if you want to run random faults
    	//int[] setFaultArray = {6}; 
    	
    	//for (int i=0; i<5; i++)
    	//{  		
    		// *****  SEARCH-BASED MAP GENERATION *****
    		
    		// Log the start time
    		long startTime = java.lang.System.currentTimeMillis();
    		
    		ps.println("Search Based Map Generation starting with iteration limit: " + iterationLimit + " at time: " + startTime + ".");

    		// Run Search-Based Map Generation to produce list of External Seeds, and a record of 
    		// situation coverage achieved
    		tempPercentCov = SearchBasedMapGeneration.generateExternalSeeds(iterationLimit);

    		ps.println("Search Based Map Generation with iteration limit: " + iterationLimit + " achieves situation coverage of: " + tempPercentCov + ".");
    		
    		// Run the model with the Search-Based seed set
    		ActuallyRunSpecificBatchFromFile.runBatch(iterationLimit, false, setFaultArray); // FALSE = We're going to supply the Fault # in runBatch

    		// Log the finish time
    		long endTime = java.lang.System.currentTimeMillis();
    		
    		ps.println("Search Based Map Generation ending with iteration limit: " + iterationLimit + " at time: " + endTime + ".  Random Map Generation starting...");
    		
    		long timeElapsed = endTime - startTime;
 
    		// *****  RANDOM MAP GENERATION *****

    		long ExternalSeed;
    		double percentageFaults = (double)5/100; // HH 20.1.15 We need to insert some faults

    		// HH 15.1.15 : Existing copies of this file will be overwritten if the output folder is not empty
    		File newOutputs = new File(Constants.outFilePath + "RandomExternalSeeds_" + iterationLimit + ".txt");
    		PrintStream ps2;

    		try{
    			ps2= new PrintStream(new FileOutputStream(newOutputs));
    		}
    		catch(FileNotFoundException e)
    		{
    			System.out.print("RandomExternalSeeds_" + iterationLimit + ".txt file not found!");
    			ps.close();
    			return;
    		}

    		COModelWithoutUI mod = new COModelWithoutUI();
    		int loopLength = 1;
    		if (setFaultArray.length > 0)
    		{
    			loopLength = setFaultArray.length;
    		}
    		
    		int iteration = 0;
    		
    		// Loop for same length of time as taken above, generating individual External seeds
    		// and running a batch of 3 simulations with each
    		while (java.lang.System.currentTimeMillis() < (endTime + timeElapsed)) // Assume that endTime of SB search is startTime for Random search
    		{
    			ExternalSeed = Math.round(new SecureRandom().nextInt()); // Get a new Map/Initial Configuration
    			
    			// HH 22.1.15 Loop through the set of individual faults that we want to activate in a loop
    			for (int i=0; i<loopLength; i++)
    			{   			
        			if (setFaultArray.length > 0)
        			{	
        				// Create the simulation model so that all the results will end up in the same output file
        				mod = new COModelWithoutUI(setFaultArray[i], (100000 + iterationLimit + iteration), false); // Make sure that the output filenames will be different from SB
        			} else {
        				mod = new COModelWithoutUI(percentageFaults, (100000 + iterationLimit), true); // Make sure that the output filenames will be different from SB
        			}
        			
        			//mod.runBatch(3, ExternalSeed); // Run a batch of 3 for each initial configuration
        			mod.runBatch(1, ExternalSeed); // Run a batch of 1 for each initial configuration to speed up
        		}
    			
    			// Add external seed to an output file
    			ps2.println(ExternalSeed);
    			
    			// Increment loop count so that we have separate files
    			if (setFaultArray.length > 0)
    			{
    				iteration = iteration + 1;
    			}
    		}
    		    		
    		ps2.close();

    		ps.println("Random Map Generation ending with iteration limit: " + iterationLimit + " at time: " + java.lang.System.currentTimeMillis() + ".");
    		
    		// Calculate and record the situation coverage achieved by the Random Map Generation 
    		tempPercentCov = CalculateSituationCoverageFromFile.calcSitCov(iterationLimit); // Okay to use iterationLimit on own here as uses different filenames
    		
    		ps.println("Random Map Generation with iteration limit: " + iterationLimit + " achieves situation coverage of: " + tempPercentCov + ".");
    		
    		// Update iterationLimit
    		iterationLimit = iterationLimit * 4; // We don't have enough run time to just double it, we need to do 4x.
    	//} 
    	
    	ps.close();
    }
}
