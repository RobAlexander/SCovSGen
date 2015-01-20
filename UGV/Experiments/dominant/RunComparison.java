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
	 * @param args
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
    	int iterationLimit = 100; 
    	double tempPercentCov;
    	
    	for (int i=0; i<5; i++)
    	{  		
    		// *****  SEARCH-BASED MAP GENERATION *****
    		
    		// Log the start time
    		long startTime = java.lang.System.currentTimeMillis();
    		
    		ps.println("Search Based Map Generation starting with iteration limit: " + iterationLimit + " at time: " + startTime + ".");

    		// Run Search-Based Map Generation to produce list of External Seeds, and a record of 
    		// situation coverage achieved
    		tempPercentCov = SearchBasedMapGeneration.generateExternalSeeds(iterationLimit);

    		ps.println("Search Based Map Generation with iteration limit: " + iterationLimit + " achieves situation coverage of: " + tempPercentCov + ".");
    		
    		// Run the model with the Search-Based seed set
    		ActuallyRunSpecificBatchFromFile.runBatch(iterationLimit); // pass in empty arguments

    		// Log the finish time
    		long endTime = java.lang.System.currentTimeMillis();
    		
    		ps.println("Search Based Map Generation ending with iteration limit: " + iterationLimit + " at time: " + endTime + ".  Random Map Generation starting...");
    		
    		long timeElapsed = endTime - startTime;
 
    		// *****  RANDOM MAP GENERATION *****

    		long ExternalSeed;
    		double percentageFaults = 0;

    		// HH 15.1.15 : Existing copies of this file will be overwritten if the output folder is not empty
    		File newOutputs = new File(Constants.outFilePath + "RandomExternalSeeds_" + iterationLimit + ".txt");
    		PrintStream ps2;

    		try{
    			ps2= new PrintStream(new FileOutputStream(newOutputs));
    		}
    		catch(FileNotFoundException e)
    		{
    			System.out.print("RandomExternalSeeds_" + iterationLimit + ".txt file not found!");
    			return;
    		}

    		// Create the simulation model so that all the results will end up in the same output file
    		COModelWithoutUI mod = new COModelWithoutUI(percentageFaults, (100000 + iterationLimit)); // Make sure that the output filenames will be different from SB

    		// Loop for same length of time as taken above, generating individual External seeds
    		// and running a batch of 3 simulations with each
    		while (java.lang.System.currentTimeMillis() < (endTime + timeElapsed)) // Assume that endTime of SB search is startTime for Random search
    		{
    			ExternalSeed = Math.round(new SecureRandom().nextInt()); // Get a new Map/Initial Configuration
    			mod.runBatch(3, ExternalSeed); // Run a batch of 3 for each initial configuration

    			// Add external seed to an output file
    			ps2.println(ExternalSeed);
    		}	

    		ps2.close();

    		ps.println("Random Map Generation ending with iteration limit: " + iterationLimit + " at time: " + java.lang.System.currentTimeMillis() + ".");
    		
    		// Calculate and record the situation coverage achieved by the Random Map Generation 
    		tempPercentCov = CalculateSituationCoverageFromFile.calcSitCov(iterationLimit); // Okay to use iterationLimit on own here as uses different filenames
    		
    		ps.println("Random Map Generation with iteration limit: " + iterationLimit + " achieves situation coverage of: " + tempPercentCov + ".");
    		
    		// Update iterationLimit
    		iterationLimit = iterationLimit * 4; // We don't have enough run time to just double it, we need to do 4x.
    	} 
    	
    	ps.close();
    }

}