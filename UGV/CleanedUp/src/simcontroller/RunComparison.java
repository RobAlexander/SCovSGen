/**
 * 
 */
package simcontroller;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.security.SecureRandom;

import modeling.COModelWithoutUI;
import modeling.Constants;

/**
 * @author hh940 
 * 
 * This Class is a wrapper for running a whole experiment to compare the situation coverage and 
 * fault-finding performance of two methods of situation generation: Random, and Search Based.
 */
public class RunComparison {

	/**
	 * This method generates a map set using a Search Based map generation method, then simulates each
	 * of these maps, instantiating either a fixed percentage of seeded faults, or by testing against a series
	 * of defined seeded faults, one at a time.  The time to complete this, for a map set limited by
	 * 'iterationLimit', which is the number of candidate maps that the search is permitted to try, is logged.
	 * The time to complete the Search Based map set generation and simulation is now used as a benchmark for
	 * the Random map set generation.  In this part of the method, a map is generated at random, simulated 
	 * against the fault set (or with the required fault level), and then added to the map set.  This is 
	 * repeated until the benchmark time has been exceeded.  A log file records the start and end times for 
	 * each part of this method.
	 * @param args (String[] - not used)
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
    	
    	int iterationLimit = 20000; // Use this as a start value if we are looping, or the #iterations if we are doing a single run 
    	double tempPercentCov;
    	
    	// Define the array of random seeds that we want to loop through
    	int[] setFaultArray = {1, 3, 7, 9, 11, 16, 17}; // Make this an empty array if you want to run random faults
    	//int[] setFaultArray = {6}; // If you just want to run a single fault
    	
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
    	double percentageFaults = (double)5/100; // In case we need to insert some faults at random

    	// Existing copies of this file will be overwritten if the output folder is not empty, this file stores
    	// each of the Random Seeds generated to be part of the map set
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

    	// Check to see if we have a series of faults to run, or if we are just running the
    	// loop once
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

    		// Loop through the set of individual faults that we want to activate in a loop
    		for (int i=0; i<loopLength; i++)
    		{   			
    			// Choose between randomly seeded faults, or seeded faults defined in the supplied fault array
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

    		// Increment loop count so that we have separate files if we are looping through the fault array
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
    		
    	
    	ps.close();
    }
}
