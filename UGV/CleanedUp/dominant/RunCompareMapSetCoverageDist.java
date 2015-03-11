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
public class RunCompareMapSetCoverageDist {

	/**
	 * @param args
	 */
	
    public static void main(String[] args)
    {
    	// Open an overall log file so that we can capture the start and end times of the various loops
    	File infoLog = new File(Constants.outFilePath + "RunCompMapSetCovDistLog.txt");
    	PrintStream ps;
    	
    	try{
    		ps = new PrintStream(new FileOutputStream(infoLog));
    	}
    	catch(FileNotFoundException e)
    	{
    		System.out.print("RunCompMapSetCovDistLog log file not found!");
    		return;
    	}
    	
    	// Log the start time
    	long startTime = java.lang.System.currentTimeMillis();
    	ps.println("Search Based Map Evaluation starting at time: " + startTime + ", " + Utility.timeToString() + ".");

    	// Run the model with the Search-Based seed set
    	CalculateSituationCoverageFromFile.calcSitCovDist(20000, true);

    	// Log the finish time
    	long endTime = java.lang.System.currentTimeMillis();
    	ps.println("Search Based Map Generation ending at time: " + endTime + ", " + Utility.timeToString() + ".");
    	
    	startTime = java.lang.System.currentTimeMillis();
    	ps.println("Random Map Evaluation starting at time: " + startTime + ", " + Utility.timeToString() + ".");
    	
    	// Run the model with the Search-Based seed set
    	CalculateSituationCoverageFromFile.calcSitCovDist(20000, false);

    	endTime = java.lang.System.currentTimeMillis();
    	ps.println("Random Map Generation ending at time: " + endTime + ", " + Utility.timeToString() + ".");
    	
    	ps.close();
    }

}
