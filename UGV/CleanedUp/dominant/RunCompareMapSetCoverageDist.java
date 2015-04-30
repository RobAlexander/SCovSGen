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
public class RunCompareMapSetCoverageDist {

	/**
	 * Evaluate the distribution of coverage of the situation set for the Search Based and
	 * Random map sets.  These results are written as matrices to their respective files, and
	 * a summary file is produced with information about how long the evaluation process took
	 * for each of the map sets. 
	 * @param args (String[] - not used) 
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
    	ps.println("Search Based Map Evaluation ending at time: " + endTime + ", " + Utility.timeToString() + ".");
    	
    	startTime = java.lang.System.currentTimeMillis();
    	ps.println("Random Map Evaluation starting at time: " + startTime + ", " + Utility.timeToString() + ".");
    	
    	// Run the model with the Search-Based seed set
    	CalculateSituationCoverageFromFile.calcSitCovDist(20000, false);

    	endTime = java.lang.System.currentTimeMillis();
    	ps.println("Random Map Evaluation ending at time: " + endTime + ", " + Utility.timeToString() + ".");
    	
    	ps.close();
    }

}
