/**
 * 
 */
package dominant;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.security.SecureRandom;

import modeling.COModelWithoutRun;
import modeling.Constants;
import modeling.Constants.coverageCriteriaInfo;
import modeling.Utility;

/**
 * @author hh940 HH 14.1.14
 *
 *	This method will use a guided search to produce a set (list) of external random seeds: each defining a unique map and initial set-up for 
 *  the UGV simulation.  This method builds the set incrementally by selecting seeds to generate maps which will improve the situation
 *  coverage of the set as a whole.  Candidate seeds are generated at random and each is used to produce a 'virtual map' - measurements are
 *  taken from this map, relating to defined coverage criteria e.g. DistanceTargetToObstacle, DistanceUGVToTarget, DistancePreviousJunctionToTarget
 *  and these are categorised (0-5) to assign the map to a 'box' in the situation space defined by the coverage criteria.  In the case above, this is a 
 *  3D space, defined by the 3 coverage criteria, each of which are divided into 6 categories (0-5) and therefore creating a space of 216 'boxes'.  The
 *  categories are designed to provide a spread across /sensible/ 'input' values for each criteria, and the theory is that by testing within each 
 *  category, we are 'covering' a wider range of possible situations.  As there are likely to be multiple criteria which define a situation, we
 *  want to try and cover each /combination/ of criteria categories (in the example above, this is the combination of 3 criteria).
 */
public class SearchBasedMapGeneration {

	/**
	 * @param args
	 */
	
	public static double generateExternalSeeds(int iterationLimit) {
		
		// First task is to set up an array to represent the 'boxes' so that we can store whether or not
		// we have achieved coverage of a particular combination of criterion categories.  In this initial 
		// case, we are looking at 3 criteria, and each is composed of 6 categories (0-5). [We'll use an
		// int array in case we want to force e.g. 5 situations per box in the future.]

		int coverageBoxes[][][] = new int[Constants.NO_CATEGORIES][Constants.NO_CATEGORIES][Constants.NO_CATEGORIES];  // Should all be initialised to 0
		int boxesCovered = 0; // This will be incremented every time we cover a *new* box
		
		// Constrain our search to a maximum number of iterations, a time limit, and/or for the search to stop once we
		// have 100% coverage
		double percentageCovered = 0;
		long noIterations = 0;
		
		long ExternalSeed;
		int percentageFaults = 0;
		COModelWithoutRun mod;
		coverageCriteriaInfo Res; 
		int c1; // DistanceTargetToObstacle
		int c2; // DistanceUGVToTarget
		int c3;	// DistancePreviousJunctionToTarget
		
		// Before we do anything, store the start time
		String startTime = Utility.timeToString();
		long lStartTime = java.lang.System.currentTimeMillis();
		
		// HH 14.1.15 : Existing copies of this file will be overwritten if the output folder is not empty
		File newOutputs = new File(Constants.outFilePath + "selectedExternalSeeds_" + iterationLimit + ".txt");
		PrintStream ps;
		
		try{
			ps= new PrintStream(new FileOutputStream(newOutputs));
		}
		catch(FileNotFoundException e)
		{
			System.out.print("selectedExternalSeeds_" + iterationLimit + ".txt file not found!");
			return -1;
		}
		
		while ((noIterations < iterationLimit) && percentageCovered < 1)
		{
			// Generate random number to use as our candidate external seed
			ExternalSeed = Math.round(new SecureRandom().nextInt()); // Get a new Map/Initial Configuration
			
			// Run map generation and analysis to calculate the values for our coverage criteria -
			// they will be categorised by a static method defined within this class so the category 
			// boundaries can easily be found and updated if necessary
			mod = new COModelWithoutRun(percentageFaults, 99, 0); // We don't actually need the internal seed or the mapNo, it's not used during the map setup
			Res = mod.getCoverageCriteria(ExternalSeed, noIterations);
			
			// Categorise the criteria results returned from the simulation
			c1 = Math.max(0, categorize(Res.distTargetToObs, 1)); // HH 21.1.15 - Little cheat to prevent -1 being returned and causing exception
			c2 = Math.max(0, categorize(Res.distUGVToTarget, 2)); // HH 21.1.15 - Little cheat to prevent -1 being returned and causing exception
			c3 = Math.max(0, categorize(Res.distPrevJctToTarget, 3)); // HH 21.1.15 - Little cheat to prevent -1 being returned and causing exception
						
			// Test whether the coverage count in the box has already reached the required number of  
			// candidate seeds.  
			if (coverageBoxes[c1][c2][c3] < Constants.REQ_COV_COUNT)
			{
				// If not, increment the count in the coverage box for this candidate seed.
				coverageBoxes[c1][c2][c3]++;
			
				// Check to see if the coverage count has now reached the required number of 
				// candidate seeds.  If so, increment the boxesCovered count. 
				if (coverageBoxes[c1][c2][c3] == Constants.REQ_COV_COUNT)
				{
					boxesCovered++;
				}
			
				// Write the candidate seed to the output file
				ps.println(ExternalSeed);
			}
			
			// Housekeeping
			noIterations++; // Increment loop count
			percentageCovered = (double)boxesCovered/(double)Constants.NO_BOXES; // Recalculate the percentage coverage
		}
		
		ps.close(); // Close the output file
		
		// Output some stats to the screen
		System.out.println("Start time = " + startTime);
		System.out.println("End time = " + Utility.timeToString());
		long lEndTime = java.lang.System.currentTimeMillis();
		
		double tempPercentCov = (double)(Math.round(percentageCovered * 100))/(double)100;
		System.out.println("Situation Coverage = " + boxesCovered + "/" + Constants.NO_BOXES + " = " + tempPercentCov + ".");
		
		matrixToFile(coverageBoxes, "SB", tempPercentCov, boxesCovered, (lEndTime - lStartTime), iterationLimit); // Dump the coverage matrix to file so we can see which boxes were covered
	
		return tempPercentCov;
	}

	// HH 14.1.15 - Convert the supplied coverage criterion to the category range 0-5 as
	// required for determining situation coverage
	public static int categorize(double inCovCriterion, int critIdx) {
		
		int retVal = 0;
		
		// Depending on which criterion we are working with (see critIdx), we
		// take the inCovCriterion measure and manipulate it to return a value
		// in the range 0-5 which allow us to map it to one of the boxes.  
		// NOTE: For consistency, the calculations used here should be the same 
		// as those used in any Excel analysis workbooks.
		switch (critIdx)
		{
			case 1 : // distTargetToObs
			{
				retVal = (int) Math.floor((Math.min(inCovCriterion, 31)-1.01)/5);
				break;
			}
			case 2: // distUGVToTarget
			{
				retVal = (int) Math.floor(inCovCriterion/40);
				break;
			}
			case 3: // distPrevJctToTarget
			{
				retVal = (int) Math.min(Math.floor(inCovCriterion/10), 5);
				break;
			}
		}
				
		return retVal;
	}
	
	// HH 14.1.15 - Dump the supplied coverageMatrix to file
	public static void matrixToFile(int[][][] inMatrix, String inText, double tempPercentCov, int boxesCovered, long duration, int iterationLimit) {
		
		// Open a file for output
		// HH 14.1.15 : Existing copies of this file will be overwritten if the output folder is not empty
		File newOutputs = new File(Constants.outFilePath + "sitCovMatrix_" + inText + "_" + iterationLimit + ".txt");
		PrintStream ps;
		String tempString = "";
		
		try{
			ps= new PrintStream(new FileOutputStream(newOutputs));
		}
		catch(FileNotFoundException e)
		{
			System.out.print(Constants.outFilePath + "sitCovMatrix_" + inText + "_" + iterationLimit + ".txt file not found!");
			return;
		}
		// Loop for each of the C3 categories (tables)
		for (int t = 0; t<Constants.NO_CATEGORIES; t++)
		{
			// Add a table header
			ps.println("Criterion #3 : Category " + t + ".");
			
			// Add column headers
			ps.println("Criterion #2, 0, 1, 2, 3, 4, 5");
			
			// Loop for each of the C2 categories (rows)
			for (int r = 0; r<Constants.NO_CATEGORIES; r++)
			{	
				// Add row header
				tempString = "" + r; 
		
				// Loop for each of the C1 categories (columns)
				for (int c = 0; c<Constants.NO_CATEGORIES; c++)
				{
					tempString+= ", " + inMatrix[c][r][t]; 
				}
				
				ps.println(tempString);
			}
			
			// Add a blank line
			ps.println("");
		}
		
		// HH 15.1.15 Add the situation coverage measurement to the bottom of the file
		ps.println("Situation Coverage = " + boxesCovered + "/" + Constants.NO_BOXES + " = " + tempPercentCov + ".");
		ps.println("Duration of Search = " + duration + ".");
		
		// Close the file
		ps.close(); // Close the output file
		
	}
	
}
