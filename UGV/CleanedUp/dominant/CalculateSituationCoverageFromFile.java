package dominant;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import modeling.COModelWithoutRun;
import modeling.Constants;
import modeling.Constants.coverageCriteriaInfo;
import modeling.Utility;

/**
 * @author hh940 HH 15.1.14
 *
 *	This class takes a list of External Random Seeds from an input file and calculates the situation 
 *  coverage achieved by the set.  This enables the set (which would be output from a set of randomly 
 *  generated runs) to be compared to sets of situations which have been generated by search-based 
 *  methods e.g. by SearchBasedMapGeneration.
 */
public class CalculateSituationCoverageFromFile {

	/**
	 * This method...
	 * @param int iterationLimit ()
	 * @return double ()
	 */
	public static double calcSitCov(int iterationLimit) {
		
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
		
		FileReader UGVInputFile = null;
		try {
			UGVInputFile = new FileReader(Constants.outFilePath + "RandomExternalSeeds_" + iterationLimit + ".txt");
		} catch (FileNotFoundException e2) {
			e2.printStackTrace();
		}
    	
    	// HH Read the input file and initialise the modelling parameters
    	final BufferedReader UGVInputFileReader= new BufferedReader(UGVInputFile);
    	
    	String inputString = "";
    	Long tempLong;
    	
		try {
			inputString = UGVInputFileReader.readLine();
		} catch (IOException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		
    	while (inputString != null)
    	{
    		// Split the line up to access the Internal Seed, External Seed and PercentageFaults
    		tempLong = new Long(inputString);
    		ExternalSeed = tempLong.longValue();
    		
    		// Run map generation and analysis to calculate the values for our coverage criteria -
    		// they will be categorised by a static method defined within this class so the category 
    		// boundaries can easily be found and updated if necessary
    		mod = new COModelWithoutRun(percentageFaults, 99, 0); // We don't actually need the internal seed or the mapNo, it's not used during the map setup
    		Res = mod.getCoverageCriteria(ExternalSeed, noIterations);

    		// Categorise the criteria results returned from the simulation
    		c1 = SearchBasedMapGeneration.categorize(Res.distTargetToObs, 1);
    		c2 = SearchBasedMapGeneration.categorize(Res.distUGVToTarget, 2);
    		c3 = SearchBasedMapGeneration.categorize(Res.distPrevJctToTarget, 3);
    		
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
    		}
    		
    		noIterations++; // Increment the seed count
    		
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
    	
		percentageCovered = (double)boxesCovered/(double)Constants.NO_BOXES; // Recalculate the percentage coverage

		// Output some stats to the screen
		System.out.println("Start time = " + startTime);
		System.out.println("End time = " + Utility.timeToString());
		long lEndTime = java.lang.System.currentTimeMillis();
		
		double tempPercentCov = (double)(Math.round(percentageCovered * 100))/(double)100;
		System.out.println("Situation Coverage = " + boxesCovered + "/" + Constants.NO_BOXES + " = " + tempPercentCov + ".");
		
		SearchBasedMapGeneration.matrixToFile(coverageBoxes, "Random", tempPercentCov, boxesCovered, (lEndTime - lStartTime), iterationLimit); // Dump the coverage matrix to file so we can see which boxes were covered
	
		return tempPercentCov;
	}

	/**
	 * This method...
	 * @param int iterationLimit ()
	 * @param boolean isSB ()
	 */
	public static void calcSitCovDist(int iterationLimit, boolean isSB) {
		
		// First task is to set up an array to represent the 'boxes' so that we can store how many maps
		// we have received for a particular combination of criterion categories.  In this initial 
		// case, we are looking at 3 criteria, and each is composed of 6 categories (0-5). 

		int coverageBoxes[][][] = new int[Constants.NO_CATEGORIES][Constants.NO_CATEGORIES][Constants.NO_CATEGORIES];  // Should all be initialised to 0
		int boxesCovered = 0; // This will be incremented every time we cover a *new* box
		
		// Constrain our search to a maximum number of iterations, a time limit, and/or for the search to stop once we
		// have 100% coverage
		long noIterations = 0;
		
		long ExternalSeed;
		int percentageFaults = 0;
		COModelWithoutRun mod;
		coverageCriteriaInfo Res; 
		int c1; // DistanceTargetToObstacle
		int c2; // DistanceUGVToTarget
		int c3;	// DistancePreviousJunctionToTarget
		
		// Before we do anything, store the start time
		long lStartTime = java.lang.System.currentTimeMillis();
		
		String myFilename = Constants.outFilePath + "SelectedExternalSeeds_" + iterationLimit + ".txt";
		if (isSB != true) {
			myFilename = Constants.outFilePath + "RandomExternalSeeds_" + iterationLimit + ".txt";
		}
		
		FileReader UGVInputFile = null;
		try {
			UGVInputFile = new FileReader(myFilename);
		} catch (FileNotFoundException e2) {
			e2.printStackTrace();
		}
    	
    	// HH Read the input file and initialise the modelling parameters
    	final BufferedReader UGVInputFileReader= new BufferedReader(UGVInputFile);
    	
    	String inputString = "";
    	Long tempLong;
    	
		try {
			inputString = UGVInputFileReader.readLine();
		} catch (IOException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		
    	while (inputString != null)
    	{
    		// Split the line up to access the Internal Seed, External Seed and PercentageFaults
    		tempLong = new Long(inputString);
    		ExternalSeed = tempLong.longValue();
    		
    		// Run map generation and analysis to calculate the values for our coverage criteria -
    		// they will be categorised by a static method defined within this class so the category 
    		// boundaries can easily be found and updated if necessary
    		mod = new COModelWithoutRun(percentageFaults, 99, 0); // We don't actually need the internal seed or the mapNo, it's not used during the map setup
    		Res = mod.getCoverageCriteria(ExternalSeed, noIterations);

    		// Categorise the criteria results returned from the simulation
    		c1 = SearchBasedMapGeneration.categorize(Res.distTargetToObs, 1);
    		c2 = SearchBasedMapGeneration.categorize(Res.distUGVToTarget, 2);
    		c3 = SearchBasedMapGeneration.categorize(Res.distPrevJctToTarget, 3);
    		
    		// Increment the coverage count in the box 
    		coverageBoxes[c1][c2][c3]++;
    		
    		noIterations++; // Increment the seed count
    		
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
    	
		long lEndTime = java.lang.System.currentTimeMillis();
		if (isSB != true) {
			SearchBasedMapGeneration.matrixToFile(coverageBoxes, "Random", -1, boxesCovered, (lEndTime - lStartTime), iterationLimit); // Dump the coverage matrix to file so we can see which boxes were covered
		} else {
			SearchBasedMapGeneration.matrixToFile(coverageBoxes, "SearchBased", -1, boxesCovered, (lEndTime - lStartTime), iterationLimit);
		}
	}
}

