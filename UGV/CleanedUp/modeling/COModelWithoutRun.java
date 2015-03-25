package modeling;

import modeling.COModel.targetInfo;
import modeling.Constants.coverageCriteriaInfo;

/**
 * A class for running a simulation without a UI, this is mainly used during the
 * search-based Situation Generation process, or to collect extra statistics about the map
 * in a run that has already been completed, where the model map is created from the
 * two random seeds which define it, and then various metrics are measured on the map
 * and returned as an output string from the start() method.
 * 
 * @author hh940, based on COModelWithUI by Robert Lee
 */
public class COModelWithoutRun
{	
	protected COModelBuilder sBuilder;

	/**
	 * This method constructs the simulation, using the COModelBuilder, and reports a status message to the console.
	 * The simulation is configured to be without a UI, and to have random faults inserted.
	 * @param percentageFaults (double - value of 0..1 (incl) to set % of faults to be injected 
	 *                                   into the model OR index of single fault to be injected)
	 * @param mapNo (int - unique identifier for results files, may include search effort, run index, R/SB differentiation)
	 * @param newInternalSeed (long - for random number generator)
	 */
    public COModelWithoutRun(double percentageFaults, int mapNo, long newInternalSeed) 
    { 
    	sBuilder = new COModelBuilder(new COModel(newInternalSeed, Constants.WorldXVal, Constants.WorldYVal, false, percentageFaults, mapNo, true));
    	System.out.println("COModelWithoutUI is being called!"+ "it's state(model)is: "+ sBuilder.getSim().toString());
    }
    
    /**
     * This method can be called to start the map generation with the supplied external random seed.
     * The simulation is reset, and a new map is generated with these parameters.  This method is 
     * used to retrieve more details about a map (or set of maps) which have already been simulated e.g.
     * in RunSpecificBatchFromFile().
     * @param newExternalSeed (long - new external seed for map generation)
     * @return String ()
     */
	public String start(long newExternalSeed)
	{
		System.out.println("COModelWithoutUI.start is called  "+ sBuilder.sim);
		sBuilder.sim.reset();
		sBuilder.sim.setExternalSeed(newExternalSeed);
		sBuilder.generateSimulation();
		
		// Some extra calculations on the map that has been created, and an output to calling method -
		// NOTE: these metrics could be edited to provide different information about the layout of the
		// constructed map.
		String outString = sBuilder.sim.getJunctionSep();
		outString = outString + ", " + sBuilder.sim.HgetMinTargetCentreSeparation(); // Error checking 1
		outString = outString + ", " + sBuilder.sim.getNoObstacles(); // Error checking 2
		
		// Report to the console that we have completed, and call finish() on the sim
		System.out.println("COModelWithoutUI finished.");
		sBuilder.sim.finish();
		return outString;
	}
	
	/** 
	 * Alternative start method to be used with the search-based situation set generation; for testing
	 * situation coverage (see dominant.SearchBasedMapGeneration class).  Method returns coverageCriteriaInfo
	 * which can be used to guide the search
	 * @param newExternalSeed (long - new external seed for map generation)
	 * @param mapNo (long - unique identifier for results files, may include search effort, run index, R/SB differentiation)
	 * @return coverageCriteriaInfo (contains values of situation coverage metrics derived from the generated map)
	 */
	public coverageCriteriaInfo getCoverageCriteria(long newExternalSeed, long mapNo)
	{
		System.out.println("COModelWithoutUI.start is called  "+ sBuilder.sim);
		sBuilder.sim.reset();
		sBuilder.sim.setExternalSeed(newExternalSeed);
		sBuilder.generateSimulation();
		
		// Some calculations on the map that has been created, and an output to calling method -
		// NOTE: these metrics could be edited to provide different information about the layout of the
		// constructed map in the event that the coverage criteria are altered.
		coverageCriteriaInfo retVal = new coverageCriteriaInfo();
		
		retVal.distTargetToObs = sBuilder.sim.HgetMinTargetObsSeparation();
		retVal.distUGVToTarget = sBuilder.sim.HgetUGVTargetSeparation();
		
		targetInfo myTargetInfo = sBuilder.sim.HgetTargetSeparations(sBuilder.sim.getTargetLoc());
		retVal.distPrevJctToTarget = myTargetInfo.fromPrevJct;
		
		// Report to the console that we have completed, and call finish() on the sim
		System.out.println("COModelWithoutUI finished mapNo = " + mapNo + ".");
		sBuilder.sim.finish();
		return retVal;
	}	
}
