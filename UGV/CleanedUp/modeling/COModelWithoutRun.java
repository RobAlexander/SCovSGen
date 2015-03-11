package modeling;

import modeling.COModel.targetInfo;
import modeling.Constants.coverageCriteriaInfo;

/**
 * A class for running a simulation without a UI
 * 
 * @author 31/7/14 HH, based on COModelWithUI by Robert Lee
 */
public class COModelWithoutRun
{	
	protected COModelBuilder sBuilder;

    public COModelWithoutRun(double percentageFaults, int mapNo, long newInternalSeed) 
    { 
    	sBuilder = new COModelBuilder(new COModel( newInternalSeed, Constants.WorldXVal, Constants.WorldYVal, false, percentageFaults, mapNo, true));
    	System.out.println("COModelWithoutUI is being called!"+ "it's state(model)is: "+ sBuilder.getSim().toString());
    }
         
	public String start(long newExternalSeed, int mapNo)
	{
		System.out.println("COModelWithoutUI.start is called  "+ sBuilder.sim);
		sBuilder.sim.reset();
		sBuilder.sim.setExternalSeed(newExternalSeed);
		
		sBuilder.generateSimulation();
		
		// TODO - Some extra calculations on the map that has been created, and an output to the output file
		String outString = sBuilder.sim.getJunctionSep();
		outString = outString + ", " + sBuilder.sim.HgetMinTargetCentreSeparation(); // Error checking 1
		outString = outString + ", " + sBuilder.sim.getNoObstacles(); // Error checking 2
		
		System.out.println("COModelWithoutUI finished.");
		sBuilder.sim.finish();
		return outString;
	}
	
	// HH 14.1.15 New Start method to be used with the search-based situation set generation; for testing
	// situation coverage (see dominant.SearchBasedMapGeneration class).  Method returns coverageCriteriaInfo
	// which can be used to guide the search
	public coverageCriteriaInfo getCoverageCriteria(long newExternalSeed, long mapNo)
	{
		System.out.println("COModelWithoutUI.start is called  "+ sBuilder.sim);
		sBuilder.sim.reset();
		sBuilder.sim.setExternalSeed(newExternalSeed);
		sBuilder.generateSimulation();
		
		// Return some extra calculations on the map that has been created
		coverageCriteriaInfo retVal = new coverageCriteriaInfo();
		
		retVal.distTargetToObs = sBuilder.sim.HgetMinTargetObsSeparation();
		retVal.distUGVToTarget = sBuilder.sim.HgetUGVTargetSeparation();
		
		targetInfo myTargetInfo = sBuilder.sim.HgetTargetSeparations(sBuilder.sim.getTargetLoc());
		retVal.distPrevJctToTarget = myTargetInfo.fromPrevJct;
		
		System.out.println("COModelWithoutUI finished mapNo = " + mapNo + ".");
		sBuilder.sim.finish();
		return retVal;
	}
	
	
}
