package modeling;

import java.io.File;
import java.security.SecureRandom;


/**
 * A class for running a simulation without a UI
 * 
 * @author 31/7/14 HH, based on COModelWithUI by Robert Lee
 */
public class COModelWithoutRun
{	
	protected COModelBuilder sBuilder;
   
	// 
    public COModelWithoutRun(double percentageFaults, int mapNo, long newInternalSeed) 
    { 
    	sBuilder = new COModelBuilder(new COModel( newInternalSeed, Constants.WorldXVal, Constants.WorldYVal, false, percentageFaults, mapNo));
    	System.out.println("COModelWithoutUI is being called!"+ "it's state(model)is: "+ sBuilder.getSim().toString());
    }
         
	public String start(long newExternalSeed, int mapNo)
	{
		System.out.println("COModelWithoutUI.start is called  "+ sBuilder.sim);
		//sBuilder.updateSeed(newInternalSeed); 
		sBuilder.sim.reset();
		sBuilder.sim.setExternalSeed(newExternalSeed);
		
		sBuilder.generateSimulation();
		//sBuilder.sim.start(); // TODO Make sure this doesn't actually start it running
		
		// TODO - Some extra calculations on the map that has been created, and an output to the output file
		String outString = sBuilder.sim.getJunctionSep();
		outString = outString + ", " + sBuilder.sim.HgetMinTargetCentreSeparation(); // Error checking 1
		outString = outString + ", " + sBuilder.sim.getNoObstacles(); // Error checking 2
		
		System.out.println("COModelWithoutUI finished.");
		sBuilder.sim.finish();
		return outString;
	}
	
}
