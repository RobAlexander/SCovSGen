package modeling;

import java.security.SecureRandom;


/**
 * A class for running a simulation without a UI
 * 
 * @author 31/7/14 HH, based on COModelWithUI by Robert Lee
 */
public class COModelWithoutUI
{	
	protected COModelBuilder sBuilder; // = new COModelBuilder((COModel) state);
   
	/** HH 27.8.14 - Added argument for percentage faults to support multi-batch runs
    * @param double percentageFaults ()
    * @param long mapNo ()
    * @param boolean inWantRandomFaults ()
    */
    public COModelWithoutUI(double percentageFaults, long mapNo, boolean inWantRandomFaults) 
    { 
    	// HH 3.9.14 - Updated the below call as seemed to be reporting that had a UI, when doesn't
    	sBuilder = new COModelBuilder(new COModel( System.nanoTime(), Constants.WorldXVal, Constants.WorldYVal, false, percentageFaults, mapNo, inWantRandomFaults));
    	System.out.println("COModelWithoutUI is being called!"+ "it's state(model)is: "+ sBuilder.getSim().toString());
    }
 
    /** 
     * HH 22.1.15 - Dummy constructor to get around some initialisation errors in RunComparison.java
     */
    public COModelWithoutUI()
    {
    	// DO nothing
    }
    
    /**
     * This method...
     * @param int noRuns ()
     * @param long newExternalSeed ()
     */
    public void runBatch(int noRuns, long newExternalSeed)
    {
    	for (int i=0; i< noRuns; i++) {
    		start(newExternalSeed);
    	}
    }
    
    /**
     * This method...
     * @param long newExternalSeed ()
     */
	public void start(long newExternalSeed)
	{
		System.out.println("COModelWithoutUI.start is called  "+ sBuilder.sim);
		sBuilder.updateSeed(new SecureRandom().nextInt()); // HH 3.9.14 Updated due to concerns about seed rounding to MAX_VALUE 
		sBuilder.sim.reset();
		
		// HH 1.9.14 - Added ability to pass External Seed from non-UI version of model
		sBuilder.sim.setExternalSeed(newExternalSeed);
		
		sBuilder.generateSimulation();
		sBuilder.sim.start();		
		do
		{
			if (!sBuilder.sim.schedule.step(sBuilder.sim))
			{
				System.out.println("COModelWithoutUI.start finished on its own after "+ sBuilder.sim.schedule.getSteps() + " steps.");
				break;
			}
		} while(sBuilder.sim.schedule.getSteps() < 5500);
		
		System.out.println("COModelWithoutUI finished.");
		sBuilder.sim.finish();
	}
}
