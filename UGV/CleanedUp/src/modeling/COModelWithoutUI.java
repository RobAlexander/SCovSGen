package modeling;

import java.security.SecureRandom;


/**
 * A class for running a simulation without a UI, this allows a batch of simulations to be
 * called from another routine, allowing variation in the internal seed, whilst maintaining the
 * same external seed (or map) 
 * 
 * @author hh940, based on COModelWithUI by Robert Lee
 */
public class COModelWithoutUI
{	
	protected COModelBuilder sBuilder;
   
	/** 
	 * This method constructs a new simulation and COModelBuilder, including an argument to specify
	 * the percentage of faults to insert (or a specific fault to activate)
     * @param percentageFaults (double - value of 0..1 (incl) to set % of faults to be injected 
	 *                                   into the model OR index of single fault to be injected)
     * @param mapNo (long - unique identifier for results files, may include search effort, run index, R/SB differentiation)
     * @param inWantRandomFaults (boolean - true if faults should be activated at random at supplied frequency, false to specify one active fault)
     */
    public COModelWithoutUI(double percentageFaults, long mapNo, boolean inWantRandomFaults) 
    { 
    	sBuilder = new COModelBuilder(new COModel( System.nanoTime(), Constants.WorldXVal, Constants.WorldYVal, false, percentageFaults, mapNo, inWantRandomFaults));
    	System.out.println("COModelWithoutUI is being called!"+ "it's state(model)is: "+ sBuilder.getSim().toString());
    }
 
    /** 
     * Dummy constructor to get around some initialisation errors in RunComparison.java
     */
    public COModelWithoutUI()
    {
    	// DO nothing
    }
    
    /**
     * This method executes a batch run with the same external seed/map - this reuses the
     * COModel object and as a result single log/output files are used for the entire batch.
     * The map is regenerated during the start routine (see below)
     * @param noRuns (int - number of times to repeat the simulation on this map)
     * @param newExternalSeed (long - external seed to use to generate the map)
     */
    public void runBatch(int noRuns, long newExternalSeed)
    {
    	for (int i=0; i<noRuns; i++) {
    		start(newExternalSeed);
    	}
    }
    
    /**
     * This method updates the internal random seed and resets other simulation parameters, it sets
     * the external seed to the supplied parameter, and then generates the corresponding map. The 
     * simulation is then executed in a loop until the simulation terminates on its own, or the number
     * of steps reaches 5500 - note that this loop constraint is unlikely to ever be reached as the
     * accident detector code contains a timeout at about 5000 steps which will cause the simulation 
     * to terminate itself earlier.
     * @param newExternalSeed (long - external random seed to generate the map)
     */
	public void start(long newExternalSeed)
	{
		System.out.println("COModelWithoutUI.start is called  "+ sBuilder.sim);
		sBuilder.updateSeed(new SecureRandom().nextInt()); 
		sBuilder.sim.reset();
		
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
		} while (sBuilder.sim.schedule.getSteps() < 5500);
		
		// Report to the console that we have completed, and call finish() on the sim
		System.out.println("COModelWithoutUI finished.");
		sBuilder.sim.finish();
	}
}
