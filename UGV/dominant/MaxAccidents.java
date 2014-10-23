/**
 * 
 */
package dominant;

import modeling.COModel;
import modeling.COModelBuilder;
import ec.EvolutionState;
import ec.Individual;
import ec.Problem;
import ec.simple.SimpleFitness;
import ec.simple.SimpleProblemForm;
import ec.vector.IntegerVectorIndividual;

/**
 * @author Xueyi Zou
 *
 */
public class MaxAccidents extends Problem implements SimpleProblemForm {

	/* (non-Javadoc)
	 * @see ec.simple.SimpleProblemForm#evaluate(ec.EvolutionState, ec.Individual, int, int)
	 */
	@Override
	public void evaluate(EvolutionState state, Individual ind,
			int subpopulation, int threadnum) {
		// TODO Auto-generated method stub
		if (ind.evaluated) return;

        if (!(ind instanceof IntegerVectorIndividual))
            state.output.fatal("Whoa!  It's not a IntegerVectorIndividual!!!",null);
        
      
        IntegerVectorIndividual ind2 = (IntegerVectorIndividual)ind;
        int sumAccidents=0;
    	int noObstacles = ind2.genome[0];
		int noCars = ind2.genome[1];
        for(int i=0;i<5; i++)
        {
        	long time = System.nanoTime();
    		COModel simState= new COModel(time, 100, 100, true);    	
    		
    		simState.setNoObstacles(noObstacles);
    		simState.setNoCars(noCars);
    		
    		COModelBuilder sBuilder = new COModelBuilder(simState);
    		sBuilder.generateSimulation(simState.getNoObstacles(),simState.getNoCars());
    		simState.start();		
    		do
    		{
    			if (!simState.schedule.step(simState))
    			{
    				break;
    			}
    		} while(simState.schedule.getSteps()< 1000);
    		
    		
    		sumAccidents += simState.aDetector.getNoAccidents();
        }
        
		float fitness= (float)sumAccidents/5;          
        
        
        if (!(ind2.fitness instanceof SimpleFitness))
            state.output.fatal("Whoa!  It's not a SimpleFitness!!!",null);
        ((SimpleFitness)ind2.fitness).setFitness(state,
            /// ...the fitness...
            fitness,
            ///... is the individual ideal?  Indicate here...
            fitness == Float.POSITIVE_INFINITY);
        ind2.evaluated = true;
        System.out.println("individual result: Obstacles("+noObstacles+"), Cars("+ noCars+"); Accidents[  " + Math.round(fitness) +" ]" );
        

	}

}
