/**
 * 
 */
package dominant;


import ec.*;
import ec.util.*;


/**
 * @author xueyi
 *
 */
public class Simulation {

	/**
	 * @param args
	 */
	public static void main(String[] args) throws Exception
	{
		
		String[] params = new String[]{"-file", "src/dominant/car-obstacle.params"};//D:/MyLibrary/KLive/EclipseWorkspace/Car-Obstacle/src/dominant/
		ParameterDatabase database = Evolve.loadParameterDatabase(params);
		EvolutionState eState= Evolve.initialize(database, 0);
		eState.startFresh();
		int result=EvolutionState.R_NOTDONE;
		while(result == EvolutionState.R_NOTDONE)
		{
			result=eState.evolve();
			System.out.println("run "+" finished :)&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&");
			
		}
		eState.finish(result);
		Evolve.cleanup(eState);		
		
	}

}
