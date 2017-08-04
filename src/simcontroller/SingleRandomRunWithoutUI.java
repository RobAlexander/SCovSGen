package simcontroller;

/**
 * @author rda2 
 */

import java.security.SecureRandom;

import modeling.COModelWithoutUI;

/** Script file to do a single random run with default settings */
public class SingleRandomRunWithoutUI {

	public static void main(String[] args) {
		COModelWithoutUI mod = new COModelWithoutUI(1,0,true); //100% of runs contain at least one random fault; not clear whether they may contain more than one 

		long ExternalSeed = Math.round(new SecureRandom().nextInt());
		mod.runBatch(1, ExternalSeed);

	}
}
