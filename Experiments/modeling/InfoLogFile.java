package modeling;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintStream;


/*
 * Class to provide a simple run logging facility to detect interesting or unusual features of 
 * program runs e.g. when the road network cannot be fully generated. 
 * 
 * NOTE: Based on File I/O from AccidentDetector.java (Xueyi)
 * 
 * @author HH 
 */
public class InfoLogFile {

	private File infoLog = new File(Constants.outFilePath + "InfoLog.txt");
	private COModel sim;
	private PrintStream ps;
	
	public InfoLogFile(){ 
				
		try{
			ps= new PrintStream(new FileOutputStream(infoLog));
		}
		catch(FileNotFoundException e)
		{
			System.out.print("File not found!");
			return;
		}
	}

	/*
	 * Simple log function to allow messages about each run to be printed to the info log file
	 */
	public void addLog(String str)
	{
		ps.println(str); 
	}
	
	/** HH 6/5/14 - Adds header information to file to enable run
	 *  to be reproduced.  An entry in the log will be made for all runs.
	 **/
	public void addHeader(COModel state)
	{
		sim = (COModel)state;
	
		ps.println("*** New Run, Seed = "+ sim.seed() + "; External Seed =" + sim.getExternalSeed() + 
			" *** NoCars = " + sim.noCars + 
			"; CarMaxDeceleration = " + sim.getCarMaxDecceleration() + "; CarMaxSpeed = "
			+ sim.getCarMaxSpeed() + "; CarMaxAcceleration = " + sim.getCarMaxAcceleration()
			+ "; CarMaxTurning = " + sim.getCarMaxTurning() + "; NoObstacles = "
			+ sim.getNoObstacles() + ".");
	}
	
}
