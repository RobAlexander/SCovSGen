package modeling;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintStream;

/*
 * Class to provide a simple run logging facility to detect interesting or unusual features of 
 * program runs e.g. when the road network cannot be fully generated. NOTE that this output file
 * is repeatedly overwritten in the case of a batch run as it is not supplied with a unique 
 * identifier to include in the filename. * 
 * NOTE: Based on File I/O from AccidentDetector.java (Xueyi)
 * @author hh940
 */
public class InfoLogFile {

	private File infoLog = new File(Constants.outFilePath + "InfoLog.txt"); // Specify the name and location of the file
	private COModel sim;
	private PrintStream ps;
	
	/**
	 * Constructor - create a new output stream at the required location/name, and 
	 * catch an exception if the file cannot be found.
	 */
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

	/**
	 * Simple log function to allow messages about each run to be printed to the info log file
	 * @param str (String - string to be printed to the output file as a single line)
	 */
	public void addLog(String str)
	{
		ps.println(str); 
	}
	
	/** 
	 * Add header information to file to enable run to be reproduced, and some basic set-up params to be
	 * reported.  An entry in the log will be made for all runs, although these can be overwritten during a 
	 * batch run. TODO: These parameters may no longer be the most relevant to report, and perhaps should be
	 * revised/updated.
	 * @param state (COModel - access to the model environment) 
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
