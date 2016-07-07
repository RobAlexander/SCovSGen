/**
 * 
 */
package simcontroller;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintStream;
import modeling.COModelWithoutRun;

/**
 * @author HH940
 * 
 * This class has been created to allow us to supply a set of existing experiment runs as a text file, and
 * use the random seeds that define these runs to re-generate the network map.  The map can then be analysed
 * and supplementary measurements taken e.g. junction separation values, and distance between target and the
 * centre of the road.  Note that contrary to the name of the class, the main method does not actually run
 * the simulation once it has generated the map.
 */
public class RunSpecificBatchFromFile {

	/**
	 * This method reads in a named file, with the format: "Internal Seed, External Seed, Percentage Faults," 
	 * on each line.  A COModelWithoutRun is created, using the supplied parameters, and the start method
	 * returns an output string containing certain metrics about the map.  This could be used to return 
	 * Ahead of Time Situation Coverage measures.   Alternatively, after a series of full simulations, for which 
	 * results have been obtained, it could be used to calculate some map measures that were not recorded at the 
	 * time (which can be determined from the map without running the simulation).  To change the metrics which
	 * are returened by the output string, the method(s) in COModelWithoutRun should be altered to give the 
	 * desired behaviour.  After each map has been evaluated, the parameters are re-written to the output file, 
	 * along with the output string.
	 * @param args (String[] - not used)
	 */
    public static void main(String[] args)
    {
    	// Parameters that we need to read from file
    	double percentageFaults;
    	Long tempLong;
    	long InternalSeed;
    	long ExternalSeed; 
    	int c;
    	
    	FileReader UGVInputFile = null;
		try {
			UGVInputFile = new FileReader("UGVInput.csv"); // Name of the input file
		} catch (FileNotFoundException e2) {
			// TODO Auto-generated catch block
			e2.printStackTrace();
		}
    	
    	// Read the input file and initialise the modelling parameters
    	final BufferedReader UGVInputFileReader= new BufferedReader(UGVInputFile);
    	
    	String inputString = "";
    	String outputString = "";
    	
		try {
			inputString = UGVInputFileReader.readLine();
		} catch (IOException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
    	
		// NOTE - differences in percentages of faults must be > 1% or files will be overwritten
		File newOutputs = new File("batchOutputs.txt");
		PrintStream ps;
		
		try{
			ps= new PrintStream(new FileOutputStream(newOutputs));
		}
		catch(FileNotFoundException e)
		{
			System.out.print("batchOuputs.txt file not found!");
			return;
		}
		
    	while (inputString != null)
    	{
    		// Split the line up to access the Internal Seed, External Seed and PercentageFaults
    		c = inputString.indexOf(",");
    		tempLong = new Long(inputString.substring(0, c));
    		InternalSeed = tempLong.longValue();
    		inputString = inputString.substring(c+1); // Promote the remainder of the string
    		
    		c = inputString.indexOf(",");
    		tempLong = new Long(inputString.substring(0, c));
    		ExternalSeed = tempLong.longValue();
    		inputString = inputString.substring(c+1); // Promote the remainder of the string
    		
    		c = inputString.indexOf(",");
    		tempLong = new Long(inputString.substring(0, c));
    		percentageFaults = (double)tempLong.longValue()/100;
    		
    		// Set up the run with these parameters
    		COModelWithoutRun mod = new COModelWithoutRun(percentageFaults, 99, InternalSeed); // 99 'cos We don't need the unique filenames for each run
    		outputString = mod.start(ExternalSeed);
    		
    		ps.println(InternalSeed + ", " + ExternalSeed + ", " + percentageFaults + ", " + outputString);
    		
    		// Get the next line
    		try {
    			inputString = UGVInputFileReader.readLine();
    		} catch (IOException e1) {
    			// TODO Auto-generated catch block
    			e1.printStackTrace();
    		}
    	}    		
    	
    	try {
    		UGVInputFileReader.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} // Close the file   	
    	
    	ps.close(); // Close the file 
    }   
}
