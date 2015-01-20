/**
 * 
 */
package dominant;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintStream;
import modeling.COModelWithoutRun;



/**
 * @author HH 3/11/14
 * 
 * This method has been created to allow us to supply a set of existing experiment runs as a text file, and
 * use the random seeds that define these runs to re-generate the network map.  The map can then be analysed
 * and supplementary measurements taken e.g. junction separation values, and distance between target and the
 * centre of the road. * 
 */
public class RunSpecificBatchFromFile {

	/**
	 * @param args
	 */
	
    public static void main(String[] args)
    {
    	// HH Parameters that we need to read from file
    	double percentageFaults;
    	Long tempLong;
    	long InternalSeed;
    	long ExternalSeed; 
    	int c;
    	
    	FileReader UGVInputFile = null;
		try {
			UGVInputFile = new FileReader("UGVInput.csv");
		} catch (FileNotFoundException e2) {
			// TODO Auto-generated catch block
			e2.printStackTrace();
		}
    	
    	// HH Read the input file and initialise the modelling parameters
    	final BufferedReader UGVInputFileReader= new BufferedReader(UGVInputFile);
    	
    	String inputString = "";
    	String outputString = "";
    	
		try {
			inputString = UGVInputFileReader.readLine();
		} catch (IOException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
    	
		// HH 28.8.14 : NOTE - differences in percentages of faults must be > 1% or files will be overwritten
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
    		outputString = mod.start(ExternalSeed, 99);
    		
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
