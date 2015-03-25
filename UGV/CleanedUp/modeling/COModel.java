package modeling;
import java.awt.Shape;
import java.awt.geom.Area;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import modeling.Constants.LineType;
import sim.util.*;
import sim.field.continuous.*;
import sim.engine.*;
import sim.field.grid.IntGrid2D;

/**
 * Class to store and control the environment, including support for
 * visualisation and reporting measurements relating to the map configuration, and
 * runtime behaviour.
 */
public class COModel extends SimState
{
	private static final long serialVersionUID = 1L;
	
	// Main collections of objects in the model
	public Bag toSchedule = new Bag(); // Incl. Entities with a step routine: Cars, UGVs
	public Bag allEntities = new Bag(); // Incl. Cars, UGVs, Obstacles and Target
	public Bag cars = new Bag(); // DumbCars/Moving Obstacles
	public Bag obstacles= new Bag(); // ParkedCars/Stationary Obstacles
	public Bag roads = new Bag(); // Roads
	public Bag junctions = new Bag(); // Junctions
	public Bag ugvs = new Bag(); // UGVs
	
	// Store some of the environmental metrics
	public int noObstacles=0;
	public int noCars=0; 
	private int noJunctions = 0;
	
	// Random Seed Parameter which can be passed in from the UI on the 'Model' tab
	// Default value of 0 will indicate that this hasn't been set, and therefore a random seed will be 
	// generated programmatically from the system time.
	public long externalSeed = 0;
		
	// ** Fault instantiation **
	// This array maps from each fault embedded in the code to a flag which indicates whether the fault is active.
	// Ultimately could upgrade this to an int array if we want to express intermittent faults, or have a % level
	// of failure, however a simple on/off should be sufficient as a starting point.
	// The array could be read in from a file, but may be simpler to probabilistically instantiate the array, unless
	// there is a particular reason for wanting to turn 'particular' faults on.  A method is provided to fill the array
	// at random with a given percentage of faults prior to the start of the simulation (e.g. 100%, 0%, 25%, ...) It is 
	// also possible to supply a single fault as an argument to the COModel constructor so that single fault can be turned on. 
	// NOTE: All faults are labelled in the code as "// New Fault #n" and can be found by simple text search.  The numbering
	// n relates to the index in the faultArray which contains the flag to indicate whether the fault is active or not.
	// Every time a new fault is added to the code, Constants.MAX_FAULTS should be incremented.
	private boolean faultArray[] = new boolean[Constants.MAX_FAULTS];  // Flag whether the fault is active or not
	private long faultCalled[] = new long[Constants.MAX_FAULTS]; // Keep a count of how many times each fault has been called during the simulation
	private double percentageFaults = 0;
	private boolean wantRandomFaults = false;
	
	private boolean runningWithUI = false; // Store whether the UI is running
	
    private int newID = 0; // Store which ID number we are up to for entities which are added to the model
    
    // Store dimensions of world model (map)
	private double xDouble;
	private double yDouble;	
	
    public Continuous2D environment; // Allows us to add/query items on the map in continuous space 
	
    // Integer Grid Based Maps to store different layers? of the environment/map
	public IntGrid2D obstacleMap; // Map (layer?) to store the obstacle locations
	private int obstacleMapResolution = 3; //multiplier used to change resolution of obstacles image
	public IntGrid2D terrainMap; // Map (layer?) to store the terrain
	private int terrainMapResolution = 3;
	public IntGrid2D wallMap; // Map (layer?) to store the wall
	private int wallMapResolution = 3;
	public IntGrid2D roadMap; // Map (layer?) to store the roads
	private int roadMapResolution = 5; // Increased from 3 to try and improve the lane markings
	public IntGrid2D junctionMap; // Map (layer?) to store the junctions
	private int junctionMapResolution = 5; // Increased from 3 to try and improve the lane markings
	public IntGrid2D jctApproachMap; // Map (layer?) to store the junction approaches
	private int jctApproachMapResolution = 5; // Increased from 3 to try and improve the lane markings	
	public IntGrid2D roadMarkingMap; // Map (layer?) to store the road markings
	private int roadMarkingMapResolution = 10; // Higher resolution due to narrow lines
	
	/**
	 * An array for storing the Junction-Junction links that have been traversed during the simulation
	 * this is for the purpose of collecting an IN-RUN coverage criterion for Junction Separation distances
	 * @author hh940
	 */
	public class jctPairInfo {
		public Double2D startLoc;
		public Double2D endLoc;
				
		public jctPairInfo ()
		{
			startLoc = new Double2D(-1,-1);
			endLoc = new Double2D(-1,-1);
		}
		
		public jctPairInfo (Double2D inStartLoc, Double2D inEndLoc) {
			startLoc = inStartLoc;
			endLoc = inEndLoc;
		}
	}
	
	private final int JCTARRAYLENGTH = 100;
	private jctPairInfo[] junctionArray[] = new jctPairInfo[JCTARRAYLENGTH][JCTARRAYLENGTH]; // Array of the above to cover all possible jct-jct pairs
	
	/**
	 * A class for storing the distance information about the target location  
	 * relative to adjacent junctions
	 * @author hh940
	 */
	public class targetInfo {
		public double fromPrevJct; // distance in m from previous junction
		public double toNextJct; // distance in m to next junction
		
		public targetInfo()
		{
			fromPrevJct = 0.0;
			toNextJct = 0.0;
		}
	}
	
	/**
	 * A class for storing the start location and bearing of the UGV, used by method
	 * snapToLane.
	 * @author hh940
	 */
	public class initialInfo {
		public initialInfo(Double2D inStartLoc, double inStartBearing) {
			startLoc = inStartLoc;
			startBearing = inStartBearing;
		}
		
		public Double2D startLoc;
		public double startBearing;
	}
	
	/**
	 * A class for storing the return type from the getRandomExit method so that we
	 * can have the junctionId returned separately from the location of the exit WP.
	 * @author hh940
	 */
	public static class jctExitInfo {
		public Double2D exitWP;
		public int jctId;
				
		public jctExitInfo ()
		{
			exitWP = new Double2D(-1,-1);
			jctId = 0;
		}
		
		public jctExitInfo (Double2D inExitWP, int inJctId) {
			exitWP = inExitWP;
			jctId = inJctId;
		}
	}
	
	/**
	 * A class for storing the return type from the getJunctionExit method so that we
	 * can have the target direction returned separately from the location of the exit WP.
	 * @author hh940
	 */
	public static class jctExitDirInfo {
		public Double2D exitWP;
		public int direction;  // Direction in degrees
				
		public jctExitDirInfo ()
		{
			exitWP = new Double2D(-1,-1);
			direction = 0;
		}
		
		public jctExitDirInfo (Double2D inExitWP, int inDir) {
			exitWP = inExitWP;
			direction = inDir;
		}
	}
	
	// These are the values per STEP.  We can make the assumption that there are 5 steps per 'real' second
	// and calculate 'realistic' parameters for maximum speed, acc, decel, etc.
	private double carMaxSpeed = 2.5;  // based on 45km/hr = 12.5m/s
	private double carMaxAcceleration = 0.1; // based on 0 to 45km/hr in 5s; (0 to 2.5m/step in 25 steps)
	private double carMaxDecceleration = 0.15625;  // based on 20m stopping distance at 45km/hr
	private double carMaxTurning = 15;
	
	public CarPerformance carStats;
	
	public AccidentDetector aDetector; // Construct this later so can pass arguments from COModel constructor
	
	public InfoLogFile infoLog = new InfoLogFile(); // Logging file for information (non-accident) messages
		
	/**
	 * Constructor used for setting up a simulation from the COModelBuilder object.  Some of the supplied arguments
	 * are used to enable results files to be constructed with appropriate unique names (at least for any given run
	 * of the model).  Other parameters are used to construct the map, or to configure run-time behaviour if the model
	 * is actually run.  This class can be used to generate a map which can be used to calculate SitCov, but then 
	 * does not need to be run as a simulation.
	 * @param seed (long - for random number generator)
	 * @param x (double - the width of the simulation environment)
	 * @param y (double - the height of the simulation environment)
	 * @param UI (boolean - true if the simulation is being run with a UI, false if it is not)
	 * @param inPercentageFaults (double - value of 0..1 (incl) to set % of faults to be injected 
	 *                                   into the model OR index of single fault to be injected)
	 * @param mapNo (long - unique identifier for results files, may include search effort, run index, R/SB differentiation)
	 * @param inWantRandomFaults (boolean - true if faults should be inserted at random at selected level, 
	 * 									  false to use supplied fault index)
	 */
    public COModel(long seed, double x, double y, boolean UI, double inPercentageFaults, long mapNo, boolean inWantRandomFaults)
    {
    	super(seed);
    	environment = new Continuous2D(1.0, x, y);
		xDouble = x;
		yDouble = y;
		
		aDetector = new AccidentDetector(inPercentageFaults, mapNo); // Construct accident detector, pass %faults to use in file name for batch runs
		
		// Construct the discrete maps at the appropriate size and resolution
		// NOTE: This may not be the most efficient way to represent the map environment, it is inherited from an 
		// earlier model.
		terrainMap = new IntGrid2D((int) (xDouble * terrainMapResolution), (int) (yDouble * terrainMapResolution), 0);
		wallMap = new IntGrid2D((int) (xDouble * wallMapResolution), (int) (yDouble * wallMapResolution), 0);
    	roadMap = new IntGrid2D((int) (xDouble * roadMapResolution), (int) (yDouble * roadMapResolution), 0);
    	junctionMap = new IntGrid2D((int) (xDouble * junctionMapResolution), (int) (yDouble * junctionMapResolution), 0);
    	jctApproachMap = new IntGrid2D((int) (xDouble * jctApproachMapResolution), (int) (yDouble * jctApproachMapResolution), 0);
		roadMarkingMap = new IntGrid2D((int) (xDouble * roadMarkingMapResolution), (int) (yDouble * roadMarkingMapResolution), 0);
    	
		runningWithUI = UI;
		carStats = new CarPerformance(carMaxSpeed, carMaxAcceleration, carMaxDecceleration, carMaxTurning);

		// Initialise the percentageFaults field
		wantRandomFaults = inWantRandomFaults; // Store whether we want to generate faults at random, or whether we are specifying only one seeded fault
		setPercentageFaults(inPercentageFaults); // This is also used to store the active fault number if we do not want random faults
	}
    
    /**
     * Update the percentageFaults field to the one required by this simulation, performing some
     * range checking on the input variable.
     * @param inPerFaults (double - the percentage of faults to instatiate, or the specific fault index if only one fault is to be seeded)
     */
    public void setPercentageFaults(double inPerFaults) {
    	// Check that it is within the required range
    	if ((inPerFaults >= 0 && inPerFaults <= 1.0 && wantRandomFaults == true) || (inPerFaults < Constants.MAX_FAULTS && wantRandomFaults == false)) {
    		percentageFaults = inPerFaults;
    	} else {
    		// The supplied input for percentage faults was not inside the recognised range - this is serious enough to force an abort
    		this.schedule.clear();
    		System.out.println("Terminating before start as percentage faults has been set outside the range [0..1]. "+ ". Game Over!");
    		System.out.println(this.schedule.scheduleComplete());
    		this.kill();
    	}
    }
    
    // Getter/setter to access the faultArray
    public boolean getFault(int idx) { return faultArray[idx]; }
    public void setFault(int idx) {	faultCalled[idx] ++; } // Increment the entry in the fault called array found at the specified index
    
    // Get/set methods for various parameters
	public int getNoJunctions() { return noJunctions; }
	public void setNoJunctions(int inNoJunctions) { noJunctions = inNoJunctions; }  
	
	public double getCarMaxSpeed() { return carMaxSpeed; }
	public void setCarMaxSpeed(double MaxSpeed) { carMaxSpeed = MaxSpeed; }
	
	public double getCarMaxAcceleration() { return carMaxAcceleration;	}
	public void setCarMaxAcceleration(double MaxAcceleration) { carMaxAcceleration = MaxAcceleration; }
	
	public double getCarMaxDecceleration() { return carMaxDecceleration; }
	public void setCarMaxDecceleration(double MaxDecceleration) { carMaxDecceleration = MaxDecceleration; }
	
	public double getCarMaxTurning() { return carMaxTurning; }
	public void setCarMaxTurning(double MaxTurning) { carMaxTurning = MaxTurning; }
	
	public int getNoObstacles() { return this.noObstacles; }
	public void setNoObstacles(int noObstacles) { this.noObstacles = noObstacles; }
	
	public int getNoCars() { return this.noCars; }
	public void setNoCars(int noCars) { this.noCars = noCars; }
	
	public long getExternalSeed() { return this.externalSeed; }
	public void setExternalSeed(long reqSeed) { this.externalSeed = reqSeed; }
	
	/**
	 * This method is used if we actually want to run the simulation for the map that has
	 * been created.  SimState.start is called, the environment is cleared and entities are
	 * loaded and scheduled.  Map representations are built for all the static features on
	 * the map/environment.  Faults, and fault tracking are instantiated (either at random
	 * or for a specific fault) and the accidentDetector log files are populated with header 
	 * information.
	 */
	public void start()
	{
		super.start();	
		environment.clear();
		
		loadEntities();
		scheduleEntities();

		{
			buildObstacleMap(); 
			buildRoadMap();
			buildJunctionMap();
			buildJctApproachMap();
			buildRoadMarkingsMap();
			buildWallMap();
		}
		
		// Set up the faults/fault tracking (do this before adding the header to aDetector so the faultArray can
		// be logged to file.
		faultArray = new boolean[Constants.MAX_FAULTS]; // Clear out the array
		faultCalled = new long[Constants.MAX_FAULTS]; // Clear out the array or results will accumulate!
		
		// Choose random faults, or to allow only one specified fault to be activated
		if (wantRandomFaults == false)
		{
			initFaultArrayWithFault(); // Instantiate the faultArray to set the supplied fault to true (uses private field percentageFaults)
		} else {
			initFaultArray(); // Instantiate the faultArray with the required percentage of faults (private field)
		}
		
		aDetector.addHeader(this); // Add header to accident log file
		aDetector.addString(this, HgetFaultArrayAsString()); // Add the fault array to the accident log file
	}
	
	/**
	 * Overrides the finish() method in SimState to allow any required termination behaviour to complete.
	 * Includes writing the faultCalled array, and a footer to the log file, and resetting the external 
	 * seed to zero if we are running with a UI.
	 **/
	public void finish()
	{
		super.finish();
		
		// Log the faultCalled array to file before writing out the footer information to the file.
		aDetector.addString(this, HgetFaultCalledAsString());
		aDetector.addFooter(this); // Add Footer information to the Accident Log file
		
		// Reset the External Seed to zero if we are running with a UI
		if (runningWithUI)
		{
			setExternalSeed(0);
		}
	}
	
	/**
	 * A method which starts the simulation
	 * @param args (String[] - the arguments for the doLoop method)
	 */
    public void beginSimulation(String[] args)
    {
   		doLoop(COModel.class, args);

    } 
		
	/**
	 * A method which provides a different number each time it is called, this is
	 * used to ensure that different entities are given different IDs
	 * @return int (a unique ID number)
	 */
	public int getNewID()
	{
		int t = newID;
		newID++;
		return t;
	}
	
	/**
	 * Method to provide the location of the target, originally required for logging information
	 * in the event that the TIMEOUT - event occurs.  Loops through the allEntities array and
	 * returns the location of the first TTARGET-type object it finds.  Assumes there will only 
	 * be one target in the simulation.
	 * @return Double2D (coordinates of location of target)
	 */
	public Double2D getTargetLoc()
	{
		for (int i=0; i< allEntities.size(); i++) {
			if (((Entity) allEntities.get(i)).type == Constants.TTARGET) {
				return ((Entity) allEntities.get(i)).getLocation();
			}			
		}
		
		return new Double2D(-1,-1);
	}
			
	/**
	 * A method which adds the entity and location of all of the entities to the simulation environment.
	 */
	public void loadEntities()
	{
		for(int i = 0; i < allEntities.size(); i++)
		{
			environment.setObjectLocation((Entity) allEntities.get(i), ((Entity) allEntities.get(i)).getLocation());
		}	
	}
		
	/**
	 * A method which adds all the entities marked as requiring scheduling to the
	 * schedule for the simulation.  These should be scheduled repeatedly, as 
	 * should the accidentDetector.  Also, sets the Bag of trackedCars in the
	 * accidentDetector so that it contains the set (containing 1) of UGVs.
	 */
	public void scheduleEntities()
	{
		// Loop across all items in toSchedule and add them all to the schedule
		for(int i = 0; i < toSchedule.size(); i++)
		{
			schedule.scheduleRepeating((Entity) toSchedule.get(i));
		}
		
		try 
		{	
			Bag trackedCars = (Bag)this.ugvs.clone();
			aDetector.setTrackedCars(trackedCars);
		}
		
		catch(CloneNotSupportedException e)
		{
			System.out.print("Clone not supported!");
			return;
		}
		
		schedule.scheduleRepeating(aDetector);
	}
		
	/**
	 * A method which resets the variables for the COModel and also clears
	 * the schedule and environment of any entities - to be called between simulations.
	 * NOTE: This method resets the newID counter so should NOT be called during a run.
	 */
	public void reset()
	{
		newID = 0;
		obstacles.clear();
		cars.clear();
		toSchedule.clear();
		allEntities.clear();
		environment.clear();
		terrainMap.setTo(Constants.NORMAL);
		roadMap.setTo(Constants.NOTROAD);
		roads.clear();
		junctions.clear();
		ugvs.clear();
		roadMarkingMap.setTo(Constants.NOPAINT);
		resetJctArray();
	}
	
    /**
     * Return a string which represents the fault array, and can be output to e.g. the Accident Log
     * @return String (string representation of the fault array)
     */
	public String HgetFaultArrayAsString() {
		
		String retString = new String("Fault Array: "); // Start by describing what is being output
		
		// Add the first entry separately then can comma-separate BEFORE each of the other
		// entries, rather than having an extra comma at the end.
		retString += 0 + "=" + faultArray[0];
		
		for (int i = 1; i < faultArray.length; i++) {
			retString += ", " + i + "=" + faultArray[i];
		}
		
		return retString;
	}
	
    /**
     * Return a string which represents the faultsCalled array, and can be output to e.g. the Accident Log
     * @return String (string representation of the faultCalled array)
     */
	public String HgetFaultCalledAsString() {
		
		String retString = new String("Faults Called: "); // Start by describing what is being output
		
		// Add the first entry separately then can comma-separate BEFORE each of the other
		// entries, rather than having an extra comma at the end.
		retString += 0 + "=" + faultCalled[0];
					
		for (int i = 1; i < faultCalled.length; i++) {
			retString += ", " + i + "=" + faultCalled[i];
		}
		
		return retString;
	}
	
    /**
     * Return a string which represents which UGV targets have been found, and can be output to 
     * e.g. the Accident Log.
     * @return String (string representation of whether the target(s) have been found)
     */
	public String HgetUGVTargetSuccess() {
		
		String retString = new String("Targets Found: "); // Start by describing what is being output
		
		for (int i = 0; i < ugvs.size(); i++) {
			retString += i + "=" + ((UGV)ugvs.get(i)).getTargetFound() + ", ";
		}
		
		return retString;
	}	

    /**
     * Construct the fault array by activating only the single fault specified in the percentageFaults
     * field (because wantRandomFaults is false).
     */
	private void initFaultArrayWithFault() {
			
		int faultIdx = (int) percentageFaults;
		
		if (faultIdx < Constants.MAX_FAULTS)
		{
			faultArray[faultIdx] = true; // set the fault to 'active' in the array			
		}
	}
	
    /**
     * Construct the fault array by initialising faults at random at a rate specified in the percentageFaults
     * field (because wantRandomFaults is true).  Selects indices into the fault array at random, and if they
     * are false, sets them to true; if true, select another at random - continues until the required 
     * percentage of faults are set to true.
     */
	private void initFaultArray() {

		// Either, loop for the appropriate number of faults (%Faults * TotalFaults) and ensure that choose 
		// a new empty slot each time (at random) - need an internal while loop so we get a repeat if we choose
		// a slot that has already been set to true. 
		
		int noFaultsRequired = 0;
		int noFaultsSet = 0;
		int tempIdx = 0; 
		
		noFaultsRequired = (int) Math.round(Constants.MAX_FAULTS * percentageFaults);
		
		while (noFaultsSet < noFaultsRequired) { 
		
			// Generate an index into the array at random
			tempIdx = (int)Math.floor(random.nextDouble() * Constants.MAX_FAULTS);
			
			while (faultArray[tempIdx] == true) {
				// Choose a new index
				tempIdx = (int)Math.floor(random.nextDouble() * Constants.MAX_FAULTS);			
			}
			
			// So we must have a false entry which we can set to true
			faultArray[tempIdx] = true; // set the fault to 'active' in the array
			noFaultsSet ++; // increase the total faults found
		}
	}
	
	/**
	 * A method which creates a discrete map of where obstacles are in the environment
	 * which can be used for outputting a visual representation of the obstacles
	 * when the simulation is run with a UI
	 */
	private void buildObstacleMap()
	{
		Bag all = environment.allObjects;
		Bag obstacles = new Bag();
		Double2D coordinate;
		int xInt = (int) (xDouble * obstacleMapResolution);
		int yInt = (int) (yDouble * obstacleMapResolution);
		obstacleMap = new IntGrid2D(xInt, yInt, 0);
		
		for (int i = 0; i < all.size(); i++)
		{
			if (((Entity) (all.get(i))).getType() == Constants.TPARKEDCAR)
			{
				obstacles.add(all.get(i));
			}
		}
		
		// Loop through all 'squares' in the obstacleMap grid and check to see whether they overlap with an obstacle
		// (ParkedCar) - if they do, set the map field to 1.
		for (int i = 0; i < xInt; i++) {
			for (int j = 0; j < yInt; j++)
			{
				// Adjust i/j values back to appropriate coordinate scale for occupancy check
				coordinate = new Double2D(((double) i) / obstacleMapResolution, ((double) j) / obstacleMapResolution);
				
				if (obstacleAtPoint(coordinate, obstacles))
				{
					obstacleMap.field[i][j] = 1;
				}				
			}
		}
	}
	
	/**
	 * A method which 'draws' the boundary of the map into the wallMap grid.
	 */
	public void buildWallMap()
	{
		int xInt = (int) (xDouble * terrainMapResolution);
		int yInt = (int) (yDouble * terrainMapResolution);
		
		for (int i = 0; i < yInt; i++) 
		{
			wallMap.field[0][i]=1;
			wallMap.field[xInt-1][i]=1;			
		}
		
		for (int j = 0; j < xInt; j++) 
		{	
			wallMap.field[j][0]=1;
			wallMap.field[j][yInt-1]=1;			
		}
	}
		
	/**
	 * Initialise the RoadMap representation.
	 */
	public void buildRoadMap()
	{
		int xInt = (int) (xDouble * roadMapResolution);
		int yInt = (int) (yDouble * roadMapResolution);
		Double2D coordinate;
		
		// Loop through all 'squares' in the obstacleMap grid and check to see whether they overlap with a road
		// - if they do, set the map field to either SINGLEONEWAY or NOTROAD.
		for (int i = 0; i < xInt; i++) {
			for (int j = 0; j < yInt; j++)
			{
				// Adjust i/j values back to appropriate coordinate scale for occupancy check
				coordinate = new Double2D(((double) i) / roadMapResolution, ((double) j) / roadMapResolution);
				
				if (roadAtPoint(coordinate, roads))
				{
					roadMap.field[i][j] = Constants.SINGLEONEWAY;
				} else {
					roadMap.field[i][j] = Constants.NOTROAD;
				}	
			}
		}
	}
	
	/**
	 * Initialise the JunctionMap representation.
	 */
	public void buildJunctionMap()
	{
		int xInt = (int) (xDouble * junctionMapResolution);
		int yInt = (int) (yDouble * junctionMapResolution);
		Double2D coordinate;
		
		// Loop through all 'squares' in the obstacleMap grid and check to see whether they overlap with a road
		// - if they do, set the map field to the junction type.
		for (int i = 0; i < xInt; i++) {
			for (int j = 0; j < yInt; j++)
			{
				// Adjust i/j values back to appropriate coordinate scale for occupancy check
				coordinate = new Double2D(((double) i) / junctionMapResolution, ((double) j) / junctionMapResolution);
				
				junctionMap.field[i][j] = junctionTypeAtPoint(coordinate, junctions);
			}
		}
	}

	/**
	 * Initialise the JunctionApproachMap representation.
	 */
	public void buildJctApproachMap()
	{
		int xInt = (int) (xDouble * jctApproachMapResolution);
		int yInt = (int) (yDouble * jctApproachMapResolution);
		Double2D coordinate;
		
		// Loop through all 'squares' in the obstacleMap grid and check to see whether they overlap with a road
		// - if they do, set to 1, else 0.
		for (int i = 0; i < xInt; i++) {
			for (int j = 0; j < yInt; j++)
			{
				// Adjust i/j values back to appropriate coordinate scale for occupancy check
				coordinate = new Double2D(((double) i) / jctApproachMapResolution, ((double) j) / jctApproachMapResolution);
				
				if (junctionAppAtPoint(coordinate, junctions)) {
					jctApproachMap.field[i][j] = 1;
				} else {
					jctApproachMap.field[i][j] = 0;
				}
			}
		}
	}
	
	/**
	 * Initialise the RoadMarkingsMap representation - note that this has a much higher resolution than
	 * other Int2DGrid and therefore takes up a substantial amount of memory - there are probably more 
	 * efficient ways to implement this.
	 */
	public void buildRoadMarkingsMap()
	{
		int xInt = (int) (xDouble * roadMarkingMapResolution);
		int yInt = (int) (yDouble * roadMarkingMapResolution);
		Double2D coordinate;
		
		// Loop through all 'squares' in the obstacleMap grid and check to see whether they overlap with a road
		// - if they do, set to WHITEPAINT, WHITERPAINT or NOPAINT.
		for (int i = 0; i < xInt; i++) {
			for (int j = 0; j < yInt; j++)
			{
				// Adjust i/j values back to appropriate coordinate scale for occupancy check
				coordinate = new Double2D(((double) i) / roadMarkingMapResolution, ((double) j) / roadMarkingMapResolution);
				// Loop over all coordinates and check if there are any road markings there
				roadMarkingMap.field[i][j] = roadMarkingAtPoint(coordinate, roads);

				// TODO - in the future, we may want to suspend line drawing through junctions 
			}
		}
	}
	
	/**
	 * A method which returns a true or false value depending on if an obstacle is
	 * at the coordinate provided
	 * @param coord (Double2D - the coordinate to check)
	 * @param obstacles (Bag - the obstacles to be checked)
	 * @return boolean (true if an obstacle is found at the supplied location)
	 */
	public boolean obstacleAtPoint(Double2D coord, Bag obstacles)
	{
		// Loop through the provided bag of obstacles and return true if the provided
		// coordinates overlap with one of the obstacles.
		// NOTE: This relies on the Obstacle child type overriding inShape
		for (int i = 0; i < obstacles.size(); i++)
		{
			if (((Obstacle) (obstacles.get(i))).inShape(coord)) 
			{
				return true;
			}
		}
		
		return false; // No overlap detected so false should be returned
	}
	
	/**
	 * A method which returns a true or false value depending on whether an obstacle is
	 * too close to the coordinate provided - too close is defined as an x AND y separation
	 * of less than Constants.OBSTACLE_LENGTH when calculated as absolute displacement in
	 * each direction (if the locations are in the same lane) or less than 
	 * (Constants.OBSTACLE_LENGTH + Constants.OBSTACLE_BUFFER) * 1.5 (if the locations are in
	 * different lanes on the same road).  The supplied coordinate is the tentative location 
	 * of a new obstacle to be inserted, and is compared to the location of all existing obstacles.  
	 * As a final check, we only reject an obstacle if it is on the same road as the supplied coordinate.
	 * @param coord (Double2D - the coordinate to check)
	 * @param obstacles (Bag - the obstacles to be checked)
	 * @param roads (Bag - used to ensure the coordinate and obstacle are on the same road)
	 * @return boolean (true if the location is too close to the obstacl)
	 */
	public boolean obstacleNearPoint(Double2D coord, Bag obstacles, Bag roads, COModel sim)
	{
		Double2D inCoord = snapToKerb(coord.x, coord.y); // Move supplied coordinate to sensible lane position
		
		// A bit of error checking on the return value
		if (inCoord.x == -1)
		{
			sim.infoLog.addLog("Problem testing for obstacle near point at: (" + coord.x + "," + coord.y + ").");
			return true; // True will prevent a ParkedCar being added at this location
		}
		
		Double2D testLoc;
		Double testDistance = Constants.OBSTACLE_LENGTH;
		
		// Loop through all supplied obstacles, retrieving the obstacle location
		for (int i = 0; i < obstacles.size(); i++)
		{
			testLoc = ((Obstacle) (obstacles.get(i))).getLocation();
			
			// When we are comparing obstacles that are in different lanes we need to ensure a
			// separation of 1.5 * Constants.OBSTACLE_LENGTH so that it is possible for the UGV to execute an overtake
			if (testLoc.x == inCoord.x || testLoc.y == inCoord.y) {
				testDistance = Constants.OBSTACLE_LENGTH; // Same lane so can be as close as OBSTACLE_LENGTH
			} else {
				testDistance = (Constants.OBSTACLE_LENGTH + Constants.OBSTACLE_BUFFER) * 1.5; // Other lane
			}
			
			// Do the comparison.  Check absolute displacement in both directions, and then make sure
			// the the two locations are on the same road
			if (Math.abs(testLoc.x - inCoord.x) < testDistance && Math.abs(testLoc.y - inCoord.y) < testDistance) 
			{
				// Check the coordinates are on the same road.
				// NOTE: this is a bit of a cheat as we can't query the road for its
				// identifier, however this method is only used when generating sensible 
				// map configurations, and does not form part of the UGV control software.
				if (getRoadIdAtPoint(testLoc, roads) == getRoadIdAtPoint(inCoord, roads)) {
					return true;
				}
			}
		}
		
		// No overlap has been detected so false should be returned
		return false;
	}	
	
	/**
	 * Method to check whether the supplied location overlaps with a road.
	 * @param coord (Double2D - location to check for overlap with road)
	 * @param roads (Bag - of roads to check for overlap with supplied coordinate)
	 * @return boolean (true if the supplied location lies on a road, false otherwise)
	 */
	public boolean roadAtPoint(Double2D coord, Bag roads)
	{
		// Loop through all supplied roads checking for overlap
		for (int i = 0; i < roads.size(); i++)
		{
			if (((Road) (roads.get(i))).inShape(coord))
			{
				return true;
			}
		}
		
		// No overlap has been detected so false should be returned
		return false;
	}

	/**
	 * Method to return the id number of the road a coordinate is located on 
	 * NOTE: it would be a cheat for the UGV to use this method, however it is only used
	 * when adding obstacles to the simulation map during initial configuration to ensure
	 * a sensible map is generated.  
	 * @param coord (Double2D - coordinates of location where we want the roadId)
	 * @param roads (Bag - of roads we want to check against the location)
	 * @return int (Id of road the point is on, or -1 if point not found on road)
	 */
	public int getRoadIdAtPoint(Double2D coord, Bag roads)
	{
		// Loop through all the supplied roads checking for overlap
		for (int i = 0; i < roads.size(); i++)
		{
			if (((Road) (roads.get(i))).inShape(coord)) 
			{
				return ((Road) (roads.get(i))).getID();
			}
		}
		
		// This should not be reached as we should only call this after we have put an obstacle on the road
		return -1; // No overlap has been detected so false should be returned
	}

	/**
	 * Method to return the lane direction (N/E or S/W) of the road a coordinate is located on.
	 * NOTE: This method is used in collision detection, it would be a cheat for the UGV to use this
	 * method in its control software.
	 * @param coord (Double2D - location at which we want to know the lane direction)
	 * @param roads (Bag - Bag of roads to check for intersection with supplied location)
	 * @return int (returns 1 for N or E; 2 for S or W)
	 */
	public int getLaneDirAtPoint(Double2D coord, Bag roads)
	{
		for (int i = 0; i < roads.size(); i++)
		{
			// Loop through all the supplied roads checking for overlap
			if (((Road) (roads.get(i))).inShape(coord)) 
			{
				return ((Road) (roads.get(i))).getLane(coord);
			}
		}
		
		// This should not be reached as we should only call this method for locations of 
		// vehicles, and these should be on the road
		return -1; // No overlap has been detected so false should be returned
	}
	
	/**
	 * Method to check for roads intersecting with each other.  This is used while the simulation map is
	 * being constructed.  NOTE: This relies on the Bag not rearranging indices spontaneously
	 * @param newRoad (Rectangle2D.Double - proposed rectangle extents of new road to check against existing roads)
	 * @param roads (Bag - existing roads on the simulation map)
	 * @param currentRoad (int - the index of the currentRoad in the roads Bag)
	 * @return boolean (return true if there is a road overlapping with the supplied newRoad)
	 */
	public boolean roadAtPoint(Rectangle2D.Double newRoad, Bag roads, int currentRoad)
	{
		for (int i = 0; i < roads.size(); i++)
		{
			// Loop for all of the roads (except the one we are creating the junction with) 
			// to check if the provided point is in it
			if (((Road) (roads.get(i))).inShape(newRoad) && (i != currentRoad))
			{
				return true;
			}
		}
		
		// No overlap has been detected so false should be returned
		return false;
	}
	
	/**
	 * Method to check whether there is a road marking at the provided location.  This method
	 * is used when constructing the roadMarkingMap.  
	 * @param coord (Double2D - the location we want to check for road markings)
	 * @param roads (Bag - access to the Roads and their Road Markings)
	 * @return int (the type of road marking found at the location)
	 */
	public int roadMarkingAtPoint(Double2D coord, Bag roads)
	{
		// This method is called for each point on the map, and so each point is compared with all
		// roads and all possible line locations (nearside, centre, offside) to see if it falls on 
		// any of them.  This method will return as soon as it finds one instance of 'white paint'
		// it will not continue the search.
		for (int i = 0; i < roads.size(); i++)
		{
			//for all of the roads check if the provided point is on any of its line markings
			if ((((Road) (roads.get(i))).getLine(LineType.NESIDE)).contains(coord.x, coord.y)) {
				return Constants.WHITEPAINT;
			}
			
			if ((((Road) (roads.get(i))).getLine(LineType.CENTRE)).contains(coord.x, coord.y)) {
				return Constants.WHITERPAINT; // Use this alternative line type as an analogy to the
											  // different markings that might be present on the road
			} 
			
			if ((((Road) (roads.get(i))).getLine(LineType.SWSIDE)).contains(coord.x, coord.y)) {
				return Constants.WHITEPAINT;
			} 
		}
		
		// No overlap has been detected so 'false' should be returned
		return Constants.NOPAINT;
	}
	
	/**
	 * Method to check for Cars intersecting with roadMarkings.  
	 * @param inShape (Shape - the shape we want to test for intersection with the road markings)
	 * @param roads (Bag - access to the Roads and Road Markings)
	 * @param inLineType (LineType - the type of line we are testing against: SESIDE, NESIDE, CENTRE)
	 * @return boolean (returns true if supplied location intersects with supplied road marking type)
	 */
	public boolean roadMarkingAtShape(Shape inShape, Bag roads, LineType inLineType)
	{
		// This method is called for each point on the map, and so each point is compared with all
		// roads and chosen line location (SESIDE, NESIDE, CENTRE) to test for intersection 
		// This method will return as soon as it finds one intersection.
		for (int i = 0; i < roads.size(); i++)
		{
			if (inShape.intersects(((Road) (roads.get(i))).getLine(inLineType)))
			{
				return true; // Intersection detected
			}		
		}
		return false; // No intersection detected
	}
	
	/**
	 * Method to work out the closest 'in lane' position for this vehicle to be initialised.  Method attempts
	 * to intersect with lane edge markings in each compass direction.  'Hits' should be detected in two of these
	 * if the coords supplied are on the road.  The return value is located on the line between these two 'hits'
	 * (intersections) and represents the in-lane driving position for the lane which the point is in.  This driving 
	 * position is the midpoint between the lane edge marking and the centre lane marking.  This method also returns
	 * the lane compass direction at the point as a bearing in degrees.
	 * @param x (double - x coordinate of approx location where we want to add the vehicle)
	 * @param y (double - y coordinate of approx location where we want to add the vehcile)
	 * @return initialInfo (in lane position closest to supplied location and driving direction for that location)
	 */
	public initialInfo snapToLane(double x, double y)
	{
		// Create a cross that is centred at x,y and find the closest intersection with either offside or nearside 
		// road markings and use this to choose a vehicle location that is roadWidth/4 away from the intersection
		Line2D.Double xLine = new Line2D.Double(x-Road.roadWidth, y, x+Road.roadWidth, y);
		Line2D.Double yLine = new Line2D.Double(x, y-Road.roadWidth, x, y+Road.roadWidth);
		Line2D.Double lineNE = new Line2D.Double();
		Line2D.Double lineSW = new Line2D.Double();
		
		Double2D nearsideIntersect;
		Double2D offsideIntersect;
		double nearsideBearing = 400;
		double offsideBearing = 400;
		boolean found = false;
		
		Double2D nearestIntersect = new Double2D(0,0);
		double nearestStartBearing = 400; // NOTE - This is an invalid measure, so we can test for it later
		
		// Allow for a small offset on either side of the 'thin lines' to allow for the assumed
		// width of the line, and the offset between the kerb and the line.
		double lineBuffer = Constants.ROADEDGEOFFSET + Constants.ROADEDGINGWIDTH;
		
		// The original algorithm allowed intersections with roads that the vehicle was
		// not actually travelling on (i.e. if close to a cross-roads), but we can't add an onRoad check
		// as that would not be appropriate for the UGV, it would need to use the sensor return values from
		// the road marking detection to 'work out' which road markings are relevant for the road it is on.
		// Hypothesis: If intersections are detected with *both* of the road markings on a road, and the 
		// location of the vehicle lies between them then everything is on the same road, and we can just use
		// whichever of the intersections is closest to calculate the lane we want to be in
		
		// Loop for all roads and outside line locations (NESIDE, SWSIDE) to see if either line intersects
		// with them.  This method will return as soon as it finds an intersection with both lines.
		// NOTE: The use of nearside and offside in this method are purely historical and in some cases
		// are inaccurate - here, nearside is synonymous with NESIDE and offside is synonymous with SWSIDE.
		for (int i = 0; i < roads.size(); i++)
		{
			// reset nearestIntersect params
			nearsideIntersect = new Double2D(0,0);
			nearsideBearing = 400;
			found = false;
			
			//check the NESIDE lane on this road against the xLine, and the yLine		
			lineNE = ((Road) (roads.get(i))).getThinLine(LineType.NESIDE);
			if (lineNE.intersectsLine(xLine)) {
				// As we know they intersect, and we know xLine is the horizontal line, that gives 
				// us the y, and therefore the other line provides the x.  			
				nearsideIntersect = new Double2D(lineNE.x1, xLine.y1);
				nearsideBearing = 180; // travelling North
			} else if (lineNE.intersectsLine(yLine)){
				// As we know they intersect, and we know yLine is the vertical line, that gives 
				// us the x, and therefore the other line provides the y.  			
				nearsideIntersect = new Double2D(yLine.x1, lineNE.y1);	
				nearsideBearing = 90; // travelling East
			}
						
			// reset offsideIntersect params
			offsideIntersect = new Double2D(0,0);
			offsideBearing = 400;
			
			// check the SWSIDE lane on this road against the xLine, and the yLine
			lineSW = ((Road) (roads.get(i))).getThinLine(LineType.SWSIDE);
			if (lineSW.intersectsLine(xLine)) {
				// As we know they intersect, and we know xLine is the horizontal line, that gives 
				// us the y, and therefore the other line provides the x.  			
				offsideIntersect = new Double2D(lineSW.x1, xLine.y1);
				offsideBearing = 0; // travelling South
			} else if (lineSW.intersectsLine(yLine)){
				// As we know they intersect, and we know yLine is the vertical line, that gives 
				// us the x, and therefore the other line provides the y.  			
				offsideIntersect = new Double2D(yLine.x1, lineSW.y1);
				offsideBearing = 270; // travelling West			
			}
								
			// If an intersection has occurred with both of the road marking lines, see
			// if the vehicle is located inbetween so we can exit the method
			if (!(offsideIntersect.x == 0 && offsideIntersect.y == 0) && !(nearsideIntersect.x == 0 && nearsideIntersect.y == 0))
			{
				// Add a buffer around each of the lines when testing to account for the distance between
				// the line and the kerb (see above)
				if (offsideIntersect.x == nearsideIntersect.x)
				{
					// Intersections are occurring with yLine (vertical) so we need to compare our y params
					if ((offsideIntersect.y - lineBuffer < y && nearsideIntersect.y + lineBuffer > y) || 
							(offsideIntersect.y + lineBuffer > y && nearsideIntersect.y - lineBuffer < y))
					{
						found = true;
					}
				} else {
					// Intersections are occurring with xLine (horizontal) so we need to compare our x params
					if ((offsideIntersect.x - lineBuffer < x && nearsideIntersect.x + lineBuffer > x) || 
							(offsideIntersect.x + lineBuffer > x && nearsideIntersect.x - lineBuffer < x))
					{
						found = true;
					}
				}
				
				// Update nearestIntersect/nearestStartBearing if necessary (check nearside first and then offside)
				if (found == true)
				{
					if ((nearsideIntersect.distance(x,y) < nearestIntersect.distance(x,y) && !(nearsideIntersect.x == 0 && nearsideIntersect.y == 0)) || (nearestIntersect.x == 0 && nearestIntersect.y == 0)) {
						nearestIntersect = new Double2D(nearsideIntersect.x, nearsideIntersect.y);
						nearestStartBearing = nearsideBearing; // overwrite with matching value
					}

					if ((offsideIntersect.distance(x,y) < nearestIntersect.distance(x,y) && !(offsideIntersect.x == 0 && offsideIntersect.y == 0)) || (nearestIntersect.x == 0 && nearestIntersect.y == 0)) {
						nearestIntersect = new Double2D(offsideIntersect.x, offsideIntersect.y);
						nearestStartBearing = offsideBearing; // overwrite with matching value
					}
					
					break; // don't need to keep looking
				}
			}
		}
				
		// Use the bearing to work out which direction to offset in, in order to work out the appropriate starting point
		// Updated as per calcRMOffset so that compensate for roadmarking offset and width when determining offset,
		// will make vehicle more central to lane
		double offset = Road.roadWidth/4 - Constants.ROADEDGEOFFSET - Constants.ROADEDGINGWIDTH;
		
		switch ((int) nearestStartBearing) {
			case 0 : nearestIntersect = new Double2D(nearestIntersect.x - offset, nearestIntersect.y); // SB
				break;
			case 90 : nearestIntersect = new Double2D(nearestIntersect.x, nearestIntersect.y + offset); // EB
				break;
			case 180 : nearestIntersect = new Double2D(nearestIntersect.x + offset, nearestIntersect.y); // NB
				break;
			case 270 : nearestIntersect = new Double2D(nearestIntersect.x, nearestIntersect.y - offset); // WB
				break;
		}
		
		initialInfo retVal = new initialInfo(nearestIntersect, nearestStartBearing);
		
		return retVal;
	}	
	
	/**
	 * Method to work out the closest 'by kerb' position for this vehicle to be initialised - this is used
	 * when placing ParkedCar obstacles into the simulation.  Similar method of operation to snapToLane, but
	 * no need to return a bearing, and offset into lane is just by OBSTACLE_WIDTH/2 from the line.
	 * @param x (double - x coordinate of approx location where we want to add the ParkedCar)
	 * @param y (double - y coordinate of approx location where we want to add the ParkedCar)
	 * @return Double2D (the location where the ParkedCar should be added, or (-1,-1) if no location was found)
	 */
	public Double2D snapToKerb(double x, double y)
	{
		// Create a cross that is centred at x,y and find the closest intersection with either offside or nearside 
		// road markings and use this to choose a vehicle location that is OBSTACLE_WIDTH/2 away from the intersection
		Line2D.Double xLine = new Line2D.Double(x-Road.roadWidth, y, x+Road.roadWidth, y);
		Line2D.Double yLine = new Line2D.Double(x, y-Road.roadWidth, x, y+Road.roadWidth);
		Line2D.Double lineRM = new Line2D.Double();
		
		Double2D nearsideIntersect;
		Double2D offsideIntersect;
		double tempBearing = 400;
		
		Double2D nearestIntersect = new Double2D(0,0);
		double nearestStartBearing = 400; // NOTE - This is an invalid measure, so we can test for it later
		
		// Loop for all roads and outside line locations (NESIDE, SWSIDE) to see if either line intersects
		// with them.  This method will return as soon as it finds an intersection with both lines.
		// NOTE: The use of nearside and offside in this method are purely historical and in some cases
		// are inaccurate - here, nearside is synonymous with NESIDE and offside is synonymous with SWSIDE.
		for (int i = 0; i < roads.size(); i++)
		{
			// reset nearestIntersect params
			nearsideIntersect = new Double2D(0,0);
			tempBearing = 400;
			
			//check the NE lanes on this road against the xLine, and the yLine		
			lineRM = ((Road) (roads.get(i))).getThinLine(LineType.NESIDE);
			if (lineRM.intersectsLine(xLine)) {
				// As we know they intersect, and we know xLine is the horizontal line, that gives 
				// us the y, and therefore the other line provides the x.  			
				nearsideIntersect = new Double2D(lineRM.x1, xLine.y1);
				tempBearing = 180; // travelling North
			} else if (lineRM.intersectsLine(yLine)){
				// As we know they intersect, and we know yLine is the vertical line, that gives 
				// us the x, and therefore the other line provides the y.  			
				nearsideIntersect = new Double2D(yLine.x1, lineRM.y1);	
				tempBearing = 90; // travelling East
			}
			
			if ((nearsideIntersect.distance(x,y) < nearestIntersect.distance(x,y) && !(nearsideIntersect.x == 0 && nearsideIntersect.y == 0)) || (nearestIntersect.x == 0 && nearestIntersect.y == 0)) {
				nearestIntersect = new Double2D(nearsideIntersect.x, nearsideIntersect.y);
				nearestStartBearing = tempBearing; // overwrite with matching value
			}
			
			// reset offsideIntersect params
			offsideIntersect = new Double2D(0,0);
			tempBearing = 400;
			
			// check the offside lane on this road against the xLine, and the yLine
			lineRM = ((Road) (roads.get(i))).getThinLine(LineType.SWSIDE);
			if (lineRM.intersectsLine(xLine)) {
				// As we know they intersect, and we know xLine is the horizontal line, that gives 
				// us the y, and therefore the other line provides the x.  			
				offsideIntersect = new Double2D(lineRM.x1, xLine.y1);
				tempBearing = 0; // travelling South
			} else if (lineRM.intersectsLine(yLine)){
				// As we know they intersect, and we know yLine is the vertical line, that gives 
				// us the x, and therefore the other line provides the y.  				
				offsideIntersect = new Double2D(yLine.x1, lineRM.y1);
				tempBearing = 270; // travelling West			
			}
						
			if ((offsideIntersect.distance(x,y) < nearestIntersect.distance(x,y) && !(offsideIntersect.x == 0 && offsideIntersect.y == 0)) || (nearestIntersect.x == 0 && nearestIntersect.y == 0)) {
				nearestIntersect = new Double2D(offsideIntersect.x, offsideIntersect.y);
				nearestStartBearing = tempBearing; // overwrite with matching value
			}
		}
		
		// Use the bearing to work out which direction to offset in, in order to work out the appropriate starting point
		switch ((int) nearestStartBearing) {
			case 0 : nearestIntersect = new Double2D(nearestIntersect.x - (Constants.OBSTACLE_WIDTH/2), nearestIntersect.y);
				break; 
			case 90 : nearestIntersect = new Double2D(nearestIntersect.x, nearestIntersect.y + (Constants.OBSTACLE_WIDTH/2));
				break; 
			case 180 : nearestIntersect = new Double2D(nearestIntersect.x + (Constants.OBSTACLE_WIDTH/2), nearestIntersect.y);
				break; 
			case 270 : nearestIntersect = new Double2D(nearestIntersect.x, nearestIntersect.y - (Constants.OBSTACLE_WIDTH/2));
				break;
			// Check for the case where we haven't been able to find a location and nearestStartBearing is still set to 400	
			default : nearestIntersect = new Double2D(-1,-1);	
		}
		
		return nearestIntersect; 
	}	
	
	/**
	 * Method to check for junctions at supplied location, returns the ID of the junction in which the point is located
	 * - if no junction is found, returns 0
	 * - if multiple junctions are found, returns -1
	 * - otherwise returns the ID of the junction
	 * @param coord (Double2D - location where we want to test for a junction)
	 * @param junctions (Bag - collection of junctions to test against)
	 * @return int (0 = no junction; -1 = multiple junctions; n = ID of junction)
	 */
	public int junctionAtPoint(Double2D coord, Bag junctions)
	{
		int jctID = 0;
		
		for (int i = 0; i < junctions.size(); i++)
		{
			//for all of the obstacles check if the provided point is in it
			if (((Junction) (junctions.get(i))).inShape(coord)) 
			{
				// Check to see if the id has already been set, so we know if there are multiple
				// junctions at the same location
				if (jctID != 0)
				{
					return -1; // We don't need to know whether there are more than 2 intersections here, 2 is sufficient
				} else {
					jctID = ((Junction) (junctions.get(i))).getID(); // Store the ID and keep looking in case there is an overlap
				}
			}
		}
		
		//at this point either return the default 0 (for no overlap), or the jctID that we have found
		return jctID;
	}
	
	/**
	 * Method to check for junctions which overlap with a supplied Area.
	 * If no junction is found, return 0, if one junction is found, return the jctId of the junction 
	 * where the overlap is found, or return -1 if the Area overlaps with multiple junctions. 
	 * @param inJunct (Area - the junction (or Area) which we want to check for intersection with junctions)
	 * @param junctions (Bag - the collection of junctions to test against)
	 * @return int (0 = no junction; -1 = multiple junctions; n = ID of junction)
	 */
	public int junctionAtArea(Area inJunct, Bag junctions)
	{
		Shape tempJctShape;
		Double2D tempJctLoc;
		int retVal = 0; // Default no overlap
		
		for (int i = 0; i < junctions.size(); i++)
		{
			// Create a shape to represent this junction
			tempJctLoc = ((Junction) (junctions.get(i))).location;
			tempJctShape = new Rectangle2D.Double((tempJctLoc.x-(Road.roadWidth/2)), (tempJctLoc.y-(Road.roadWidth/2)), Road.roadWidth, Road.roadWidth);
			Area jctArea = new Area (tempJctShape);
			jctArea.intersect(inJunct);
			if (!jctArea.isEmpty()) {
				if (retVal == 0) {				
					retVal = ((Junction) (junctions.get(i))).getID();
				} else {
					retVal = -1; // We've already found another junction that overlaps with this Area, so return -1
				}
			}
		}
		
		return retVal;
	}
	
	/**
	 * Method to unOccupy the specified junction - can't use Bag.get() as that refers to the idx, not the ID 
	 * @param jctID (int - ID of junction we wish to unoccupy)
	 * @param junctions (Bag - collection of all junctions)
	 * @param vehID (long - ID of vehicle which has left the junction)
	 */
	public void unOccupyJunction(int jctID, Bag junctions, long vehID)
	{
		Junction tempJunction;
		
		for (int i = 0; i < junctions.size(); i++)
		{
			// Search for the supplied junction in the bag, and un-occupy it
			tempJunction = (Junction) (junctions.get(i));
			if (tempJunction.getID() == jctID)
			{
				tempJunction.unOccupy(vehID);
				break; // We don't need to keep searching
			}
		}
	}	
	
	/**
	 * Method to check for junction at provided coordinate, returns junction type, or NOJUNCTION if no junction 
	 * @param coord (Double2D - location where we want to check for a junction)
	 * @param junctions (Bag - collection of all junctions)
	 * @return int (type identifier for the junction, or NOJUNCTION if no junction is found)
	 */
	public int junctionTypeAtPoint(Double2D coord, Bag junctions)
	{
		for (int i = 0; i < junctions.size(); i++)
		{
			// for all of the junctions check if the provided point is in it
			if (((Junction) (junctions.get(i))).inShape(coord))
			{
				return ((Junction) junctions.get(i)).getType();
			}
		}
		// no overlap has been detected so NOJUNCTION should be returned
		return Constants.NOJUNCTION;
	}
	
	/**
	 * Method to check for junction approaches at provided location.
	 * @param coord (Double2D - location where we want to check for a junction approach)
	 * @param junctions (Bag - collection of junctions)
	 * @return boolean (true if location is in a junction approach, false otherwise)
	 */
	public boolean junctionAppAtPoint(Double2D coord, Bag junctions)
	{
		for (int i = 0; i < junctions.size(); i++)
		{
			// for all of the junctions check if the provided point is in their approach
			if (((Junction) (junctions.get(i))).inApproach(coord) == true) 
			{
				return true;
			}
		}
		
		// no overlap has been detected so false should be returned
		return false;
	}
	
	/**
	 * Method to check for junction exits at provided location.
	 * @param coord (Double2D - location where we want to check for a junction exit)
	 * @param junctions (Bag - collection of junctions)
	 * @return boolean (true if location is in a junction exit, false otherwise)
	 */
	public boolean junctionExitAtPoint(Double2D coord, Bag junctions)
	{
		for (int i = 0; i < junctions.size(); i++)
		{
			// for all of the junctions check if the provided point is in their exit
			if (((Junction) (junctions.get(i))).inExit(coord)) 
			{
				return true;
			}
		}
		
		// no overlap has been detected so false should be returned
		return false;
	}
	
	/**
	 * This method kills the simulation if there are no longer any active UGVs on the 
	 * schedule.  This is currently called at the end of the DumbCar step method and the 
	 * end of the UGV step method (and also once in the middle of the DC step).  It is 
	 * probably only necessary to call it once in UGV.step though, as this is the only
	 * routine that can alter whether the UGV is active.  
	 */
    public void dealWithTermination()
	{
    	int noActiveAgents =0;
    	for(Object o: this.toSchedule)
    	{
    		if(((Car)o).isActive  && ((Car)o).getType() == Constants.TUGV)
    		{
    			noActiveAgents++;
    		}
    	}
    	
		if(noActiveAgents < 1)
		{
			this.schedule.clear();
			this.kill();
		}
	 }
	
    /** 
     * This method is called by AccidentDetector to record the location at which a vehicle
     * has left the road (in case we want to visualise it).  It does this by adding a new
     * failure object to the environment at the supplied location.
     * @param location (Double2D - location at which the off-road event occurred)
     */
    public void recordOffRoad(Double2D location)
    {
    	int fID = this.getNewID();
    	Failure fp = new Failure(fID, Constants.TFAILURE);
		fp.setLocation(location);
		environment.setObjectLocation(fp, location);	
    }

    /** 
     * This method is called by AccidentDetector to record the location where a vehicle has crossed
     * a line e.g. centre or lane edge markings.  It does this by adding a new failure
     * object to the environment at the supplied location.
     * @param location (Double2D - the location at which the line was crossed)
     */
    public void recordCrossLine(Double2D location)
    {
    	int fID = this.getNewID();
    	Failure fp = new Failure(fID, Constants.TFAILURE);
		fp.setLocation(location);
		environment.setObjectLocation(fp, location);	
    }
    
    /** 
     * This method is called by AccidentDetector to record the location where a vehicle has crashed 
     * with another vehicle.  It does this by adding a new crash object to the environment at
     * the supplied location.  
     * @param location (Double2D - the location where the crash occurred)
     */
    public void recordCrash(Double2D location)
    {
    	int fID = this.getNewID();
    	Crash fp = new Crash(fID, Constants.TFAILURE);
		fp.setLocation(location);
		environment.setObjectLocation(fp, location);	
    }
    
    /** 
     * Return candidate coverage metric - MinJctSeparation, as required for sit coverage experiments.
     * Loops through all junction pairs checking the distance between their centres, and keeping
     * track of the lowest value observed.  This minimum junction separation distance is returned.
     * @return double (the minimum distance between any junctions on the map)
     */
    public double HgetMinJctSeparation()
    {
    	double retVal = Math.max(Constants.WorldXVal, Constants.WorldYVal);
    	double tempDist = 0;
    	
    	for (int j=0; j<junctions.size(); j++) 
    	{
        	for (int i=0; i<junctions.size(); i++) 
        	{
        		tempDist = ((Junction)junctions.get(j)).location.distance(((Junction)junctions.get(i)).location);
        		
        		// Make sure we are not checking the distance between an object and itself (=0)
        		if (tempDist < retVal && i != j)
        		{
        			retVal = tempDist;
        		}
        	}
    	}
    	
    	return retVal;
    }
    
    /**
     * This method returns the initial separation distance (crow flies) between the UGV and its target
     * - note that this is the centre-to-centre distance, rather than the separation of their
     * closest points.
     * NOTE: this assumes there is only one UGV, if there are more than one, this will only ever return
     * the distance between the UGV found at index 0 in the UGV Bag, and its target.  
     * @return double (distance between UGV and Target at start of simulation)
     */
    public double HgetUGVTargetSeparation()
    {
    	double retVal = Math.max(Constants.WorldXVal, Constants.WorldYVal); // default return value
    	
    	if (ugvs.size() > 0)
    	{
    		Entity e;
    		int targetId = ((UGV)ugvs.get(0)).getTargetID();
    		Double2D targetLoc = new Double2D(-1,-1);

    		// Find the target from the bag of all entities and store location
    		for(int i = 0; i < allEntities.size(); i++)
    		{
    			e = (Entity) allEntities.get(i);			
    			if (e.getID() == targetId)
    			{
    				targetLoc = e.getLocation();
    			}
    		}

    		if (targetLoc.x != -1) {
    			retVal = ((UGV)ugvs.get(0)).location.distance(targetLoc);   			
    		}
    	}
			
		return retVal;		
    }

    /**
     * This method returns the minimum distance between the target and any static obstacles
     * - note that this is the centre-to-centre distance, rather than the separation of their
     * closest points.
     * @return double (distance between target and the obstacle closest to it)
     */
    public double HgetMinTargetObsSeparation()
    {
    	double retVal = Math.max(Constants.WorldXVal, Constants.WorldYVal);
		
    	if (ugvs.size() > 0)
    	{
    		Entity e;
    	
    		int targetId = ((UGV)ugvs.get(0)).getTargetID();
    		Double2D targetLoc = new Double2D(-1,-1);

    		// Find the target from the bag of all entities and store location
    		for(int i = 0; i < allEntities.size(); i++)
    		{
    			e = (Entity) allEntities.get(i);			
    			if (e.getID() == targetId)
    			{
    				targetLoc = e.getLocation();
    			}
    		}

    		double tempDist = 0;

    		// loop through all the obstacles and test each one
    		for (int o=0; o < obstacles.size(); o++)
    		{
    			tempDist = ((Obstacle)obstacles.get(o)).location.distance(targetLoc); 
    			if (tempDist < retVal)
    			{
    				retVal = tempDist;
    			}
    		}
    	} 
    	
    	return retVal;		
    }
    
    /** 
    * This method returns the minimum separation distance (crow flies, perpendicular) between the target and any kerb.
    * @return double (the minimum perpendicular distance between the target and a kerb)
    */
    public double HgetMinTargetKerbSeparation()
    {
    	double retVal = Math.max(Constants.WorldXVal, Constants.WorldYVal); // default return value
		
    	if (ugvs.size() > 0)
    	{
    		Entity e;
    	
    		int targetId = ((UGV)ugvs.get(0)).getTargetID();
    		Double2D targetLoc = new Double2D(-1,-1);

    		// Find the target from the bag of all entities and store location
    		for(int i = 0; i < allEntities.size(); i++)
    		{
    			e = (Entity) allEntities.get(i);			
    			if (e.getID() == targetId)
    			{
    				targetLoc = e.getLocation();
    			}
    		}

    		if (targetLoc.x != -1) {

    			// Loop through all the roads to find which one(s) contain(s) the target
    			Road tempRoad;
    			double tempVal = Math.max(Constants.WorldXVal, Constants.WorldYVal); // default return value
    			for (int r = 0; r < roads.size(); r++)
    			{
    				tempRoad = (Road)roads.get(r);

    				if (tempRoad.inShape(targetLoc) == true) 
    				{
    					if (tempRoad.x1 == tempRoad.x2)	{ 
    						// Is N/S so compare x val of targetLoc to x values of edge of road
    						tempVal = Math.min(Math.abs(tempRoad.x1 - targetLoc.x - Road.roadWidth/2),
    								Math.abs(tempRoad.x1 + Road.roadWidth/2 - targetLoc.x));
    					} else {
    						// Is E/W so compare y val of targetLoc to y values of edge of road
    						tempVal = Math.min(Math.abs(tempRoad.y1 - targetLoc.y - Road.roadWidth/2),
    								Math.abs(tempRoad.y1 + Road.roadWidth/2 - targetLoc.y));
    					}

    					// Is this the shortest distance so far?
    					if (tempVal < retVal) {retVal = tempVal;}
    				}
    			}
    		}
    	}
		
		return retVal;		
    }
    
    /**
     * This method returns the  minimum separation (crow flies) between target and the centre of the road
     * @return double (the minimum perpendicular distance between the target and the centre of the road)
     */
    public double HgetMinTargetCentreSeparation()
    {
    	double retVal = Math.max(Constants.WorldXVal, Constants.WorldYVal); // default return value
		
    	if (ugvs.size() > 0)
    	{
    		Entity e;
    		int targetId = ((UGV)ugvs.get(0)).getTargetID();
    		Double2D targetLoc = new Double2D(-1,-1);

    		// Find the target from the bag of all entities and store location
    		for(int i = 0; i < allEntities.size(); i++)
    		{
    			e = (Entity) allEntities.get(i);			
    			if (e.getID() == targetId)
    			{
    				targetLoc = e.getLocation();
    			}
    		}

    		if (targetLoc.x != -1) {

    			// Loop through all the roads to find which one(s) contain(s) the target
    			Road tempRoad;
    			double tempVal = Math.max(Constants.WorldXVal, Constants.WorldYVal); // default return value
    			for (int r = 0; r < roads.size(); r++)
    			{
    				tempRoad = (Road)roads.get(r);

    				if (tempRoad.inShape(targetLoc) == true) 
    				{
    					if (tempRoad.x1 == tempRoad.x2)	{ 
    						// Is N/S so compare x val of targetLoc to x value of centre of road
    						tempVal = Math.abs(targetLoc.x - tempRoad.x1);
    					} else {
    						// Is E/W so compare y val of targetLoc to y values of centre of road
    						tempVal = Math.abs(targetLoc.y - tempRoad.y1);
    					}

    					// Is this the shortest distance so far?
    					if (tempVal < retVal) {retVal = tempVal;}
    				}
    			}
    		}
    	}
    	
		return retVal;		
    }
    
    /** 
     * Return number of obstacles which are separated by between OBSTACLE_HEADWAY and 2x OBSTACLE_HEADWAY (actually test for
     * between Obs.length + OBSTACLE_HEADWAY and Obs.length + 2*OBSTACLE_HEADWAY as Obstacle location is centre of obs)
     * @return int (number of occasions where 2 obstacles are separated by a gap of length in specified range)
     */
    public int HgetCriticalObsSeparation() 
    {
    	int retVal = 0;
    	double tempDist = 0;
    	
    	for (int j=0; j<obstacles.size(); j++) 
    	{
        	for (int i=0; i<obstacles.size(); i++) 
        	{
        		tempDist = ((Obstacle)obstacles.get(j)).location.distance(((Obstacle)obstacles.get(i)).location);
        		
        		// Make sure we are not checking the distance between an object and itself (=0)
        		if (tempDist >= Constants.OBSTACLE_LENGTH + Constants.OBSTACLE_HEADWAY &&
        			tempDist <= Constants.OBSTACLE_LENGTH + (2*Constants.OBSTACLE_HEADWAY) && i != j)
        		{
        			retVal++;
        		}
        	}
    	}
    	
    	return retVal/2; // Divide by 2 as method counts from i to j and from j to i.
    }
    
   /**
    *  Return proportion of initial separation (crow flies) between target and UGV that is actually on the road surface
    *  We make an approximation to this value by incrementally testing locations on the line between the two objects
    *  to see if they fall on the road surface.
    * @return double (proportion of crow flies distance between UGV start location and target which is on road surface)
    */ 
    public double HgetUGVTargetRoadSeparation() 
    {
		if (ugvs.size() > 0)
		{
			Entity e;
			int targetId = ((UGV)ugvs.get(0)).getTargetID();
			Double2D targetLoc = new Double2D(-1,-1);
			Double2D UGVLoc = ((UGV)ugvs.get(0)).location;
			double noRoadHits = 0;
			double noNonRoadHits = 0;

			// Find the target from the bag of all entities and store location
			for(int i = 0; i < allEntities.size(); i++)
			{
				e = (Entity) allEntities.get(i);			
				if (e.getID() == targetId)
				{
					targetLoc = e.getLocation();
				}
			}

			if (targetLoc.x != -1) {

				// Work out bearing from UGVLoc to targetLoc
				double angle = Utility.calculateAngle(UGVLoc, targetLoc);
				double moveV = Utility.yMovement(angle, 1);
				double moveH = Utility.xMovement(angle, 1);

				MutableDouble2D sumForces = new MutableDouble2D();
				sumForces.addIn(UGVLoc);

				while (sumForces.distance(targetLoc) > 1) {
					sumForces.addIn(new Double2D(moveH, moveV));

					// Test the tempLoc against the Road network
					if (roadAtPoint(new Double2D(sumForces), roads) == true) {
						noRoadHits++;// Add to count of road hits
					} else {
						noNonRoadHits++;// Add to count of non-road hits
					}
				}

				// Return success/(success + fail) - % onRoad
				if (noRoadHits+noNonRoadHits == 0)
				{
					return 0;
				} else
				{
					return (noRoadHits/(noRoadHits+noNonRoadHits));
				}
			}
		}
		
		return -1;
    }
        
   /**
    * This method is to be called when an existing dumbCar in the network is terminated
    * due to leaving the road surface.  The new car should be added at a randomly chosen entrance to the 
    * network i.e. a dead-end.
    */
    public void addNewDumbCar()
    {
    	// Choose a junction at random and retrieve the entry location opposite an exit from the junction 
    	int j = (int) (random.nextDouble() * junctions.size());
    	Double2D entryLoc = ((Junction) junctions.get(j)).getDeadEndEntry();
    	
    	// Loop until we get a junction at the end of the network (will only have one exit)
    	while (((Junction) junctions.get(j)).isDeadEnd() == false || entryLoc.x == -1)
    	{
    		j = (int) (random.nextDouble() * junctions.size());
    		entryLoc = ((Junction) junctions.get(j)).getDeadEndEntry();
    	}
    	
    	// Now add a car at that location
    	initialInfo startInfo = snapToLane(entryLoc.x, entryLoc.y);

		DumbCar theCar = new DumbCar(getNewID(), carStats, startInfo.startBearing);
		cars.add(theCar);
		theCar.setLocation(startInfo.startLoc); 
		theCar.isSchedulable = true;
		environment.setObjectLocation(theCar, startInfo.startLoc);
		allEntities.add(theCar);
		toSchedule.add(theCar); 
		schedule.scheduleRepeating((Entity) theCar); // Put the car on the schedule so it gets stepped!
    }

   /**
    * This method tells us the orientation of the lane in which the testPt falls.  This method
    * is called during map construction.  This method should not be called by the UGV as that
    * would be cheating behaviour as the UGV can't query the road as to its direction at any
    * given location.
    * @param testPt (Double2D - the location at which we want to know the road direction)
    * @return double (0 if road is N/S, 90 if road is E/W)
    */
	public double getRoadDirectionAtPoint(Double2D testPt) {
	
		// For now we are still going to assume that the roads are in a grid-formation
		// and therefore run either NS or EW.  
		// Loop through all the roads to find which one(s) contain(s) the target
		Road tempRoad;
		
		for (int r = 0; r < roads.size(); r++)
		{
			tempRoad = (Road)roads.get(r);

			if (tempRoad.inShape(testPt) == true) 
			{
				if (tempRoad.x1 == tempRoad.x2)	{ 
					// Is N/S so return 0
					return 0;
				} else {
					// Is E/W so return 90
					return 90;
				}
			}
		}
		return -1;
	}

   /** 
    * This method calculates the separation between each adjacent junction on the network and returns a string
	* with comma-separated occurrence counts in the following 8 categories (including lower bound but excluding
	* upper: <3m, 3-6m, 6-12m, 12-23m, 23-46m, 46-100m, 100-150m, >150m 
	* @return String (comma separated junction separation counts in each category)
	*/
	public String getJunctionSep() {
		
		int[] sepCount = new int[8];
		Double2D startJunctionLoc;
		Double2D endJunctionLoc;
		Double2D tempJctLoc;
		Road tempRoad;
		boolean middleJct;
		double sepDist;
		
		// Loop through all the junctions
		for (int j = 0; j < junctions.size(); j++)
		{
			startJunctionLoc = ((Junction) junctions.get(j)).getLocation();
			
			// Test each junction against all the other junctions that are
			// of a higher index (because we don't want to count any twice)
			for(int k = j+1; k < junctions.size(); k++)
			{
				endJunctionLoc = ((Junction) junctions.get(k)).getLocation();
				
				// Loop through all the roads and see if there is a road on which 
				// both the junctions appear
				for (int r = 0; r < roads.size(); r++)
				{
					tempRoad = (Road) roads.get(r);
					
					if (tempRoad.inShape(startJunctionLoc) && tempRoad.inShape(endJunctionLoc))
					{
						// The two junctions are on the same road, now we need to know whether there
						// are any intermediate junctions
						middleJct = false;
						
						for (int x = 0; x < junctions.size(); x++)
						{
							if (x != j && x != k)
							{
								tempJctLoc = ((Junction) junctions.get(x)).getLocation();
								
								if (tempRoad.inShape(tempJctLoc))
								{
									// Check to see if the junction is actually between the 
									// two other junctions
									// TODO - If we want to move to non-grid roads then this method
									// will have to be updated
									if (startJunctionLoc.x == endJunctionLoc.x)
									{
										// N/S so check the y values
										if ((startJunctionLoc.y > tempJctLoc.y && endJunctionLoc.y < tempJctLoc.y) || 
										     (startJunctionLoc.y < tempJctLoc.y && endJunctionLoc.y > tempJctLoc.y))
										{
											// tempJctLoc is in the middle
											middleJct = true;
										}
									} else {
										// E/W so check the x values
										if ((startJunctionLoc.x > tempJctLoc.x && endJunctionLoc.x < tempJctLoc.x) || 
											     (startJunctionLoc.x < tempJctLoc.x && endJunctionLoc.x > tempJctLoc.x))
										{
											// tempJctLoc is in the middle
											middleJct = true;
										}
									}
								}
							}
						}
						
						if (middleJct == false)
						{
							// We didn't find a junction in the middle, so calculate the distance between the
							// two points and store it in the appropriate 'bin'
							sepDist = startJunctionLoc.distance(endJunctionLoc);
							
							if (sepDist < 3) {
								sepCount[0]++;
							} else if (sepDist < 6) {
								sepCount[1]++;
							} else if (sepDist < 12) {
								sepCount[2]++;
							} else if (sepDist < 23) {
								sepCount[3]++;
							} else if (sepDist < 46) {
								sepCount[4]++;
							} else if (sepDist < 100) {
								sepCount[5]++;
							} else if (sepDist < 150) {
								sepCount[6]++;
							} else {
								sepCount[7]++;
							}
						}
					}
				}
			}
		}
		
		String retString = "" + sepCount[0];
		// Iterate through the array, constructing the string
		for (int i = 1; i < 8; i++)
		{
			retString = retString + ", ";
			retString = retString + sepCount[i];
		}
		
		return retString;
	}

   /**
	* This method stores a pair of junctions in the jctArray to facilitate the IN-RUN
	* recording of the situation coverage measure of junction separation
	* @param prevJct (int - ID of previous junction)
	* @param currentJct (int - ID of next junction)
	* @param inJctLocs (jctPairInfo - pair of Double2Ds storing locations of current and previous junctions)
	*/
	public void HsetJctArray(int prevJct, int currentJct, jctPairInfo inJctLocs) 
	{
		// Check that we haven't been supplied with invalid values (we'll just ignore them if we have)
		if (prevJct == -1 || currentJct == -1 || prevJct == currentJct) {
			// DO nothing
		} else {
			// We've got valid data, so update the array accordingly (don't need to check what is currently there, we're only
			// ever storing the junction locations, and these can't change)
			junctionArray[prevJct][currentJct] = inJctLocs;
		}
	}
	
	/**
	 * This method resets all of the entries in the junctionArray to null so that data is not 
	 * carried over between runs.
	 */
	public void resetJctArray() 
	{
		// Loop through the entire junction array and ensure each entry is null
		for (int i=0; i<JCTARRAYLENGTH; i++)
		{
			for (int j=0; j<JCTARRAYLENGTH; j++)
			{
				junctionArray[i][j] = null;
			}
		}			
	}
	
   /**
	* Calculate the separation between each adjacent junction pair that IS ACTUALLY VISITED BY THE UGV and return a string
	* with comma-separated occurrence counts in the following 8 categories (including lower bound but excluding
	* upper: <3m, 3-6m, 6-12m, 12-23m, 23-46m, 46-100m, 100-150m, >150m)  NOTE: This is not the number of times that
	* each of these stretches of road have been traversed, it is the number of stretches of road (of each length
	* category) that have been visited at least once by the UGV.
	* @return String (comma-separated list of the number of junction separation categories visited by UGV)
	*/
	public String HgetIRJunctionSep() {
		
		int[] sepCount = new int[8];
		Double2D startJunctionLoc;
		Double2D endJunctionLoc;
		double sepDist;
		
		// Loop through the junctionArray
		for (int j = 0; j < JCTARRAYLENGTH; j++)
		{
			// Test each junction against all the other junctions that are
			// of a higher index (because we don't want to count any twice)
			// NOTE : see below where we check whether we have driven down the
			// road in the other direction
			for(int k = 0; k < j; k++)
			{
				sepDist = 0; // Reset this so we can check it later
				
				// Check to see if we have actually visited this pair of locations
				if (junctionArray[j][k] != null) // Heading down the road one way...
				{
					startJunctionLoc = junctionArray[j][k].startLoc;
					endJunctionLoc = junctionArray[j][k].endLoc;
					
					sepDist = startJunctionLoc.distance(endJunctionLoc); // Find out what the distance between the two locations is
					
				} else if (junctionArray[k][j] != null)	{ // Heading down the road the other way...
					startJunctionLoc = junctionArray[k][j].startLoc;
					endJunctionLoc = junctionArray[k][j].endLoc;
					
					sepDist = startJunctionLoc.distance(endJunctionLoc); // Find out what the distance between the two locations is
				}
				
				// Check to see if we found a value in either of the matrix locations above
				if (sepDist > 0)
				{
					if (sepDist < 3) {
						sepCount[0]++;
					} else if (sepDist < 6) {
						sepCount[1]++;
					} else if (sepDist < 12) {
						sepCount[2]++;
					} else if (sepDist < 23) {
						sepCount[3]++;
					} else if (sepDist < 46) {
						sepCount[4]++;
					} else if (sepDist < 100) {
						sepCount[5]++;
					} else if (sepDist < 150) {
						sepCount[6]++;
					} else {
						sepCount[7]++;
					}
				}
			}
		}
		
		String retString = "" + sepCount[0];
		// Iterate through the array, constructing the string
		for (int i = 1; i < 8; i++)
		{
			retString = retString + ", ";
			retString = retString + sepCount[i];
		}
		
		return retString;	
	}
	
   /** 
    * Work out how far the target is from the junctions in the downstream and upstream directions
    * return an object containing both of these measurements in metres.  If the return values are both zero
	* this indicates an error condition e.g. the road on which the target is located could not be identified.
    * @param targetLoc (Double2D - location of the target)
    * @return targetInfo (pair of distances (m) from previous junction/to next junction)
    */
	public targetInfo HgetTargetSeparations(Double2D targetLoc)
	{
		targetInfo retVal = new targetInfo();
		Double2D tempJunctionLoc;
		Road currentRoad = null;
		boolean roadFound = false;
		
		// Find out which road the target is on - it can only be on one road as it can't be added in a 
		// junction.
		for (int r = 0; r < roads.size(); r++)
		{
			currentRoad = (Road) roads.get(r);
			
			if (currentRoad.inShape(targetLoc) == true)
			{
				roadFound = true;
				break; // so that currentRoad will still contain the right road
			}
		}
		
		// Just check that we actually found the road
		if (roadFound == false)
		{
			return retVal; // both values are still set to zero (not possible), so can ignore such return values
		}
		
		double plusDistance = Constants.WorldXVal; // the distance to the junction that is in the positive direction
		double minusDistance = Constants.WorldXVal; // the distance to the junction that is in the negative direction
		double tempDistance;
		
		// Loop through all the junctions and work out which junctions are on the same road as the target
		for (int j = 0; j < junctions.size(); j++)
		{
			tempJunctionLoc = ((Junction) junctions.get(j)).getLocation();
			tempDistance = Math.abs(tempJunctionLoc.distance(targetLoc));
			
			if (currentRoad.inShape(tempJunctionLoc) == true)
			{
				// Depending on the orientation of the road, work out which
				// side of the Target the junction is on (increasing/decreasing x/y)
				if (currentRoad.getIsNS() == true)
				{
					// is it in the positive direction
					if (tempJunctionLoc.y - targetLoc.y > 0)
					{
						if (tempDistance < plusDistance)
						{
							plusDistance = tempDistance;
						}
					} else {// it is in the negative direction
						if (tempDistance < minusDistance)
						{
							minusDistance = tempDistance;
						}
					}
				} else {
					// is it in the positive direction
					if (tempJunctionLoc.x - targetLoc.x > 0)
					{
						if (tempDistance < plusDistance)
						{
							plusDistance = tempDistance;
						}
					} else {// it is in the negative direction
						if (tempDistance < minusDistance)
						{
							minusDistance = tempDistance;
						}
					}
				}
			}
		}
		
		// Work out whether it is the increasing x/y direction or the decreasing x/y direction that is the 
		// prev/next measurement.  This depends on the 'direction' of the target, or more accurately the 
		// direction of the lane in which the Target is located.
		
		// Returns 1 for N/E or 2 for S/W
		if (currentRoad.getLane(targetLoc) == 1) {
			// North or East
			if (currentRoad.getIsNS() == true) {
				// Target 'facing' North
				retVal.fromPrevJct = plusDistance;
				retVal.toNextJct = minusDistance;
			} else {
				// Target 'facing' East
				retVal.fromPrevJct = minusDistance;
				retVal.toNextJct = plusDistance;
			}
		} else {
			// South or West
			if (currentRoad.getIsNS() == true) {
				// Target 'facing' South
				retVal.fromPrevJct = minusDistance;
				retVal.toNextJct = plusDistance;
			} else {
				// Target 'facing' West
				retVal.fromPrevJct = plusDistance;
				retVal.toNextJct = minusDistance;
			}
		}
		
		return retVal;
	}		
}
