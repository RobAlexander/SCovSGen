package modeling;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

import modeling.Constants.LineType;
import sim.util.*;
import sim.field.continuous.*;
import sim.engine.*;
import sim.field.grid.IntGrid2D;

public class COModel extends SimState
{
	public Bag toSchedule = new Bag();
	public Bag allEntities = new Bag();
	public Bag cars = new Bag();
	public Bag obstacles= new Bag();
	
	// HH 30/4/14 - Road collection
	public Bag roads = new Bag();
	public Bag junctions = new Bag();
	public Bag ugvs = new Bag();
	//end
	
	public int noObstacles=0; // HH 30.7.14 - not used
	public int noCars=0; // HH 7.5.14 - not used
	private int noJunctions = 0; // HH 31/7/14 - added but has a set and get
	
	// HH 1.9.14 - New Random Seed Parameter which can be passed in from the UI on the 'Model' tab
	// Default value of 0 will indicate that this hasn't been set, and therefore a random seed will be 
	// generated programmatically from the system time.
	public long externalSeed = 0;
	
	// HH 30.4.14 - Road parameters
	//public int noRoads = 6;
	//public int noJunctions = 10; // HH 30.7.14 - Make this a randomly generated variable from MIN_JUNCTIONS to MAX_JUNCTIONS
	//public int noTjunctions = 3;
	//public int noXjunctions = 3;
	//
	
	// HH 22/7/14 - Fault instantiation
	//    This array maps from each fault embedded in the code to a flag which indicates whether the fault is active
	//    Ultimately could upgrade this to an int array if we want to express intermittent faults, or have a % level
	//    of failure, however a simple on/off should be sufficient as a starting point.
	//    The array could be read in from a file, but may be simpler to probabilistically instantiate the array, unless
	//    there is a particular reason for wanting to turn 'particular' faults on.  A method is provided to fill the array
	//    at random with a given percentage of faults prior to the start of the simulation (e.g. 100%, 0%, 25%, ...)
	//    NOTE: All faults are labelled in the code as "// FAULT #n" and can be found by simple text search.  The numbering
	//    n relates to the index in the faultArray which contains the flag to indicate whether the fault is active or not.
	//    Every time a new fault is added to the code, Constants.MAX_FAULTS should be incremented.
	private boolean faultArray[] = new boolean[Constants.MAX_FAULTS];  // Flag whether the fault is active or not
	private long faultCalled[] = new long[Constants.MAX_FAULTS]; // Keep a count of how many times each fault has been called during the simulation
	private double percentageFaults = 0;
	// HH end
	
	private boolean runningWithUI = false; 
	
    private int newID = 0;	
    
	private double xDouble;
	private double yDouble;	
	
    public Continuous2D environment;
	
	public IntGrid2D obstacleMap;
	private int obstacleMapResolution = 3; //multiplier used to change resolution of obstacles image

	public IntGrid2D terrainMap;
	private int terrainMapResolution = 3;
	
	public IntGrid2D wallMap;
	private int wallMapResolution =3;
	
	// HH 30.4.14 - Road parameters
	public IntGrid2D roadMap;
	private int roadMapResolution = 5; // HH 17.6.14 - increased from 3 to try and improve the lane markings
	public IntGrid2D junctionMap;
	private int junctionMapResolution = 5; // HH 17.6.14 - increased from 3 to try and improve the lane markings
	public IntGrid2D jctApproachMap;
	private int jctApproachMapResolution = 5; // HH 17.6.14 - increased from 3 to try and improve the lane markings	
	// HH 17.6.14 - Additional road params
	public IntGrid2D roadMarkingMap;
	private int roadMarkingMapResolution = 10; // higher resolution due to narrow lines
	// HH 24.9.14 - For testing getShape for dumbCar
	//public IntGrid2D dumbCarMap;
	//private int dumbCarMapResolution = 3; //multiplier used to change resolution of obstacles image
	
	// HH 19.6.14
	public class initialInfo {
		public initialInfo(Double2D inStartLoc, double inStartBearing) {
			startLoc = inStartLoc;
			startBearing = inStartBearing;
		}
		public Double2D startLoc;
		public double startBearing;
	}
	
	// HH end
	
	// HH - Original value, replaced below
	//private double carMaxSpeed=1.0; 
	//private double carMaxAcceleration = 0.05;
	//private double carMaxDecceleration = 0.05;
	//private double carMaxTurning = 15;
	
	// HH 29/7/14 - These are the values per STEP.  We can make the assumption that there are 5 steps per 'real' second
	// and calculate 'realistic' parameters for maximum speed, acc, decel, etc.
	private double carMaxSpeed = 2.5;  // HH 29/7/14 based on 45km/hr = 12.5m/s
	private double carMaxAcceleration = 0.1; // HH 29/7/14 based on 0 to 45km/hr in 5s; (0 to 2.5m/step in 25 steps)
	private double carMaxDecceleration = 0.15625;  // HH 29/7/14 based on 20m stopping distance at 45km/hr
	private double carMaxTurning = 15;
	// HH end
	
	public CarPerformance carStats;
	
	public AccidentDetector aDetector; // HH 28/8/14 Construct this later so can pass argument 
	
	// HH 06/05/14 - Logging file
	public InfoLogFile infoLog = new InfoLogFile();
	// HH end
	
	/**
	 * Constructor method used in the simState.doLoop process for running the simulation
	 * 
	 * @param seed for random number generator
	 */
//	public COModel(long seed)//, int noCars, int noObstacles)
//	{
//		super(seed);
//		environment = new Continuous2D(1.0, xDouble, yDouble);
//		obstacleMap = new IntGrid2D((int) (xDouble * obstacleMapResolution), (int) (yDouble * obstacleMapResolution), 0);
//		terrainMap = new IntGrid2D((int) (xDouble * terrainMapResolution), (int) (yDouble * terrainMapResolution), 0);
//		wallMap = new IntGrid2D((int) (xDouble * wallMapResolution), (int) (yDouble * wallMapResolution), 0);
//		carStats = new CarPerformance(carMaxSpeed, carMaxAcceleration, carMaxDecceleration, carMaxTurning);	
////		this.noCars= noCars;
////		this.noObstacles= noObstacles;
//		System.out.println("Simulation1 is being called!!!!!!!!!!!");
//	}
	
	/**
	 * Constructor used for setting up a simulation from the COModelBuilder object.
	 * 
	 * @param seed for random number generator
	 * @param x the width of the simulation environment
	 * @param y the height of the simulation environment
	 * @param UI pass true if the simulation is being ran with a UI false if it is not.
	 * 
	 * // HH 22/7/14 - Added extra input parameter 
	 * @param inPercentageFaults - value of 0..1 (inclusive) to set the percentage of faults to be injected into the model
	 */
    public COModel(long seed, double x, double y, boolean UI, double inPercentageFaults, int mapNo)//,int noCars, int noObstacles)
    {
    	super(seed);
    	System.out.println("At the start of COModel Constructor: Free =" + Runtime.getRuntime().freeMemory());
    	System.out.println("At the start of COModel Constructor: Max =" + Runtime.getRuntime().maxMemory());
    	System.out.println("At the start of COModel Constructor: Total =" + Runtime.getRuntime().totalMemory());
    	environment = new Continuous2D(1.0, x, y);
		xDouble = x;
		yDouble = y;
		
		aDetector = new AccidentDetector(inPercentageFaults, mapNo); // HH 28.8.14 - Pass %faults to use in file name for batch
		
		System.out.println("After creating environment: Free =" + Runtime.getRuntime().freeMemory());
    	System.out.println("After creating environment: Total =" + Runtime.getRuntime().totalMemory());
    	// HH 30/4/14 - Remove existing environment features and build road features instead
		//obstacleMap = new IntGrid2D((int) (xDouble * obstacleMapResolution), (int) (yDouble * obstacleMapResolution), 0);
		terrainMap = new IntGrid2D((int) (xDouble * terrainMapResolution), (int) (yDouble * terrainMapResolution), 0);
		System.out.println("After creating Int2DGrid: Free =" + Runtime.getRuntime().freeMemory());
    	System.out.println("After creating Int2DGrid: Total =" + Runtime.getRuntime().totalMemory());
    	wallMap = new IntGrid2D((int) (xDouble * wallMapResolution), (int) (yDouble * wallMapResolution), 0);
    	System.out.println("After creating Int2DGrid: Free =" + Runtime.getRuntime().freeMemory());
    	System.out.println("After creating Int2DGrid: Total =" + Runtime.getRuntime().totalMemory());
    	roadMap = new IntGrid2D((int) (xDouble * roadMapResolution), (int) (yDouble * roadMapResolution), 0);
    	System.out.println("After creating Int2DGrid: Free =" + Runtime.getRuntime().freeMemory());
    	System.out.println("After creating Int2DGrid: Total =" + Runtime.getRuntime().totalMemory());
    	junctionMap = new IntGrid2D((int) (xDouble * junctionMapResolution), (int) (yDouble * junctionMapResolution), 0);
    	System.out.println("After creating Int2DGrid: Free =" + Runtime.getRuntime().freeMemory());
    	System.out.println("After creating Int2DGrid: Total =" + Runtime.getRuntime().totalMemory());
    	jctApproachMap = new IntGrid2D((int) (xDouble * jctApproachMapResolution), (int) (yDouble * jctApproachMapResolution), 0);
		// HH 17.6.14 - Additional map for road markings
    	System.out.println("After creating Int2DGrid: Free =" + Runtime.getRuntime().freeMemory());
    	System.out.println("After creating Int2DGrid: Total =" + Runtime.getRuntime().totalMemory());
    	roadMarkingMap = new IntGrid2D((int) (xDouble * roadMarkingMapResolution), (int) (yDouble * roadMarkingMapResolution), 0);
		// HH end
    	System.out.println("After creating Int2DGrid: Free =" + Runtime.getRuntime().freeMemory());
    	System.out.println("After creating Int2DGrid: Total =" + Runtime.getRuntime().totalMemory());
    	System.out.println("MAX = " + Runtime.getRuntime().maxMemory());
		
		runningWithUI = UI;
		carStats = new CarPerformance(carMaxSpeed, carMaxAcceleration, carMaxDecceleration, carMaxTurning);
//		this.noCars= noCars;
//		this.noObstacles= noObstacles;
//		System.out.println("Simulation2 is being called!!!!!!!!!!! the model is: "+ this.toString());

		// HH 22/7/14 initialise the percentageFaults field
		setPercentageFaults(inPercentageFaults);
	}
    
    // HH 22/7/14 - Update the percentageFaults parameter to the one required by this simulation
    public void setPercentageFaults(double inPerFaults) {
    	// Check that it is within the required range:
    	if (inPerFaults >= 0 && inPerFaults <= 1.0) {
    		percentageFaults = inPerFaults;
    	} else {
    		this.schedule.clear();
    		System.out.println("Terminating before start as percentage faults has been set outside the range [0..1]. "+ ". Game Over!");
    		System.out.println(this.schedule.scheduleComplete());
    		this.kill();
    	}
    }
    
    // HH 22/7/14 - Return the entry in the fault array found at the specified index
    public boolean getFault(int idx) {
    	return faultArray[idx];
    }

    // HH 22/7/14 - Increment the entry in the fault called array found at the specified index
    public void setFault(int idx) {
    	faultCalled[idx] ++;
    }
    
    // HH 31/7/14 Added set/get methods for the number of junctions
	public  int getNoJunctions() {
		return noJunctions;
	}

	public  void setNoJunctions(int inNoJunctions) {
		noJunctions = inNoJunctions;
	}  
	// end
    	
	
	public  double getCarMaxSpeed() {
		return carMaxSpeed;
	}


	public  void setCarMaxSpeed(double MaxSpeed) {
		carMaxSpeed = MaxSpeed;
	}

	
	public  double getCarMaxAcceleration() {
		return carMaxAcceleration;
	}


	public  void setCarMaxAcceleration(double MaxAcceleration) {
		carMaxAcceleration = MaxAcceleration;
	}


	public  double getCarMaxDecceleration() {
		return carMaxDecceleration;
	}


	public  void setCarMaxDecceleration(double MaxDecceleration) {
		carMaxDecceleration = MaxDecceleration;
	}


	public  double getCarMaxTurning() {
		return carMaxTurning;
	}


	public  void setCarMaxTurning(double MaxTurning) {
		carMaxTurning = MaxTurning;
	}
	
	public int getNoObstacles() {
		return this.noObstacles;
	}

	public void setNoObstacles(int noObstacles) {
		this.noObstacles = noObstacles;
	}

	public int getNoCars() {
		return this.noCars;
	}

	public void setNoCars(int noCars) {
		this.noCars = noCars;
	}
	
	// HH 1.9.14 - Added get/set methods in case they are required by the GUI
	public long getExternalSeed() {
		return this.externalSeed;
	}

	public void setExternalSeed(long reqSeed) {
		this.externalSeed = reqSeed;
	}
	
	public void start()
	{
		super.start();	
		environment.clear();
		
		loadEntities();
		scheduleEntities();
		//if (runningWithUI)
		{
			// HH - 30/4/14 Remove existing environment features and build road features instead
			buildObstacleMap(); //draw the map of integers for obstacle representation
			//buildTerrainMap();
			buildRoadMap();
			buildJunctionMap();
			buildJctApproachMap();
			// HH - 17.6.14 Add road markings
			buildRoadMarkingsMap();
			// HH 24.9.14 - For testing getShape in DumbCar
			//buildDumbCarMap();
			// end
			
			buildWallMap();
		}
		
		// HH 22/7/14 Update for faults (do this before adding the header to aDetector so the faultArray can
		// be logged to file.
		faultArray = new boolean[Constants.MAX_FAULTS]; // Clear out the array
		faultCalled = new long[Constants.MAX_FAULTS]; // Clear out the array or results will accumulate!
		initFaultArray(percentageFaults);// Instantiate the faultArray with the required percentage of faults
		// HH end		
		
		aDetector.addHeader(this); // HH 30/4/14 - Add header to accident log file
		aDetector.addString(this, HgetFaultArrayAsString()); // HH 22/7/14 - Add the fault array to the accident log file
	}
	
	/**
	 * HH 17.7.14 Override the finish() method in SimState to allow any
	 * required termination behaviour to complete
	 **/
	
	public void finish()
	{
		super.finish();
		
		// HH 22/7/14 Update for faults (do this before adding the footer to aDetector so the faultCalled array can
		// be logged to file).
		aDetector.addString(this, HgetFaultCalledAsString());
		// HH end	
		
		// Add Header information to the Accident Log file
		aDetector.addFooter(this);
		
		// HH 3.9.14 - Reset the External Seed to zero if we are running with a UI
		if (runningWithUI)
		{
			setExternalSeed(0);
		}
	}
	
//	/**
//	 * A method which constructs an example simulation, this exists for testing
//	 * purposes and can be removed without affecting the COModel class.
//	 */
//	private void exampleSim()
//	{
//		Double2D targ = new Double2D(0,0);
//
//		//Add target to the environment at a random location
//		int targetID = getNewID();
//		Target t = new Target(targetID);
//		addEntity(t, targ, false);
//
//		CircleObstacle b = new CircleObstacle(getNewID(), 4);
//		addEntity(b, new Double2D(50, 50), false);
//
//		Car AV = new Car(getNewID(), targetID);
//		addEntity(AV, new Double2D(100, 100), true);
//	}
//     
//	
	/**
	 * A method which starts the simulation
	 * 
	 * @param args the arguments for the doLoop method
	 */
    public void beginSimulation(String[] args)
    {
   		doLoop(COModel.class, args);

    } 
		
	
	/**
	 * A method which provides a different number each time it is called, this is
	 * used to ensure that different entities are given different IDs
	 * 
	 * @return a unique ID number
	 */
	public int getNewID()
	{
		int t = newID;
		newID++;
		return t;
	}
	
	
	/**
	 * HH 21/7/14 - Method to provide the location of the target, original required for logging information
	 * in the event that the TIMEOUT - event occurs.
	 */
	public Double2D getTargetLoc()
	{
		if (allEntities.size() > 0) {
			if (((Entity) allEntities.get(0)).type == Constants.TTARGET) {
				return ((Entity) allEntities.get(0)).getLocation();
			} else {
				return new Double2D(-1,-1);
			}
		} else {
			return new Double2D(-1,-1);
		}
	}
			
	
	/**
	 * A method which adds all of the entities to the simulations environment.
	 */
	public void loadEntities()
	{
		for(int i = 0; i < allEntities.size(); i++)
		{
			environment.setObjectLocation((Entity) allEntities.get(i), ((Entity) allEntities.get(i)).getLocation());
			//getNewID(); //need to count off the ids inputted to make sure they aren't reused.
		}
		//allEntities.clear();
	}
	
	
	/**
	 * A method which adds an entity to the simulation which is to be ran
	 * 
	 * @param e the entity to be added
	 * @param location the location to add the entity
	 * @param schedule does the entity require being added to the simulations schedule
	 */
//	public void addEntity(Entity e, Double2D location, boolean schedule)
//	{
//		e.setLocation(location);
//		//environment.setObjectLocation(e, location);
//		allEntities.add(e);
//
//		if (schedule == true)
//		{
//			toSchedule.add(e);
//		}
//		
//
//	}
	
	
	/**
	 * A method which adds all the entities marked as requiring scheduling to the
	 * schedule for the simulation
	 */
	public void scheduleEntities()
	{
		//loop across all items in toSchedule and add them all to the schedule
		for(int i = 0; i < toSchedule.size(); i++)
		{
			schedule.scheduleRepeating((Entity) toSchedule.get(i));
		}
		//toSchedule.clear();
		
		try{
			
			//HH 15.7.14 - Replaced with UGVs
			//Bag trackedCars = (Bag)this.cars.clone();
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
	 * the schedule and environment of any entities, to be called between simulations.
	 * 
	 * This method resets the newID counter so should NOT be called during a run.
	 */
	public void reset()
	{
		newID = 0;
		obstacles.clear();
		cars.clear();
		toSchedule.clear();
		allEntities.clear();
		environment.clear(); //clear the environment
		// HH 30/4/14 Update for road model
		terrainMap.setTo(Constants.NORMAL);
		roadMap.setTo(Constants.NOTROAD);
		roads.clear();
		junctions.clear();
		ugvs.clear();
		// HH 17.6.14 Update for road markings
		roadMarkingMap.setTo(Constants.NOPAINT);
		// HH end

	}
	
    /*
     * HH 22/7/14 - Return a string which represents the fault array, and can be output to e.g. the Accident Log
     */
	public String HgetFaultArrayAsString() {
		
		String retString = new String("Fault Array: "); // Start by describing what is being output
		
		for (int i = 0; i < faultArray.length; i++) {
			retString += i + "=" + faultArray[i] + ", ";
		}
		
		return retString;
	}
	
    /*
     * HH 22/7/14 - Return a string which represents the faultsCalled array, and can be output to e.g. the Accident Log
     */
	public String HgetFaultCalledAsString() {
		
		String retString = new String("Faults Called: "); // Start by describing what is being output
		
		for (int i = 0; i < faultCalled.length; i++) {
			retString += i + "=" + faultCalled[i] + ", ";
		}
		
		return retString;
	}
	
    /*
     * HH 22/7/14 - Return a string which represents which UGV targets have been found, and can be output to e.g. the Accident Log
     */
	public String HgetUGVTargetSuccess() {
		
		String retString = new String("Targets Found: "); // Start by describing what is being output
		
		for (int i = 0; i < ugvs.size(); i++) {
			retString += i + "=" + ((UGV)ugvs.get(i)).getTargetFound() + ", ";
		}
		
		return retString;
	}	

    /*
     * HH 22/7/14 - Return a string which represents the fault array, and can be output to e.g. the Accident Log
     */
	private void initFaultArray(double percentageFaults) {

		// Either, loop for the appropriate number of faults (%Faults * TotalFaults) and ensure that choose 
		// a new empty slot each time (at random) - need an internal while loop so we get a repeat if we choose
		// a slot that has already been set to true. OR Loop through the array and at each 'slot' flip a biased coin
		// to choose whether to set it to true.  May need multiple pass-throughs to get to the right number of faults
		// though.  Second approach may be more efficient?
		
		
		// Loop through the array and 'flip a biased coin' to determine whether to set the entry to true
		int noFaultsRequired = 0;
		int noFaultsSet = 0;
		
		noFaultsRequired = (int) Math.round(Constants.MAX_FAULTS * percentageFaults);
		
		while (noFaultsSet < noFaultsRequired) { 
		
			for (int i = 0; i < faultArray.length; i++) {
				
				// Only need to check on this iteration if this fault is not active
				if (faultArray[i] == false) {

					// Repeat the check for exceeding the number of faults in case we have exceeded the 
					// required number on the previous iteration
					if (noFaultsSet < noFaultsRequired) { 

						if (random.nextDouble() < percentageFaults) {
							faultArray[i] = true; // set the fault to 'active' in the array
							noFaultsSet ++; // increase the total faults found
						}
					} else {
						// Exit, we've set it up
						return;
					}
				}
			}
		}
	}
	
	
	/**
	 * A method which creates a discrete map of where obstacles are in the environment
	 * which can be used for outputting a visual representation of the obstacles
	 * when the simulation is run with a UI
	 */
	private void buildObstacleMap()
	{
		//[TODO] This will need to be called more than once if the simulations can have moving obstacles
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
		
		for (int i = 0; i < xInt; i++) {
			for (int j = 0; j < yInt; j++)
			{
				coordinate = new Double2D(((double) i) / obstacleMapResolution, ((double) j) / obstacleMapResolution);
				//loop over all coordinates and check if there is an obstacle there
				if (obstacleAtPoint(coordinate, obstacles))
				{
					//System.out.println("setting the point at x:" + Integer.toString(i) + " y:" + Integer.toString(j) + " to 1");
					obstacleMap.field[i][j] = 1;
				}				
			}
		}
	}

//	/**
//	 * A method which creates a discrete map of where dumb cars are in the environment
//	 * which can be used for outputting a visual representation of the obstacles
//	 * when the simulation is run with a UI
//	 */
//	public void buildDumbCarMap()
//	{
//		//[TODO] This will need to be called more than once if the simulations can have moving obstacles
//		Bag all = environment.allObjects;
//		Bag cars = new Bag();
//		Double2D coordinate;
//		int xInt = (int) (xDouble * dumbCarMapResolution);
//		int yInt = (int) (yDouble * dumbCarMapResolution);
//		dumbCarMap = new IntGrid2D(xInt, yInt, 0);
//		
//		for (int i = 0; i < all.size(); i++)
//		{
//			if (((Entity) (all.get(i))).getType() == Constants.DUMBCAR)
//			{
//				cars.add(all.get(i));
//			}
//			
//		}
//		
//		for (int i = 0; i < xInt; i++) {
//			for (int j = 0; j < yInt; j++)
//			{
//				coordinate = new Double2D(((double) i) / dumbCarMapResolution, ((double) j) / dumbCarMapResolution);
//				//loop over all coordinates and check if there is an obstacle there
//				if (dumbCarAtPoint(coordinate, cars))
//				{
//					//System.out.println("setting the point at x:" + Integer.toString(i) + " y:" + Integer.toString(j) + " to 1");
//					dumbCarMap.field[i][j] = 1;
//				}				
//			}
//		}
//	}
	
	/**
	 * A method which draws two circles onto the terrain map for different terrain types.
	 */
	public void buildWallMap()
	{
		int xInt = (int) (xDouble * terrainMapResolution);
		int yInt = (int) (yDouble * terrainMapResolution);
		
		// Hh 30.7.14 Swapped the xInt and yInt over in both methods below as seemed to be wrong way around.
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
	
	
	public void buildTerrainMap()
	{

		int xInt = (int) (xDouble * wallMapResolution);
		int yInt = (int) (yDouble * wallMapResolution);
		
		for (int i = 0; i < xInt; i++) {
			for (int j = 0; j < yInt; j++)
			{
				if ((((i - (50 * terrainMapResolution)) * (i - (50 * terrainMapResolution))) + ((j - (50 * terrainMapResolution)) * (j - (50 * terrainMapResolution)))) < ((20 * terrainMapResolution) * (20 * terrainMapResolution)))
				{
					terrainMap.field[i][j] = Constants.GRAVEL;
				} else if ((((i - (50 * terrainMapResolution)) * (i - (50 * terrainMapResolution))) + ((j - (50 * terrainMapResolution)) * (j - (50 * terrainMapResolution)))) < ((40 * terrainMapResolution) * (40 * terrainMapResolution))){
					terrainMap.field[i][j] = Constants.ICE;
				} else {
					terrainMap.field[i][j] = Constants.NORMAL;
				}
			}
		}
		
	}
	
	/**
	 * Method to construct a road network layout 
	 * 
	 * @author HH
	 */
	public void buildRoadMap()
	{
		// initialise the whole area to non-road and then add in the roads
		int xInt = (int) (xDouble * roadMapResolution);
		int yInt = (int) (yDouble * roadMapResolution);
		
		Double2D coordinate;
		
		for (int i = 0; i < xInt; i++) {
			for (int j = 0; j < yInt; j++)
			{
				coordinate = new Double2D(((double) i) / roadMapResolution, ((double) j) / roadMapResolution);
				//loop over all coordinates and check if there is an obstacle there
				if (roadAtPoint(coordinate, roads))
				{
					//System.out.println("setting the point at x:" + Integer.toString(i) + " y:" + Integer.toString(j) + " to 1");
					roadMap.field[i][j] = Constants.SINGLEONEWAY;
				} else {
					roadMap.field[i][j] = Constants.NOTROAD;
				}	
			}
		}
	}
	
	/**
	 * Method to construct junctions within the road network layout 
	 * 
	 * @author HH
	 */
	public void buildJunctionMap()
	{
		// initialise the whole area to non-road and then add in the roads
		int xInt = (int) (xDouble * junctionMapResolution);
		int yInt = (int) (yDouble * junctionMapResolution);
		
		Double2D coordinate;
		
		for (int i = 0; i < xInt; i++) {
			for (int j = 0; j < yInt; j++)
			{
				coordinate = new Double2D(((double) i) / junctionMapResolution, ((double) j) / junctionMapResolution);
				//loop over all coordinates and check if there is an obstacle there
				junctionMap.field[i][j] = junctionTypeAtPoint(coordinate, junctions);
			}
		}
	}

	/**
	 * Method to construct junctions within the road network layout 
	 * 
	 * @author HH
	 */
	public void buildJctApproachMap()
	{
		// initialise the whole area to non-road and then add in the roads
		int xInt = (int) (xDouble * jctApproachMapResolution);
		int yInt = (int) (yDouble * jctApproachMapResolution);
		
		Double2D coordinate;
		
		for (int i = 0; i < xInt; i++) {
			for (int j = 0; j < yInt; j++)
			{
				coordinate = new Double2D(((double) i) / jctApproachMapResolution, ((double) j) / jctApproachMapResolution);
				//loop over all coordinates and check if there is an obstacle there
				if (junctionAppAtPoint(coordinate, junctions)) {
					jctApproachMap.field[i][j] = 1;
				} else {
					jctApproachMap.field[i][j] = 0;
				}
			}
		}
	}
	
	/**
	 * Method to 'draw' road markings on the road network layout
	 * 
	 * @author HH
	 */
	public void buildRoadMarkingsMap()
	{
		// initialise the whole area to non-road and then add in the roads
		int xInt = (int) (xDouble * roadMarkingMapResolution);
		int yInt = (int) (yDouble * roadMarkingMapResolution);
		
		Double2D coordinate;
		
		for (int i = 0; i < xInt; i++) {
			for (int j = 0; j < yInt; j++)
			{
				coordinate = new Double2D(((double) i) / roadMarkingMapResolution, ((double) j) / roadMarkingMapResolution);
				
				//loop over all coordinates and check if there are any road markings there
				roadMarkingMap.field[i][j] = roadMarkingAtPoint(coordinate, roads);

				// TO DO - in the future, we may want to suspend line drawing through junctions, or perhaps 
			}
		}
	}
	
	/**
	 * A method which returns a true or false value depending on if an obstacle is
	 * at the coordinate provided
	 * 
	 * @param coord the coordinate to check
	 * @param obstacles the obstacles to be checked
	 * @return 
	 */
	public boolean obstacleAtPoint(Double2D coord, Bag obstacles)
	{
		for (int i = 0; i < obstacles.size(); i++)
		{
			//for all of the obstacles check if the provided point is in it
			if (((Obstacle) (obstacles.get(i))).inShape(coord)) //[TODO] this might not work it depends on if whatever the object is will override inShape
			{
				return true;
			}
		}
		//at this point no cross over has been detected so false should be returned
		return false;
	}
	
	/**
	 * A method which returns a true or false value depending on if a dumbCar is
	 * at the coordinate provided
	 * 
	 * @param coord the coordinate to check
	 * @param cars the Cars to be checked
	 * @return 
	 */
	public boolean dumbCarAtPoint(Double2D coord, Bag cars)
	{
		for (int i = 0; i < cars.size(); i++)
		{
			//for all of the obstacles check if the provided point is in it
			if (((DumbCar) (cars.get(i))).inShape(coord)) //[TODO] this might not work it depends on if whatever the object is will override inShape
			{
				return true;
			}
		}
		//at this point no cross over has been detected so false should be returned
		return false;
	}
	
	/**
	 * A method which returns a true or false value depending on if an obstacle is
	 * too close to the coordinate provided - too close is defined as an x or y separation
	 * of less than Constants.OBSTACLE_LENGTH when calculate as absolute displacement in
	 * each direction.  The supplied coordinate is the tentative centre of a new obstacle
	 * to be inserted, and is compared to the centre of all existing obstacles.  As a final
	 * check, we only reject an obstacle if it is on the same road as the supplied coordinate.
	 * 
	 * @param coord the coordinate to check
	 * @param obstacles the obstacles to be checked
	 * @return 
	 */
	public boolean obstacleNearPoint(Double2D coord, Bag obstacles, Bag roads)
	{
		Double2D testLoc;
		
		for (int i = 0; i < obstacles.size(); i++)
		{
			testLoc = ((Obstacle) (obstacles.get(i))).getLocation();
			
			//for all of the obstacles check if the provided point is in it
			//[TODO] this might not work in all cases, not 100% sure of maths behind it - could be overly strong
			if (Math.abs(testLoc.x - coord.x) < Constants.OBSTACLE_LENGTH && Math.abs(testLoc.y - coord.y) < Constants.OBSTACLE_LENGTH) 
			{
				// Check the coordinates are on the same roads
				if (getRoadIdAtPoint(testLoc, roads) == getRoadIdAtPoint(coord, roads)) {
					return true;
				}
			}
		}
		//at this point no cross over has been detected so false should be returned
		return false;
	}	
	
	
	/**
	 * A method which checks the terrain at a given location
	 * 
	 * @param coord coordinate to check the terrain at
	 * @return 
	 */
	public int terrainAtPoint(Double2D coord)
	{
		int x = (int)coord.x * terrainMapResolution;
		int y = (int)coord.y * terrainMapResolution;
		int tTerrain =0;
		try
		{
			tTerrain = terrainMap.get(x, y);
		}
		catch(ArrayIndexOutOfBoundsException e)
		{
			System.out.println("too fast to detect the clash with wall accident, go on!");
		}
		return tTerrain;
	}
	
	/**
	 * Method to check for roads 
	 * 
	 * @author HH
	 */
	public boolean roadAtPoint(Double2D coord, Bag roads)
	{
		for (int i = 0; i < roads.size(); i++)
		{
			//for all of the obstacles check if the provided point is in it
			if (((Road) (roads.get(i))).inShape(coord)) //[TODO] this might not work it depends on if whatever the object is will override inShape
			{
				return true;
			}
		}
		//at this point no cross over has been detected so false should be returned
		return false;
	}

	/**
	 * Method to check whether the road is N/S at the point we are interested in (cf. E/W) 
	 * 
	 * @author HH
	 */
	public boolean isRoadNSAtPoint(Double2D coord, Bag roads)
	{
		for (int i = 0; i < roads.size(); i++)
		{
			//for all of the obstacles check if the provided point is in it
			if (((Road) (roads.get(i))).inShape(coord)) //[TODO] this might not work it depends on if whatever the object is will override inShape
			{
				return ((Road) (roads.get(i))).getIsNS();
			}
		}
		//at this point no cross over has been detected so false should be returned
		return false; // This should not be called as we should only call this after we have put an obstacle on the road
	}

	/**
	 * Method to return the id number of the road a coordinate is located on (used when adding an Obstacle)
	 * 
	 * @author HH 13.8.14
	 */
	public int getRoadIdAtPoint(Double2D coord, Bag roads)
	{
		for (int i = 0; i < roads.size(); i++)
		{
			//for all of the obstacles check if the provided point is in it
			if (((Road) (roads.get(i))).inShape(coord)) //[TODO] this might not work it depends on if whatever the object is will override inShape
			{
				return ((Road) (roads.get(i))).getID();
			}
		}
		//at this point no cross over has been detected so false should be returned
		return -1; // This should not be called as we should only call this after we have put an obstacle on the road
	}

	/**
	 * Method to return the lane direction (N/E or S/W) of the road a coordinate is located on
	 * 
	 * @author HH 4.9.14
	 */
	public int getLaneDirAtPoint(Double2D coord, Bag roads)
	{
		for (int i = 0; i < roads.size(); i++)
		{
			//for all of the roads check if the provided point is in it
			if (((Road) (roads.get(i))).inShape(coord)) //[TODO] this might not work it depends on if whatever the object is will override inShape
			{
				return ((Road) (roads.get(i))).getLane(coord);
			}
		}
		//at this point no cross over has been detected so false should be returned
		return -1; // This should not be called as we should only call this after we have put an obstacle on the road
	}
	
	/**
	 * Method to check for roads intersecting with each other.  
	 * 
	 * @author HH
	 */
	public boolean roadAtPoint(Rectangle2D.Double newRoad, Bag roads, int currentRoad)
	{
		for (int i = 0; i < roads.size(); i++)
		{
			//for all of the roads (except the one we are creating the junction with) check if the provided point is in it
			if (((Road) (roads.get(i))).inShape(newRoad) && (i != currentRoad)) //[TODO] this might not work it depends on if whatever the object is will override inShape
			{
				return true;
			}
		}
		//at this point no cross over has been detected so false should be returned
		return false;
	}
	
	/**
	 * Method to check for roads intersecting with each other.  
	 * 
	 * @author HH 17.6.14
	 */
	public int roadMarkingAtPoint(Double2D coord, Bag roads)
	{
		// This method is called for each point on the map, and so each point is compared with all
		// roads and all possible line locations (nearside, centre, offside) to see if it falls on 
		// any of them.  This method will return as soon as it finds one instance of 'white paint'
		// it will not continue the search.
		for (int i = 0; i < roads.size(); i++)
		{
			//for all of the roads (except the one we are creating the junction with) check if the provided point is in it
			if ((((Road) (roads.get(i))).getLine(LineType.SESIDE)).contains(coord.x, coord.y)) {
				return Constants.WHITEPAINT;
			}
			
			if ((((Road) (roads.get(i))).getLine(LineType.CENTRE)).contains(coord.x, coord.y)) {
				return Constants.WHITERPAINT;
			} 
			
			if ((((Road) (roads.get(i))).getLine(LineType.NWSIDE)).contains(coord.x, coord.y)) {
				return Constants.WHITEPAINT;
			} 
				
		}
		//at this point no cross over has been detected so false should be returned
		return Constants.NOPAINT;
	}
	
	/**
	 * Method to work out the closest 'in lane' position for this vehicle to be initialised.  
	 * 
	 * @author HH 18.6.14
	 */
	public initialInfo snapToLane(double x, double y)
	{
		// Create a cross that is centred at x,y and find the closest intersection with either offside or nearside 
		// road markings and use this to choose a vehicle location that is roadWidth/4 away from the intersection
		Line2D.Double xLine = new Line2D.Double(x-Road.roadWidth, y, x+Road.roadWidth, y);
		Line2D.Double yLine = new Line2D.Double(x, y-Road.roadWidth, x, y+Road.roadWidth);
		Line2D.Double lineSE = new Line2D.Double(); // HH 30.9.14 Changed name
		Line2D.Double lineNW = new Line2D.Double(); // HH 30.9.14 Added
		
		Double2D nearsideIntersect;
		Double2D offsideIntersect;
		double nearsideBearing = 400;
		double offsideBearing = 400;
		boolean found = false;
		
		Double2D nearestIntersect = new Double2D(0,0);
		double nearestStartBearing = 400; // NOTE - This is an invalid measure, so we can test for it later
		
		// HH 15.10.14 - Allow for a small offset on either side of the 'thin lines' to allow for the assumed
		// width of the line, and the offset between the kerb and the line.
		double lineBuffer = Constants.ROADEDGEOFFSET + Constants.ROADEDGINGWIDTH;
		
		// HH 30.9.14 - The original algorithm allowed intersections with roads that the vehicle was
		// not actually travelling on (i.e. if close to a cross-roads), but we can't add an onRoad check
		// as that would not be appropriate for the UGV, it would need to use the sensor return values from
		// the road marking detection to 'work out' which road markings are relevant for the road it is on.
		// Hypothesis: If intersections are detected with *both* of the road markings on a road, and the 
		// location of the vehicle lies between them then everything is on the same road, and we can just use
		// whichever of the intersections is closest to calculate the lane we want to be in
		
		
		// Loop for all roads and outside line locations (nearside, offside) to see if either line intersects
		// with any of them.  This method will return as soon as it finds one instance of 'white paint'
		// it will not continue the search.
		for (int i = 0; i < roads.size(); i++)
		{
			// reset nearestIntersect params
			nearsideIntersect = new Double2D(0,0);
			nearsideBearing = 400;
			found = false;
			
			//check the nearside lane on this road against the xLine, and the yLine		
			lineSE = ((Road) (roads.get(i))).getThinLine(LineType.SESIDE);
			if (lineSE.intersectsLine(xLine)) {
				// As we know they intersect, and we know xLine is the horizontal line, that gives 
				// us the y, and therefore the other line provides the x.  Really just need a median method though.				
				nearsideIntersect = new Double2D(lineSE.x1, xLine.y1);
				nearsideBearing = 180; // travelling North
			} else if (lineSE.intersectsLine(yLine)){
				// As we know they intersect, and we know yLine is the vertical line, that gives 
				// us the x, and therefore the other line provides the y.  Really just need a median method though.				
				nearsideIntersect = new Double2D(yLine.x1, lineSE.y1);	
				nearsideBearing = 90; // travelling East
			}
						
			// reset offsideIntersect params
			offsideIntersect = new Double2D(0,0);
			offsideBearing = 400;
			
			// check the offside lane on this road against the xLine, and the yLine
			lineNW = ((Road) (roads.get(i))).getThinLine(LineType.NWSIDE);
			if (lineNW.intersectsLine(xLine)) {
				// As we know they intersect, and we know xLine is the horizontal line, that gives 
				// us the y, and therefore the other line provides the x.  Really just need a median method though.				
				offsideIntersect = new Double2D(lineNW.x1, xLine.y1);
				offsideBearing = 0; // travelling South
			} else if (lineNW.intersectsLine(yLine)){
				// As we know they intersect, and we know yLine is the vertical line, that gives 
				// us the x, and therefore the other line provides the y.  Really just need a median method though.				
				offsideIntersect = new Double2D(yLine.x1, lineNW.y1);
				offsideBearing = 270; // travelling West			
			}
								
			// HH 30.9.14 - If an intersection has occurred with both of the road marking lines, see
			// if the vehicle is located inbetween so we can exit the method
			if (!(offsideIntersect.x == 0 && offsideIntersect.y == 0) && !(nearsideIntersect.x == 0 && nearsideIntersect.y == 0))
			{
				// HH 15.10.14 - Added a buffer around each of the lines when testing to account for the distance between
				// the line and the kerb (see above)
				if (offsideIntersect.x == nearsideIntersect.x)
				{
					// Intersections are occurring with yLine (vertical) so we need to compare our y params
					if ((offsideIntersect.y - lineBuffer < y && nearsideIntersect.y + lineBuffer > y) || 
							(offsideIntersect.y + lineBuffer > y && nearsideIntersect.y - lineBuffer < y))
					{
						found = true;
					}
				} else
				{
					// Intersections are occurring with xLine (horizontal) so we need to compare our x params
					if ((offsideIntersect.x - lineBuffer < x && nearsideIntersect.x + lineBuffer > x) || 
							(offsideIntersect.x + lineBuffer > x && nearsideIntersect.x - lineBuffer < x))
					{
						found = true;
					}
				}

				// HH 15.10.14 - Added a buffer around each of the lines when testing to account for the distance between
				// the line and the kerb (see above)
//				if (offsideIntersect.x == nearsideIntersect.x)
//				{
//					// Intersections are occurring with yLine (vertical) so we need to compare our y params
//					if ((offsideIntersect.y < y && nearsideIntersect.y > y) || (offsideIntersect.y > y && nearsideIntersect.y < y))
//					{
//						found = true;
//					}
//				} else
//				{
//					// Intersections are occurring with xLine (horizontal) so we need to compare our x params
//					if ((offsideIntersect.x < x && nearsideIntersect.x > x) || (offsideIntersect.x > x && nearsideIntersect.x < x))
//					{
//						found = true;
//					}
//				}
				
				// HH 30.9.14 Update nearestIntersect/nearestStartBearing if necessary (check nearside first and then offside)
				if (found == true)
				{
					if ((nearsideIntersect.distance(x,y) < nearestIntersect.distance(x,y) && !(nearsideIntersect.x == 0 && nearsideIntersect.y == 0)) || (nearestIntersect.x == 0 && nearestIntersect.y == 0)) {
						nearestIntersect = new Double2D(nearsideIntersect.x, nearsideIntersect.y);
						nearestStartBearing = nearsideBearing; // overwrite with matching value
					}

					if (offsideIntersect.distance(x,y) < nearestIntersect.distance(x,y) && !(offsideIntersect.x == 0 && offsideIntersect.y == 0) || (nearestIntersect.x == 0 && nearestIntersect.y == 0)) {
						nearestIntersect = new Double2D(offsideIntersect.x, offsideIntersect.y);
						nearestStartBearing = offsideBearing; // overwrite with matching value
					}
					
					break; // don't need to keep looking
				}
			}
		}
		
//		// TO DO - Remove - debugging
//		if (nearestIntersect.x == 0 && nearestIntersect.y == 0)
//		{
//			found = false;
//		}
		
		// Use the bearing to work out which direction to offset in, in order to work out the appropriate starting point
		// HH 27.8.14 - Updated as per calcRMOffset so that compensate for roadmarking offset and width when determining offset,
		// will make vehicle more central to lane
		double offset = Road.roadWidth/4 - Constants.ROADEDGEOFFSET - Constants.ROADEDGINGWIDTH;
		
		switch ((int) nearestStartBearing) {
			case 0 : nearestIntersect = new Double2D(nearestIntersect.x - offset, nearestIntersect.y);
				break;
			case 90 : nearestIntersect = new Double2D(nearestIntersect.x, nearestIntersect.y + offset);
				break;
			case 180 : nearestIntersect = new Double2D(nearestIntersect.x + offset, nearestIntersect.y);
				break;
			case 270 : nearestIntersect = new Double2D(nearestIntersect.x, nearestIntersect.y - offset);
				break;
		}
		
		initialInfo retVal = new initialInfo(nearestIntersect, nearestStartBearing);
		
		return retVal;
	}	
	
	/**
	 * Method to work out the closest 'in lane' position for this vehicle to be initialised.  
	 * 
	 * @author HH 18.6.14
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
		
		// Loop for all roads and outside line locations (nearside, offside) to see if either line intersects
		// with any of them.  This method will return as soon as it finds one instance of 'white paint'
		// it will not continue the search.
		for (int i = 0; i < roads.size(); i++)
		{
			// reset nearestIntersect params
			nearsideIntersect = new Double2D(0,0);
			tempBearing = 400;
			
			//check the nearside lane on this road against the xLine, and the yLine		
			lineRM = ((Road) (roads.get(i))).getThinLine(LineType.SESIDE);
			if (lineRM.intersectsLine(xLine)) {
				// As we know they intersect, and we know xLine is the horizontal line, that gives 
				// us the y, and therefore the other line provides the x.  Really just need a median method though.				
				nearsideIntersect = new Double2D(lineRM.x1, xLine.y1);
				tempBearing = 180; // travelling North
			} else if (lineRM.intersectsLine(yLine)){
				// As we know they intersect, and we know yLine is the vertical line, that gives 
				// us the x, and therefore the other line provides the y.  Really just need a median method though.				
				nearsideIntersect = new Double2D(yLine.x1, lineRM.y1);	
				tempBearing = 90; // travelling East
			}
			
			if (nearsideIntersect.distance(x,y) < nearestIntersect.distance(x,y) && !(nearsideIntersect.x == 0 && nearsideIntersect.y == 0) || (nearestIntersect.x == 0 && nearestIntersect.y == 0)) {
				nearestIntersect = new Double2D(nearsideIntersect.x, nearsideIntersect.y);
				nearestStartBearing = tempBearing; // overwrite with matching value
			}
			
			// reset offsideIntersect params
			offsideIntersect = new Double2D(0,0);
			tempBearing = 400;
			
			// check the offside lane on this road against the xLine, and the yLine
			lineRM = ((Road) (roads.get(i))).getThinLine(LineType.NWSIDE);
			if (lineRM.intersectsLine(xLine)) {
				// As we know they intersect, and we know xLine is the horizontal line, that gives 
				// us the y, and therefore the other line provides the x.  Really just need a median method though.				
				offsideIntersect = new Double2D(lineRM.x1, xLine.y1);
				tempBearing = 0; // travelling South
			} else if (lineRM.intersectsLine(yLine)){
				// As we know they intersect, and we know yLine is the vertical line, that gives 
				// us the x, and therefore the other line provides the y.  Really just need a median method though.				
				offsideIntersect = new Double2D(yLine.x1, lineRM.y1);
				tempBearing = 270; // travelling West			
			}
						
			if (offsideIntersect.distance(x,y) < nearestIntersect.distance(x,y) && !(offsideIntersect.x == 0 && offsideIntersect.y == 0) || (nearestIntersect.x == 0 && nearestIntersect.y == 0)) {
				nearestIntersect = new Double2D(offsideIntersect.x, offsideIntersect.y);
				nearestStartBearing = tempBearing; // overwrite with matching value
			}
			
		}
		
		// [TODO] Use the bearing to work out which direction to offset in, in order to work out the appropriate starting point
		switch ((int) nearestStartBearing) {
			case 0 : nearestIntersect = new Double2D(nearestIntersect.x - (Constants.OBSTACLE_WIDTH/2), nearestIntersect.y);
				break;
			case 90 : nearestIntersect = new Double2D(nearestIntersect.x, nearestIntersect.y + (Constants.OBSTACLE_WIDTH/2));
				break;
			case 180 : nearestIntersect = new Double2D(nearestIntersect.x + (Constants.OBSTACLE_WIDTH/2), nearestIntersect.y);
				break;
			case 270 : nearestIntersect = new Double2D(nearestIntersect.x, nearestIntersect.y - (Constants.OBSTACLE_WIDTH/2));
				break;
		}
		
		return nearestIntersect; // TO DO - HH 7.8.14 - There may be an easier way to do this...
	}	
	
	
	/**
	 * Method to check for junctions 
	 * HH 16.10.14 - Updated so that returns the ID of the junction in which the point is located
	 * - if no junction is found, returns 0
	 * - if multiple junctions are found, returns -1
	 * - otherwise returns the ID of the junction
	 * 
	 * @author HH
	 */
	public int junctionAtPoint(Double2D coord, Bag junctions)
	{
		int jctID = 0;
		
		for (int i = 0; i < junctions.size(); i++)
		{
			//for all of the obstacles check if the provided point is in it
			if (((Junction) (junctions.get(i))).inShape(coord)) //[TODO] this might not work it depends on if whatever the object is will override inShape
			{
				// HH 16.10.14 Check to see if the id has always been set, so we know if there are multiple
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
	 * HH 16.10.14 Method to unOccupy the specified junction - can't use Bag.get() as that refers to the idx, not the ID 
	 * 
	 * @author HH
	 */
	public void unOccupyJunction(int jctID, Bag junctions)
	{
		Junction tempJunction;
		
		for (int i = 0; i < junctions.size(); i++)
		{
			//for all of the obstacles check if the provided point is in it
			tempJunction = (Junction) (junctions.get(i));
			if (tempJunction.getID() == jctID) //[TODO] this might not work it depends on if whatever the object is will override inShape
			{
				tempJunction.unOccupy();
			}
		}
	}	
	
	/**
	 * Method to check for junctions, returns junction type, incl. 0 if no junction 
	 * 
	 * @author HH
	 */
	public int junctionTypeAtPoint(Double2D coord, Bag junctions)
	{
		for (int i = 0; i < junctions.size(); i++)
		{
			//for all of the obstacles check if the provided point is in it
			if (((Junction) (junctions.get(i))).inShape(coord)) //[TODO] this might not work it depends on if whatever the object is will override inShape
			{
				return ((Junction) junctions.get(i)).getType();
			}
		}
		//at this point no cross over has been detected so false should be returned
		return Constants.NOJUNCTION;
	}
	
	/**
	 * Method to check for junction approaches
	 * 
	 * @author HH
	 */
	public boolean junctionAppAtPoint(Double2D coord, Bag junctions)
	{
		for (int i = 0; i < junctions.size(); i++)
		{
			//for all of the obstacles check if the provided point is in it
			if (((Junction) (junctions.get(i))).inApproach(coord)) //[TODO] this might not work it depends on if whatever the object is will override inShape
			{
				return true;
			}
		}
		//at this point no cross over has been detected so false should be returned
		return false;
	}
	
	/**
	 * Method to check for junction approaches
	 * 
	 * @author HH
	 */
	public boolean junctionExitAtPoint(Double2D coord, Bag junctions)
	{
		for (int i = 0; i < junctions.size(); i++)
		{
			//for all of the obstacles check if the provided point is in it
			if (((Junction) (junctions.get(i))).inExit(coord)) //[TODO] this might not work it depends on if whatever the object is will override inShape
			{
				return true;
			}
		}
		//at this point no cross over has been detected so false should be returned
		return false;
	}
	
    public void dealWithTermination()
	{
    	int noActiveAgents =0;
    	//System.out.println(COModel.toSchedule.size());
    	for(Object o: this.toSchedule)
    	{
    		if(((Car)o).isActive  && ((Car)o).getType() == Constants.TUGV)
    		{
    			noActiveAgents++;
    		}
    		
    	}
    	//System.out.println("NO. of active cars is: "+ noActiveAgents);
    	
		if(noActiveAgents < 1)
		{
			this.schedule.clear();
			//System.out.println("NO. of active cars is: "+ noActiveAgents+". Game Over!");
			//System.out.println(this.schedule.scheduleComplete());
			this.kill();
		}
	 }
	
    // HH 15.7.14 - Called by AccidentDetector to record the location at which a vehicle
    // has left the road (in case we want to visualise it)
    public void recordOffRoad(Double2D location)
    {
    	int fID = this.getNewID();
    	Failure fp = new Failure(fID, Constants.TFAILURE);
		fp.setLocation(location);
		environment.setObjectLocation(fp, location);	
    }

    // HH 16.7.14 - Called by AccidentDetector to record where a vehicle has crossed
    // a line
    public void recordCrossLine(Double2D location)
    {
    	int fID = this.getNewID();
    	Failure fp = new Failure(fID, Constants.TFAILURE);
		fp.setLocation(location);
		environment.setObjectLocation(fp, location);	
    }
    
    // HH 26.8.14 - Called by AccidentDetector to record where a vehicle has crashed with another vehicle
    public void recordCrash(Double2D location)
    {
    	int fID = this.getNewID();
    	Crash fp = new Crash(fID, Constants.TFAILURE);
		fp.setLocation(location);
		environment.setObjectLocation(fp, location);	
    }
    
    // HH 27.8.14 - Return complexity metrics, as required for sit coverage experiments
    public double HgetMinJctSeparation() // Return minimum distance between junctions (assess all pairs)
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
    
    public double HgetUGVTargetSeparation() // Return initial separation (crow flies) between target and UGV
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

    public double HgetMinTargetObsSeparation() // Return minimum distance between target and all static obstacles
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
    		for (int o=0; o<obstacles.size(); o++)
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
    // HH end - 27.8.14
    
    // HH 2.9.14 Addition of more network/map complexity measures
    public double HgetMinTargetKerbSeparation() // Return minimum separation (crow flies) between target and kerb
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
    						tempVal = Math.min(Math.abs(targetLoc.x - tempRoad.x1+Road.roadWidth/2), // HH 22.10.14 -RW/2 -> +RW/2
    								Math.abs(tempRoad.x1+Road.roadWidth/2-targetLoc.x));
    					} else {
    						// Is E/W so compare y val of targetLoc to y values of edge of road
    						tempVal = Math.min(Math.abs(targetLoc.y - tempRoad.y1+Road.roadWidth/2), // HH 22.10.14 -RW/2 -> +RW/2
    								Math.abs(tempRoad.y1+Road.roadWidth/2-targetLoc.y));
    					}

    					// Is this the shortest distance so far?
    					if (tempVal < retVal) {retVal = tempVal;}
    				}
    			}
    		}
    	}
		
		return retVal;		
    }
    
    public double HgetMinTargetCentreSeparation() // Return minimum separation (crow flies) between target and kerb
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
    
    // Return number of obstacles which are separated by between 1 car length and 2 car lengths (actually test for
    // between Obs.length + UGV.length and Obs.length + 2*UGV.length as Obstacle location is centre of obs)
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
    
    // Return proportion of initial separation (crow flies) between target and UGV that is actually on the road surface
    // We make an approximation to this value by incrementally testing locations on the line between the two objects
    // to see if they fall on the road surface.
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
				double angle = Car.calculateAngle(UGVLoc, targetLoc);
				double moveV = Car.yMovement(angle, 1);
				double moveH = Car.xMovement(angle, 1);

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
    // HH - end
    
//	public static void main(String[] args) throws FileNotFoundException
//	{
//		//setting up the logger
//		///*
//		DateFormat dateFormat = new SimpleDateFormat("yyyyMMdd-HHmmss");
//		Date date = new Date();
//		
//		File file = new File("Run" + dateFormat.format(date) + ".txt");
//		PrintStream logger = new PrintStream(new FileOutputStream(file));
//				
//		//System.setOut(logger);
//		//*/
//		
//		//setting up the simulation
//		long time = System.nanoTime();
//		COModel state= new COModel(time, 100, 100, true);
//		System.out.println("Seed: " + Long.toString(time));
//		state.setNoObstacles(18);
//		state.setNoCars(4);
//				
//		COModelBuilder sBuilder = new COModelBuilder(state);
//		sBuilder.generateSimulation(state.getNoObstacles(),state.getNoCars());
//		state.start();
//		
//		//simB.setUpSim(l);
//				
//		//String[] withEndTime = COModelBuilder.addEndTime(args); //add a max number of steps to the arguments passed to the simulation
//        //int endingSteps = Integer.parseInt(args[1]);
////		System.out.println("Initial seed value: " + Long.toString(l));
////		System.out.println("testSim:");
//		do
//		{
//			if (!state.schedule.step(state))
//			{
//				break;
//			}
//		} while(state.schedule.getSteps()< 1000);
//		
//		state.finish();
//		System.out.println("finished :)");
//		//state.beginSimulation(withEndTime);
//		
//	}
    
    /* 
     * HH 9.9.14 - addNewDumbCar() : to be called when an existing dumbCar in the network is terminated
     * due to leaving the road surface.  The new car should be added at a randomly chosen entrance to the 
     * network i.e. a dead-end.
     */
    public void addNewDumbCar()
    {
    	// Choose a junction at random and check whether it is at the edge of the network (will only have one exit)
    	int j = (int) (random.nextDouble() * junctions.size());
    	Double2D entryLoc = ((Junction) junctions.get(j)).getDeadEndEntry();
    	
    	// Loop until we get a junction at the end of the network
    	while (((Junction) junctions.get(j)).isDeadEnd() == false || entryLoc.x == -1)
    	{
    		j = (int) (random.nextDouble() * junctions.size());
    		entryLoc = ((Junction) junctions.get(j)).getDeadEndEntry();
    	}
    	
    	// Now add a car at that location
    	initialInfo startInfo = snapToLane(entryLoc.x, entryLoc.y);

		DumbCar theCar = new DumbCar(getNewID(), carStats, startInfo.startBearing);
		cars.add(theCar);
		theCar.setLocation(startInfo.startLoc); // HH 18.6.14 edited from "new Double2D(x,y)"
		theCar.isSchedulable = true;
		environment.setObjectLocation(theCar, startInfo.startLoc);
		allEntities.add(theCar);
		toSchedule.add(theCar); // This may not actually do anything, but retain just in case
		schedule.scheduleRepeating((Entity) theCar);
    }

    /* 
     * HH 2.10.14 New method to tell us the direction of the lane in which the testPt falls
     */
	public double getRoadDirectionAtPoint(Double2D testPt, Bag roads2) {
	
		// 7.10.14 For now we are still going to assume that the roads are in a grid-formation
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

	// HH 3/11/14 Calculate the separation between each adjacent junction on the network and return a string
	// with comma-separated occurence counts in the following 8 categories (including lower bound but excluding
	// upper: <3m, 3-6m, 6-12m, 12-23m, 23-46m, 46-100m, 100-150m, >150m 
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

}
