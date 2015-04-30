package modeling;

import java.awt.geom.Area;
import java.awt.geom.Line2D;
import java.awt.geom.PathIterator;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintStream;
import modeling.COModel.targetInfo;
import sim.engine.SimState;
import sim.engine.Steppable;
import sim.util.Bag;
import sim.util.Double2D;



/**
 * Class to facilitate detection, tracking and logging of failures which occur during simulation.  
 * 'Step' method checks for different types of failure, including: leaving the road; crossing lines 
 * (e.g. centre, and kerb markings); crashing into obstacles; crashing into cars; and timeout
 * (failure to find target before end of simulation).  Two output files are created, a summary file
 * which is a useful overview, and a detailed log of every accident/failure.  These files are created
 * and populated by methods in this class.
 * 
 * @author xueyi/hh940
 *
 */
public class AccidentDetector implements Constants,Steppable {

	private static final long serialVersionUID = 1L;

	private File accidentLog; // HH 28.8.14 - Init this in the constructor so we can use diff names on batch run
	private File accidentSummary; // HH 28.8.14 - Summary file for one line per experiment for data analysis 
	private COModel sim;
	private PrintStream ps;
	private PrintStream psSummary; // HH 28.8.14 New stream for the summary log file
	private Bag trackedCars=new Bag();
	
	private int noAccidents=0;
	private int AccLeaveRoad = 0;
	private int AccCrossCentre = 0;
	private int AccCrossSE = 0;
	private int AccCrossNW = 0;
	private int AccCrashObs = 0;
	private int AccCrashCar = 0;
	private int AccTimeout = 0;
		
	private String summaryString;
	
	/**
	 * Return total number of accidents logged
	 * @return noAccidents
	 */
	public int getNoAccidents() {
		return noAccidents;
	}

	/**
	 * Sets total number of accidents logged
	 * @param noAccidents (int - number of accidents)
	 */
	public void setNoAccidents(int noAccidents) {
		this.noAccidents = noAccidents;
	}

	/**
	 * Return bag of tracked cars (those that are checked for failures on each timestep).
	 * @return Bag trackedCars
	 */
	public Bag getTrackedCars() {
		return this.trackedCars;
	}

	/**
	 * Sets bag of tracked cars (those that are checked for failures on each timestep).
	 * @param trackedCars (Bag - bag of tracked cars to set)
	 */
	public void setTrackedCars(Bag trackedCars) {
		this.trackedCars = trackedCars;
	}

	/**
	 * Constructor.  Creates accidentLog and accidentSummary output files by combining 
	 * input parameters to generate a more unique filename, and one which can potentially
	 * record the configuration of the run.  For example: in a run where the active faults
	 * are actually specified, percentageFaults will record the fault number; in a run
	 * where faults are selected at random, it will record the percentage of faults which
	 * are being selected to be active.  mapNo is commonly used to record the level of search
	 * effort which is applied, and/or to separate outputs from repetition runs with different 
	 * maps.  Adds header information to the summary file.
	 * @param percentageFaults (double - used to generate a more unique filename)
	 * @param mapNo (long - used to generate a more unique filename)
	 */	
	public AccidentDetector(double percentageFaults, long mapNo){ 
		
		// HH 28.8.14 : NOTE - differences in percentages of faults must be > 1% or files will be overwritten
		accidentLog = new File(Constants.outFilePath + "AccidentLog" + Math.round(percentageFaults * 100) + "_" + mapNo + ".txt");
		
		try{
			ps= new PrintStream(new FileOutputStream(accidentLog));
		}
		catch(FileNotFoundException e)
		{
			System.out.print("Accident log file not found!");
			return;
		}
		
		// HH 28.8.14 - Create new summary log file for logging pertinent data about the run as a whole for easy analysis
		accidentSummary = new File(Constants.outFilePath + "AccidentSummary" + Math.round(percentageFaults * 100) + "_" + mapNo + ".txt");
		
		try{
			psSummary= new PrintStream(new FileOutputStream(accidentSummary));
		}
		catch(FileNotFoundException e)
		{
			System.out.print("Summary file not found!");
			return;
		}		

		summaryString = "";
		
		// HH 28.8.14 - Add some header information to the summary file
		psSummary.println("RandomSeed, ExternalRandomSeed, #Junctions, #Roads, #Obstacles, #Cars, MinJunctionSep, DistanceUGVtoTarget, " +
						  "DistanceTargetToObs, UGVTargetRoadSep, CriticalObsSep, MinTargetCentreSep, MinTargetKerbSep, JctSep<3m, JctSep3-6m, " +
						  "JctSep6-12m, JctSep12-23m, JctSep23-46m, JctSep46-100m, JctSep100-150m, JctSep150m+, #Faults, " +
						  "#Steps, #Accidents, #LeaveRoad, #CrossCentre, #CrossSE, #CrossNW, #CrashObs, #CrashCar, #Timeout");
	}

	/* (non-Javadoc)
	 * @see sim.engine.Steppable#step(sim.engine.SimState)
	 * 
	 * Method called on every step to check the current simulation state for any collisions or
	 * failures relating to the tracked vehicles.  Checks for different types of failure, including: 
	 * leaving the road; crossing lines (e.g. centre, and kerb markings); crashing into obstacles; 
	 * crashing into cars; and timeout (failure to find target before end of simulation).  Accident 
	 * Log file is updated if necessary, as are the accident counts, and for physical interactions, 
	 * the location of the tracked car is reported back to the SimState so the failure location can be 
	 * recorded for display purposes.  In the event of a timeout failure, the simulation is terminated.
	 * @param state (SimState - access to the simulation environment)
	 */
	@Override
	public void step(SimState state) {
		
		sim = (COModel)state;
		ParkedCar obstacle;
		Car car1;
		Car car2;

		// For each tracked car, check for the different types of failure.  
		// Update log file and appropriate accident counts in the event of a collision.  Record a crash
		// at the location of the tracked vehicle and pass this information to the SimState so the crash
		// can be visualised.
		for (int i=0; i<trackedCars.size(); i++)
		{
			car1= (Car)trackedCars.get(i);
			
			// Check for a collision against all obstacles present in the simulation
			for(int j=0; j<sim.obstacles.size(); j++)
			{
				obstacle=(ParkedCar)sim.obstacles.get(j);
				
				if(detectCollisionWithObstacle(car1, obstacle))
				{
					addLog(AccidentType.CLASHWITHOBSTACLE, car1.getID(), sim.schedule.getSteps(), car1.getLocation(), "with obstacle id = "+ obstacle.getID() ); // HH 30/4/14 - Corrected typo
					noAccidents++;
					AccCrashObs++;
					
					// Add visualisation of crashes with obstacles
					sim.recordCrash(car1.getLocation()); // TODO : Update to actual collision point between vehicles
				}
			}
			
			// Check for a collision with the pavement
			if(detectCollisionWithPavement(car1, sim.roads) == true)
			{
				addLog(AccidentType.LEAVEROAD, car1.getID(), sim.schedule.getSteps(), car1.getLocation(), null);
				noAccidents++;
				AccLeaveRoad++;		
				sim.recordOffRoad(car1.location);
			}
			
			// Check for crossing of centre line
			Double2D intersect = detectLineCrossing(car1, sim.roads, LineType.CENTRE, sim.junctions);
			if(intersect.x > -1)
			{
				addLog(AccidentType.CROSSCENTRELINE, car1.getID(), sim.schedule.getSteps(), intersect, null);
				noAccidents++;
				AccCrossCentre++;		
				sim.recordCrossLine(intersect);
			}
			
			// Check for line crossing - NB or EB Kerb line
			Double2D intersectNE = detectLineCrossing(car1, sim.roads, LineType.NESIDE, sim.junctions);
			if(intersectNE.x > -1)
			{
				addLog(AccidentType.CROSS_NE_LINE, car1.getID(), sim.schedule.getSteps(), intersectNE, null);
				noAccidents++;
				AccCrossNW++;		
				sim.recordCrossLine(intersectNE);
			}
			
			// Check for line crossing - SB or WB Kerb line
			Double2D intersectSW = detectLineCrossing(car1, sim.roads, LineType.SWSIDE, sim.junctions);
			if(intersectSW.x > -1)
			{
				addLog(AccidentType.CROSS_SW_LINE, car1.getID(), sim.schedule.getSteps(), intersectSW, null);
				noAccidents++;
				AccCrossSE++;	
				sim.recordCrossLine(intersectSW);
			}
			
			// Check for a collision against all other cars present in the simulation
			for (int j=0; j<sim.cars.size(); j++)
			{
				car2= (Car)sim.cars.get(j);
				if(car2 == car1)
				{
					continue;
				}
				else if (detectCollisionWithOtherCar(car1, car2))
				{
					addLog(AccidentType.CLASHWITHOTHERCAR, car1.getID(), sim.schedule.getSteps(), car1.getLocation(), 
							" Collision with Car: " + car2.getID() + " (" + car2.getLocation() + "." );
					noAccidents++;
					AccCrashCar++;
					sim.recordCrash(car1.getLocation()); // TODO : Update to actual collision point between vehicles
				}
			}
			
		}
		
		// Check limit on the number of steps that have been executed (this is also an failure condition)
		// Update log file and appropriate accident counts in the event of a 'timeout' failure.  Kill the simulation.
		if (sim.schedule.getSteps() > 5000)
		{
			Double2D targetLoc = sim.getTargetLoc();
			String locString = "(" + targetLoc.x + "," + targetLoc.y + ")";
			
			ps.println(AccidentType.TIMEOUT.toString() +"; time: "+ sim.schedule.getSteps() + " steps.  Run Terminated without reaching Target location: " + locString + ".");
			noAccidents++;
			AccTimeout++;
			
			// Now do something to stop the run
			sim.kill(); // NOTE: it should be okay to call this directly as we are 'in a Steppable', so according to the comments
						// by the SimState methods, this is ok.
		}
	}
	
	/**
	 * Add a structured log entry to the AccidentLog file containing the supplied parameters,
	 * including a free-text string.
	 * @param t (AccidentType - Type of accident, for logging)
	 * @param carID (int - Unique id of tracked vehicle involved in failure)
	 * @param step (long - Simulation time step)
	 * @param coor (Double2D - Location of tracked vehicle at time of failure)
	 * @param str (String - A string containing any additional information about the failure)
	 */
	public void addLog(AccidentType t, int carID, long step, Double2D coor, String str)
	{
		ps.println(t.toString() +":- car: "+ carID + "; time: "+ step + " steps; location: (" + coor.x + ", " + coor.y + "); " + str); // HH 30/4/14 Tidied up the formatting here
	}
	
	/** 
	 * Add header information to file to enable accident results to be reproduced (using the two 
	 * seeds that are reported); Describes various possible measures of map complexity.  Also 
	 * ensures that an entry is present in the log for runs which do not produce any accidents.
	 * Sets up the output string for the summary output file by clearing it, and adding any
	 * information that is known about the run a priori (other information is appended at the end
	 * and the string is written to file by method addFooter).
	 * @param state (COModel - Simulation state which can be interrogated for model configuration)
	 **/
	public void addHeader(COModel state)
	{
		sim = (COModel)state;
	
		// Add some more logging
		targetInfo myTargetInfo = sim.HgetTargetSeparations(sim.getTargetLoc());
		
		ps.println("*** New Run, Seed = "+ sim.seed() + "*** External Seed = " + sim.getExternalSeed() + 
			"; Start time = " + Utility.timeToString() + ".");
		
		ps.println("Map/Network Complexity Measures: MinJunctSep = " + sim.HgetMinJctSeparation() +
				   "; UGVTargetSep = " + sim.HgetUGVTargetSeparation() + "; TargetObstacleSep = " +
				   sim.HgetMinTargetObsSeparation() + "; UGVTargetRoadSep = " + sim.HgetUGVTargetRoadSeparation() +
				   "; CriticalObsSep = " + sim.HgetCriticalObsSeparation() + "; MinTargetCentreSep = " +
					sim.HgetMinTargetCentreSeparation() + "; MinTargetKerbSep = " + sim.HgetMinTargetKerbSeparation() + 
					"; DistPrevJctToTarget = " + myTargetInfo.fromPrevJct + "; DistTargetToNextJct = " + myTargetInfo.toNextJct + ".");
		
		// Reset the summary string, and start to add information to it.  We'll 
		// write it all out in one go at the end
		// FORMAT:
		// "RandomSeed, ExternalRandomSeed, #Junctions, #Roads, #Obstacles, #Cars, MinJunctionSep, DistanceUGVtoTarget, " +
		// "DistanceTargetToObs, UGVTargetRoadSep, CriticalObsSep, MinTargetCentreSep, MinTargetKerbSep, "
		// "DistPrevJctToTarget, DistTargetToNextJct, "
		// "#Faults, #Steps, #Accidents, #LeaveRoad, #CrossCentre, #CrossSE, #CrossNW, #CrashObs, #CrashCar");
		
		summaryString = "";
		summaryString += sim.seed() + ", ";
		summaryString += sim.getExternalSeed() + ", ";
		summaryString += sim.junctions.size() + ", ";
		summaryString += sim.roads.size() + ", ";
		summaryString += sim.noObstacles + ", ";
		summaryString += sim.noCars + ", ";
		summaryString += sim.HgetMinJctSeparation() + ", ";
		summaryString += sim.HgetUGVTargetSeparation() + ", ";
		summaryString += sim.HgetMinTargetObsSeparation() + ", ";
		summaryString += sim.HgetUGVTargetRoadSeparation()  + ", ";
		summaryString += sim.HgetCriticalObsSeparation()  + ", ";
		summaryString += sim.HgetMinTargetCentreSeparation()  + ", ";
		summaryString += sim.HgetMinTargetKerbSeparation()  + ", ";
		
		// Add some more logging
		summaryString += myTargetInfo.fromPrevJct + ", ";
		summaryString += myTargetInfo.toNextJct + ", ";
	}
	
	/** 
	 * Add footer information to log file to report whether the target was found, number of accidents,
	 * number of steps and % faults active, along with various other parameters relevant to the 
	 * simulation.  Note that information on which faults are active and how many times each fault
	 * has been called, is added by the COModel.start and COModel.finish respectively.  
	 * The remaining AccidentSummary information, including the total accident count, and the
	 * count for each accident type is formatted and the final output string (which
	 * represents the results of the current simulation) is printed to the output file.  Object 
	 * fields relating to accident tracking are all reset for the next simulation run.
	 * @param state (COModel - Simulation state which can be interrogated for model configuration)
	 **/
	public void addFooter(COModel state)
	{
		sim = (COModel)state;
	
		// Work out the % faults that are active
		int noFaults = 0;
		for (int i = 0; i < Constants.MAX_FAULTS; i++)
		{
			if (sim.getFault(i) == true) {
				noFaults++;
			}
		}
		
		ps.println(state.HgetUGVTargetSuccess()); // Add extra footer re: targets found
		
		ps.println("*** End of run, Seed = "+ sim.seed() + ", External Seed = " + sim.getExternalSeed()  + 
				"*** NoCars = " + sim.noCars + "; CarMaxDeceleration = " + sim.getCarMaxDecceleration() + 
				"; CarMaxSpeed = " + sim.getCarMaxSpeed() + "; CarMaxAcceleration = " + sim.getCarMaxAcceleration()
				+ "; CarMaxTurning = " + sim.getCarMaxTurning() + "; NoObstacles = "
				+ sim.getNoObstacles() + "; NoJunctions = " + sim.getNoJunctions()
				+ "; NoAccidents = " + sim.aDetector.getNoAccidents() + 
				"; NoSteps = " + sim.schedule.getSteps() + "; % Faults = " + noFaults + "/" + 
				Constants.MAX_FAULTS + "=" + (noFaults/Constants.MAX_FAULTS) + "; End time = " + Utility.timeToString() + "."); // HH 30.7.14 Updated to include % faults
		
		// Add the remaining summary log information and write to the file
		// FORMAT:
		// "RandomSeed, #Junctions, #Roads, #Obstacles, #Cars, MinJunctionSep, DistanceUGVtoTarget, DistanceTargetToObs, " +
		// "#Faults, #Steps, #Accidents, #LeaveRoad, #CrossCentre, #CrossSE, #CrossNW, #CrashObs, #CrashCar");
		
		summaryString += sim.HgetIRJunctionSep() + ", "; // HH 6.11.14 Output the different categories of junctionSeparation
		summaryString += noFaults + ", ";
		summaryString += sim.schedule.getSteps()  + ", ";
		summaryString += sim.aDetector.getNoAccidents() + ", "; // TOTAL
		summaryString += sim.aDetector.AccLeaveRoad + ", "; // LeaveRoad
		summaryString += sim.aDetector.AccCrossCentre + ", "; // CrossCentre
		summaryString += sim.aDetector.AccCrossSE + ", "; // CrossSE
		summaryString += sim.aDetector.AccCrossNW + ", "; // CrossNW
		summaryString += sim.aDetector.AccCrashObs + ", "; // CrashObs
		summaryString += sim.aDetector.AccCrashCar + ", "; // CrashCar
		summaryString += sim.aDetector.AccTimeout; // Timeout
				
		psSummary.println(summaryString);
		
		// Reset noAccidents and associated measures
		this.setNoAccidents(0);
		AccLeaveRoad = 0;
		AccCrossCentre = 0;
		AccCrossSE = 0;
		AccCrossNW = 0;
		AccCrashObs = 0;
		AccCrashCar = 0;
		AccTimeout = 0;
	}
	
	/** 
	 * Add a passed string to the log file, alongside the current #steps; used by
	 * COModel.start and COModel.finish to report which faults are active and how
	 * many times each has been called.
	 * @param state (COModel - Simulation state which can be interrogated for current step count)
	 * @param inString (String - to be output to the log file)
	 **/
	public void addString(COModel state, String inString)
	{
		sim = (COModel)state;
	
		ps.println("STEP #" + sim.schedule.getSteps() + " - " + inString + ".");
	}
	
	/** 
	 * Check to see whether there is an overlap between the Shape objects which
	 * represent the footprints of the car and the obstacle.  These Shape objects
	 * are appropriately sized and rotated for the simulation objects that they
	 * represent, and the java.awt.geom.Area.intersect(Area rhs) method is used 
	 * on an Area created from the obstacle Shape, to check for intersection 
	 * with an Area created from the car Shape.  If this intersection is null,
	 * there is no collision.
	 * @param car (Car - in use this would be the tracked car, UGV)
	 * @param obstacle (ParkedCar - object we are checking for collision against)
	 * @return boolean (true if a collision is detected)
	 **/	
	private boolean detectCollisionWithObstacle(Car car, ParkedCar obstacle)
	{
		return obstacle.inShape(car.getShape());
	}
	
	/** 
	 * If the tracked car is stationary, it is not considered to be at fault for
	 * any collision, return false.  Otherwise... check to see whether there 
	 * is an overlap between the Shape objects which
	 * represent the footprints of the trackedCar and car2.  These Shape objects
	 * are appropriately sized and rotated for the simulation objects that they
	 * represent, and the java.awt.geom.Area.intersect(Area rhs) method is used 
	 * on an Area created from the trackedCar Shape, to check for intersection 
	 * with an Area created from the car2 Shape.  If this intersection is null,
	 * there is no collision.  If the intersection is not null, we need to check 
	 * that both cars are active, and that they are not the same vehicle.  This
	 * method then checks a series of conditions under which the trackedCar is
	 * not deemed to be at fault (see code comments no. 1-3 for details): 
	 * @param trackedCar (Car - the tracked car, UGV)
	 * @param car2 (Car - Car we are checking for collision against)
	 * @return boolean (true if a collision is detected)
	 **/
	private boolean detectCollisionWithOtherCar(Car trackedCar, Car car2)
	{
		// Firstly let's see if the UGV is stationary, because if so, we are going to attribute
		// any accident to one of the other cars (so we can ignore it)
		if (trackedCar.getSpeed() == 0)
		{
			return false;
		}
				
		Area shape1 = new Area(trackedCar.getShape());
		Area shape2 = new Area(car2.getShape());
		Area intersection = new Area(shape1);
		intersection.intersect(shape2); // replace shape1 with the intersection of shape1 and shape2
		
		// See if they actually intersected
		if (intersection.isEmpty() == true)
		{
			return false; // they didn't
		} else if (trackedCar.isActive && car2.isActive && (trackedCar.ID != car2.ID)) {
			
			// We want to make sure this is a crash we should be reporting as it is not 
			// fair to blame the UGV when the Dumb Car is entirely at fault i.e. back end swings out and 
			// wipes out UGV during a turn.  
			//
			// It can be quite difficult to find general rules for determining fault, so we will just use
			// simple one(s) and hope that this will eliminate a few unnecessary failures, whilst retaining those
			// in which the UGV is at fault.
			
			Boolean retVal = true;
			
			// Work out simplified compass direction corresponding to each vehicle's direction of 
			// travel; this may differ from the direction of the lane it is currently in. 
			UGV_Direction UGVLaneDir = Utility.getDirection(trackedCar.getDirection());
			int UGVLaneDirSimple;
			if (UGVLaneDir == UGV_Direction.NORTH || UGVLaneDir == UGV_Direction.EAST) {
				UGVLaneDirSimple = 1; 
			} else {
				UGVLaneDirSimple = 2;
			}
			UGV_Direction DCLaneDir = Utility.getDirection(car2.getDirection());
			int DCLaneDirSimple;
			if (DCLaneDir == UGV_Direction.NORTH || DCLaneDir == UGV_Direction.EAST) {
				DCLaneDirSimple = 1;
			} else {
				DCLaneDirSimple = 2;
			}
			
			// 1. If the UGV is not in a junction, and the DumbCar is mostly located in the other lane (we'll just
			// check which side of the road the centre of each car is on) then we can assume that the collision is
			// the fault of the DumbCar.  Alternatively, as long as the UGV is on the side of the road corresponding to
			// its direction of travel, if it is hit by a vehicle travelling in the other direction, assume the DC is at fault.
			if ((sim.junctionAtArea(new Area(trackedCar.getShape()), sim.junctions) == 0) && 
				((sim.getLaneDirAtPoint(trackedCar.getLocation(), sim.roads) != sim.getLaneDirAtPoint(car2.getLocation(), sim.roads)) ||
				 ((UGVLaneDirSimple != DCLaneDirSimple) && UGVLaneDirSimple == sim.getLaneDirAtPoint(trackedCar.getLocation(), sim.roads))))
			{
				retVal = false;			
			}
			
			// 2. If the DumbCar is travelling faster than the UGV, on approximately the same bearing (+/- 45 deg) then
			// we can assume that the collision is the fault of the DumbCar - hopefully this will eliminate the failure
			// of a DumbCar driving over the top of a UGV that is slowing down to a stop, but has not quite stopped yet.
			double angleDiff = Math.abs(trackedCar.getDirection() - car2.getDirection());
			if ((trackedCar.getSpeed() < car2.getSpeed()) && ((angleDiff < 45) || (angleDiff > 315)))
			{
				retVal = false;			
			}
			
			// 3. If the DumbCar is spanning two lanes i.e. intersecting with the centreline, then
			// we can assume it is at fault in the crash
			if (sim.roadMarkingAtShape(car2.getShape(), sim.roads, LineType.CENTRE)){
				retVal = false;
			}
						
			if (retVal == true) {
				// Output the info about the crash
				addLog(AccidentType.CLASHWITHOTHERCAR, trackedCar.getID(), sim.schedule.getSteps(), trackedCar.getLocation(), 
					" Shape1: " + areaToString(shape1) + ", Shape2: " + areaToString(shape2) + ", Intersection: " +
					areaToString(intersection) + ".");
			
				return true; // Tests above suggest collision could have been the fault of the UGV
			} else {
				return false; // Tests above suggest collision was the fault of the DumbCar not the UGV
			}
		} else {
			return false; // they did, but one of the cars is already dead, or the cars are both the same
		}
	}
	
	/** 
	 * Format the supplied inArea as a String suitable for the log file (to enable debugging).
	 * The string is enclosed in square brackets, and contains all the coordinates that make 
	 * up the Area, each enclosed in curly brackets, and annotated with the type of each point.  
	 * @param inArea (Area - Area to be formatted as a String)
	 * @return String (inArea formatted as a String)
	 **/
	private String areaToString(Area inArea)
	{
		String outString = "[";
		double type;
		double[] coords = new double[6];
		
		for (PathIterator theOutline = inArea.getPathIterator(null); !theOutline.isDone(); theOutline.next())
		{
			type = theOutline.currentSegment(coords);
			
			// Build the string
			outString += "(" + type + ", " + coords[0] + ", " + coords[1] + ") ";
		}
		
		outString.trim(); // Remove the last space added
		outString += "]"; // Close the brackets
				
		return outString;
	}
	
	/** 
	 * Detect whether the centre point of the car (car.getLocation) has left the road
	 * by checking for intersection of this location with all of the road Shapes on the
	 * map.  If the location does not intersect with any of the roads, false is returned.
	 * @param car (Car - being checked for 'LEAVEROAD')
	 * @param roads (Bag - all roads on the map)
	 * @return boolean (true if the centre of the car has left the road)
	 **/
	private boolean detectCollisionWithPavement(Car car, Bag roads)
	{
		// TODO - This method only checks to see whether the centre point of the UGV (getLocation())
		// has left the road, which means that almost half of the vehicle could be overlapping the pavement
		// without this failure being detected.  As we are in an exercise to reduce the number of failures being
		// logged, we will accept this weaker form of collision detection (which will detect serious pavement
		// invasions), but perhaps in the future this should be updated so that any part of the UGV leaving the
		// road would be sufficient to trigger a failure.
		
		boolean onRoad = false;
		
		for (int r=0; r < roads.size(); r++)
		{
			if (((Road) roads.get(r)).inShape(car.getLocation()) == true)
			{
				onRoad = true;
			}
		}
		
		return !onRoad;
	}
	
	/** 
	 * Detect whether the centre point of the car (car.getLocation) has left the road
	 * by checking for intersection of this location with all of the road Shapes on the
	 * map.  If the location does not intersect with any of the roads, false is returned.
	 * NOTE - Where the car has intersected with part of the centre line, but not actually crossed the midpoint of the
	 * line, this method will assume that the vehicle maintains the same trajectory and projects this onto the midpoint
	 * of the line, to return a *likely* future intersection point.  This method *may* return two detections for a single
	 * crossing if the UGV ends the step 'on' the line i.e. within the rectangle which represents the painted line of 10cm 
	 * width.
	 * @param car (Car - being checked for 'LINE CROSSING')
	 * @param roads (Bag - all roads on the map)
	 * @param inLineType (LineType - which line we are checking against: NEARSIDE, CENTRE, OFFSIDE)
	 * @param junctions (Bag - all junctions on the map)
	 * @return Double2D (location at which centre of vehicle crosses centre of line, or -1,0 to indicate no line crossing failure)
	 **/
	private Double2D detectLineCrossing(Car car, Bag roads, LineType inLineType, Bag junctions)
	{
		Double2D retVal = new Double2D(-1,0);
		
		// Because at the beginning, this does not have a value
		if (car.getPrevLoc() == null) {
			return retVal; // return value of x = -1 indicates no failure
		}
		
		// If the car is a UGV, check to see whether it is in overtaking mode, as it is okay to cross
		// the centre line in this case
		if (car.getType() == TUGV) {
			if (((UGV) car).isOvertaking() == true) {
				return retVal; // return value of x = -1 indicates no failure
			}
			
			// If the car is a UGV it is also okay to cross the centre line when performing a UTurn manouevre
			if (((UGV) car).isUTurning() == true) {
				return retVal; // return value of x = -1 indicates no failure
			}
		}
		
		Line2D.Double trajectory = new Line2D.Double(car.getPrevLoc().x, car.getPrevLoc().y, car.getLocation().x, car.getLocation().y);
		
		// For each road, access the centre line and check for intersection with the
		// line that is made by connecting the previous and current locations of the
		// car
		for (int r=0; r < roads.size(); r++)
		{
			if (((Road) roads.get(r)).getLine(inLineType).intersectsLine(trajectory) == true)
			{
				// Check whether the road is N/S or E/W as this will impact upon the calculations below.
				if (((Road) roads.get(r)).x1 == ((Road) roads.get(r)).x2) {
					// This road runs N/S
					
					// See below for comment on simplicity of this calculation.
					double angle = Math.atan((trajectory.y1 - trajectory.y2)/(trajectory.x1 - trajectory.x2));
					double adjacent = trajectory.x2 - ((Road) roads.get(r)).getLine(inLineType).getCenterX();
					double opposite = adjacent * Math.tan(angle);
					
					// Need to make sure that all calculated values are valid numbers
					if (!Double.isNaN(angle) && !Double.isNaN(adjacent) && !Double.isNaN(opposite)) {
						retVal = new Double2D(((Road) roads.get(r)).getLine(inLineType).getCenterX(), trajectory.y2 - opposite);
					}
										
				} else {
					// The road must run E/W
					
					// Note: although these calculations seems simple for taking into account the different
					// possibilities for intersection i.e. slope angle or direction, and whether the centre of the 
					// centre line has already been crossed, pen & paper calcs suggest that the geometry falls out
					// correctly due to signs cancelling etc.
					double angle = Math.atan((trajectory.x1 - trajectory.x2)/(trajectory.y1 - trajectory.y2));
					double adjacent = trajectory.y2 - ((Road) roads.get(r)).getLine(inLineType).getCenterY();
					double opposite = adjacent * Math.tan(angle);

					// Need to make sure that all calculated values are valid numbers
					if (!Double.isNaN(angle) && !Double.isNaN(adjacent) && !Double.isNaN(opposite)) {
						retVal = new Double2D(trajectory.x2 - opposite, ((Road) roads.get(r)).getLine(inLineType).getCenterY());
					}
				}
			}
		}
				
		if (sim.junctionAtPoint(retVal, junctions) != 0 || retVal.x == -1) {
			return new Double2D(-1,0); // return value of x = -1 indicates no failure
		} else {
			return retVal;
		}
	}	
}
