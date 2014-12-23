package modeling;
import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.Area;
import java.awt.geom.Line2D;
import java.awt.geom.PathIterator;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.util.Calendar;

import modeling.COModel.targetInfo;
import sim.engine.SimState;
import sim.engine.Steppable;
import sim.util.Bag;
import sim.util.Double2D;


/**
 * 
 */

/**
 * @author xueyi
 *
 */
public class AccidentDetector implements Constants,Steppable {

	/**
	 * 
	 */
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
	
	public int getNoAccidents() {
		return noAccidents;
	}

	public void setNoAccidents(int noAccidents) {
		this.noAccidents = noAccidents;
	}

	public Bag getTrackedCars() {
		return this.trackedCars;
	}

	public void setTrackedCars(Bag trackedCars) {
		this.trackedCars = trackedCars;
	}

	public AccidentDetector(double percentageFaults, int mapNo){ 
		
		// HH 28.8.14 : NOTE - differences in percentages of faults must be > 1% or files will be overwritten
		accidentLog = new File("AccidentLog" + Math.round(percentageFaults * 100) + "_" + mapNo + ".txt");
		
		try{
			ps= new PrintStream(new FileOutputStream(accidentLog));
		}
		catch(FileNotFoundException e)
		{
			System.out.print("Accident log file not found!");
			return;
		}
		
		// HH 28.8.14 - Create new summary log file for logging pertinent data about the run as a whole for easy analysis
		accidentSummary = new File("AccidentSummary" + Math.round(percentageFaults * 100) + "_" + mapNo + ".txt");
		
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
	 */
	@Override
	public void step(SimState state) {
		// TODO Auto-generated method stub
		sim = (COModel)state;
		ParkedCar obstacle;
		//Bag cars = COModel.cars;
		Car car1;
		Car car2;

		for (int i=0; i<trackedCars.size(); i++)
		{
			car1= (Car)trackedCars.get(i);
			
			for(int j=0; j<sim.obstacles.size(); j++)
			{
				obstacle=(ParkedCar)sim.obstacles.get(j);
				//System.out.println("Obstacle"+obstacle.getID());
				if(detectCollisionWithObstacle(car1, obstacle))
				{
					addLog(AccidentType.CLASHWITHOBSTACLE, car1.getID(), sim.schedule.getSteps(), car1.getLocation(), "with obstacle id = "+ obstacle.getID() ); // HH 30/4/14 - Corrected typo
					noAccidents++;
					AccCrashObs++;
					
					// HH 13.11.14 - Updated so have visualisation of crashes with obstacles
					sim.recordCrash(car1.getLocation()); // TO DO : Update to actual collision point between vehicles
					// HH 7.8.14 Reinstate this so detect collisions with obstacles
					//car1.isActive=false;
					//trackedCars.remove(car1);
				}
			}
			
// HH 15.7.14 Had to remove this as was terminating execution on wall collision
//			if(detectCollisionWithWall(car1))
//			{
//				addLog(AccidentType.CLASHWITHWALL, car1.getID(), sim.schedule.getSteps(), car1.getLocation(), null);
//				noAccidents++;
//				car1.isActive=false;
//				trackedCars.remove(car1);
//			}
			
			// HH 15.7.14 Added some road departure logging
			if(detectCollisionWithPavement(car1, sim.roads) == true)
			{
				addLog(AccidentType.LEAVEROAD, car1.getID(), sim.schedule.getSteps(), car1.getLocation(), null);
				noAccidents++;
				AccLeaveRoad++;
				//car1.isActive=false;
				//trackedCars.remove(car1);			
				sim.recordOffRoad(car1.location);
			}
			
			// HH 16.7.14 Added some line crossing logging - Centre Line
			Double2D intersect = detectLineCrossing(car1, sim.roads, LineType.CENTRE, sim.junctions);
			if(intersect.x > -1)
			{
				addLog(AccidentType.CROSSCENTRELINE, car1.getID(), sim.schedule.getSteps(), intersect, null);
				noAccidents++;
				AccCrossCentre++;
				//car1.isActive=false;
				//trackedCars.remove(car1);			
				sim.recordCrossLine(intersect);
			}
			
			// HH 17.7.14 Added some line crossing logging - Centre Line
			Double2D intersectNW = detectLineCrossing(car1, sim.roads, LineType.NWSIDE, sim.junctions);
			if(intersectNW.x > -1)
			{
				addLog(AccidentType.CROSS_NW_LINE, car1.getID(), sim.schedule.getSteps(), intersectNW, null);
				noAccidents++;
				AccCrossNW++;
				//car1.isActive=false;
				//trackedCars.remove(car1);			
				sim.recordCrossLine(intersectNW);
			}
			
			// HH 17.7.14 Added some line crossing logging - Centre Line
			Double2D intersectSE = detectLineCrossing(car1, sim.roads, LineType.SESIDE, sim.junctions);
			if(intersectSE.x > -1)
			{
				addLog(AccidentType.CROSS_SE_LINE, car1.getID(), sim.schedule.getSteps(), intersectSE, null);
				noAccidents++;
				AccCrossSE++;
				//car1.isActive=false;
				//trackedCars.remove(car1);			
				sim.recordCrossLine(intersectSE);
			}
			
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
					sim.recordCrash(car1.getLocation()); // TO DO : Update to actual collision point between vehicles
					//car1.isActive=false;
					//car2.isActive=false;
					//trackedCars.remove(car1);
					//trackedCars.remove(car2);
				}
			}
			
		}
		
		// HH - 21/07/14 Added a limiter on the number of steps that have been executed (this is also an failure condition)
		if (sim.schedule.getSteps() > 5000)
		{
			Double2D targetLoc = sim.getTargetLoc();
			String locString = "(" + targetLoc.x + "," + targetLoc.y + ")";
			
			ps.println(AccidentType.TIMEOUT.toString() +"; time: "+ sim.schedule.getSteps() + " steps.  Run Terminated without reaching Target location: " + locString + ".");
			noAccidents++;
			AccTimeout++;
			
			// Now do something to stop the run
			sim.kill(); // NOTE: it should be okay to call this directly as we are 'in a Steppable', so according to the comments by the SimState methods, this is ok.
			
		}
		
	}
	
	public void addLog(AccidentType t, int carID, long step, Double2D coor, String str)
	{
		ps.println(t.toString() +":- car: "+ carID + "; time: "+ step + " steps; location: (" + coor.x + ", " + coor.y + "); " + str); // HH 30/4/14 Tidied up the formatting here
	}
	
	/** HH 30/4/14 - Adds header information to file to enable accident results
	 *  to be reproduced.  An entry in the log can also be made for runs
	 *  which do not produce any accidents.
	 **/
	public void addHeader(COModel state)
	{
		sim = (COModel)state;
	
		// HH 26.11.14 Added some more logging
		targetInfo myTargetInfo = sim.HgetTargetSeparations(sim.getTargetLoc());
		
		ps.println("*** New Run, Seed = "+ sim.seed() + "*** External Seed = " + sim.getExternalSeed() + 
			"; Start time = " + timeToString() + ".");
		
		ps.println("Map/Network Complexity Measures: MinJunctSep = " + sim.HgetMinJctSeparation() +
				   "; UGVTargetSep = " + sim.HgetUGVTargetSeparation() + "; TargetObstacleSep = " +
				   sim.HgetMinTargetObsSeparation() + "; UGVTargetRoadSep = " + sim.HgetUGVTargetRoadSeparation() +
				   "; CriticalObsSep = " + sim.HgetCriticalObsSeparation() + "; MinTargetCentreSep = " +
					sim.HgetMinTargetCentreSeparation() + "; MinTargetKerbSep = " + sim.HgetMinTargetKerbSeparation() + 
					"; DistPrevJctToTarget = " + myTargetInfo.fromPrevJct + "; DistTargetToNextJct = " + myTargetInfo.toNextJct + ".");
		
		// HH 28.8.14 - reset the summary string, and start to add information to it.  We'll 
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
		
		// HH 26.11.14 Added some more logging
		summaryString += myTargetInfo.fromPrevJct + ", ";
		summaryString += myTargetInfo.toNextJct + ", ";
	}
	
	/** HH 17/7/14 - Adds footer information to file to report the number of accidents
	 **/
	public void addFooter(COModel state)
	{
		sim = (COModel)state;
	
		// HH 30.7.14 - Work out the % faults that are active
		int noFaults = 0;
		for (int i = 0; i < Constants.MAX_FAULTS; i++)
		{
			if (sim.getFault(i) == true) {
				noFaults++;
			}
		}
		
		ps.println(state.HgetUGVTargetSuccess()); // HH 22.7.14 Add extra footer re: targets found
		
		ps.println("*** End of run, Seed = "+ sim.seed() + ", External Seed = " + sim.getExternalSeed()  + 
				"*** NoCars = " + sim.noCars + "; CarMaxDeceleration = " + sim.getCarMaxDecceleration() + 
				"; CarMaxSpeed = " + sim.getCarMaxSpeed() + "; CarMaxAcceleration = " + sim.getCarMaxAcceleration()
				+ "; CarMaxTurning = " + sim.getCarMaxTurning() + "; NoObstacles = "
				+ sim.getNoObstacles() + "; NoJunctions = " + sim.getNoJunctions()
				+ "; NoAccidents = " + sim.aDetector.getNoAccidents() + 
				"; NoSteps = " + sim.schedule.getSteps() + "; % Faults = " + noFaults + "/" + 
				Constants.MAX_FAULTS + "=" + (noFaults/Constants.MAX_FAULTS) + "; End time = " + timeToString() + "."); // HH 30.7.14 Updated to include % faults
		
		// HH 28.8.14 - Add the remaining summary log information and write to the file
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
		
		// Reset noAccidents // HH 28.8.14 and associated measures
		this.setNoAccidents(0);
		AccLeaveRoad = 0;
		AccCrossCentre = 0;
		AccCrossSE = 0;
		AccCrossNW = 0;
		AccCrashObs = 0;
		AccCrashCar = 0;
		AccTimeout = 0;
	}
	
	/** HH 22/7/14 - Adds a passed string to file, alongside the current #steps
	 **/
	public void addString(COModel state, String inString)
	{
		sim = (COModel)state;
	
		ps.println("STEP #" + sim.schedule.getSteps() + " - " + inString + ".");
	}
	
	private boolean detectCollisionWithObstacle(Car car, ParkedCar obstacle)
	{
		//return obstacle.inShape(car.getLocation()); // HH 2.10.14 - Crude, only checked centre of Car
		return obstacle.inShape(car.getShape());
	}
	
	private boolean detectCollisionWithWall(Car car)
	{
        Double2D me = car.getLocation();
		
		if(me.x <= 1 )
		{
			//System.out.println("clash with the left wall!");
			return true;
		}
		else if(Constants.WorldXVal - me.x <= 1)
		{
			//System.out.println("clash with the right wall!");
			return true;
		}
		
		if (me.y <= 1)
		{
			//System.out.println("clash with the upper wall!");
			return true;
		}
		else if(Constants.WorldYVal- me.y <= 1)
		{
			//System.out.println("clash with the lower wall!");
			return true;
		}
		
		return false;
	}
	
	// HH 24.9.14 - Improved this method so that it uses the actual vehicle outlines
	// rather than some approximation based on whether the centres of each object are 
	// less than 2m apart
	// HH 3.12.14 - Force the first parameter to be a UGV so that we can ignore any
	// collisions where the UGV is stationary (assume these to be the fault of one
	// of the other DumbCars)
	private boolean detectCollisionWithOtherCar(Car trackedCar, Car car2)
	{
//		Double2D location1=car1.getLocation();
//		Double2D location2=car2.getLocation();
//		
//		// HH 26.8.14 Changed from 1 to 2 as assume diameter of vehicle is 2m so distance between centres is 2m
//		// HH 27.8.14 Added check to make sure both cars are active
//		if(location1.distance(location2) < 2 && car1.isActive && car2.isActive) {
//			return true;
//		}
//		else
//		{
//			return false;
//		}
		
		// HH 3.12.14 Firstly let's see if the UGV is stationary, because if so, we are going to attribute
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
			
			// HH 17.12.14 - We want to make sure this is a crash we should be reporting as it is not 
			// fair to blame the UGV when the Dumb Car is entirely at fault i.e. back end swings out and 
			// wipes out UGV during a turn.  
			//
			// It can be quite difficult to find general rules for determining fault, so we will just use
			// simple one(s) and hope that this will eliminate a few unnecessary failures, whilst retaining those
			// in which the UGV is at fault.
			
			Boolean retVal = true;
			
			// If the UGV is not in a junction, and the DumbCar is mostly located in the other lane (we'll just
			// check which side of the road the centre of the DumbCar is on) then we can assume that the collision is
			// the fault of the DumbCar.  Alternatively, as long as the UGV is on the side of the road corresponding to
			// its direction of travel, if it is hit by a vehcile travelling in the other direction, assume the DC is at fault
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
			
			if ((sim.junctionAtArea(new Area(trackedCar.getShape()), sim.junctions) == 0) && // HH 17.12.14 Changed retVal Check from false to 0
				((sim.getLaneDirAtPoint(trackedCar.getLocation(), sim.roads) != sim.getLaneDirAtPoint(car2.getLocation(), sim.roads)) ||
				 ((UGVLaneDirSimple != DCLaneDirSimple) && UGVLaneDirSimple == sim.getLaneDirAtPoint(trackedCar.getLocation(), sim.roads))))
			{
				retVal = false;			
			}
			
			// If the DumbCar is travelling faster than the UGV, on approximately the same bearing (+/- 15 deg) then
			// we can assume that the collision is the fault of the DumbCar - hopefully this will eliminate the failure
			// of a DumbCar driving over the top of a UGV that is slowing down to a stop, but has not quite stopped yet.
			double angleDiff = Math.abs(trackedCar.getDirection() - car2.getDirection());
			if ((trackedCar.getSpeed() < car2.getSpeed()) && ((angleDiff < 45) ||(angleDiff > 315))) // HH 17.12.14 Changed from 15 to 45
			{
				retVal = false;			
			}
			
			// HH 22.12.14 If the DumbCar is spanning two lanes i.e. intersecting with the centreline, then
			// we can assume it is at fault in the crash
			if (sim.roadMarkingAtShape(car2.getShape(), sim.roads, LineType.CENTRE)){
				retVal = false;
			}
						
			if (retVal == true) {
				// HH 25.9.14 Let's output the info to make sure there was really a crash
				addLog(AccidentType.CLASHWITHOTHERCAR, trackedCar.getID(), sim.schedule.getSteps(), trackedCar.getLocation(), 
					" Shape1: " + areaToString(shape1) + ", Shape2: " + areaToString(shape2) + ", Intersection: " +
					areaToString(intersection) + ".");
			
				return true; // they did
			} else {
				return false; // HH 17.12.14 - tests above suggest collision was the fault of the DumbCar not the UGV
			}
		} else {
			return false; // they did, but one of the cars is already dead
		}
	}
	
	/*
	 * HH 25.9.14 - Describe the area as a String to enable debugging
	 */
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
	
	/** HH 15/7/14 - Detect whether the 'car' has left the road
	 **/
	private boolean detectCollisionWithPavement(Car car, Bag roads)
	{
		// HH 3.12.14 TODO - This method only checks to see whether the centre point of the UGV (getLocation())
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
	
	/** HH 16/7/14 - Detect whether the 'car' has crossed the centre line
	 * 
	 * NOTE - Where the car has intersected with part of the centre line, but not actually crossed the midpoint of the
	 * line, this method will assume that the vehicle maintains the same trajectory and projects this onto the midpoint
	 * of the line, to return a *likely* future intersection point.  This method *may* return two detections for a single
	 * crossing if the UGV ends the step 'on' the line i.e. within the rectangle which represents the painted line of 10cm 
	 * width.
	 **/
	private Double2D detectLineCrossing(Car car, Bag roads, LineType inLineType, Bag junctions)
	{
		Double2D retVal = new Double2D(-1,0);
		
		// Because at the beginning, this does not have a value
		if (car.getPrevLoc() == null) {
			return retVal; // return value of x=-1 indicates failure
		}
		
		// HH 25.8.14 - if the car is a UGV, check to see whether it is in overtaking mode, as it is okay to cross
		// the centre line in this case
		if (car.getType() == TUGV) {
			if (((UGV) car).isOvertaking() == true) {
				return retVal; // return value of x=-1 indicates failure
			}
			
			// HH 16.12.14 - if the car is a UGV it is also okay to cross the centre line when performing a UTurn manouevre
			if (((UGV) car).isUTurning() == true) {
				return retVal; // return value of x=-1 indicates failure
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
					
					// HH 23/7/14 See below for comment on simplicity of this calculation.
					double angle = Math.atan((trajectory.y1 - trajectory.y2)/(trajectory.x1 - trajectory.x2));
					double adjacent = trajectory.x2 - ((Road) roads.get(r)).getLine(inLineType).getCenterX();
					double opposite = adjacent * Math.tan(angle);
					
					// HH 22.12.14 Need to make sure that all calculated values are valid numbers
					if (!Double.isNaN(angle) && !Double.isNaN(adjacent) && !Double.isNaN(opposite)) {
						retVal = new Double2D(((Road) roads.get(r)).getLine(inLineType).getCenterX(), trajectory.y2 - opposite);
					}
										
				} else {
					// The road must run E/W
					
					// HH 23/7/14 Note: although these calculations seems simple for taking into account the different
					// possibilities for intersection i.e. slope angle or direction, and whether the centre of the 
					// centre line has already been crossed, pen & paper calcs suggest that the geometry falls out
					// correctly due to signs cancelling etc.
					double angle = Math.atan((trajectory.x1 - trajectory.x2)/(trajectory.y1 - trajectory.y2));
					double adjacent = trajectory.y2 - ((Road) roads.get(r)).getLine(inLineType).getCenterY();
					double opposite = adjacent * Math.tan(angle);

					// HH 22.12.14 Need to make sure that all calculated values are valid numbers
					if (!Double.isNaN(angle) && !Double.isNaN(adjacent) && !Double.isNaN(opposite)) {
						retVal = new Double2D(trajectory.x2 - opposite, ((Road) roads.get(r)).getLine(inLineType).getCenterY());
					}
				}
			}
		}
		
		
		if (sim.junctionAtPoint(retVal, junctions) != 0 || retVal.x == -1) {
			return new Double2D(-1,0); // return value of x=-1 indicates failure
		} else {
			return retVal;
		}

	}	
	
	private String timeToString() {
		
		Calendar timeNow = Calendar.getInstance();
		String timeString = "";
		
		if (timeNow.get(Calendar.HOUR) == 0) {
			if (timeNow.get(Calendar.AM_PM) == Calendar.AM) {
				timeString += "00:";
			} else {
				timeString += "12:";
			}
		} else if (timeNow.get(Calendar.HOUR) < 10) {
			timeString += "0";
			timeString += timeNow.get(Calendar.HOUR);
			timeString += ":";
		} else {
			timeString += timeNow.get(Calendar.HOUR);
			timeString += ":";
		}
		
		if (timeNow.get(Calendar.MINUTE) < 10) {
			timeString += "0";
			timeString += timeNow.get(Calendar.MINUTE);
			timeString += ":";
		} else {
			timeString += timeNow.get(Calendar.MINUTE);
			timeString += ":";
		}		

		if (timeNow.get(Calendar.SECOND) < 10) {
			timeString += "0";
			timeString += timeNow.get(Calendar.SECOND);
			timeString += ":";
		} else {
			timeString += timeNow.get(Calendar.SECOND);
			timeString += ":";
		}
		
		timeString += timeNow.get(Calendar.MILLISECOND);
		
		return timeString;
	}
	
	

}
