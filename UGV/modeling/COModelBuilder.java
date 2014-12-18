package modeling;
import java.awt.geom.Area;
import java.awt.geom.Rectangle2D;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.io.FileOutputStream;
import java.security.SecureRandom;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;

import ec.util.MersenneTwisterFast;
import modeling.COModel.initialInfo;
import sim.util.*;
/**
 *
 * @author Robert Lee
 * This class is used to build/initiate the simulation.
 * There is a "main" method for running the simulation without GUI
 * Called for by simulationWithUI.class
 */
public class COModelBuilder
{
	private static String simLength = "1000";
	
	public  COModel sim;
	
	// HH 30.7.14 Made these Global Constants
	//private static double worldXVal = 200; // HH 30.7.14 Changed from 100 to quadruple world size
	//private static double worldYVal = 200; // HH 30.7.14 Changed from 100 to quadruple world size
	//private int noCars=3;
	
		
//	public COModelBuilder()
//	{
//		//System.out.println("SimBuilder1 is being called!!!!!!!!!!");
//		setUpSim((Calendar.SECOND * 1000)+ Calendar.MILLISECOND);
//		System.out.println("In COModelBuilder " + Runtime.getRuntime().freeMemory());
//	}
	
	
	public COModelBuilder(COModel s)
	{
		//System.out.println("SimBuilder2 is being called!!!!!!!!!! ");
		sim = s;
	}
	
	
//	private void setUpSim(long initSeed)
//	{
//		System.out.println("COModelBuilder.setUpSim is called!!!!!!!! ");
//		sim = new COModel(initSeed, Constants.WorldXVal, Constants.WorldYVal, false);
//		
//	}
	
	// HH 31/7/14 - Method to update the seed used by the underlying COModel so that can 
	// do a batch run using different seeds if required
	public void updateSeed(long newSeed)
	{
		sim.setSeed(newSeed);
	}

	
//	public static void testSim()
//	{
//		int tID = sim.getNewID();
//		Target t = new Target(tID);
//		entAdder(t, 0, 0, false);
//		
//		CircleObstacle ob = new CircleObstacle(sim.getNewID(), 10);
//		entAdder(ob, 50, 50, false);		
//		
//		Car AV = new Car(sim.getNewID(), tID);
//		entAdder(AV, 99, 99, true);
//	}
//	
//	
//	public static void testSim2()
//	{
//		int tID = sim.getNewID();
//		double xVal = sim.random.nextDouble() * 100;
//		double yVal = sim.random.nextDouble() * 100;
//		
//		Target t = new Target(tID);
//		entAdder(t, xVal, yVal, false);
//		
//		CircleObstacle ob = new CircleObstacle(sim.getNewID(), 10);
//		entAdder(ob, 50, 50, false);		
//		
//		if (xVal < 50)
//		{
//			xVal += 50;
//		} else if (xVal > 50) {
//			xVal -= 50;
//		}
//		
//		if (yVal < 50)
//		{
//			yVal += 50;
//		} else if (xVal > 50) {
//			yVal -= 50;
//		}
//		
//		Car AV = new Car(sim.getNewID(), tID);
//		entAdder(AV, xVal, yVal, true);		
//	}
	
	// HH 30.7.14 Removed noJcts input parameter (determined at random below)
	public void generateSimulation()//int noObstacles, int noCars) // HH 30/4/14 - added new road related parameters
	{		
		// HH 1.9.14 - This method cannot use sim.random if we want to provide the ability to run multiple
		// fault patterns on the same base map (network).
		if (sim.getExternalSeed() == 0) {
			// The seed hasn't been set in the UI, or when the COModel was created, so set one now
			// based on the system time (use 
			sim.setExternalSeed(new SecureRandom().nextInt()); // HH 3.9.14 Updated to use SecureRandom rather than nanoTime
		}
			
		// Now create the new random generator, it can be local as we only use it here to generate the map
		MersenneTwisterFast mapGenRandom = new MersenneTwisterFast(sim.getExternalSeed());
		
		// HH - end
		
		sim.infoLog.addHeader(sim); // HH 6.5.14 - Add header to info log file

		// HH 30/7/14 Decide how many junctions we are going to try to add.  In the future this might need to be done
		// using a different RN generator so that it is not linked to the random seed - these should maybe be independent 
		// for helping score situations.
		int noJcts = (int) (mapGenRandom.nextDouble() * (Constants.MAX_JUNCTIONS - Constants.MIN_JUNCTIONS)) + Constants.MIN_JUNCTIONS;
		
// HH 7.5.14 - Removed obstacles from the environment as currently prototyping driving on road behaviour - static obstacles on 
// road may not be appropriate, until we want to add on-road furniture.  Parked vehicles should probably be some kind of vehicle
// obstacle object with a speed of 0. Also, replaced target code as want to add it on the road network, rather than to avoid the
// obstacles. NOTE : New code is supplied below, after the road network has been constructed.
		
//		
//		
//		int tID = sim.getNewID();
//		Target t = new Target(tID);
//		do
//		{
//			x = sim.random.nextDouble() * worldXVal;
//			y = sim.random.nextDouble() * worldYVal;
//		}  while (sim.obstacleAtPoint(new Double2D(x,y), obstacles) || );
//		//entAdder(t, x, y, false);
//		t.setLocation(new Double2D(x,y));
//		t.isSchedulable = false;
//		sim.allEntities.add(t);
//		//System.out.println("target is at (" + Double.toString(x) + ", " + Double.toString(y) + ") ID is"+ t.ID);
				
// HH 7.5.14 - Replaced existing code to add cars to the environment as only want to add one vehicle and want to add it to the 
// road network, rather than anywhere in the environment.  This will be the 'start' location, and the vehicle is intended to 
// travel from the start to the target.  NOTE : New code is supplied below, after the road network has been constructed.
		
//		//Bag cars=new Bag();
//		for(int i=1; i<=noCars; i++)
//		{
//			do
//			{
//				x = sim.random.nextDouble() * worldXVal;
//				y = sim.random.nextDouble() * worldYVal;
//			}  while (sim.obstacleAtPoint(new Double2D(x,y), obstacles));
//			Car car = new Car(sim.getNewID(), tID, sim.carStats);	
//			sim.cars.add(car);
//			//entAdder(car, x, y, true);	
//			car.setLocation(new Double2D(x,y));
//			car.isSchedulable = true;
//			sim.allEntities.add(car);
//			sim.toSchedule.add(car);
//			//System.out.println("car is at (" + Double.toString(x) + ", " + Double.toString(y) + ")ID is"+ car.ID);
//			//System.out.println("COModelBuilder.genereteSimulation is called, car's max speed is: " + sim.carStats.getMaxSpeed());
//		}
		

		
		// HH 30/4/14 Support for adding Road features to the model
		double x1, y1, length, jct;
		Road road;
		int r;
		
		do // Choose a random location for start of road (must not intersect with an existing road - although there shouldn't be any other roads)
		{
			// HH 16.7.14 Ensure that the road cannot overlap (in parallel) with the edge of the map by constraining the
			// coordinates to be offset from the edge of the map by half the roadWidth
			x1 = (mapGenRandom.nextDouble() * (Constants.WorldXVal - Road.roadWidth)) + Road.roadWidth/2;
			y1 = (mapGenRandom.nextDouble() * (Constants.WorldYVal - Road.roadWidth)) + Road.roadWidth/2;
		}  while (sim.roadAtPoint(new Double2D(x1,y1), sim.roads));
		
		length = 1; // reset length before we use it
		
		// decide whether the road should be N/S or E/W
		if (mapGenRandom.nextBoolean()) {
			// E/W
			// HH 16.7.14 - Allow lengths that will take us outside the map, but truncate at the boundary
			//while ((Math.abs(length) < Junction.jctApproachLen) || ((x1 + length) < 0) || ((x1 + length) > 100)) {
			while (Math.abs(length) < Junction.jctApproachLen) {
				// decide whether this should be applied in negative or positive (we will use these as multipliers for the results below)
				if (mapGenRandom.nextBoolean()) {
					length = -1;
				} else {
					length = 1;
				}
				
				length = length * mapGenRandom.nextDouble() * Constants.WorldXVal; // select a random length
				
				// HH 16.7.14 - Constrain the road to end at the edge of the map.  Note: roads can go right to the edge of the map
				// when they are perpendicular to the map edge, it is only parallel offsets that must be roadWidth/2 away.
				if ((x1 + length) < 0) {
					length = 0 - x1;
				} else if ((x1 + length) > Constants.WorldXVal) {
					length = Constants.WorldXVal - x1;
				}
			}
			road = new Road(sim.getNewID(), Constants.SINGLETWOWAY, new Double2D(x1,y1), new Double2D(x1+length,y1));
		} else {
			// N/S
			// HH 16.7.14 - Allow lengths that will take us outside the map, but truncate at the boundary
			//while ((Math.abs(length) < Junction.jctApproachLen) || ((y1 + length) < 0) || ((y1 + length) > 100)) {
			while (Math.abs(length) < Junction.jctApproachLen) {
				// decide whether this should be applied in negative or positive (we will use these as multipliers for the results below)
				if (mapGenRandom.nextBoolean()) {
					length = -1;
				} else {
					length = 1;
				}
				
				length = length * mapGenRandom.nextDouble() * Constants.WorldYVal; // select a random length
				
				// HH 16.7.14 - Constrain the road to end at the edge of the map.  Note: roads can go right to the edge of the map
				// when they are perpendicular to the map edge, it is only parallel offsets that must be roadWidth/2 away.
				if ((y1 + length) < 0) {
					length = 0 - y1;
				} else if ((y1 + length) > Constants.WorldYVal) {
					length = Constants.WorldYVal - y1;
				}
			}
			road = new Road(sim.getNewID(), Constants.SINGLETWOWAY, new Double2D(x1,y1), new Double2D(x1,y1+length));
		}
		sim.roads.add(road);
		
// Removed to support grid based roads
//		// For simplicity allow line to be drawn on any angle and any length - TO DO: May want to impose a rectangular grid later
//		do // Choose a random location for end of road (must not intersect with an existing road)
//		{
//			x2 = sim.random.nextDouble() * worldXVal;
//			y2 = sim.random.nextDouble() * worldYVal;
//		}  while (sim.roadAtPoint(new Double2D(x,y), sim.roads));		
//		
//		
//		Road road = new Road(sim.getNewID(), Constants.SINGLETWOWAY, new Double2D(x1,y1), new Double2D(x2,y2));
//		sim.roads.add(road);
		
		double cx = 0;
		double cy = 0;
		boolean jctFound = false;
		int noSmallIterations = 0;
		int noBigIterations = 0;
		Road cRoad, nRoad; 
		Junction nJct;
		//int jctDir = -1;
		int dirLen = 1;
		
		// HH 4.12.14 Added to support junction overlap check below.
		Area jctArea;
		
		// Init cRoad/jct to prevent compilation errors below
		cRoad = (Road) sim.roads.get(0); 
		r = 0;
		jct = 0;
		
		// For each junction, choose a location on the road (random constrained by length of road) TO DO - Implement!
		for(int i=1; i<=noJcts; i++)
		{
			jctFound = false; // init loop exit
			noBigIterations = 0; // reset counter for this junction
			
			while (jctFound == false && noBigIterations < 10) {
				
				// Choose a road at random from the Bag of roads
				r = mapGenRandom.nextInt(sim.roads.size());
				cRoad = (Road) sim.roads.get(r);

				noSmallIterations = 0; // reset counter for this road
				noBigIterations += 1; // increment counter for this junction
					
				// Calculate length of road
				length = cRoad.getLength();

				// Generate random from 0 to length (but not within roadWidth * 1.5 (changed from roadWidth/2 on 25.8.14) 
				// of either end of the road and make sure it is within the model bounds)

				// TODO - HH 2.10.14 - this method will need to be updated when we revert to non-grid based roads
				do {
					jct = ((mapGenRandom.nextDouble(true, true) * (length - (3*Road.roadWidth))) + (Road.roadWidth*1.5));
					// work out the 'start' coordinates for the road (so we add from the right end)
					cx = Math.min(cRoad.x1,  cRoad.x2);
					cy = Math.min(cRoad.y1,  cRoad.y2);
					
					noSmallIterations += 1; // increment counter so we don't get stuck in an infinite loop
					
					// HH 4.12.14 Better check for overlapping junctions:
					Double2D jctLoc;
					if (cRoad.getIsNS()) {
						jctLoc = new Double2D(cx, (cy+jct));
					} else {
						jctLoc = new Double2D((cx+jct), cy);
					}
					Rectangle2D jctRect = new Rectangle2D.Double((jctLoc.x-(Road.roadWidth/2)), (jctLoc.y-(Road.roadWidth/2)), Road.roadWidth, Road.roadWidth);
					jctArea = new Area(jctRect);
					
					
//				} while (((cRoad.getIsNS() && (((cy + jct >= (Constants.WorldYVal-Road.roadWidth))) || (cy + jct < 0) || sim.junctionAppAtPoint(new Double2D(cx, (cy+jct)), sim.junctions))) || 
//						 (!cRoad.getIsNS() && (((cx + jct >= (Constants.WorldXVal-Road.roadWidth))) || (cx + jct < 0) || sim.junctionAppAtPoint(new Double2D((cx+jct), cy), sim.junctions)))) &&
//						 noSmallIterations < 10);
				} while (((cRoad.getIsNS() && (((cy + jct >= (Constants.WorldYVal-Road.roadWidth))) || (cy + jct < 0) || (sim.junctionAtArea(jctArea, sim.junctions) != 0))) || 
						 (!cRoad.getIsNS() && (((cx + jct >= (Constants.WorldXVal-Road.roadWidth))) || (cx + jct < 0) || (sim.junctionAtArea(jctArea, sim.junctions) != 0)))) &&
						 noSmallIterations < 10);
				
				if (noSmallIterations < 10){
					jctFound = true;
				} else {
					// HH - 6.5.14, removed as additional logging surplus to requirements
					// sim.infoLog.addLog("Unable to find a location for junction " + i + " of " + noJcts + ", on road id =" + r + ".");
				}
			}
			
//			// TODO - Remove this, here for debugging purposes
//			Double2D jctLoc;
//			if (cRoad.getIsNS()) {
//				jctLoc = new Double2D(cx, (cy+jct));
//			} else {
//				jctLoc = new Double2D((cx+jct), cy);
//			}
//			Rectangle2D jctRect = new Rectangle2D.Double((jctLoc.x-(Road.roadWidth/2)), (jctLoc.y-(Road.roadWidth/2)), Road.roadWidth, Road.roadWidth);
//			Area jctArea = new Area(jctRect);
//			if (sim.junctionAtArea(jctArea, sim.junctions)) {
//				System.out.println("We've just been allowed to add a new junction overlapping an existing one!");
//			}		

			if (jctFound == true) {

				// We're going to reuse length to work out the length of our new road, and first we'll decide whether it should be applied in negative or positive 
				// i.e. which way will the new road extend from the junction we have just selected (we will use these as multipliers for the results below)
				if (mapGenRandom.nextBoolean()) {
					dirLen = -1;
				} else {
					dirLen = 1;
				}

				noSmallIterations = 0; // HH 16.7.14 reset counter for this road
				
				// Work out coordinate corresponding to displacement of random from start of road and choose a length for the new road
				// and add the new road
				if (cRoad.getIsNS()) {
					
					// HH 16/7/14 - The chosen road cRoad (where we are placing the junction) is N/S, so this new road will be E/W
					x1 = cRoad.x1;
					y1 = Math.min(cRoad.y1, cRoad.y2) + jct;
					do {
						
						// HH 16.7.14 Once we get half-way through the allocated number of tries, try adding roads in the other direction
						// HH 26.11.14 Moved this to the start so that don't inadverently change dirLen in the middle of the loop (when
						// we use it to sort out the direction of the junction at the end)
						if (noSmallIterations == 10) {
							dirLen = -dirLen;
						}
						
						length = dirLen * mapGenRandom.nextDouble() * Constants.WorldXVal;
						
						// HH 16.7.14 - Constrain the road to end at the edge of the map.  Note: roads can go right to the edge of the map
						// when they are perpendicular to the map edge, it is only parallel offsets that must be roadWidth/2 away.
						if ((x1 + length) < 0) {
							length = 0 - x1;
						} else if ((x1 + length) > Constants.WorldXVal) {
							length = Constants.WorldXVal - x1;
						}						
						
						noSmallIterations += 1; // HH 16.7.14 Increment counter so we don't get stuck in an infinite loop
						
						// HH 16.7.14 - Updated to check enclosing rectangle for overlaps during the loop
					} while ((Math.abs(length) < Junction.jctApproachLen || sim.roadAtPoint(new Rectangle2D.Double((length > 0 ? x1-(Road.roadWidth/2) : x1+length), y1-(Road.roadWidth/2), Math.abs(length), Road.roadWidth), sim.roads, r)) && noSmallIterations < 20);
					
//					// Check to see if the enclosing rectangle (i.e. including road width) of this road that we want to create will
//					// intersect with roads other than the one it is supposed to join TO DO - Could enforce that a perpendicular
//					// crossroads intersection is okay (as long as we then implement another method for detecting new junctions)
//					// HH 16.7.14 - Also try flipping the road direction if we have exceeded 10 iterations
//					if (sim.roadAtPoint(new Rectangle2D.Double((length > 0 ? x1-(Road.roadWidth/2) : x1+length), y1-(Road.roadWidth/2), Math.abs(length), Road.roadWidth), sim.roads, r) || noSmallIterations >= 10) {
//						// Intersection with existing road has been detected, so try flipping the direction of the road
//						// Test for intersection will occur again before the road is added to the roads array.
//						length = -length; 
//						dirLen = -dirLen;
//					} 

					//jctDir = (dirLen > 0) ? Constants.T_WEST : Constants.T_EAST;
					//nJct = new Junction(sim.getNewID(), x1, y1, jctDir);	
					nJct = new Junction(sim.getNewID(), x1, y1, jct, (dirLen < 0 ? 0 : Math.abs(length)), (cRoad.getLength()-jct), (dirLen < 0 ? Math.abs(length) : 0));
					nRoad = new Road(sim.getNewID(), Constants.SINGLETWOWAY, new Double2D(x1,y1), new Double2D(x1+ length,y1));
				} else {
					
					noSmallIterations = 0; // HH 16.7.14 reset counter for this road
					
					// HH 16/7/14 - The chosen road cRoad (where we are placing the junction) is E/W, so this new road will be N/S
					x1 = Math.min(cRoad.x1, cRoad.x2) + jct;
					y1 = cRoad.y1;
					do {

						// HH 16.7.14 Once we get half-way through the allocated number of tries, try adding roads in the other direction
						// HH 26.11.14 Moved this to the start so that don't inadverently change dirLen in the middle of the loop (when
						// we use it to sort out the direction of the junction at the end)
						if (noSmallIterations == 10) {
							dirLen = -dirLen;
						}
						
						length = dirLen * mapGenRandom.nextDouble() * Constants.WorldYVal;
						
						// HH 16.7.14 - Constrain the road to end at the edge of the map.  Note: roads can go right to the edge of the map
						// when they are perpendicular to the map edge, it is only parallel offsets that must be roadWidth/2 away.
						if ((y1 + length) < 0) {
							length = 0 - y1;
						} else if ((y1 + length) > Constants.WorldYVal) {
							length = Constants.WorldYVal - y1;
						}	
						
						noSmallIterations += 1; // HH 16.7.14 Increment counter so we don't get stuck in an infinite loop
						
						
						// HH 16.7.14 - Updated to check enclosing rectangle for overlaps during the loop					
					} while ((Math.abs(length) < Junction.jctApproachLen || sim.roadAtPoint(new Rectangle2D.Double(x1-(Road.roadWidth/2), (length > 0 ? y1-(Road.roadWidth/2) : y1+length), Road.roadWidth, Math.abs(length)), sim.roads, r)) && noSmallIterations < 20);

//					// Check to see if the enclosing rectangle (i.e. including road width) of this road that we want to create will
//					// intersect with roads other than the one it is supposed to join TODO - Could enforce that a perpendicular
//					// crossroads intersection is okay (as long as we then implement another method for detecting new junctions)
//					if (sim.roadAtPoint(new Rectangle2D.Double(x1-(Road.roadWidth/2), (length > 0 ? y1-(Road.roadWidth/2) : y1+length), Road.roadWidth, Math.abs(length)), sim.roads, r)) {
//						// Intersection with existing road has been detected, so try flipping the direction of the road
//						// Test for intersection will occur again before the road is added to the roads array.
//						length = -length;
//						dirLen = -dirLen;
//					} 
					
					//jctDir = (dirLen > 0) ? Constants.T_NORTH : Constants.T_SOUTH;
					//nJct = new Junction(sim.getNewID(), x1, y1, jctDir);
					nJct = new Junction(sim.getNewID(), x1, y1, (dirLen < 0 ? Math.abs(length) : 0), (cRoad.getLength() - jct), (dirLen < 0 ? 0 : length), jct);
					nRoad = new Road(sim.getNewID(), Constants.SINGLETWOWAY, new Double2D(x1,y1), new Double2D(x1,y1+length));
				}
				
				if (!sim.roadAtPoint(nRoad.getSurface(), sim.roads, r) && noSmallIterations < 20) {
					// No intersection with existing roads have been detected, so add the road to the roads array.
					sim.junctions.add(nJct);
					sim.roads.add(nRoad);
				} else if (noSmallIterations >= 20){
					sim.infoLog.addLog("Failure when adding junction: " + i + "; unable to locate intersection on road.");
				} else {
					sim.infoLog.addLog("Failure when adding junction: " + i + "; intersections detected in both directions.");
				}
				
			}
			
			if (noBigIterations >= 10) {
				sim.infoLog.addLog("Unable to find a location for junction " + i + " of " + noJcts + ", on any road after 10 attempts, aborting search...");
			}
			
			// TO DO - At some point implement X junctions in addition to T junctions
		}
		
		sim.infoLog.addLog("Junctions added: " + sim.junctions.size() + "/" + noJcts + "; roads added: " + sim.roads.size() + ".");
		// HH end
		
		sim.setNoJunctions(sim.junctions.size()); // HH 31/7/14 - So we know how many junctions were added at random
		
		// HH 16.7.14 Add junctions at the end of each road (if there is not already a junction there) - this is to ensure that 
		// vehicles will slow down as they approach the end of a road, and (hopefully) make them more likely to be able to complete
		// any necessary Uturn within the junction footprint and without straying onto the sidewalk.
		addJunctionsAtDeadEnds();
				
//		System.out.println("Simultaion stepping begins!");
//		System.out.println("==============================================================");
//		
		
		// HH 7.8.14 Reinstated obstacles for static obstacles class of ParkedCar
		Bag obstacles = sim.obstacles;
		double x;
		double y;	
		Double2D testPt;
		
		int noObstacles = (int) (mapGenRandom.nextDouble() * (Constants.MAX_OBSTACLES - Constants.MIN_OBSTACLES)) + Constants.MIN_OBSTACLES;
		
		int noIterations = 0; // HH 27.8.14 - Limit loop
		for (int i = 0; i < noObstacles; i++)
		{
			noIterations = 0; // reset for each added obstacle
			do {
				x = mapGenRandom.nextDouble() * Constants.WorldXVal;
				y = mapGenRandom.nextDouble() * Constants.WorldYVal;
				testPt = new Double2D(x,y);
				noIterations++; // HH 27.8.14 - Limit loop
			} while ((!sim.roadAtPoint(testPt, sim.roads) || sim.junctionAtPoint(testPt, sim.junctions) != 0 
					|| sim.junctionAppAtPoint(testPt, sim.junctions) || sim.junctionExitAtPoint(testPt, sim.junctions)
					|| sim.obstacleNearPoint(testPt, obstacles, sim.roads)) && noIterations < Constants.MAX_ITERATIONS); // HH 23.7.14 - edited to prevent adding Obstacles in Junction
					// HH 20.8.14 - edited to prevent adding Obstacles near junctions or other obstacles
			
			if (noIterations < Constants.MAX_ITERATIONS) // HH 27.8.14 - Make sure we didn't timeout on the loop
			{
				double inDirection = sim.getRoadDirectionAtPoint(testPt, sim.roads); // Check on the orientation of the road
				int roadId = sim.getRoadIdAtPoint(testPt, sim.roads); // HH 13.8.14 Get the ID of the road we are adding this car on

				ParkedCar ob = new ParkedCar(sim.getNewID(), Constants.TPARKEDCAR, inDirection, roadId);
				ob.setLocation(sim.snapToKerb(x,y));
				ob.isSchedulable = false;
				sim.allEntities.add(ob);
				obstacles.add(ob);
				//System.out.println("obstacle " + Integer.toString(i) + " is at (" + Double.toString(x) + ", " + Double.toString(y) + "), ID is"+ ob.ID);
			}
		}		
		
		sim.infoLog.addLog("Obstacles added: " + sim.obstacles.size() + "/" + noObstacles + ".");
		sim.setNoObstacles(obstacles.size()); // Store count
		
		// HH 7.5.14 - New code to add target (destination)
		int tID = sim.getNewID();
		Target t = new Target(tID);
		
		do {
			x = mapGenRandom.nextDouble() * Constants.WorldXVal;
			y = mapGenRandom.nextDouble() * Constants.WorldYVal;
		} while (!sim.roadAtPoint(new Double2D(x,y), sim.roads) || sim.junctionAtPoint(new Double2D(x,y), sim.junctions) != 0 || 
				sim.obstacleAtPoint(new Double2D(x,y), obstacles)); // HH 23.7.14 - edited to prevent adding Target in Junction + 7.8.14 or in Obstacles
		t.setLocation(new Double2D(x,y));
		t.isSchedulable = false;
		sim.allEntities.add(t);
		
		// HH 7.5.14 - new code to add UGV (start)
		do {
			x = mapGenRandom.nextDouble() * Constants.WorldXVal;
			y = mapGenRandom.nextDouble() * Constants.WorldYVal;
		} while (!sim.roadAtPoint(new Double2D(x,y), sim.roads) || sim.junctionAtPoint(new Double2D(x,y),  sim.junctions) != 0 || 
				sim.obstacleAtPoint(new Double2D(x,y), obstacles)); // HH 18.6.14 - edited to prevent adding UGV in Junction + 7.8.14 or in Obstacles
		
		// HH 18.6.14 - new code to snap the UGV to the nearest side of the road, and orient it 
		// so that it is facing in the right direction to start driving.
		initialInfo startInfo = sim.snapToLane(x,y);
				
		UGV theUGV = new UGV(sim.getNewID(), tID, sim.carStats, startInfo.startBearing, sim.junctions.size(), sim); // HH modified 18.6.14 to use new constructor with initial bearing
		sim.ugvs.add(theUGV);
		theUGV.setLocation(startInfo.startLoc); // HH 18.6.14 edited from "new Double2D(x,y)"
		theUGV.isSchedulable = true;
		sim.allEntities.add(theUGV);
		sim.toSchedule.add(theUGV);
		
		int noCars = (int) (mapGenRandom.nextDouble() * (Constants.MAX_CARS - Constants.MIN_CARS)) + Constants.MIN_CARS;
		
		for (int i = 0; i < noCars; i++)
		{		
			noIterations = 0; // reset for each loop
			// HH 26.8.14 - Add some car obstacles (not inside junctions or obstacles)
			do {
				x = mapGenRandom.nextDouble() * Constants.WorldXVal;
				y = mapGenRandom.nextDouble() * Constants.WorldYVal;
				noIterations++; // HH 27.8.14 limit loop
			} while ((!sim.roadAtPoint(new Double2D(x,y), sim.roads) || sim.junctionAtPoint(new Double2D(x,y),  sim.junctions) != 0 || 
					sim.obstacleAtPoint(new Double2D(x,y), obstacles) || theUGV.getLocation().distance(new Double2D(x,y)) < 5) && 
					noIterations < Constants.MAX_ITERATIONS) ; // HH 2.9.14 Prevent Car being added within 5m of UGV

			// Snap the Cars to the nearest side of the road, and orient it 
			// so that it is facing in the right direction to start driving.
			
			if (noIterations < Constants.MAX_ITERATIONS) // HH 27.8.14 - Make sure we didn't timeout on the loop
			{
				startInfo = sim.snapToLane(x,y);

				DumbCar theCar = new DumbCar(sim.getNewID(), sim.carStats, startInfo.startBearing); // HH modified 18.6.14 to use new constructor with initial bearing
				sim.cars.add(theCar);
				theCar.setLocation(startInfo.startLoc); // HH 18.6.14 edited from "new Double2D(x,y)"
				theCar.isSchedulable = true;
				sim.allEntities.add(theCar);
				sim.toSchedule.add(theCar);
			}
		}

		sim.infoLog.addLog("Car obstacles added: " + sim.cars.size() + "/" + noCars + ".");
		sim.setNoCars(sim.cars.size()); // store the count
		
	}
	
	
	/**
	 * A method which can be used with constructing a random simulation, this method
	 * will add a specified number of waypoints to a simulation which will give the 
	 * car a route to follow.
	 * 
	 * @param target the id of the eventual target of the car
	 * @param noWaypoints the number of waypoints to randomly add
	 * @return the id number of the first waypoint for the car to reach
	 */
//	private int addWaypoints(int target, int noWaypoints)
//	{
//		int newTarget = target;
//		double x;
//		double y;
//		
//		for (int i = 0; i < noWaypoints; i++)
//		{
//			x = sim.random.nextDouble() * worldXVal;
//			y = sim.random.nextDouble() * worldYVal;
//			newTarget = sim.getNewID();
//			Waypoint wp = new Waypoint(newTarget, target);
//			entAdder(wp, x, y, false);
//		}
//		
//		return newTarget;
//	}
	
	
	/**
	 * A method which contructs a Double2D coordinate from two values
	 * 
	 * @param x the x component of the coordinate
	 * @param y the y component of the coordinate
	 * @return the contructed Double2D coordinate
	 */
//	private static Double2D coordBuilder(double x, double y) {return new Double2D(x, y);}
	
	
	/**
	 * A method which adds an entity to the list of entities to be added to the
	 * environment when the simulation begins.
	 * 
	 * @param e the entity to be added
	 * @param x the x component of it's location
	 * @param y the y component of it's location
	 * @param schedulable a true if the entity needs to be added to the schedule, false if it doesn't
	 */
//	private void entAdder(Entity e, double x, double y, boolean schedulable)
//	{
//		Double2D loc = coordBuilder(x, y);
//		sim.addEntity(e, loc, schedulable);
//	}
//	

	
	
	/**
	 * 
	 * @param args
	 * @return 
	 */
	public static String[] addEndTime(String[] args)
	{
		String[] x = new String[args.length + 2];
		
		if (args.length != 0)
		{
			x = args;
		}
		
		x[args.length] = "-for";
		x[args.length + 1] = simLength;
		
		return x;
	}
	
	public COModel getSim() {return sim;}
	
	
	/**
	 * HH 16.7.14 - Method to add 'junctions' at the end of each road (where a junction does not already exist
	 * at that location).  Junctions will be added roadWidth/2 away from the end of the junction so that they 
	 * will remain within the existing footprint of the road (and therefore the map).
	 */
	private void addJunctionsAtDeadEnds()
	{
		Road tempRoad;
		Junction nJct;
		
		for (int r = 0; r < sim.roads.size(); r++)
		{
			tempRoad = (Road) sim.roads.get(r);
			
			// Is the road N/S?
			if (tempRoad.x1 == tempRoad.x2) {
			
				// See whether there is a junction at the top end of the road, if not, add one
				if (sim.junctionAtPoint(new Double2D(tempRoad.x1, Math.min(tempRoad.y1, tempRoad.y2)), sim.junctions) == 0)
				{
					nJct = new Junction(sim.getNewID(), tempRoad.x1, Math.min(tempRoad.y1, tempRoad.y2)+Road.roadWidth/2, 0, 0, tempRoad.getLength(), 0);
					sim.junctions.add(nJct);
				}
				
				// See whether there is a junction at the bottom end of the road, if not, add one
				if (sim.junctionAtPoint(new Double2D(tempRoad.x1, Math.max(tempRoad.y1, tempRoad.y2)), sim.junctions) == 0)
				{
					nJct = new Junction(sim.getNewID(), tempRoad.x1, Math.max(tempRoad.y1, tempRoad.y2)-Road.roadWidth/2, tempRoad.getLength(), 0, 0, 0);
					sim.junctions.add(nJct);					
				}				
				
			// Otherwise must be E/W
			} else {
				
				// See whether there is a junction at the west end of the road, if not, add one
				if (sim.junctionAtPoint(new Double2D(Math.min(tempRoad.x1, tempRoad.x2), tempRoad.y1), sim.junctions) == 0)
				{
					nJct = new Junction(sim.getNewID(), Math.min(tempRoad.x1, tempRoad.x2)+Road.roadWidth/2, tempRoad.y1, 0, tempRoad.getLength(), 0, 0);
					sim.junctions.add(nJct);
				}
				
				// See whether there is a junction at the bottom end of the road, if not, add one
				if (sim.junctionAtPoint(new Double2D(Math.max(tempRoad.x1, tempRoad.x2), tempRoad.y1), sim.junctions) == 0)
				{
					nJct = new Junction(sim.getNewID(), Math.max(tempRoad.x1, tempRoad.x2)-Road.roadWidth/2, tempRoad.y1, 0, 0, 0, tempRoad.getLength());
					sim.junctions.add(nJct);					
				}				
			}
		}
	}
	
}
