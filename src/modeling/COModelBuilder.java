package modeling;
import java.awt.geom.Area;
import java.awt.geom.Rectangle2D;
import java.security.SecureRandom;

import ec.util.MersenneTwisterFast;
import modeling.COModel.initialInfo;
import sim.util.*;

/**
 * @author Robert Lee/hh940
 * This class is used to build/initiate the simulation.
 * There are "main" methods for running the simulation with and without GUI
 * or without running the model (i.e. just constructing the map).  This class
 * generates the initial simulation map using the 'external' random number seed
 * which held by COModel sim.  The seed is used to calculate how many of each
 * entity type to attempt to add, and where they should be located.
 */
public class COModelBuilder
{	
	public  COModel sim;
	
	/**
	 * Constructor - initialise the sim object to be the supplied COModel parameter
	 * @param s (COModel - the COModel object to use for the simulation)
	 */
	public COModelBuilder(COModel s)
	{
		sim = s;
	}
		
	/**
	 * Method to update the internal seed used by the underlying COModel so that can 
	 * do a batch run using different seeds if required (allows for different 
	 * run-time behaviour on the same map). 
	 * @param newSeed (long - new internal seed to use in the underlying COModel sim)
	 */
	public void updateSeed(long newSeed)
	{
		sim.setSeed(newSeed);
	}
	
	/**
	 * Main method to generate the simulation environment.  
	 */
	public void generateSimulation()
	{		
		// NOTE: This method cannot use sim.random if we want to provide the ability to run multiple
		// fault patterns on the same base map (network), so we use a separate External Seed when
		// generating the map itself.
		if (sim.getExternalSeed() == 0) {
			// The seed hasn't been set in the UI, or when the COModel was created, so set one now
			// based on a pseudo-random number generated from the Java SecureRandom class 
			sim.setExternalSeed(new SecureRandom().nextInt()); 
		}
			
		// Now create the new random generator, it can be local as we only use it here to generate the map
		MersenneTwisterFast mapGenRandom = new MersenneTwisterFast(sim.getExternalSeed());
		
		sim.infoLog.addHeader(sim); // Add header to info log file

		// ****  Add the FIRST ROAD  ****
		
		// Decide how many junctions we are going to try to add to the map by generating a number at 
		// random in the appropriate range
		int noJcts = (int) (mapGenRandom.nextDouble() * (Constants.MAX_JUNCTIONS - Constants.MIN_JUNCTIONS)) + Constants.MIN_JUNCTIONS;
				
		// Support variables for adding Road features to the model
		double x1, y1, length, jct;
		Road road;
		int r;
		
		// Choose a random location for start of road (must not intersect with an existing road 
		// - although there shouldn't be any other roads)
		do {
			// Ensure that the road cannot overlap (in parallel) with the edge of the map by constraining the
			// coordinates to be offset from the edge of the map by half the roadWidth
			x1 = (mapGenRandom.nextDouble() * (Constants.WorldXVal - Road.roadWidth)) + Road.roadWidth/2;
			y1 = (mapGenRandom.nextDouble() * (Constants.WorldYVal - Road.roadWidth)) + Road.roadWidth/2;
		}  while (sim.roadAtPoint(new Double2D(x1,y1), sim.roads));
		
		length = 1; // reset length before we use it
		
		// decide whether the road should be N/S or E/W
		if (mapGenRandom.nextBoolean()) {
			// 'Coin toss = true' so choose E/W
			// Allow lengths that will take us outside the map, but truncate at the boundary
			while (Math.abs(length) < Junction.jctApproachLen) {
				// decide whether this should be applied in negative or positive (we will use these as 
				// multipliers for the results below)
				if (mapGenRandom.nextBoolean()) {
					length = -1;
				} else {
					length = 1;
				}
				
				length = length * mapGenRandom.nextDouble() * Constants.WorldXVal; // select a length at random
				
				// Constrain the road to end at the edge of the map.  Note: roads can go right to the edge of the map
				// when they are perpendicular to the map edge, it is only parallel offsets that must be roadWidth/2 away.
				if ((x1 + length) < 0) {
					length = 0 - x1;
				} else if ((x1 + length) > Constants.WorldXVal) {
					length = Constants.WorldXVal - x1;
				}
			}
			
			road = new Road(sim.getNewID(), Constants.SINGLETWOWAY, new Double2D(x1,y1), new Double2D(x1+length,y1));
		
		} else {
			// 'Coin toss = false' so choose N/S
			// Allow lengths that will take us outside the map, but truncate at the boundary
			while (Math.abs(length) < Junction.jctApproachLen) {
				// decide whether this should be applied in negative or positive (we will use these as multipliers for the results below)
				if (mapGenRandom.nextBoolean()) {
					length = -1;
				} else {
					length = 1;
				}
				
				length = length * mapGenRandom.nextDouble() * Constants.WorldYVal; // select a length at random
				
				// Constrain the road to end at the edge of the map.  Note: roads can go right to the edge of the map
				// when they are perpendicular to the map edge, it is only parallel offsets that must be roadWidth/2 away.
				if ((y1 + length) < 0) {
					length = 0 - y1;
				} else if ((y1 + length) > Constants.WorldYVal) {
					length = Constants.WorldYVal - y1;
				}
			}
			
			road = new Road(sim.getNewID(), Constants.SINGLETWOWAY, new Double2D(x1,y1), new Double2D(x1,y1+length));
		}
		
		sim.roads.add(road); // Add this (first) road to the collection
				
		// ****  Add the remaining JUNCTIONS/ROADS  ****
		
		// variables required to support addition of more junctions and roads
		double cx = 0;
		double cy = 0;
		boolean jctFound = false;
		int noSmallIterations = 0; // limit the number of times we try to place a junction on a selected road
		int noBigIterations = 0; // limit the number of times that we select a new road and try and place the junction on it
		Road cRoad, nRoad; // chosen road, new road
		Junction nJct;
		int dirLen = 1;
		Area jctArea; // Supports junction overlap check below.
		
		// Init cRoad/jct to prevent compilation errors below
		cRoad = (Road) sim.roads.get(0); 
		r = 0;
		jct = 0;
		
		// For each junction, choose a location on the road (random but constrained by length of road)
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

				// Generate random from 0 to length (but not within roadWidth * 1.5 of either end of the road 
				// and make sure it is within the model bounds)

				// TODO - this method will need to be updated when we revert to non-grid based roads
				do {
					jct = ((mapGenRandom.nextDouble(true, true) * (length - (3*Road.roadWidth))) + (Road.roadWidth*1.5));
					
					// work out the 'start' coordinates for the road (so we add from the right end)
					cx = Math.min(cRoad.x1,  cRoad.x2);
					cy = Math.min(cRoad.y1,  cRoad.y2);
					
					noSmallIterations += 1; // increment counter so we don't get stuck in an infinite loop
					
					// Check for overlapping junctions by creating a rectangle area to represent the proposed
					// junction, and in the do-while loop force another iteration if the junction overlaps
					// with an existing junction
					Double2D jctLoc;
					if (cRoad.getIsNS()) {
						jctLoc = new Double2D(cx, (cy+jct));
					} else {
						jctLoc = new Double2D((cx+jct), cy);
					}
					Rectangle2D jctRect = new Rectangle2D.Double((jctLoc.x-(Road.roadWidth/2)), (jctLoc.y-(Road.roadWidth/2)), Road.roadWidth, Road.roadWidth);
					jctArea = new Area(jctRect);
					
				} while (((cRoad.getIsNS() && (((cy + jct >= (Constants.WorldYVal-Road.roadWidth))) || (cy + jct < 0) || (sim.junctionAtArea(jctArea, sim.junctions) != 0))) || 
						 (!cRoad.getIsNS() && (((cx + jct >= (Constants.WorldXVal-Road.roadWidth))) || (cx + jct < 0) || (sim.junctionAtArea(jctArea, sim.junctions) != 0)))) &&
						 noSmallIterations < 10);
				
				// If we've reached here and we haven't timed out, then we know that we've found a location for the junction
				if (noSmallIterations < 10){
					jctFound = true;
				}
			}	

			if (jctFound == true) {

				// We're going to reuse length to work out the length of our new road, and first we'll decide whether it should 
				// be applied in negative or positive i.e. which way will the new road extend from the junction we have just 
				// selected (we will use these as multipliers for the results below)
				if (mapGenRandom.nextBoolean()) {
					dirLen = -1;
				} else {
					dirLen = 1;
				}

				noSmallIterations = 0; // Reset counter for this junction location
				
				// Work out coordinate corresponding to displacement of random from start of road, choose a length for the new road
				// and add the new junction and road unless the road length is less than the junction approach length, or the new
				// road overlaps with an existing road, or we have exceeded the 20 attempts to locate a junction on this chosen road
				if (cRoad.getIsNS()) {
					
					// The chosen road cRoad (where we are placing the junction) is N/S, so this new road will be E/W
					x1 = cRoad.x1;
					y1 = Math.min(cRoad.y1, cRoad.y2) + jct;
					do {
						
						// Once we get half-way through the allocated number of tries (20), try adding roads in the other direction
						// This needs to be here at the start so that we don't inadvertently change dirLen in the middle of the loop (as
						// we use it to sort out the direction of the junction at the end)
						if (noSmallIterations == 10) {
							dirLen = -dirLen;
						}
						
						length = dirLen * mapGenRandom.nextDouble() * Constants.WorldXVal;
						
						// Constrain the road to end at the edge of the map.  Note: roads can go right to the edge of the map
						// when they are perpendicular to the map edge, it is only parallel offsets that must be roadWidth/2 away.
						if ((x1 + length) < 0) {
							length = 0 - x1;
						} else if ((x1 + length) > Constants.WorldXVal) {
							length = Constants.WorldXVal - x1;
						}						
						
						noSmallIterations += 1; // Increment counter so we don't get stuck in an infinite loop
						
					// Check that the road is long enough, that it does not overlap with other roads, and that we
					// haven't exceeded the maximum number of attempts to add a road at this junction location
					} while ((Math.abs(length) < Junction.jctApproachLen || 
							  sim.roadAtPoint(new Rectangle2D.Double((length > 0 ? x1-(Road.roadWidth/2) : x1+length), y1-(Road.roadWidth/2), Math.abs(length), Road.roadWidth), sim.roads, r)) 
							 && noSmallIterations < 20);
					
					// Add the new junction, providing the lengths of each junction arm e.g. if dirLen is negative, the eastern arm length is 0,
					// but if dirLen is positive, this will be the length of the new road; add the new road.
					nJct = new Junction(sim.getNewID(), x1, y1, jct, (dirLen < 0 ? 0 : Math.abs(length)), (cRoad.getLength()-jct), (dirLen < 0 ? Math.abs(length) : 0));
					nRoad = new Road(sim.getNewID(), Constants.SINGLETWOWAY, new Double2D(x1,y1), new Double2D(x1+ length,y1));
					
				} else {
					
					noSmallIterations = 0; // Reset counter for this junction location
					
					// The chosen road cRoad (where we are placing the junction) is E/W, so this new road will be N/S
					x1 = Math.min(cRoad.x1, cRoad.x2) + jct;
					y1 = cRoad.y1;
					do {

						// Once we get half-way through the allocated number of tries (20), try adding roads in the other direction
						// This needs to be here at the start so that we don't inadvertently change dirLen in the middle of the loop (as
						// we use it to sort out the direction of the junction at the end)
						if (noSmallIterations == 10) {
							dirLen = -dirLen;
						}
						
						length = dirLen * mapGenRandom.nextDouble() * Constants.WorldYVal;
						
						// Constrain the road to end at the edge of the map.  Note: roads can go right to the edge of the map
						// when they are perpendicular to the map edge, it is only parallel offsets that must be roadWidth/2 away.
						if ((y1 + length) < 0) {
							length = 0 - y1;
						} else if ((y1 + length) > Constants.WorldYVal) {
							length = Constants.WorldYVal - y1;
						}	
						
						noSmallIterations += 1; // Increment counter so we don't get stuck in an infinite loop
						
						
					// Check that the road is long enough, that it does not overlap with other roads, and that we
					// haven't exceeded the maximum number of attempts to add a road at this junction location
					} while ((Math.abs(length) < Junction.jctApproachLen || 
							  sim.roadAtPoint(new Rectangle2D.Double(x1-(Road.roadWidth/2), (length > 0 ? y1-(Road.roadWidth/2) : y1+length), Road.roadWidth, Math.abs(length)), sim.roads, r)) 
							 && noSmallIterations < 20);

					// Add the new junction, providing the lengths of each junction arm e.g. if dirLen is negative, the eastern arm length is 0,
					// but if dirLen is positive, this will be the length of the new road; add the new road.
					nJct = new Junction(sim.getNewID(), x1, y1, (dirLen < 0 ? Math.abs(length) : 0), (cRoad.getLength() - jct), (dirLen < 0 ? 0 : length), jct);
					nRoad = new Road(sim.getNewID(), Constants.SINGLETWOWAY, new Double2D(x1,y1), new Double2D(x1,y1+length));
				}
				
				// If our new road doesn't overlap with an existing road, and we haven't timed out, then we can
				// add the new road and junction to their respective collections.
				if (!sim.roadAtPoint(nRoad.getSurface(), sim.roads, r) && noSmallIterations < 20) {
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
			
			// TODO - At some point implement X junctions in addition to T junctions
		}
		
		sim.infoLog.addLog("Junctions added: " + sim.junctions.size() + "/" + noJcts + "; roads added: " + sim.roads.size() + ".");
		
		sim.setNoJunctions(sim.junctions.size()); // So we know how many junctions were actually added at random
		
		// Add junctions at the end of each road (if there is not already a junction there) - this is to ensure that 
		// vehicles will slow down as they approach the end of a road, and (hopefully) make them more likely to be able to complete
		// any necessary Uturn within the junction footprint and without straying onto the sidewalk.
		addJunctionsAtDeadEnds();
					
		// ****  Add the PARKED CARS  ****
		
		// Parameters for static obstacles class of ParkedCar
		Bag obstacles = sim.obstacles;
		double x;
		double y;	
		Double2D testPt;
		Double2D tempLoc;
		
		int noObstacles = (int) (mapGenRandom.nextDouble() * (Constants.MAX_OBSTACLES - Constants.MIN_OBSTACLES)) + Constants.MIN_OBSTACLES;
		
		int noIterations = 0; // To limit number of iterations
		
		// Loop for number of obstacles that we want to attempt to add.  Choose up to 50 map locations at random,
		// each time checking that the chosen location is on a road, but not in a junction, junction approach, or
		// junction exit.  The location must also not be too near any other obstacles (see COModel.obstacleNearPoint)
		// As the chosen location will be the centre of the parked car, it is possible that the full vehicle might overlap
		// partially with a junction exit, or a junction approach.
		for (int i = 0; i < noObstacles; i++)
		{
			noIterations = 0; // reset loop counter for each added obstacle
			
			do {
				x = mapGenRandom.nextDouble() * Constants.WorldXVal;
				y = mapGenRandom.nextDouble() * Constants.WorldYVal;
				testPt = new Double2D(x,y);
				noIterations++; 
			} while ((!sim.roadAtPoint(testPt, sim.roads) || sim.junctionAtPoint(testPt, sim.junctions) != 0 
					|| sim.junctionAppAtPoint(testPt, sim.junctions) || sim.junctionExitAtPoint(testPt, sim.junctions)
					|| sim.obstacleNearPoint(testPt, obstacles, sim.roads, sim)) && noIterations < Constants.MAX_ITERATIONS);
			
			// If we didn't timeout from the above loop, then we must have found a suitable location to 
			// add the ParkedCar.  Now we need to work out which side of the road
			// our chosen location is on, snap it to the kerb, and add the new ParkedCar object.
			if (noIterations < Constants.MAX_ITERATIONS) 
			{
				double inDirection = sim.getRoadDirectionAtPoint(testPt); // Check on the orientation of the road
				int roadId = sim.getRoadIdAtPoint(testPt, sim.roads); // Get the ID of the road we are adding this car on

				tempLoc = sim.snapToKerb(x,y);
				if (tempLoc.x != -1) {
					ParkedCar ob = new ParkedCar(sim.getNewID(), Constants.TPARKEDCAR, inDirection, roadId);
					ob.setLocation(tempLoc);
					ob.isSchedulable = false;
					sim.allEntities.add(ob);
					obstacles.add(ob);
				} else {
					sim.infoLog.addLog("Problem adding car at: (" + x + "," + y + ").");
				}
			}
		}		
		
		// Record and store the number of obstacles that we were actually able to add (this may
		// differ from the number that we attempted to add - from the random number generator)
		sim.infoLog.addLog("Obstacles added: " + sim.obstacles.size() + "/" + noObstacles + ".");
		sim.setNoObstacles(obstacles.size()); 
		
		// ****  Add the TARGET  ****
		
		// Choose a location at random for the target, note that there is no limit to the number 
		// of iterations of this loop, the target must be added somewhere.  Suitable locations 
		// are limited to 'on road' locations, which must not be in junctions, or overlapping 
		// with obstacles (ParkedCar); the loop will repeat until these constraints are met.
		int tID = sim.getNewID();
		Target t = new Target(tID);
		
		do {
			x = mapGenRandom.nextDouble() * Constants.WorldXVal;
			y = mapGenRandom.nextDouble() * Constants.WorldYVal;
		} while (!sim.roadAtPoint(new Double2D(x,y), sim.roads) || sim.junctionAtPoint(new Double2D(x,y), sim.junctions) != 0 || 
				sim.obstacleAtPoint(new Double2D(x,y), obstacles));
		t.setLocation(new Double2D(x,y));
		t.isSchedulable = false;
		sim.allEntities.add(t);
		
		// ****  Add the UGV  ****
		
		// Choose a location at random for the UGV start point, note that there is no limit 
		// to the number of iterations of this loop, the target must be added somewhere.  
		// Suitable locations are limited to 'on road' locations, which must not be in junctions, 
		// or overlapping with obstacles (ParkedCar); the loop will repeat until these 
		// constraints are met.
		do {
			x = mapGenRandom.nextDouble() * Constants.WorldXVal;
			y = mapGenRandom.nextDouble() * Constants.WorldYVal;
		} while (!sim.roadAtPoint(new Double2D(x,y), sim.roads) || sim.junctionAtPoint(new Double2D(x,y),  sim.junctions) != 0 || 
				sim.obstacleAtPoint(new Double2D(x,y), obstacles));
		
		// Snap the UGV to the nearest side of the road, and orient it 
		// so that it is facing in the right direction to start driving.
		initialInfo startInfo = sim.snapToLane(x,y);	
		UGV theUGV = new UGV(sim.getNewID(), tID, sim.carStats, startInfo.startBearing, sim.junctions.size(), sim); 
		sim.ugvs.add(theUGV);
		theUGV.setLocation(startInfo.startLoc);
		theUGV.isSchedulable = true;
		sim.allEntities.add(theUGV);
		sim.toSchedule.add(theUGV);
		
		// ****  Add the MOVING CARS  ****
		
		// Generate a random number in the appropriate range to specify the number of cars
		// that we would like to add to the model.  Loop for each of these cars, allowing each car 50
		// attempts to choose a location at random for its start point.  Suitable locations are
		// limited to 'on road' locations, which must not be in junctions, or overlapping with obstacles (ParkedCar).  
		// They must also be at least 5m from the UGV location.  The loop will repeat until these 
		// constraints are met, or the iteration limit is reached.
		int noCars = (int) (mapGenRandom.nextDouble() * (Constants.MAX_CARS - Constants.MIN_CARS)) + Constants.MIN_CARS;
		
		for (int i = 0; i < noCars; i++)
		{		
			noIterations = 0; // reset for each loop
			
			do {
				x = mapGenRandom.nextDouble() * Constants.WorldXVal;
				y = mapGenRandom.nextDouble() * Constants.WorldYVal;
				noIterations++;
			} while ((!sim.roadAtPoint(new Double2D(x,y), sim.roads) || sim.junctionAtPoint(new Double2D(x,y),  sim.junctions) != 0 || 
					sim.obstacleAtPoint(new Double2D(x,y), obstacles) || theUGV.getLocation().distance(new Double2D(x,y)) < 5) && 
					noIterations < Constants.MAX_ITERATIONS) ; 

			// If we didn't timeout from the above loop then we have found a suitable location to add the Car
			// Snap the Car to the nearest side of the road, and orient it so that it is facing in the right 
			// direction to start driving.
			if (noIterations < Constants.MAX_ITERATIONS) 
			{
				startInfo = sim.snapToLane(x,y);
				DumbCar theCar = new DumbCar(sim.getNewID(), sim.carStats, startInfo.startBearing);
				sim.cars.add(theCar);
				theCar.setLocation(startInfo.startLoc);
				theCar.isSchedulable = true;
				sim.allEntities.add(theCar);
				sim.toSchedule.add(theCar);
			}
		}

		// Record and store the number of cars that we were actually able to add (this may
		// differ from the number that we attempted to add - from the random number generator)
		sim.infoLog.addLog("Car obstacles added: " + sim.cars.size() + "/" + noCars + ".");
		sim.setNoCars(sim.cars.size());
	}
			
	/**
	 * This method returns the COModel simulation which is used by this COModelBuilder
	 * @return COModel (the COModel simulation object in use)
	 */
	public COModel getSim() {return sim;}
	
	/**
	 * Method to add 'junctions' at the end of each road (where a junction does not already exist
	 * at that location).  Junctions will be added roadWidth/2 away from the ends of the road so that they 
	 * will remain within the existing footprint of the road (and therefore the map).
	 */
	private void addJunctionsAtDeadEnds()
	{
		Road tempRoad;
		Junction nJct;
		
		// Loop through each road so that we can add the (extra) junctions at the ends of each road
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
				
				// See whether there is a junction at the western end of the road, if not, add one
				if (sim.junctionAtPoint(new Double2D(Math.min(tempRoad.x1, tempRoad.x2), tempRoad.y1), sim.junctions) == 0)
				{
					nJct = new Junction(sim.getNewID(), Math.min(tempRoad.x1, tempRoad.x2)+Road.roadWidth/2, tempRoad.y1, 0, tempRoad.getLength(), 0, 0);
					sim.junctions.add(nJct);
				}
				
				// See whether there is a junction at the eastern end of the road, if not, add one
				if (sim.junctionAtPoint(new Double2D(Math.max(tempRoad.x1, tempRoad.x2), tempRoad.y1), sim.junctions) == 0)
				{
					nJct = new Junction(sim.getNewID(), Math.max(tempRoad.x1, tempRoad.x2)-Road.roadWidth/2, tempRoad.y1, 0, 0, 0, tempRoad.getLength());
					sim.junctions.add(nJct);					
				}				
			}
		}
	}
}
