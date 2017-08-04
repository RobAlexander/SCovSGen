package modeling;

import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.Area;
import java.awt.geom.Rectangle2D;

import modeling.COModel.jctExitInfo;
import sim.engine.SimState;
import sim.field.continuous.Continuous2D;
import sim.util.Bag;
import sim.util.Double2D;
import sim.util.MutableDouble2D;

/**
 * Moving obstacles which can start anywhere on the road network, choose junction exits at random,
 * and follow a trajectory that involves them following a fixed offset from the kerb - based on 'knowing'
 * their location on the road.  If the vehicle chooses a junction exit at a dead-end that results in it
 * leaving the road network (synonymous with reaching a car park), another vehicle is added to appear in
 * a dead-end (chosen at random) to maintain a relatively constants number of moving obstacles in the 
 * simulation.  TODO - In the future could vary the road position based on a double sine wave
 * to simulate drunk-driving as a future test scenario.  Other similar movement variation could be introduced.
 * 
 * This vehicle type is supposed to simulate a vehicle operated by a 'real' driver and therefore we
 * assume more knowledge of the environment.  We do not 'care' about whether DumbCars crash, or about 
 * crashes which are the fault of the DumbCar.  The behaviour of these vehicles is not fault free, as 
 * more emphasis has been placed on improving the UGV algorithms, and so faults/failures in this class 
 * are ignored to reduce their impact on the results of the simulations.  Note that if the UGV algorithms
 * had been used by the other cars in the network, it would probably have led to gridlock, or deadlock
 * situations as the UGV is programmed to be very cautious.  To prevent deadlock, the DumbCar is more
 * aggressive.
 */
public class DumbCar extends Car {
	
	private static final long serialVersionUID = 1L;

	/**
	 * Constructor - create a moving car obstacle of type DUMBCAR.  
	 * @param idNo (int - unique identifier)
	 * @param performance (CarPerformance - the performance parameters of the car)
	 * @param initialBearing (double - direction in which the vehicle start pointing)
	 */
	public DumbCar(int idNo, CarPerformance performance,
			double initialBearing) {
		super(idNo, -1, performance, initialBearing, DUMBCAR);
		// NOTE: The targetId of a DumbCar will be -1 as it does not have a target
	}

	/**
	 * This method overrides Car.step as the DumbCar has more knowledge about its environment than 
	 * for example the UGV 
	 * @param state (SimState - access to the simulation environment)
	 */
	@Override
	public void step(SimState state)
	{
		sim = (COModel) state;
		
		if(this.isActive == true)
		{
			resetSpeedParams(); // Reset these speed change vote counters for this step
			
			Continuous2D environment = sim.environment;
			
			Double2D me = environment.getObjectLocation(this);
			
			// Check we are still on the road, and if we have left it, remove self from sim,
			// do necessary housekeeping, and add a new DumbCar to the network.
			if (onRoad(sim.roads, getShape()) == false) {
				
				// Log speed of car leaving the road
				sim.infoLog.addLog("Step: " + sim.schedule.getSteps() + ", Car: " + this.getID() + 
								   " has left the road at: " + me.toString() + ", bearing: " + this.getDirection() + ".");
				
				// Need to make sure we aren't blocking junctions unnecessarily 
				if (getJctID() > 0)
				{
					// Log car actually leaving junction
					sim.infoLog.addLog("Step: " + sim.schedule.getSteps() + ", All of Car: " + this.getID() + " has left junction" + 
							", at speed: " + this.getSpeed() + ", bearing: " + this.getDirection() + "; unOccupy called on " +
							"Junction #" + getJctID() + ".");
					
					// We've left the junction, so reset the junction occupancy so someone else can enter
					sim.unOccupyJunction(getJctID(), sim.junctions, this.getID());
					setJctID(0);
				}
				
				this.isActive = false;
				environment.remove(this); // Remove from environment
				sim.dealWithTermination();
				
				// Add a new car to the simulation at a random entry point to the network
				sim.addNewDumbCar();
				return;
			}	
			
			MutableDouble2D sumForces = new MutableDouble2D(); // Record the changes to be made to the location of the car

			// Store the previous location now, before we do anything with it
			storePrevLoc(me);
			
			double moveV; // Vertical component of the cars movement
			double moveH; // Horizontal component of the cars movement
			
			// Get location of target
			Bag everything = environment.getAllObjects(); // This will get all of the objects in the world, then start filtering
			
			this.setStats(sim.getCarMaxSpeed(),sim.getCarMaxAcceleration(), sim.getCarMaxDecceleration(), sim.getCarMaxTurning()); 
			
			Entity e; // Temp variable for iterating
			Entity eTarget = new Entity(-1, TOTHER); // This id for the target is illegal, to get ids one should use COModel.getNewID()
			
			// Find the current target from the bag of all entities (if there is one)
			if (this.getTargetID() != -1) {
				for(int i = 0; i < everything.size(); i++)
				{
					e = (Entity) everything.get(i);			
					if (e.getID() == this.getTargetID())
					{
						eTarget =  e;
					}
				}
			}
			
			// Check to see if we have recently started to leave a junction, and are just
			// waiting for the 'back-end' to vacate the junction so we can mark it as cleared.
			
			if (getJctID() > 0)
			{
				// Check that we have started to leave the junction
				if (sim.junctionAtPoint(me, sim.junctions) == 0 || sim.junctionAtPoint(me, sim.junctions) != this.getJctID())
				{
					// Check to make sure that the original junction has been completely vacated (there are 
					// accidents caused by the DC entering a second junction whilst remaining in the first junction 
					// too - and then the first junction is marked as empty; however this shouldn't happen anymore.
					int jctRetVal = sim.junctionAtArea(new Area(this.getShape()), sim.junctions);
					
					// See if we have entirely left the junction
					if (jctRetVal == 0) 
					{
						// We've left the junction, so reset the junction occupancy so someone else can enter
						sim.unOccupyJunction(getJctID(), sim.junctions, this.getID());
						setJctID(0);
					} else if (jctRetVal < 0) { // -1 return code suggests vehicle overlaps multiple junctions
						
						// Need to double-check that the vehicle has actually left the first junction, 
						// if it has, we can free up the junction
						for (int i=0; i < sim.junctions.size(); i++)
						{
							Junction tempJct = (Junction) sim.junctions.get(i);
							
							if (((Junction)sim.junctions.get(i)).ID == getJctID())
							{
								// Create a shape to represent this junction
								Rectangle2D tempJctShape = new Rectangle2D.Double((tempJct.location.x-(Road.roadWidth/2)), (tempJct.location.y-(Road.roadWidth/2)), Road.roadWidth, Road.roadWidth);
								Area jctArea = new Area (tempJctShape);
								jctArea.intersect(new Area(this.getShape())); // Intersect with the DumbCar
								if (jctArea.isEmpty()) {
									// We've left the junction, so reset the junction occupancy so someone else can enter
									sim.unOccupyJunction(getJctID(), sim.junctions, this.getID());
									setJctID(0);
								}
							}
						}
					}
				}
			}
						
			// Check whether we are approaching a junction (slow down) - if we are already executing a turning manoeuvre, 
			// we don't need to check the junctions as it's immaterial until we have finished the turn.
			boolean inJunctionOrApproach = false;
					
			if (eTarget.getType() != TUTURNWP)
			{
				for(int i = 0; i < sim.junctions.size(); i++) 
				{				
					// ARE WE INSIDE A JUNCTION (only checking whether the 'location' of the DumbCar is in a junction
					// this is the centre of the front of the vehicle, so we should pick up a vehicle as soon as it 
					// starts to enter the junction.
					if (((Junction) sim.junctions.get(i)).inShape(me)) {
								
						// Log speed of car entering junction
						sim.infoLog.addLog("Step: " + sim.schedule.getSteps() + ", Car: " + this.getID() + " entering junction " + ((Junction)sim.junctions.get(i)).getID() + 
								           ", at speed: " + this.getSpeed() + ", bearing: " + this.getDirection() + ".");
						
						// Vehicle currently within the junction, we need a new waypoint to redirect
						// towards the destination (junction exit)
						jctExitInfo junctionInfo = ((Junction) sim.junctions.get(i)).getRandomExit(this, i, sim);
						Double2D junctionWP = junctionInfo.exitWP;

						// Implementing 4-way Stop						
						// Check on the return value, if we don't get a valid WP back, we haven't
						// succeeded in entering the junction.  We need to slow down to zero (perhaps log an emergency
						// stop if that exceeds the maximum deceleration of the vehicle), and we shouldn't set 
						// the inJunctionFlag.  ALSO, make sure that we can't execute any movements by following the 
						// methods which follow - may need to set a new mode for TWAIT.
						if (junctionWP.x == -1 && junctionWP.y == -1) 
						{
							// Something has gone wrong, an exit has not been chosen - maybe the junction is
							// already occupied - Execute emergency stop (entering in the log file if this 
							// would not have been possible in reality, given the performance characteristics of the
							// vehicle.
							double overshoot = emergencyStop();
							
							if (overshoot > 0) {
								sim.infoLog.addLog("Step: " + sim.schedule.getSteps() + ", Car: " + this.getID() + " would not have stopped in time, excess speed = " + 
										overshoot + " in junction " + ((Junction)sim.junctions.get(i)).getID() + 
							           ", at speed: " + this.getSpeed() + ", bearing: " + this.getDirection() + ".");
							}
							
							startWaiting(); // Put the DumbCar into waiting mode
							
						} else {
							eTarget = createWaypoint(junctionWP, sim, me, TUTURNWP, false, null); // set eTarget to point to new WP
							
							goSlow(); // Reduce the speed, we're entering a junction
						}
						
						inJunctionOrApproach = true;  // Need to set this to true regardless to prevent acceleration
											
						// ARE WE INSIDE A JUNCTION APPROACH
					} else if (((Junction) sim.junctions.get(i)).inApproach(me)) {
						
						// Vehicle currently within the junction approach, slow down and maintain current direction
						inJunctionOrApproach = true;
						goReallySlow(); // A stronger vote for a speed decrease to try and prevent overshoot
					}
				}
			} else {
				
				// Try to ensure turn is as tight as possible
				goSlow();
			}
									
			// If the DumbCar is turning in a junction, and not in waiting mode, check to 
			// see if it has managed to leave the junction.  If not, the DumbCar direction should be
			// set so that it continues to turn towards the junction exit.
			if (eTarget.getType() == TUTURNWP && isWaiting() == false) 
			{				
				// If we have left the junction, assume we can clear the WP - regardless of the
				// distance separation. If two junctions are close together/overlapping, then the vehicle
				// can leave the first junction, and appear in the second one before the WP has been cleared so 
				// we need to check for a change in the junctionId (or jctID == -1), in addition to leaving the junction.
				// NOTE : An issue may still be present when there is an actual overlap of junctions because the 
				// code above for detecting junctions will only execute for the first junction with which an overlap is
				// detected (which could be the one that the vehicle is supposed to have left) - although as the WP is placed
				// slightly outside of the original junction, the vehicle should not actually be inside the original junction
				// when the WP is cleared and the code above executes to find another junction entry.
				if (sim.junctionAtPoint(me, sim.junctions) == 0 || sim.junctionAtPoint(me, sim.junctions) != this.getJctID())
				{
					// Log speed of car leaving junction
					sim.infoLog.addLog("Step: " + sim.schedule.getSteps() + ", Front of Car: " + this.getID() + " leaving junction" + 
							           ", at speed: " + this.getSpeed() + ", bearing: " + this.getDirection() + ".");
										
					this.setTargetID(-1); // back to default as have 'reached' target
					environment.remove(eTarget);
										
				} else if (sim.junctionAtPoint(me, sim.junctions) != 0) { // Only set the direction when we are in the junction
				
					setDirection(me, eTarget.getLocation());
					
					inJunctionOrApproach = true; // This isn't being set when the vehicle is already in a junction
				}
			}
			
			// OBSTACLE AVOIDANCE - Need a different method for DumbCar (cf. Car) as it's taking far too long to conduct 
			// the lidar-based search instead will use something that will be much 'cheaper' and is not a 'cheat' as 
			// these vehicles are meant to simulate human-drivers anyway, so should have access to better information.
			double distToObs = findImminentCrash(sim.cars);
			distToObs = Math.min(distToObs, findImminentCrash(sim.ugvs));
			
			double prevDist = getStoppingDistance();
			this.setStoppingDistance(distToObs); // Update for next step
			
			// Adjust spped requests depending upon whether we are getting closer to, or further from the
			// car/ugv in front.
			if ((distToObs < 20 && distToObs >= 0) && distToObs < prevDist) 
			{
				goSlowStop(); // We are close to the vehicle in front, and getting closer so slow down!
			} else if (distToObs <= 0 && distToObs > -10 && isWaiting() == false) { 
				goFaster(true); // Force a speed up in case 2 vehicles are too close/overlapping (this
				// should only cause the front one to speed up, as it is the only one which should return
				// a negative distance to obstacle in that range.
			} else {
				goFaster(false); // This is just a vote to speed up, in case the vehicle has got stuck
			}			
			
			// We need to ensure that when a vehicle is supposed to be waiting at a junction, it must
			// not be allowed to *creep* forward 
			if (isWaiting() == true)
			{
				goSlowStop(); // Make sure that we actually slow down to 0
			}
						
			// If the vehicle is not in a junction, or has recently completed a turn (as evidenced by
			// not having a target id), check that the vehicle is oriented correctly, and at an
			// appropriate offset from the kerb.  If not, adjust the direction in an attempt to correct
			// NOTE - this may require multiple steps, but we are not going to set a waypoint, we will just
			// evaluate on each step.
			if (getTargetID() == -1  && isWaiting() == false) {
				
				// Need to check to make sure that we aren't about to enter a junction
				// because if we are, snapToLane can do some strange things - like try to orient the
				// vehicle to align with the perpendicular road.
				// Work out where a vehicle at this location and with this bearing would
				// be on a next step at max speed
				moveV = Utility.yMovement(this.getDirection(), sim.getCarMaxSpeed());
				moveH = Utility.xMovement(this.getDirection(), sim.getCarMaxSpeed());
				sumForces.zero();
				sumForces.addIn(new Double2D(moveH, moveV));	
		        sumForces.addIn(me);
		        
		        // Only execute this code if we aren't about to move into a junction
		        if (sim.junctionAtPoint(new Double2D(sumForces), sim.junctions) == 0) 
		        {
		        	COModel.initialInfo desiredLoc = sim.snapToLane(me.x, me.y);	        	
		        	
		        	// Make sure a valid result has been returned (non-valid might suggest that vehicle is about to leave road
		        	if (desiredLoc.startBearing < 400)
		        	{
		        		// Work out where a vehicle at this location and with this bearing would
		        		// be on a next step at max speed
		        		moveV = Utility.yMovement(desiredLoc.startBearing, sim.getCarMaxSpeed());
		        		moveH = Utility.xMovement(desiredLoc.startBearing, sim.getCarMaxSpeed());
		        		sumForces.zero();
		        		sumForces.addIn(new Double2D(moveH, moveV));	
		        		sumForces.addIn(desiredLoc.startLoc);

		        		// Set the direction of this Car to point to this location
		        		setDirection(me, new Double2D(sumForces));
		        	}
		        }
			}
			
			// Accelerate the Car if it is not in a junction or approach
			if (inJunctionOrApproach == false  && isWaiting() == false) 
			{
				goFaster(false); // Just in case we have become stuck
			} 
			
			// Sort out all the speed requirements - changes the speed field
			doSpeedCalcs();			
			
			// Call the operations to calculate how much the car moves in the x
			// and y directions at the new speed.  
			moveV = Utility.yMovement(getDirection(), getSpeed());
			moveH = Utility.xMovement(getDirection(), getSpeed());
			
			sumForces.zero(); // Before we use it again, make sure it has been reset
			sumForces.addIn(new Double2D(moveH, moveV));	
	        sumForces.addIn(me);
			sim.environment.setObjectLocation(this, new Double2D(sumForces));
			this.setLocation( new Double2D(sumForces));
						
			location = new Double2D(sumForces);
		}
		
		if(sim != null)
		{
			sim.dealWithTermination();
		}
    }
	
	/**
	 * This method produces a string representation of the DumbCar which is simply a string containing
	 * the ID number of the vehicle.
	 * 
	 * @return String (Unique ID of DumbCar as a string.)
	 */
	@Override
	public String toString()
	{
		return "" + getID() + "";
	}
	
	/**
	 * Method which returns a rectangle representing the moving car obstacle and centred at location
	 * @return Shape (the actual vehicle footprint, as a rectangle rotated according to the vehicle heading)
	 */
	public Shape getShape()
	{
		// The 'location' is the front and centre of this shape which we will assume is of the size given in
		// the Constants file for an obstacle
		double widthOffset = Constants.OBSTACLE_WIDTH/2;
		
		// Return a shape and align it with the oriented position of the vehicle
		Rectangle2D.Double carRectangle = new Rectangle2D.Double();
		
		// NOTE: the location is used as the front of the vehicle
		// Assume the basic shape is as it would appear when pointed along the x-axis, so this 
		// means some swapping around of width/length.  NOTE that the anchor point specified is the centre-
		// front of the vehicle, rather than where this point might actually be in a 4-wheeled vehicle. A
		// change to this might reduce the 'flipping'/jumping behaviour seen when the DumbCars manoeuvre
		// around in junctions. TODO - consider adjusting the anchor point for more realistic turning.
		carRectangle = new Rectangle2D.Double(location.x - Constants.OBSTACLE_LENGTH, location.y - widthOffset, Constants.OBSTACLE_LENGTH, Constants.OBSTACLE_WIDTH);		
		AffineTransform rotateTransform = AffineTransform.getRotateInstance(((Car) this).orientation2D(), location.x, location.y);		
				
		Shape carShape = rotateTransform.createTransformedShape(carRectangle);		
		return carShape;
	}	
	
	/**
	 * Method which returns true or false if a provided coordinate is inside the shape
	 * boundary.
	 * @param coord (Double2D - location we want to test for overlap with the shape)
	 * @return boolean (returns true if the supplied location is inside the shape)
	 */
	public boolean inShape(Double2D coord)
	{
		Shape carShape = getShape();		
		return carShape.contains(coord.x, coord.y); // Uses Shape.contains - it's not entirely clear whether points on the boundary are included
	}
	
	/**
	 * This method searches through the supplied Bag of vehicles and tests to see whether they 
	 * are very close (within 20 + OBSTACLE_LENGTH) to the DumbCar.  If so, it calculates the 
	 * bearing 	`
	 * @param cars (Bag - )
	 * @return double ()
	 */
	private double findImminentCrash(Bag cars)
	{
		double retVal = Constants.WorldXVal * 2; // Default return value, suggests nothing found in range
		Car currentCar;
		double tempDist;
		
		// Loop through all the other cars, and check the separation
		for (int c=0; c<cars.size(); c++)
		{
			currentCar = (Car)cars.get(c);
			tempDist = location.distance(currentCar.getLocation());
			if (tempDist < (20 + Constants.OBSTACLE_LENGTH) && (this.getID() != currentCar.getID()))
			{
				// There is a vehicle in range so now we should check to see whether it is ahead of this one
				// or behind it.
				double bearing = getDirection();
				
				// Check that the vehicles are likely to be in the same lane (in-lane displacement <2.5)
				// and then work out whether the detected vehicle is in front/behind the DumbCar. The aim
				// is to find the closest vehicle to the DumbCar, in the same lane, and then report the 
				// distance to it, and whether it is in front, or behind.
				// NOTE: tempDist is always positive, but the stored retVal can be positive or negative
				if (bearing >= 315 && bearing < 45 && Math.abs(currentCar.getLocation().x - location.x) < 2.5) { // SB
					if (currentCar.getLocation().y >= location.y)
					{
						if (tempDist < Math.abs(retVal)) {retVal = tempDist;} // Object is in front
					} else if (currentCar.getLocation().y < location.y)
					{
						if (tempDist < Math.abs(retVal)) {retVal = -tempDist;} // Object is behind
					}
				} else if (bearing >= 45 && bearing < 135 && Math.abs(currentCar.getLocation().y - location.y) < 2.5) { // EB
					if (currentCar.getLocation().x >= location.x)
					{
						if (tempDist < Math.abs(retVal)) {retVal = tempDist;} // Object is in front
					} else if (currentCar.getLocation().x < location.x)
					{
						if (tempDist < Math.abs(retVal)) {retVal = -tempDist;} // Object is behind
					}
				} else if (bearing >= 135 && bearing < 225 && Math.abs(currentCar.getLocation().x - location.x) < 2.5) { // NB
					if (currentCar.getLocation().y <= location.y)
					{
						if (tempDist < Math.abs(retVal)) {retVal = tempDist;} // Object is in front
					} else if (currentCar.getLocation().y > location.y)
					{
						if (tempDist < Math.abs(retVal)) {retVal = -tempDist;} // Object is behind
					}
				} else if (bearing >= 225 && bearing < 315 && Math.abs(currentCar.getLocation().y - location.y) < 2.5) { // WB
					if (currentCar.getLocation().x <= location.x)
					{
						if (tempDist < Math.abs(retVal)) {retVal = tempDist;} // Object is in front
					} else if (currentCar.getLocation().x > location.x)
					{
						if (tempDist < Math.abs(retVal)) {retVal = -tempDist;} // Object is behind
					}
				}
			}
		}
		
		return retVal;
	}	
}
