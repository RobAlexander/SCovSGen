package modeling;

import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.Rectangle2D;

import sim.engine.SimState;
import sim.field.continuous.Continuous2D;
import sim.util.Bag;
import sim.util.Double2D;
import sim.util.MutableDouble2D;

/*
 * HH 25.8.14 - Moving obstacles which appear at entrances to the network, choose junction exits at random,
 * and follow a trajectory that involves them following a fixed offset from the kerb - based on 'knowing'
 * their location on the road.  TO DO - Rob has suggested varying the road position based on a double sine wave
 * to simulate drunk-driving as a future test scenario.  Other similar movement variation could be introduced.
 */

public class DumbCar extends Car {
	
	public DumbCar(int idNo, CarPerformance performance,
			double initialBearing) {
		super(idNo, -1, performance, initialBearing, DUMBCAR);
		// NOTE: The targetId of a DumbCar will be -1 as it does not have a target
	}

	@Override
	public void step(SimState state)
	{
		sim = (COModel) state;
		
		if(this.isActive == true)
		{
			resetSpeedParams(); // HH 22.9.14 - Reset these for this step
			
			Continuous2D environment = sim.environment;
			
			Double2D me = environment.getObjectLocation(this);
			
			// Check we are still on the road, and if we have left it, remove self from sim
			if (onRoad(sim.roads, getShape()) == false) {
				
				// HH - 27.8.14 Log speed of car entering junction
				sim.infoLog.addLog("Step: " + sim.schedule.getSteps() + ", Car: " + this.getID() + 
								   " has left the road at: " + me.toString() + ", bearing: " + this.getDirection() + ".");
				
				// HH 29.9.14 - Need to make sure we aren't blocking junctions unnecessarily 
				if (getJctID() > 0)
				{
					// We've left the junction, so reset the junction occupancy so someone else can enter
					// HH 16.10.14 Update to new method which doesn't confuse ID and Idx
					sim.unOccupyJunction(getJctID(), sim.junctions);
					//((Junction) sim.junctions.get(getJctID())).unOccupy();
					setJctID(0);
				}
				
				this.isActive = false;
				environment.remove(this); // HH 18.9.14 - Remove from environment
				sim.dealWithTermination();
				
				// HH 9.9.14 Add a new car to the simulation at a random entry point to the network
				sim.addNewDumbCar();
				
				return;
			}
			
			MutableDouble2D sumForces = new MutableDouble2D(); //used to record the changes to be made to the location of the car

			// HH 16/7/14 - Store the previous location now, before we do anything with it
			storePrevLoc(me);
			// HH - end
			
			double moveV; //vertical component of the cars movement
			double moveH; //horizontal component of the cars movement
			
			//get location of target
			Bag everything = environment.getAllObjects(); //this will get all of the objects in the world, then start filtering :)
			
			this.setStats(new CarPerformance(sim.getCarMaxSpeed(),sim.getCarMaxAcceleration(), sim.getCarMaxDecceleration(), sim.getCarMaxTurning()));
			
			Entity e; // Temp variable for iterating
			Entity eTarget = new Entity(-1, TOTHER); //this id for the target is illegal, to get ids one should use COModel.getNewID()
			
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
			
			dealWithTerrain();
			
			// check whether we are approaching a junction (slow down)
			// Check to see whether we are already executing a turning manoeuvre, if so, 
			// don't need to check the junctions as it's immaterial until we have finished the turn.
			boolean inJunctionOrApproach = false;
			
			// TODO - Remove: for debug only
			if (this.ID == 259)
			{
				inJunctionOrApproach = false;
			}
			
			if (eTarget.getType() != TUTURNWP)
			{
				for(int i = 0; i < sim.junctions.size(); i++) 
				{				
					// ARE WE INSIDE A JUNCTION
					if (((Junction) sim.junctions.get(i)).inShape(me)) {
								
						// HH - 27.8.14 Log speed of car entering junction
						sim.infoLog.addLog("Step: " + sim.schedule.getSteps() + ", Car: " + this.getID() + " entering junction " + ((Junction)sim.junctions.get(i)).getID() + 
								           ", at speed: " + this.getSpeed() + ", bearing: " + this.getDirection() + ".");
						
						// Vehicle currently within the junction, ensure that we are checking whether we need a new waypoint to redirect
						// towards the destination
						Double2D junctionWP = ((Junction) sim.junctions.get(i)).getRandomExit(this, i, sim);

						// HH 3.9.14 Implementing 4-way Stop						
						// TO DO : Add a check on the return value, if we don't get a valid WP back, we haven't
						// succeeded in entering the junction.  We need to slow down to zero (perhaps log an emergency
						// stop if that exceeds the maximum deceleration of the vehicle), and we shouldn't set 
						// the inJunctionFlag.  ALSO, make sure that we can't execute any movements by following the 
						// methods which follow - may need to set a new mode for TWAIT.
						// HH 16.10.14 - getRandomExit returns (-1, ID) as an error code sometimes, not always (-1,-1)
						// so adjusted the code below so that we stop the vehicle
						// if (junctionWP.x == -1 && junctionWP.y == -1)
						if (junctionWP.x == -1) 
						{
							// Something has gone wrong, an exit has not been chosen - maybe the junction is
							// already occupied
							// TO DO - do we need to set a flag, or stop the vehicle
							double overshoot = emergencyStop();
							
							if (overshoot > 0) {
								sim.infoLog.addLog("Step: " + sim.schedule.getSteps() + ", Car: " + this.getID() + " would not have stopped in time, excess speed = " + 
										overshoot + " in junction " + ((Junction)sim.junctions.get(i)).getID() + 
							           ", at speed: " + this.getSpeed() + ", bearing: " + this.getDirection() + ".");
							}
							
							startWaiting();
							
						} else {
							eTarget = createWaypoint(junctionWP, sim, me, TUTURNWP); //set eTarget to be new WP
							
							goSlow();
						}
						
						inJunctionOrApproach = true;  // Need to set this to true regardless to prevent acceleration
											
						// ARE WE INSIDE A JUNCTION APPROACH
					} else if (((Junction) sim.junctions.get(i)).inApproach(me)) {
						// Vehicle currently within the junction approach, slow down and maintain current direction
						inJunctionOrApproach = true;
						goReallySlow(); // HH 9.9.14 Decreased speed further to try and prevent overshoot
					}
				}
			} else {
				
				// HH 3.9.14 Move this from the code below to prevent it being called twice
				// Try to ensure turn is as tight as possible
				goSlow();
			}
									
			if (eTarget.getType() == TUTURNWP && isWaiting() == false) 
			{				
				// HH 3.9.14 - Moved above, and redundant check removed
				//// Try to ensure turn is as tight as possible
				//if (eTarget.getType() == TUTURNWP) {
				//	goSlow(sim);
				//}
				
				// HH 29.9.14 Moved this below to try and solve the strange junction behaviour
				//setDirection(me, eTarget.getLocation());
				
				// HH 29.9.14 - Added a restriction to prevent clearing the junction WP until have left junction
				//if (me.distance(eTarget.getLocation()) < 1.5 && sim.junctionAtPoint(me, sim.junctions) == false)  
				// HH 30.9.14 - If we have left the junction, assume we can clear the WP - regardless of the
				// distance separation.
				// HH 16.10.14 If two junctions are close together/overlapping, then the vehicle
				// can leave the first junction, and appear in the second one before the WP has been cleared so 
				// we need to check for a change in the junctionId (or jctID == -1), in addition to leaving the junction.
				// NOTE : An issue may still be present when there is an actual overlap of junctions because the 
				// code above for detecting junctions will only execute for the first junction with which an overlap is
				// detected (which could be the one that the vehicle is supposed to have left) - although as the WP is placed
				// slightly outside of the original junction, the vehicle should not actually be inside the original junction
				// when the WP is cleared and the code above executes to find another junction entry.
				if (sim.junctionAtPoint(me, sim.junctions) == 0 || sim.junctionAtPoint(me, sim.junctions) != this.getJctID())
				{
					// HH - 27.8.14 Log speed of car leaving junction
					sim.infoLog.addLog("Step: " + sim.schedule.getSteps() + ", Car: " + this.getID() + " leaving junction" + 
							           ", at speed: " + this.getSpeed() + ", bearing: " + this.getDirection() + ".");
										
					this.setTargetID(-1); // back to default as have 'reached' target
					environment.remove(eTarget);
					
					// HH 4.9.14 - Junction Priority 
					if (getJctID() > 0)
					{
						// We've left the junction, so reset the junction occupancy so someone else can enter
						// HH 16.10.14 Updated to use method which doesn't confuse ID and idx
						sim.unOccupyJunction(getJctID(), sim.junctions);
						//((Junction) sim.junctions.get(getJctID())).unOccupy();
						setJctID(0);
					}
					
//					// HH 30.9.14 - Make sure that we are pointing in the right direction (actually gets run twice!)
//					// Work out which way we should be pointing?
//					initialInfo newLocInfo= sim.snapToLane(location.x, location.y);
//					
//					// Make sure a valid result has been returned
//					if (newLocInfo.startBearing < 400) {
//						
//						// Work out where a vehicle at this location and with this bearing would
//						// be on a next step at max speed
//						moveV = yMovement(newLocInfo.startBearing, sim.getCarMaxSpeed());
//						moveH = xMovement(newLocInfo.startBearing, sim.getCarMaxSpeed());
//						sumForces.zero();
//						sumForces.addIn(new Double2D(moveH, moveV));	
//				        sumForces.addIn(newLocInfo.startLoc);
//						
//				        // Set the direction of this Car to point to this location
//				        setDirection(me, new Double2D(sumForces));
//					}
					
				} else if (sim.junctionAtPoint(me, sim.junctions) != 0) // Only set the direction when we are in the junction
				{
					setDirection(me, eTarget.getLocation());
					
					inJunctionOrApproach = true; // HH 30.9.14 This isn't being set when the vehicle is already in a junction
				}
			}
			
			// HH 29.9.14 - Need a different method for DumbCar as it's taking far too long to conduct the lidar-based search
			// instead will use something that will be much 'cheaper' and is not a 'cheat' as these vehicles are meant to
			// simulate human-drivers anyway, so should have access to better information.
			double distToObs = findImminentCrash(sim.cars);
			distToObs = Math.min(distToObs, findImminentCrash(sim.ugvs));
			
			double prevDist = getStoppingDistance();
			this.setStoppingDistance(distToObs); // Update for next step
			
			// HH 29.9.14 - Adjust params to take into account previous stopping distances
			if ((distToObs < 20 && distToObs >= 0) && distToObs < prevDist) // HH 24.9.14 - Added >0 to try and separate vehicles which are on top of one another
			{
				goSlowStop(); 
			} else if (distToObs <= 0 && distToObs > -10 && isWaiting() == false) { // HH 30.9.14 Added isWaiting check
				goFaster(true); // HH 24.9.14 - Force a speed up in case 2 vehicles are too close
			} else {
				goFaster(false); // HH 23.9.14 - This is just a vote to speed up, in case the vehicle has got stuck
			}			
			
//			// HH 23.9.14 Regardless of what is going on - if there is another moving vehicle on the road ahead, and within 10m of us, then we need to slow down.
//			// TO DO: 10m is sort of arbitrary at the moment (COModel sim, boolean sameLane, double inAngle, double inRange, double inSensitivity)
////			double distToObs = location.distance(checkAllMovingObstacles(sim, sim.cars, true, DCMovObsViewingAngle, DCMovObsViewingRange, SensitivityForRoadTracking));
////			distToObs = Math.min(distToObs, location.distance(checkAllMovingObstacles(sim, sim.ugvs, true, DCMovObsViewingAngle, DCMovObsViewingRange, SensitivityForRoadTracking))); // HH 24.9.14 - Check UGV
//			double distToObs = calcDistance(checkAllMovingObstacles(sim, sim.cars, true, DCMovObsViewingAngle, DCMovObsViewingRange, SensitivityForRoadTracking));
//			distToObs = Math.min(distToObs, calcDistance(checkAllMovingObstacles(sim, sim.ugvs, true, DCMovObsViewingAngle, DCMovObsViewingRange, SensitivityForRoadTracking))); // HH 24.9.14 - Check UGV
//			
//			// HH 29.9.14 - Get the stopping distance recorded on the previous run
//			double prevDist = getStoppingDistance();
//			this.setStoppingDistance(distToObs); // Update for next step
//			
//			// HH 29.9.14 - Adjust params to take into account previous stopping distances
//			if (distToObs < 20 && distToObs < prevDist) // HH 24.9.14 - Added >0 to try and separate vehicles which are on top of one another
//			{
//				goSlowStop(); 
////			} else if (distToObs < 0 && distToObs > -10) {
////				goFaster(true); // HH 24.9.14 - Force a speed up in case 2 vehicles are too close
//			} else {
//				goFaster(false); // HH 23.9.14 - This is just a vote to speed up, in case the vehicle has got stuck
//			}			
			
//			if (distToObs < 10 && distToObs > 0) // HH 24.9.14 - Added >0 to try and separate vehicles which are on top of one another
//			{
//				goSlowStop(); 
//			} else if (distToObs < 0 && distToObs > -10) {
//				goFaster(true); // HH 24.9.14 - Force a speed up in case 2 vehicles are too close
//			} else {
//				goFaster(false); // HH 23.9.14 - This is just a vote to speed up, in case the vehicle has got stuck
//			}
			
			// If the vehicle is not in a junction, or has recently completed a turn (as evidenced by
			// not having a target id), check that the vehicle is oriented correctly, and at an
			// appropriate offset from the kerb.  If not, adjust the direction in an attempt to correct
			// NOTE - this may require multiple steps, but we are not going to set a waypoint, we will just
			// evaluate on each step.
			if (getTargetID() == -1  && isWaiting() == false) {
				
				// HH 30.9.14 - Need to check to make sure that we aren't about to enter a junction
				// because if we are, snapToLane can do some strange things - like try to orient the
				// vehicle to align with the perpendicular road.
				// Work out where a vehicle at this location and with this bearing would
				// be on a next step at max speed
				moveV = yMovement(this.getDirection(), sim.getCarMaxSpeed());
				moveH = xMovement(this.getDirection(), sim.getCarMaxSpeed());
				sumForces.zero();
				sumForces.addIn(new Double2D(moveH, moveV));	
		        sumForces.addIn(me);
		        
		        // Only execute this code if we aren't about to move into a junction
		        if (sim.junctionAtPoint(new Double2D(sumForces), sim.junctions) == 0) 
		        {
				
		        	COModel.initialInfo desiredLoc = sim.snapToLane(me.x, me.y);	        	
		        	
//		        	// HH - TO DO : Remove debugging
//		        	double tempAngle = correctAngle(360 + desiredLoc.startBearing - getDirection());
//		        	if (tempAngle > 10 && tempAngle < 350) 
//		        	{
//		        		moveV = 1;
//		        	}
		        			        	
		        	// Make sure a valid result has been returned (non-valid might suggest that vehicle is about to leave road
		        	if (desiredLoc.startBearing < 400) {

		        		// Work out where a vehicle at this location and with this bearing would
		        		// be on a next step at max speed
		        		moveV = yMovement(desiredLoc.startBearing, sim.getCarMaxSpeed());
		        		moveH = xMovement(desiredLoc.startBearing, sim.getCarMaxSpeed());
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
				goFaster(false); // HH 22.9.14 - Replaced the below
				//changeSpeed(ACCELERATE);
			} // Just in case we have become stuck
			
			// HH 22.9.14 - Sort out all the speed requirements
			doSpeedCalcs();			
			
			//call the operations to calculate how much the car moves in the x
			//and y directions.  
			moveV = yMovement(getDirection(), getSpeed());
			moveH = xMovement(getDirection(), getSpeed());
			
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
	
	@Override
	public String toString()
	{
		return "" + getID() + "";
	}
	
	/**
	 * method which returns a rectangle representing the moving car obstacle and centred at location
	 */
	public Shape getShape()
	{
		// HH 8.10.14 - Changed the location to be the front of the vehicle so that we don't get 
		// silly crashes in junctions.  
		// The location is the front and centre of this shape which we will assume is of the size given in
		// the Constants file for an obstacle
		double widthOffset = Constants.OBSTACLE_WIDTH/2;
		
		// 24.9.14 Return a shape aligned with the oriented vehicles
		Rectangle2D.Double carRectangle = new Rectangle2D.Double();
		
		// HH 8.10.14 - Adjusted the code below so that the location is used as the front of the vehicle
		// HH 24.9.14 - Assume the basic shape is as it would appear when pointed along the x-axis, so this means some swapping around of width/length
		carRectangle = new Rectangle2D.Double(location.x - Constants.OBSTACLE_LENGTH, location.y - widthOffset, Constants.OBSTACLE_LENGTH, Constants.OBSTACLE_WIDTH);		
		AffineTransform rotateTransform = AffineTransform.getRotateInstance(((Car) this).orientation2D(), location.x, location.y);		
		
//		// HH 24.9.14 - Assume the basic shape is as it would appear when pointed along the x-axis, so this means some swapping around of width/length
//		double lengthOffset = Constants.OBSTACLE_LENGTH/2;
//		carRectangle = new Rectangle2D.Double(location.x - lengthOffset, location.y - widthOffset, Constants.OBSTACLE_LENGTH, Constants.OBSTACLE_WIDTH);		
//		AffineTransform rotateTransform = AffineTransform.getRotateInstance(((Car) this).orientation2D(), location.x, location.y);
		
		Shape carShape = rotateTransform.createTransformedShape(carRectangle);
			
//		// NOTE - this needs to take into account the orientation of the moving obstacle as
//		// this will affect which direction to apply the widthOffset and lengthOffset in (NB. Just 
//		// using the UGV.getDirection static method for converting bearing to compass direction)
//		if (UGV.getDirection(getDirection()) == UGV_Direction.NORTH || UGV.getDirection(getDirection()) == UGV_Direction.SOUTH) {
//			carShape = new Rectangle2D.Double(location.x-widthOffset, location.y-lengthOffset, 
//					Constants.OBSTACLE_WIDTH, Constants.OBSTACLE_LENGTH);
//		} else {
//			carShape = new Rectangle2D.Double(location.x-lengthOffset, location.y-widthOffset, 
//					Constants.OBSTACLE_LENGTH, Constants.OBSTACLE_WIDTH);
//		}
		
		return carShape;
	}	
	
	/**
	 * method which returns true or false if a provided coordinate is in the shape
	 * would have to be overwritten when implemented
	 */
	public boolean inShape(Double2D coord)
	{
		Shape carShape = getShape();		
		return carShape.contains(coord.x, coord.y);
	}
	
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
				
				// If this vehicle is pointing towards the one that has been detected, return the separation
				// NOTE: tempDist is always positive, but the stored retVal can be positive or negative
				if (bearing >= 315 && bearing < 45) {
					if (currentCar.getLocation().y >= location.y && Math.abs(currentCar.getLocation().x - location.x) < 2.5)
					{
						if (tempDist < Math.abs(retVal)) {retVal = tempDist;} // Object is in front
					} else if (currentCar.getLocation().y < location.y)
					{
						if (tempDist < Math.abs(retVal)) {retVal = -tempDist;} // Object is behind
					}
				} else if (bearing >= 45 && bearing < 135 && Math.abs(currentCar.getLocation().y - location.y) < 2.5) {
					if (currentCar.getLocation().x >= location.x)
					{
						if (tempDist < Math.abs(retVal)) {retVal = tempDist;} // Object is in front
					} else if (currentCar.getLocation().x < location.x)
					{
						if (tempDist < Math.abs(retVal)) {retVal = -tempDist;} // Object is behind
					}
				} else if (bearing >= 135 && bearing < 225 && Math.abs(currentCar.getLocation().x - location.x) < 2.5) {
					if (currentCar.getLocation().y <= location.y)
					{
						if (tempDist < Math.abs(retVal)) {retVal = tempDist;} // Object is in front
					} else if (currentCar.getLocation().y > location.y)
					{
						if (tempDist < Math.abs(retVal)) {retVal = -tempDist;} // Object is behind
					}
				} else if (bearing >= 225 && bearing < 315 && Math.abs(currentCar.getLocation().y - location.y) < 2.5) {
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
