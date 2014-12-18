package modeling;

import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.Area;
import java.awt.geom.Rectangle2D;

import modeling.COModel.jctPairInfo;
import modeling.Constants.UGV_Direction;
import modeling.Constants.genLineType;
import sim.engine.SimState;
import sim.field.continuous.Continuous2D;
import sim.util.Bag;
import sim.util.Double2D;
import sim.util.MutableDouble2D;

/* 
 * HH 7.5.14 - derived class created to provide road-driving functionality to existing Car class
 * 
 * @author HH
 * 
 */

public class UGV extends Car {

	//private final double sensitivityForRoadTracking = 0.5; // HH - Original
	private final double sensitivityForRoadTracking = 0.5;
	//private double UGVViewingRange = 10; // HH - Original
	private double UGVViewingRange = 10;
	private double UGVViewingAngle = 90; // HH - Original
	
	// HH 6.8.14 - Obstacle Detection Params
	private double UGVObsViewingRange = 25;
	private double UGVObsViewingAngle = 180; // HH 20.8.14 Had to expand range so can detect obstacle length better.
	
	// HH 8.9.14 - Moving Obstacle Detection Params
	private double UGVMovObsViewingRange = 100;
	private double UGVMovObsViewingAngle = 10; // We will assume that the sensor is placed on the offside front corner of the vehicle
											   // so this should provide sufficient search breadth to cover most of the road (>7m away)
	
	private int[][] junctionHistory; // HH 15.7.14 Store true/false for each junction index and direction to show where we have already been, 16.7.14 updated to int
	private Entity finalTarget;
	private boolean targetFound = false;
	
	private OvertakeStage overtakeStage = OvertakeStage.NOT_OVERTAKING;
	
	private int prevJct = -1;
	private Double2D prevJctLoc;
	
	// HH 18.6.14 - New constructor to allow initialisation of direction/bearing
	// HH 22.7.14 - Added COModel as argument to allow faults
	public UGV(int idNo, int idTarget, CarPerformance performance, double bearing, int noJcts, COModel sim) {
		super(idNo, idTarget, performance, bearing, TUGV);
				
		// HH 15.7.14 For junction navigation - to prevent loops which always check the same incorrect 
		// turning, when the 'nearest' junction arm is not the one that will bring the UGV to the target
		int jIdx = 0;
		
		junctionHistory = new int[noJcts][UGV_Direction.values().length];
		
		for (int i = 0; i < noJcts; i++)
		{
			// HH 15/7/14 Loop for compass directions, using correct Enum index, and init to false
			for (UGV_Direction j : UGV_Direction.values()) {
				
				jIdx = j.ordinal();
				junctionHistory[i][jIdx] = 0; // HH 16.7.14 - a zero indicates that this approach has not been visited
	
				
				// HH 3.12.14 This Fault was never begin called, so perhaps there is an issue with this code loop???
				// TODO - CHECK!!!
//				// FAULT #0 - HH 22/7/14 - Initialise all values to 1
//				if (sim.getFault(0) == true) {
//					junctionHistory[i][jIdx] = 1;
//					sim.setFault(0);
//				}
			}
		}
		
		finalTarget = new Entity(-1, TOTHER); // Invalid type for instantiation
		// HH end
		
		targetFound = false; // HH 22.7.14 
	}
		
	/*
	 * Based on modeling.Car.step (Robert Lee) - functionality changed to force vehicle to restrict movement to
	 * remain on road network.
	 * 
	 * (non-Javadoc)
	 * @see modeling.Car#step(sim.engine.SimState)
	 */
	@Override
	public void step(SimState state)
	{
		// Set these variables outside of the main loop as they are used to call the dealWithTermination code below
		sim = (COModel) state;
		
		if(this.isActive == true)
		{
			resetSpeedParams(); // HH 22.9.14 - Reset these for this step
			
			Continuous2D environment = sim.environment;
			
			Double2D me = environment.getObjectLocation(this);
						
			MutableDouble2D sumForces = new MutableDouble2D(); //used to record the changes to be made to the location of the car

			// HH 16/7/14 - Store the previous location now, before we do anything with it
			storePrevLoc(me);
			// HH - end
			
			//Double2D targetCoor = me; // HH - not sure why we init this to point to the car location, maybe just so it is initialised?
			double moveV; //vertical component of the cars movement
			double moveH; //horizontal component of the cars movement
			
			//get location of target
			Bag everything = environment.getAllObjects(); //this will get all of the objects in the world, then start filtering :)
			//Bag obstacles = new Bag();
			
			this.setStats(sim.getCarMaxSpeed(), sim.getCarMaxAcceleration(), sim.getCarMaxDecceleration(), sim.getCarMaxTurning());
			//System.out.println("Car.step is called, car"+this.getID()+"'s coordinate: ("+ me.x+" , "+me.y+")");
	        
			Entity e;
			
			Entity eTarget = new Entity(-1, TOTHER); //this id for the target is illegal, to get ids one should use COModel.getNewID()
							
			// Find the target from the bag of all entities (is probably item 0)
			for(int i = 0; i < everything.size(); i++)
			{
				e = (Entity) everything.get(i);			
				if (e.getID() == this.getTargetID())
				{
					eTarget =  e;
					//targetCoor = eTarget.getLocation();
//				} else if (e.getType() == TCIROBSTACLE) {
//					obstacles.add(e);
				}
				
				// If the final target has not been set yet, extract the info from here too
				if (finalTarget.getType() == TOTHER) 
				{
					if (e.getType() == TTARGET)
					{
						finalTarget = e;
					}
				}
			}
			
			//dealWithTerrain();

			// HH 16.12.14 Check to see if we have recently started to leave a junction, and are just
			// waiting for the 'back-end' to vacate the junction so we can mark it as cleared.
			if (getJctID() > 0)
			{
				// Check that we have started to leave the junction
				if (sim.junctionAtPoint(me, sim.junctions) == 0 || sim.junctionAtPoint(me, sim.junctions) != this.getJctID())
				{
					// See if we have entirely left the junction
					if (sim.junctionAtArea(new Area(this.getShape()), sim.junctions) <= 0) // HH 17.12.14 Include if now in a different jct
					{
						// We've left the junction, so reset the junction occupancy so someone else can enter
						sim.unOccupyJunction(getJctID(), sim.junctions);
						setJctID(0);
					}
				}
			}			
			
			// HH 12.12.14 Work out the stopping distance for the vehicle based on speed and maxDeceleratione
			double stoppingDistance = 0;
			double tempSpeed = getSpeed();
			
			while (tempSpeed > 0) {
				stoppingDistance = stoppingDistance + tempSpeed; 
				tempSpeed = tempSpeed - getStats().getMaxDeceleration(); // Being extra cautious, assume hasn't started to decelerate on this step
			}
			
			stoppingDistance = Math.max(stoppingDistance, Constants.MIN_STOPPINGDISTANCE); // Make sure we are at least above a minimum distance
			
// HH - 17.6.14 Removed reliance on remaining on the road and trying not to go further away from the target.  New mode of operation 
// assumes that the vehicle will detect lane markings, using its forward vision sensors (these would probably need to be a camera,
// but implementation is not relevant here).  Vehicles should continue in their current 
// direction (and lane) until they reach a junction, at which point they can determine whether another approach would be favourable
// to reach the target (taking into account whether the approach has already been explored, and whether moving in that direction is
// likely to bring them nearer to the target).  
//
/////////////////////////////////////////////// OLD CODE ///////////////////////////////////////////////////////////////////////////			
//			// HH - to retain existing methodology, will try and use the angle to the target as the favoured direction choice
//			// but instead of looking for obstacles in our path, determine whether the path will remain on the road, and if so, 
//			// will the new location be no further from the target than the current one (TO DO: undoubtably this will cause
//			// problems later as the vehicle may be unable to follow the only valid paths to a destination: assuming the
//			// vehicle must move further away in order to get nearer).
//			
//			// Check whether the current course is going to take the vehicle off the road within the current viewing range.
//			// If so, turn right or left to try and avoid this by adding a new waypoint.
//			if (!checkRoads(sim.roads, this.getDirection())) 
//			{
//				int wpID = sim.getNewID();
//				alterCourse(sim.roads, me, wpID, sim.environment, ((Entity) everything.get(0)).getLocation());						
//			}
/////////////////////////////////////////////// OLD CODE ENDS ///////////////////////////////////////////////////////////////////////
			
			// HH 12.12.14 Regardless of what we are doing, if we are travelling at a slow enough speed, and
			// moving looks likely to cause a collision, then we should stop immediately.  As the speed threshold
			// is set at the maxDeceleration, we aren't breaking any motion laws to just stop entirely.
			if (getSpeed() <= getStats().getCurrentMaxDecel())
			{
				// Check the area around the UGV to ensure that there is no overlap.  Method will be to 
				// extend the UGV shape slightly (+1m) in all directions and check for overlapping with any
				// Moving obstacles.  If any overlap is detected, the vehicle should stop immediately.  To try 
				// and simulate sensing appropriately, we will check outwards in a circle from the centre of the
				// UGV to a range of 3m
				if (checkAllMovingObsCircle(sim, sim.cars, sensitivityForRoadTracking) == true)
				{
					return; // No need to continue with Step, we're just going to stay still!
				}
			}
			
			// Check whether the current course is going to take the vehicle further away from the detected lane marking at the limit 
			// of the current viewing range, or whether we need to change direction at a junction to get nearer to the destination, or
			// explore a new area (that may not be directly nearer to the destination). If so, turn right or left to adjust heading 
			// by adding a new waypoint.
			//boolean inJunction = false;
			boolean inJunctionApproach = false;
			
			// HH 15.7.14 - Check to see whether we are already executing a turning manoeuvre, if so, 
			// don't need to check the junctions as it's immaterial until we have finished the turn.
			if (eTarget.getType() != TUTURNWP)
			{
				for(int i = 0; i < sim.junctions.size(); i++) 
				{
					// New Fault #7 (FAULT #12) - HH 28/7/14 - Exit the loop half-way through
					if (sim.getFault(7) == true) {
						if (i == (sim.junctions.size()/2)) {
							break;
						}
						sim.setFault(7); // HH 13.11.14 Added
					}
										
					// ARE WE INSIDE A JUNCTION
					if (((Junction) sim.junctions.get(i)).inShape(me)) {
								
						// Vehicle currently within the junction, ensure that we are checking whether we need a new waypoint to redirect
						// towards the destination
						Double2D junctionWP = ((Junction) sim.junctions.get(i)).getJunctionExit(finalTarget.getLocation(), this, i, sim);
						
						// HH 3.9.14 Implementing 4-way Stop						
						// Check on the return value, if we don't get a valid WP back, we haven't
						// succeeded in entering the junction.  We need to slow down to zero (perhaps log an emergency
						// stop if that exceeds the maximum deceleration of the vehicle), and we shouldn't set 
						// the inJunctionFlag.  ALSO, make sure that we can't execute any movements by following the 
						// methods which follow - may need to set a new mode for TWAIT.
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
							
							startWaiting(); // Go into waiting mode
							
						} else {
							// HH 14.7.14 - Make sure the WP is on the road surface
							if (ptOnRoad(sim.roads, junctionWP) == true) 
							{
								// HH 15.7.14 Created generic method to create and return the new waypoint
								eTarget = createWaypoint(junctionWP, sim, me, TUTURNWP, false, null); //set eTarget to be new WP - HH 21.11.14 added new params
								
								// HH 6.11.14 Store the combination of prevJct and thisJct so that we can use it to calculate IN_RUN
								// Situation Metrics
								int currentJct = ((Junction)sim.junctions.get(i)).getID();
								Double2D currentLoc = ((Junction)sim.junctions.get(i)).getLocation();
								if (prevJct > -1 && prevJct != currentJct)
								{
									jctPairInfo jctLocs = sim.new jctPairInfo(prevJctLoc, currentLoc);
									
									sim.HsetJctArray(prevJct, currentJct, jctLocs);
									prevJct = currentJct; // Update for next Junction
									prevJctLoc = currentLoc;
								} else {
									prevJct = currentJct; // This must be the first time we have encountered a junction
									prevJctLoc = currentLoc;
								}
							}
							
							goSlow();
						}

						// HH 24.7.14 - To try and work out why vehicles are leaving the road during turns, create
						// a log of the speed, location, and bearing of the UGV when it enters a junction
						sim.infoLog.addLog("Step: " + sim.schedule.getSteps() + ", UGV = (" + me.x + "," + me.y + "), bearing = " +
								this.getDirection() + " : " + Utility.getDirection(this.getDirection()) + ", speed = " + this.getSpeed() + ".");
						
						// ARE WE INSIDE A JUNCTION APPROACH
					} else if (((Junction) sim.junctions.get(i)).inApproach(me)) {
						// Vehicle currently within the junction approach, slow down and maintain current direction
						goSlow(); 
						inJunctionApproach = true;
					}
				}
			}
			
			// HH 10.7.4 - The next thing we need to check is that we are not coming to the edge of the map, or the edge of
			// the road.  In either of these cases, we want to execute a U-turn.  This can be achieved in the same way as a U-turn
			// within a junction i.e. by inserting a waypoint at the same location in the adjacent lane (opposite direction) after
			// slowing the vehicle down almost to a stop (to allow it to make the tight turn).
			
			// Firstly make sure we are not about to reach the target, as that can confuse everything due to strange bearings etc.
			// HH adjusted distance to target from 10 to 3 as causing issues at junctions)
			// HH 8.10.14 - Fixed a bug here by replacing everything.get(0) with finalTarget
			if (me.distance(finalTarget.getLocation()) >= 3 && eTarget.getType() != TUTURNWP && isWaiting() == false) // Check we haven't already set a WP to follow
			{
				// HH 15/7/14 - Are we about to leave the road, or hit the wall
				if (((nearlyOffRoad(sim.roads, me, this.getDirection()) == true) || (checkWallClose(this.getDirection()) == true)))
				{
					// The vehicle is going to leave the road or hit the wall if it remains on this bearing so slow down
					// to prepare for manoeuvre
					goSlow();

					// HH 15/7/14 - We only want to execute a Uturn if we are about to leave the road, or hit the wall, not just
					// because we happen to have just collected another Uturn WP.
					if (checkWall() == true || nearlyOffRoad(sim.roads, me, this.getDirection()) == true) 
					{
						// We've run out of road or map
						Double2D uTurnWP = getUTurn(me, this.getDirection());

						// HH 15.7.14 Created generic method to create and return the new waypoint
						eTarget = createWaypoint(uTurnWP, sim, me, TUTURNWP, false, null); //set eTarget to be new WP - HH 21.11.14 added new params

						//inJunction = true; // same behaviour required as for junction, i.e. skip below code
					}
				}
			} 
			
			// HH 7.8.14 - before we set a straight ahead, we need to check for any static obstacles on the path 
			// in front of us.  Only do this if we are not in a junction or a junction approach (as the map 
			// generator has been configured to prevent obstacles being added in these locations - similar to 
			// rules of the road and not parking near or in junctions)
			if (eTarget.getType() != TUTURNWP && isWaiting() == false) { 
				
				// HH 21.11.14 Need these outside of the if stmt below
				Double2D maxDistanceToObsCoord;
				Double2D minDistanceToObsCoord;
				
				// HH 21.11.14 Add a seedable fault to replicate a bug found in the code
				// HH 3.12.14 New Fault #18 (Fault #31)
				if (sim.getFault(18) == true) {
					// Look for an obstacle in front and evaluate the distance
					maxDistanceToObsCoord = this.checkAllObstacles(sim, this.getDirection(), true);
					minDistanceToObsCoord = this.checkAllObstacles(sim, this.getDirection(), false);
					sim.setFault(18);
				} else {
					// HH 21.11.14 Try this with the bearing instead as we can't see obstacles properly once
					// we have pulled out into the overtake as we are not looking in the direction of travel
					// anymore - (but our sensor could be)
					maxDistanceToObsCoord = this.checkAllObstacles(sim, Utility.getDirectionDeg(this.getDirection()), true);
					minDistanceToObsCoord = this.checkAllObstacles(sim, Utility.getDirectionDeg(this.getDirection()), false);
				}

				// HH 9.9.14 Look for a moving obstacle in the next lane
				Double2D minDistanceToMovObsCoord = this.checkAllMovingObstacles(sim, sim.cars, false, UGVMovObsViewingAngle, UGVMovObsViewingRange, sensitivityForRoadTracking);			
								
				// Check to see if we are already in a parked car manoeuvre
				if (eTarget.getType() == TPARKEDCAR) {				
				
					// Check to see if we are close to the waypoint (adjusted to account for 2.5m/step speed)
					if (me.distance(eTarget.getLocation()) <= (Math.max(1, getSpeed()/2)) || 
						overshotWaypoint(eTarget.getLocation(), getDirection()) == true) {
					
						// Work out whether this is the first waypoint or the second one?
						if (overtakeStage == OvertakeStage.OVERTAKE_START) {

							// This must be the first waypoint as we can still see the back of the 
							// parked vehicle with the sensor, so add the second waypoint and remove this one
							Double2D pCarWP = new Double2D(-1,-1);  // This will be set below
							
							// HH 18.11.14 Discovered some strange behaviour in the code below, so have replaced one of the 
							// conditions so the second if condition is only triggered in the presence of a seeded fault. 
							// Various edits have been made to the code below to support this:
							
							// HH 19.8.14 Need to check whether we are seeing the back of this vehicle, or some point
							// on a subsequent parked car.  For simplicity, just use some rough boundary conditions
							// HH 8.12.14 Added check of the return value maxDistanceToObsCoord as can be (-1,-1) and
							// is then meaningless when used in distance calcs.
							if ((me.distance(maxDistanceToObsCoord) <= (Constants.OBSTACLE_LENGTH + 1) || 
								//((me.distance(maxDistanceToObsCoord) <= (Constants.OBSTACLE_LENGTH * 3) && sim.getFault(29) == false))) {
								((me.distance(maxDistanceToObsCoord) <= UGVObsViewingRange && sim.getFault(16) == false))) && maxDistanceToObsCoord.x != -1) {
								
								// HH 21.8.14 - Changed so that use the location of the START waypoint to determine the location
								// of the new waypoint
								//pCarWP = getOvertakeWP(me, getDirection(), maxDistanceToObsCoord, OvertakeStage.OVERTAKE_PULLEDOUT);
								pCarWP = getOvertakeWP(eTarget.getLocation(), getDirection(), maxDistanceToObsCoord, OvertakeStage.OVERTAKE_PULLEDOUT);
								
								// HH 17.12.14 If it possible that we haven't seen the end of the parked platoon, we shouldn't change
								// the overtake stage
								if (Math.abs(me.distance(maxDistanceToObsCoord) - UGVObsViewingRange) < 2) {
									// Let's assume that there is more platoon outside of our viewing range, so we'll
									// leave the stage at _START so that we can execute this code again; log what we're
									// doing
									sim.infoLog.addLog("Step: " + sim.schedule.getSteps() + ", UGV = (" + me.x + "," + me.y + "), bearing = " +
											this.getDirection() + " : " + Utility.getDirection(this.getDirection()) + ", speed = " + this.getSpeed() + 
											". Remaining in overtake stage: START; obstacles may extend past range limit.");
									
								} else {
									overtakeStage = OvertakeStage.OVERTAKE_PULLEDOUT;
								
									// HH 18.8.14 - To try and work out why vehicles are leaving the road during overtakes
									sim.infoLog.addLog("Step: " + sim.schedule.getSteps() + ", UGV = (" + me.x + "," + me.y + "), bearing = " +
										this.getDirection() + " : " + Utility.getDirection(this.getDirection()) + ", speed = " + this.getSpeed() + 
										". Entering overtake stage: PULLED OUT.");
								}
							//} else if (me.distance(maxDistanceToObsCoord) <= (Constants.OBSTACLE_LENGTH * 3) && sim.getFault(29) == true) {
							// HH 3.12.14 New Fault #16 (Fault #29)
							} else if (me.distance(maxDistanceToObsCoord) <= UGVObsViewingRange && sim.getFault(16) == true && maxDistanceToObsCoord.x != -1) {
								// HH 21.8.14 - Changed so that use the location of the START waypoint to determine the location
								// of the new waypoint
								//pCarWP = getOvertakeWP(me, getDirection(), maxDistanceToObsCoord, OvertakeStage.OVERTAKE_START);
								pCarWP = getOvertakeWP(eTarget.getLocation(), getDirection(), maxDistanceToObsCoord, OvertakeStage.OVERTAKE_START);
								
								// HH 18.8.14 - To try and work out why vehicles are leaving the road during overtakes
								sim.infoLog.addLog("Step: " + sim.schedule.getSteps() + ", UGV = (" + me.x + "," + me.y + "), bearing = " +
										this.getDirection() + " : " + Utility.getDirection(this.getDirection()) + ", speed = " + this.getSpeed() + 
										". Extending overtake stage: START, another obstacle detected ahead.");
								// NOTE - keep overtakeStage as OVERTAKE_START
								sim.setFault(16); // HH 18.11.14 - Update fault called
								
							} else {
								// We don't want to offset the next waypoint to be as far away as the max distance as the
								// UGV should pull back into its lane between obstacles.  NOTE: there is a danger with this 
								// strategy that the vehicle will not have time to stop safely before the second obstacle 
								// if an oncoming vehicle is detected that was far enough away when it began the manoeuvre 
								// around the first obstacle. By offsetting from the current position by the required offset
								// in both direction, the waypoint will be inserted at the right distance (as the algorithm only 
								// uses one of the dimensions of the point (depending on the direction of travel of the UGV)
								Double2D tempObstaclePt;
								
								if (Utility.getDirection(this.getDirection()) == UGV_Direction.EAST || Utility.getDirection(this.getDirection()) == UGV_Direction.SOUTH ) {
									tempObstaclePt = new Double2D(me.x + Constants.OBSTACLE_LENGTH, me.y + Constants.OBSTACLE_LENGTH);
								} else {
									tempObstaclePt = new Double2D(me.x - Constants.OBSTACLE_LENGTH, me.y - Constants.OBSTACLE_LENGTH);
								}
								
								// HH 21.8.14 - Changed so that use the location of the START waypoint to determine the location
								// of the new waypoint
								//pCarWP = getOvertakeWP(me, getDirection(), tempObstaclePt, OvertakeStage.OVERTAKE_PULLEDOUT);
								pCarWP = getOvertakeWP(eTarget.getLocation(), getDirection(), tempObstaclePt, OvertakeStage.OVERTAKE_PULLEDOUT);
								overtakeStage = OvertakeStage.OVERTAKE_PULLEDOUT;
								
								// HH 18.8.14 - To try and work out why vehicles are leaving the road during overtakes
								sim.infoLog.addLog("Step: " + sim.schedule.getSteps() + ", UGV = (" + me.x + "," + me.y + "), bearing = " +
										this.getDirection() + " : " + Utility.getDirection(this.getDirection()) + ", speed = " + this.getSpeed() + 
										". Entering overtake stage: PULLED OUT, but obstacle detected ahead.");
							}
														
							eTarget = createWaypoint(pCarWP, sim, me, TPARKEDCAR, true, eTarget); //set eTarget to be new WP - HH 21.11.14 - added new params				
							
						} else if (overtakeStage == OvertakeStage.OVERTAKE_PULLEDOUT) {
							
							// We may have finished the overtaking manoeuvre, but need to make sure we are
							// tracking back to the appropriate offset from the kerb.  
							Double2D pCarWP = getOvertakeWP(me, getDirection(), new Double2D(Constants.OBSTACLE_HEADWAY, Constants.OBSTACLE_HEADWAY), OvertakeStage.OVERTAKE_FINISH);
							eTarget = createWaypoint(pCarWP, sim, me, TPARKEDCAR, true, eTarget); //set eTarget to be new WP - HH 21.11.14 - added new params		

							// HH 18.8.14 - To try and work out why vehicles are leaving the road during overtakes
							sim.infoLog.addLog("Step: " + sim.schedule.getSteps() + ", UGV = (" + me.x + "," + me.y + "), bearing = " +
									this.getDirection() + " : " + Utility.getDirection(this.getDirection()) + ", speed = " + this.getSpeed() + 
									". Entering overtake stage: FINISH.");
							
							overtakeStage = OvertakeStage.OVERTAKE_FINISH;
							
							// HH 17.12.14 Get the UGV to slow down as it pulls back in, as it needs to re-evaluate surroundings
							goSlow();
							
						} else if (overtakeStage == OvertakeStage.OVERTAKE_FINISH) {

							// HH 18.8.14 - To try and work out why vehicles are leaving the road during overtakes
							sim.infoLog.addLog("Step: " + sim.schedule.getSteps() + ", UGV = (" + me.x + "," + me.y + "), bearing = " +
									this.getDirection() + " : " + Utility.getDirection(this.getDirection()) + ", speed = " + this.getSpeed() + 
									". Entering overtake stage: NOT OVERTAKING.");
							
							overtakeStage = OvertakeStage.NOT_OVERTAKING;
							
							// HH 8.10.14 - 'Eat' the waypoint
							setTargetID(((Waypoint) eTarget).getNextPoint());
							environment.remove(eTarget);
						}
					
					// HH 3.12.14 - New Fault #17 (Fault #30)
					} else if (sim.getFault(17) == false) {
						// HH 18.11.14 - we want to make sure that the UGV is continuing to turn towards its WP
						setDirection(me, eTarget.getLocation());
						
						sim.infoLog.addLog("Step: " + sim.schedule.getSteps() + ", UGV = (" + me.x + "," + me.y + "), bearing = " +
								this.getDirection() + " : " + Utility.getDirection(this.getDirection()) + ", speed = " + this.getSpeed() + 
								". Continuing in current overtaking stage.");
						
						// HH 17.12.14 Get the UGV to slow down as it pulls back in, as it needs to re-evaluate surroundings
						if (overtakeStage == OvertakeStage.OVERTAKE_FINISH)
						{
							goSlow();
						}
						
					} else {
						sim.setFault(17); // HH 18.11.14 Call above must not have happened, so log the fault
					}
					
				} 
				
				// HH 21.8.14 - Separated this out so can be run by a vehicle pulling back in after an overtake
				if (eTarget.getType() != TPARKEDCAR || overtakeStage == OvertakeStage.NOT_OVERTAKING) {
					if (minDistanceToObsCoord.x > 0 && me.distance(minDistanceToObsCoord) <= UGVObsViewingRange) {
						// An obstacle has been detected within the viewing range
					
						// HH 9.9.14 See if we can overtake yet? (this will depend on the presence of
						// an oncoming vehicle),  
						// we don't start the manoeuvre until we are at the right distance from the object
						double minDistanceToMovObs = UGVMovObsViewingRange * 2;
						if (minDistanceToMovObsCoord.x > 0) // Make sure that we've actually found an obstacle in range before we use the value
						{
							minDistanceToMovObs = location.distance(minDistanceToMovObsCoord);
						}
						
						// HH 16.12.14 Need to take into account that we need the oncoming vehicle to pass the 
						// UGV before the UGV starts its manoeuvre to ensure that there is
						// no collision.  Previously we have actually just been checking to ensure that the
						// UGV completes its manoeuvre before the oncoming car reaches the UGV starting
						// location - this is a bit meaningless, and could easily lead to a collision
						double distMovObsToEndOvertake = UGVMovObsViewingRange * 2;
						if (maxDistanceToObsCoord.x > 0) {
							distMovObsToEndOvertake = minDistanceToMovObs - me.distance(maxDistanceToObsCoord) - Constants.OBSTACLE_HEADWAY;
						} else {
							// We'll just have to estimate this distance based on the minimum distance
							distMovObsToEndOvertake = minDistanceToMovObs - me.distance(minDistanceToObsCoord) - Constants.OBSTACLE_LENGTH - Constants.OBSTACLE_HEADWAY;
						}
						
						if ((distMovObsToEndOvertake / Car.CAR_SPEED) > (getAvgManoeuvreTime(me.distance(minDistanceToObsCoord)))) 
						{ 
							if (me.distance(minDistanceToObsCoord) <= (Constants.OBSTACLE_HEADWAY + getSpeed())) 
							{
								// Insert a waypoint at this distance, but offset from the kerb by the 
								// obstacle width + half UGV width + obstacle safety margin
								Double2D pCarWP = getOvertakeWP(me, getDirection(), minDistanceToObsCoord, OvertakeStage.OVERTAKE_START);
								eTarget = createWaypoint(pCarWP, sim, me, TPARKEDCAR, true, eTarget); //set eTarget to be new WP - HH 21.11.14 - added new params	
								//changeSpeed(ACCELERATE); // We want to be at max speed for an overtake
								
								// HH 18.8.14 - To try and work out why vehicles are leaving the road during overtakes
								sim.infoLog.addLog("Step: " + sim.schedule.getSteps() + ", UGV = (" + me.x + "," + me.y + "), bearing = " +
										this.getDirection() + " : " + Utility.getDirection(this.getDirection()) + ", speed = " + this.getSpeed() + 
										". Entering overtake stage: START.");
								
								overtakeStage = OvertakeStage.OVERTAKE_START;
							}
						} else {
							
							// Check to see if we are at the point where we have to actually stop to be able to complete
							// the manoeuvre safely
							if (me.distance(minDistanceToObsCoord) <= (Constants.OBSTACLE_HEADWAY + getSpeed()))
							{
								double overshoot = emergencyStop();
								
								if (overshoot > 0) {
									sim.infoLog.addLog("Step: " + sim.schedule.getSteps() + ", Car: " + this.getID() + " would not have stopped in time for overtake, excess speed = " + 
											overshoot + 
								           ", at speed: " + this.getSpeed() + ", bearing: " + this.getDirection() + ".");
								}
								
							} else {
								
								// If we can't overtake then we need to slow down
								goReallySlow();
							}
							
							// Set flag
							overtakeStage = OvertakeStage.WAIT; // Don't really use this yet
						}
					}					
				}
			}
						
			// HH 15/7/14 - this should be a catch-all, and if we don't already have a turning WP set, we should try to 
			// set a 'straight ahead' one by searching for road markings.
			//if (eTarget.getType() != TUTURNWP && (eTarget.getType() != TPARKEDCAR || overtakeStage == OvertakeStage.OVERTAKE_FINISH)) { // HH 19.8.14 Change as loses the final overtakeWP
			if (eTarget.getType() != TUTURNWP && (eTarget.getType() != TPARKEDCAR || overtakeStage == OvertakeStage.NOT_OVERTAKING) &&
					isWaiting() == false) { 
			
				// HH 2.10.14 - Need to check to make sure that we aren't about to enter a junction
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
		        	if (desiredLoc.startBearing < 400) {

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

		        // HH 2.10.14 - Replaced with code above to try and improve the 
//				// Survey the sensors to detect road markings within range (and update the internal map)
//				Double2D lastRM = findAllRoadMarkings(sim.roads, this.getDirection(), me, sim);
//			
//				// HH 14.7.14 - Added check on return value from findAllRoadMarkings
//				// HH [TODO] - might want to update this 
//				if (!(lastRM.x == 0 && lastRM.y == 0))
//				{
//					// Offset to the centre of the lane to see if we need to alter course
//					Double2D roadMarkingWP = calcRMOffset(lastRM, me);
//
//					// HH 14.7.14 - Make sure the WP is on the road surface
//					if (onRoad(sim.roads, roadMarkingWP) == true) 
//					{
//						// HH 15.7.14 Created generic method to create and return the new waypoint
//						eTarget = createWaypoint(roadMarkingWP, sim, me, TWAYPOINT); //set eTarget to be new WP
//					}
//						
//				} 
			}
			
			// HH 23.9.14 Regardless of what is going on - if there is another moving vehicle on the road ahead, and within 10m of us, then we need to slow down.
			// TO DO: 10m is sort of arbitrary at the moment (COModel sim, boolean sameLane, double inAngle, double inRange, double inSensitivity)
			// TO DO: Maybe make this so that we slow down if we are going faster than the car in front - so maybe keep track of the distance to the car in front
			// but should be allowed to keep travelling at the speed of the car in front, with a separation of about 2m if we are e.g. in a junction approach, or 
			// travelling at low speeds.
			// HH 8.12.14 Moved the calc outside of the if condition so that we can test for default return value
			Double2D tempMovObsLoc = checkAllMovingObstacles(sim, sim.cars, true, UGVMovObsViewingAngle, UGVMovObsViewingRange, sensitivityForRoadTracking);
			// HH 12.12.14 - Changed 20 -> stoppingDistance to try and make vehicles slightly more aggressive
			if (location.distance(tempMovObsLoc) < stoppingDistance && tempMovObsLoc.x != -1) // HH 17.11.14 Changed to 20m so have time to slow down
			{
				goSlowStop();
			} else {
				goFaster(false); // HH 23.9.14 - This is just a vote to speed up, in case the vehicle has got stuck
			}
			
			// HH 13.11.14 We need to ensure that when a vehicle is supposed to be waiting at a junction, it must
			// not be allowed to *creep* forward 
			if (isWaiting() == true)
			{
				// HH 13.11.14 This was a fault that was found during testing so it has been inserted as a seeded fault
				// so if the fault is active, we don't slow down in waiting mode, but in most cases (when fault inactive)
				// the UGV will slow to a stop in waiting mode.
				
				// New Fault #15 (FAULT #28) - HH 17/11/14 - Don't enforce the slow down to zero condition (can cause creep through jct)
				if (sim.getFault(15) == true) {
					// We're in the fault condition, so don't slow the vehicle down.
					sim.setFault(15);
				} else {
					goSlowStop(); // Make sure that we actually slow down to 0
				}
			}
			
			// TO DO - Fix this, bit of a botch to enforce termination as waypoint locations mean that the vehicle
			// is missing the target, in order to stay on the road so we'll check to see if we are near to the
			// 'actual' target, and if so, we'll throw away the way point chain.
// HH 10.7.14 Altered this as is changing direction of UGV in vicinity of any waypoint
			//if (me.distance( ((Entity) everything.get(0)).getLocation()) < 10) {
			
			// HH 1.10.14 - A little pre-processing so we don't get a null ptr exception in the loop
			double tempDist = 0;
			if (eTarget.getID() == -1)
			{
				tempDist = 4; // Set higher than the threshold below
				
				// HH 12.12.14 Let's at least set the target to point to the final target
				eTarget = finalTarget;
				setTargetID(finalTarget.getID()); 
				
			} else
			{
				tempDist = me.distance(eTarget.getLocation()); // HH 2.10.14 Swapped with above (wrong way around)
			}
			
//			if (me.distance( (finalTarget).getLocation()) < 3 && eTarget.getID() != -1) // HH 1.10.14 Added check for eTarget
			if (me.distance( (finalTarget).getLocation()) < 3) // HH 8.12.14 Removed check for eTarget as it can sometimes get set to -1 and Target is never found
			{
				// HH - 4/9/14 We don't really want vehicles straying towards targets in the other lane as 
				// can produce some weird behaviour.  However, we'll retain this as a seedable fault
				
				// New Fault #14 (FAULT #27) - HH 4/9/14 - Forget to check if the target is in the same lane as the UGV
				// TODO HH - 2.10.14 - This is cheating; the UGV can't ask the road which lane it's in!
				// HH 4.12.14 - For now, correct this so sim.getLaneDirAtPoint(eTarget.getLocation(), sim.roads) looks at the
				// finalTarget location instead
				if (sim.getFault(14) == true || 
					(sim.getLaneDirAtPoint(me, sim.roads) == sim.getLaneDirAtPoint(finalTarget.getLocation(), sim.roads)))
				{
					eTarget = finalTarget;
					setTargetID(finalTarget.getID()); // HH 8.12.14 - If we don't set the ID too then we end up setting tempDist to 4 on next iter
					setDirection(me, eTarget.getLocation());
					
					// HH 14.7.14 - Added this test in case vehicle is added next to a waypoint
					if (getSpeed() >= 1) {
						goSlow(); // HH 22.9.14 - Replaced the below
						//changeSpeed(DECELERATE); 
					} else if (getSpeed() < 1) {
						goFaster(false); // HH 22.9.14 - Replaced the below
						//changeSpeed(ACCELERATE);
					}	
					
					if (sim.getFault(14) == true) {
						sim.setFault(14); // HH 13.11.14 Added
					}
				}
			// HH 1.10.14 - Updated to try and fix null ptr bug	
//			} else if (((me.distance(eTarget.getLocation()) > 3 && eTarget.getType() != TUTURNWP && inJunctionApproach == false) ||
//					   (eTarget.getType() == TPARKEDCAR && overtakeStage != OvertakeStage.WAIT)) && isWaiting() == false) { 
			} else if (((tempDist > 3 && eTarget.getType() != TUTURNWP && inJunctionApproach == false) ||
					   (eTarget.getType() == TPARKEDCAR && overtakeStage != OvertakeStage.WAIT)) && isWaiting() == false) { 

				
				// HH 14.7.14 Prevent accelerate during turn
				goFaster(false); // HH 22.9.14 - Replaced the below
				//changeSpeed(ACCELERATE);
				//setDirection(me, targetCoor); // HH 19.6.14 - removed this 


			} else if (isWaiting() == false) {
				
				// HH 7.10.14 - Seems to be causing some strange behaviour, so removed. Don't want to direct towards 
				// final target unless we are close, or choosing a direction at a junction.				
//				if (eTarget.getType() == TTARGET)
//				{
//					goSlow(); // HH 22.9.14 - Replaced the below
//					//changeSpeed(DECELERATE);
//					setDirection(me, eTarget.getLocation()); // HH 10.7.14 - Make sure it goes to the target
//				// HH 9.7.14 - Added this so that keeps moving towards the waypoint
//				// even after leaving the junction
//				// HH 14.7.14 - Added TUTURNWP to ensure keep turning towards waypoint on each step
//				} else 
					
				if (eTarget.getType() == TWAYPOINT || eTarget.getType() == TUTURNWP) {
					
					// HH 14.7.14 - Added this cos turn not tight enough
					if (eTarget.getType() == TUTURNWP) {
						goSlow();
					}
					
					setDirection(me, eTarget.getLocation());
				}
			} 
			
			// If we are really close to our target, whether it is a waypoint or a real target, we may want to
			// finish the run, or 'eat' the target and get the next one.
//			if (me.distance(eTarget.getLocation()) < 1) {
			if (tempDist < 1) // HH 1.10.14 - Updated to prevent null ptr error
			{
				
				if (eTarget.getID() == -1)
				{
					//flag an error as -1 is an illegal id so at this point it can only be that there isn't
					//an existing target for the car
					//System.out.println("Car"+this.getID()+"arrived at fake destination, die!");
					//System.out.println("Ending Sim after failure to read from target");
					//sim.schedule.clear();
					this.isActive = false;
					
				} else {
					if (eTarget.getType() == TTARGET)
					{
						// HH 4.12.14 - Add a check to make sure the Target is in the same lane as the 
						// vehicle before we 'eat' the target
						// double inRange, double inAngle, double inSensitivity, double inBearing
						if (checkSameLane(sim, eTarget.location, UGVViewingRange, UGVViewingAngle, sensitivityForRoadTracking, getDirection()) == true) {
							this.isActive = false;
							targetFound = true;
						}
						
					// HH 14.7.14 - Added TUTURNWP to ensure keep turning towards waypoint on each step
					} else if (eTarget.getType() == TWAYPOINT || eTarget.getType() == TUTURNWP) {
					//} else if (eTarget.getType() == TWAYPOINT ) {
						//get rid of wp and get new target ID
						//System.out.println("Car"+this.getID()+"gets a new target and removing waypoint");
						setTargetID(((Waypoint) eTarget).getNextPoint());
						environment.remove(eTarget);
						
						// HH 15.12.14 - We don't actually want to clear the junction until the whole DumbCar
						// has left the junction, not just the centre location.  This method has been moved to 
						// the top of the Step() routine
//						// H 4.9.14 We've left the junction, so reset the junction occupancy so someone else can enter
//						if (eTarget.getType() == TUTURNWP  && getJctID() > 0) {
//							// HH 16.10.14 Use new method for unOccupying junction as previous one muddles ID and idx
//							sim.unOccupyJunction(getJctID(), sim.junctions);
//							//((Junction) sim.junctions.get(getJctID())).unOccupy();
//							setJctID(0);
//						}
					}
				}			
			}		
						
			//call the operations to calculate how much the car moves in the x
			//and y directions.

			// HH 18.11.14 Don't want to contravene any laws of motion using the commented out code below so have changed the code 
			// so that we only request a slow down if we are going to overshoot
			if (getSpeed() >= tempDist && eTarget.getType() == TPARKEDCAR)
			{
				goSlow(); // Request a slow down before we do the speed calcs and move the UGV
			}
			
			// HH 22.9.14 - Sort out all the speed requirements
			doSpeedCalcs();			
			
			// HH 18.8.14 - reduce the distance to travel if we are going to overshoot the waypoint
//			if (getSpeed() >= location.distance(eTarget.getLocation()) && eTarget.getType() == TPARKEDCAR)
//			if (getSpeed() >= tempDist && eTarget.getType() == TPARKEDCAR)
//			{
//				// HH - we're going to overshoot, so restrict our movement to the distance to the waypoint (and 
//				// hope this doesn't contravene too many laws of motion TO DO - May want to revisit this...
//				moveV = yMovement(getDirection(), location.distance(eTarget.getLocation()));
//				moveH = xMovement(getDirection(), location.distance(eTarget.getLocation()));
//			} else {
				// HH - original methods for determining movement
				moveV = Utility.yMovement(getDirection(), getSpeed());
				moveH = Utility.xMovement(getDirection(), getSpeed());
//			}
			
			sumForces.zero();
			sumForces.addIn(new Double2D(moveH, moveV));	
	        sumForces.addIn(me);
			sim.environment.setObjectLocation(this, new Double2D(sumForces));
			this.setLocation( new Double2D(sumForces));
			
//			if(checkWall() == true)
//			{
//				System.out.println("Car"+this.getID()+"clashes with the wall!");
//				//sim.schedule.clear();
//				this.isActive = false;
//				
//			}
			
			location = new Double2D(sumForces);
//			if (detectCollision(obstacles))
//			{
//				System.out.println("Car"+this.getID()+"has clashed with one of the obstacles!");
//				//sim.schedule.clear();
//				this.isActive = false;
//				
//			} else {
				//proximityToDanger(obstacles, location); // HH 7.5.14 - TO DO - May want to reinstate this line
//			}
			
			
		}
		
		// Check to see if there are actually any agents left, or should we stop
		if(sim!=null)
		{
			sim.dealWithTermination();
		}
    }
	
	// HH 2.10.14 - This code is no longer used as step() calls the Car.snapToLane() which
	// will hopefully be more compatible with a non-grid road formation in the future.
	/**
	 * HH 18.6.14 - Work out the location we will want to travel to if we want to maintain
	 * our lateral position relative to the road marking observation.  This method works out
	 * which orientation the vehicle is currently in, and calculates the required position
	 * which will be used as a waypoint by alterCourseRM.
	 *  
	 * @param roads
	 * @param bearing
	 * @return true if going to remain on road, false if not
	 */
	private Double2D calcRMOffset(Double2D lastRM, Double2D me)
	{
		// Depending on the orientation of lastRM, relative to 'me', we will know
		// how to offset from the road marking location.
		
		Double2D newLoc = new Double2D();
		
		// HH 15.7.14 Updated to ensure vehicles remain central to the lane, regardless of road 
		// marking width/offset
		double offset = Road.roadWidth/4 - Constants.ROADEDGEOFFSET - Constants.ROADEDGINGWIDTH; // was previously just Road.roadWidth/4;
		
		if (lastRM.x >= me.x && lastRM.y >= me.y) {
			// lastRM is SE of current location
			newLoc = new Double2D(lastRM.x - offset, lastRM.y);
		} else if (lastRM.x >= me.x && lastRM.y < me.y) {
			// lastRM is NE of current location
			newLoc = new Double2D(lastRM.x, lastRM.y + offset);
		} else if (lastRM.x < me.x && lastRM.y >= me.y) {
			// lastRM is SW of current location
			newLoc = new Double2D(lastRM.x, lastRM.y - offset);
		} else if (lastRM.x < me.x && lastRM.y < me.y) {
			// lastRM is NW od current location
			newLoc = new Double2D(lastRM.x + offset, lastRM.y);
		} else {
			// [TODO] Error here!
		}
		
		return newLoc;
	}	
	
//	/**
//	 * HH 7.5.14 - Based on Car.checkCourse (Robert Lee)
//	 *  
//	 * @param roads
//	 * @param bearing
//	 * @param sim // HH 28.7.14 Added for fault insertion
//	 * @return true if going to remain on road, false if not
//	 */
//	private boolean checkRoads(Bag roads, double bearing, COModel sim)
//	{
//		for(int i = 0; i < roads.size(); i++)
//		{
//			// FAULT #13 - HH 28/7/14 - Exit the loop half-way through
//			if (sim.getFault(13) == true) {
//				if (i == (sim.roads.size()/2)) {
//					break;
//				}
//				sim.setFault(13); // HH 13.11.14 Added
//			}			
//			
//			if (onCourse((Road) roads.get(i), bearing, sim))
//			{
//				return true;
//			}
//		}
//		
//		return false;
//	}
	
	
//	// HH 2.10.14 - Stopped using this as it 'cheats' and narrows down the options by only considering the 
//	// lines on the road the vehicle is on ...but how does the UGV know which line is on which road, and 
//	// indeed which road it is actually on?
//	/**
//	 * HH 7.5.14 - Based on Car.checkCourse (Robert Lee)
//	 *  
//	 * @param roads
//	 * @param bearing
//	 * @param sim // HH 28.7.14 - added for fault insertion
//	 * @return coordinates of the furthermost road marking detected.
//	 */
//	private Double2D findAllRoadMarkings(Bag roads, double bearing, Double2D me, COModel sim)
//	{
//		Double2D currentXY = new Double2D(0,0);
//		Double2D furthestXY = new Double2D(0,0);
//
//		// HH 10.7.14 - Only look at the road that the UGV is on
//		for(int i = 0; i < roads.size(); i++)
//		{
//			// FAULT #14 - HH 28/7/14 - Exit the loop half-way through
//			if (sim.getFault(14) == true) {
//				if (i == (roads.size()/2)) {
//					break;
//				}
//				sim.setFault(14); // HH 13.11.14 Added
//			}
//			
//			if (((Road)roads.get(i)).inShape(me) == true ) 
//			{
//				currentXY = findRoadMarkings((Road) roads.get(i), bearing, me, sim);
//
//				if ( (onMap(currentXY) == true) && ((me.distance(currentXY) > me.distance(furthestXY)) || (furthestXY.x == 0 && furthestXY.y == 0)))
//				{
//					if (currentXY.x != 0 || currentXY.y != 0) {
//						furthestXY = currentXY;
//					}
//				}
//			}
//		}
//		
//		
//// HH 10.7.14 - updated so that only consider the road that the UGV is actually on at the time
////		for(int i = 0; i < roads.size(); i++)
////		{
////			currentXY = findRoadMarkings((Road) roads.get(i), bearing, me);
////			
////			if ((me.distance(currentXY) > me.distance(furthestXY)) || (furthestXY.x == 0 && furthestXY.y == 0))
////			{
////				if (currentXY.x != 0 || currentXY.y != 0) {
////					furthestXY = currentXY;
////				}
////			}
////		}
//		
//		return furthestXY;
//	}
	
//	/** 
//	 * HH 7.5.14 - Based on Car.onCourse (Robert Lee)
//	 * 
//	 * @param road
//	 * @param bearing
//	 * @param sim // HH 28.7.14 - added for fault insertion
//	 * @return true if going to remain on road on provided course (as far as it can see) false if not
//	 */
//	private boolean onCourse(Road road, double bearing, COModel sim)
//	{
//		//simple and dirty method which checks the coordinates between 0 and 
//		//the viewing range away from the target in certain increments and see 
//		//if they're on the road
//		MutableDouble2D testCoord = new MutableDouble2D();
//		Double2D amountAdd = new Double2D(Utility.xMovement(bearing, sensitivityForRoadTracking), Utility.yMovement(bearing, sensitivityForRoadTracking));
//		testCoord.addIn(location);
//		double startAngle = 0; // HH 28.7.14 - added to replace fixed start for iteration of i=0 (supports fault insertion)
//		double sensorSensitivity = sensitivityForRoadTracking; // HH 28.7.14 - added to replace fixed sensitivity (supports fault insertion)
//		
//		// FAULT #16 - HH 28/7/14 - Force the loop to start half-way through
//		if (sim.getFault(16) == true) {
//			startAngle = UGVViewingRange/2;
//			sim.setFault(16); // HH 13.11.14 Added
//		} 
//
//		// FAULT #20 - HH 28/7/14 - Force the sensor to operate with degraded vision sensitivity
//		if (sim.getFault(20) == true) {
//			sensorSensitivity = sensitivityForRoadTracking*2;
//			sim.setFault(20); // HH 13.11.14 Added
//		} 
//		
//		for(double i = startAngle; i < UGVViewingRange; i += sensorSensitivity)
//		{
//			// FAULT #15 - HH 28/7/14 - Exit the loop half-way through
//			if (sim.getFault(15) == true) {
//				if (i == (UGVViewingRange/2)) {
//					break;
//				}
//				sim.setFault(15); // HH 13.11.14 Added
//			}
//			
//			//keep adding the amountAdd on and seeing if the coordinate is on the road
//			if (!road.inShape(new Double2D(testCoord)))
//			{
//				return false; //the testing doesn't need to continue if it would leave the road at any point
//			}
//			
//			testCoord.addIn(amountAdd);
//			
//		}
//		
//		return true; //the car does not leave the road at any point it can see on it's current course
//	}
	
	/** 
	 * HH 10.7.14 - Based on Car.checkWall (Robert Lee)
	 * 
	 *  Check the direction in which the vehicle is travelling and reports whether the 
	 *  vehicle is getting close to the wall.
	 * 
	 * @param bearing
	 * @return true if getting too close to the wall (i.e. within vision)
	 */
	private boolean checkWallClose(double bearing)
	{
		Double2D me = this.location;
		UGV_Direction myDirection = Utility.getDirection(bearing);
		
		if(me.x <= UGVViewingRange && myDirection == UGV_Direction.WEST)
		{
			//System.out.println("clash with the left wall!");
			return true;
		}
		else if(Constants.WorldXVal - me.x <= UGVViewingRange && myDirection == UGV_Direction.EAST)
		{
			//System.out.println("clash with the right wall!");
			return true;
		}
		
		if (me.y <= UGVViewingRange && myDirection == UGV_Direction.NORTH)
		{
			//System.out.println("clash with the upper wall!");
			return true;
		}
		else if(Constants.WorldYVal- me.y <= UGVViewingRange && myDirection == UGV_Direction.SOUTH)
		{
			//System.out.println("clash with the lower wall!");
			return true;
		}
		
		return false;
	}
	
//	/** 
//	 * HH 17.6.14 - Based on Car.onCourse (Robert Lee), and some code in Car.alterCourse (Robert Lee)
//	 * 
//	 * @param road
//	 * @param bearing
//	 * @param sim // HH 28.7.14 - added to support failure insertion
//	 * @return the road markings that are detected within the range of the vehicle sensor (the farthest ones detected)
//	 */
//	private Double2D findRoadMarkings(Road road, double bearing, Double2D me, COModel sim)
//	{
//		//simple and dirty method which checks the coordinates between 0 and 
//		//the viewing range away from the target in certain increments and see 
//		//if they intersect with road markings
//		MutableDouble2D testCoord = new MutableDouble2D();
//		Double2D amountAdd = new Double2D();
//		testCoord.addIn(location);
//		Double2D lastRM = new Double2D();
//		
//		// For each angle that the sensor is able to view, turning in realistic increments
//		double resolution = 0.5;
//		double newBearing = 0.0;
//		LineType myNearside = LineType.SESIDE;
//		
//		// HH 28.7.14 - Added variables for the loop to support fault insertion
//		double startAngle = -(UGVViewingAngle/2);
//		double endAngle = (UGVViewingAngle / 2);
//		double startRange = 0;
//		double endRange = UGVViewingRange;
//		double rangeSensitivity = sensitivityForRoadTracking;
//		
//		// New Fault #8 (FAULT #17) - HH 28/7/14 - Force the angle loop to start half-way through
//		if (sim.getFault(8) == true) {
//			startAngle = 0;
//			sim.setFault(8); // HH 13.11.14 Added
//		} 
//
//		// New Fault #9 (FAULT #18) - HH 28/7/14 - Force the angle loop to end half-way through
//		if (sim.getFault(9) == true) {
//			endAngle = 0;
//			sim.setFault(9); // HH 13.11.14 Added
//		} 
//		
//		// New Fault #10 (FAULT #19) - HH 28/7/14 - Force the angle sensor to 'reduce' range resolution (double iterator step)
//		if (sim.getFault(10) == true) {
//			resolution = resolution*2;
//			sim.setFault(10); // HH 13.11.14 Added
//		} 
//		
//		for(double i = startAngle; i < endAngle; i += resolution)
//		{
//			// HH 6.8.14 - Each iteration we need to reset the testCoord so it is back at the current location
//			// of the vehicle (sensor)
//			testCoord.setTo(0,0);
//			testCoord.addIn(location);
//			newBearing = Utility.correctAngle(bearing + i); // HH 6.8.14 - Reset the bearing for this iteration
//			
//			// HH 6.8.14 - Looks like this was happening with the wrong resolution - should be the range we adjust
//			//amountAdd = new Double2D(xMovement(newBearing, resolution), yMovement(newBearing, resolution));
//			amountAdd = new Double2D(Utility.xMovement(newBearing, rangeSensitivity), Utility.yMovement(newBearing, rangeSensitivity));
//			
//			// New Fault #11 (FAULT #21) - HH 28/7/14 - Force the angle loop to start half-way through
//			if (sim.getFault(11) == true) {
//				startRange = UGVViewingRange/2;
//				sim.setFault(11); // HH 13.11.14 Added
//			} 
//
//			// New Fault #12 (FAULT #22) - HH 28/7/14 - Force the angle loop to end half-way through
//			if (sim.getFault(12) == true) {
//				endRange = UGVViewingRange/2;
//				sim.setFault(12); // HH 13.11.14 Added
//			} 
//			
//			// New Fault #13 (FAULT #23) - HH 28/7/14 - Force the sensor to 'reduce' angular resolution (double iterator step)
//			if (sim.getFault(13) == true) {
//				rangeSensitivity = sensitivityForRoadTracking*2;
//				sim.setFault(13); // HH 13.11.14 Added
//			} 
//						
//			// HH 6.8.14 - we don't use j, this just ensures we run the loop the right num
//			for(double j = startRange; j < endRange; j += rangeSensitivity)
//			{
//				// keep adding the amountAdd on and seeing if the coordinate is on the road marking
//				// depending on which direction we are facing, we will either need to look at the 
//				// neside road markings (which is actually equivalent to the NORTH/EAST side of the road), 
//				// or the offside road markings (SOUTH/WEST side of the road)
//				if (Utility.getDirection(bearing) == UGV_Direction.NORTH || Utility.getDirection(bearing) == UGV_Direction.EAST) {
//					myNearside = LineType.NWSIDE;
//				} else { // must be travelling SOUTH or WEST
//					myNearside = LineType.SESIDE;
//				}
//				
//				// HH 6.8.14 - Adding in the first increment prior to the test, rather than after as no point
//				// testing the location the vehicle is already in
//				testCoord.addIn(amountAdd);
//				
//				if ((((Road) road).getLine(myNearside)).contains(testCoord.x, testCoord.y))
//				{
//					if ((me.distance(testCoord.x, testCoord.y) > me.distance(lastRM)) || (lastRM.x == 0 && lastRM.y == 0)) {
//						lastRM = new Double2D(testCoord.x, testCoord.y);
//					}
//				}
//			}
//		}
//		
//		return lastRM; 
//	}
	
//	/** 
//	 * HH 9.9.14 - Based on Car.onCourse (Robert Lee), and some code in Car.alterCourse (Robert Lee)
//	 * New version of findRoadMarkings which is less of a 'cheat' as it moves left and right from the
//	 * current bearing until it reaches a road marking (or exceeds range).  It will only continue to look
//	 * for as long as it detects either road surface, or a line, so this should be more aligned with image
//	 * processing methods for finding the lines on the roads.  This should restrict the algorithm to finding
//	 * lines that are on the same road as the UGV without needing 'special' access to the road that the
//	 * UGV is currently on as was done in the previous method.
//	 * 
//	 * @param road // passed in from the Bag of all roads (assume we will be in a loop searching all of them)
//	 * @param findNearest // true = return the nearest point found; false = return the furthest
//	 * @param sim // supports failure insertion
//	 * @param reqLine - which line are we searching for (nearside, offside, centre)
//	 * @return the coordinate location of road markings that are detected within the range of the vehicle sensor
//	 */
//	private Double2D locateRoadMarkings(Road road, boolean findNearest, COModel sim, genLineType reqLine)
//	{
//		//simple and dirty method which checks the coordinates between 0 and 
//		//the viewing range away from the target in certain increments and see 
//		//if they intersect with road markings
//		MutableDouble2D testCoord = new MutableDouble2D();
//		Double2D amountAdd = new Double2D();
//		testCoord.addIn(location);
//		
//		Double2D RM = new Double2D(-1, -1); // Default value
//		
//		// For each angle that the sensor is able to view, turning in realistic increments
//		double resolution = 0.5;
//		double newBearing = 0.0;
//		
//		// Set defaults (appropriate for centre line and offside line)
//		double startAngle = 0;
//		double endAngle = (UGVViewingAngle / 2);
//		double startRange = 0;
//		double endRange = UGVViewingRange;
//		double rangeSensitivity = sensitivityForRoadTracking;		
//		
//		// Alter the search params for nearside search
//		if (reqLine == genLineType.NEARSIDE)
//		{
//			startAngle = -(UGVViewingAngle/2);
//			endAngle = 0;
//		}
//		
//		// New Fault #8 (FAULT #17) - HH 28/7/14 - Force the angle loop to start half-way through
//		if (sim.getFault(8) == true) {
//			startAngle = 0;
//			sim.setFault(8); // HH 13.11.14 Added
//		} 
//
//		// New Fault #9 (FAULT #18) - HH 28/7/14 - Force the angle loop to end half-way through
//		if (sim.getFault(9) == true) {
//			endAngle = 0;
//			sim.setFault(9); // HH 13.11.14 Added
//		} 
//		
//		// New Fault #10 (FAULT #19) - HH 28/7/14 - Force the angle sensor to 'reduce' range resolution (double iterator step)
//		if (sim.getFault(10) == true) {
//			resolution = resolution*2;
//			sim.setFault(10); // HH 13.11.14 Added
//		}  
//		
//		// Check the viewable range at each angle		
//		for(double i = startAngle; i < endAngle; i += resolution)
//		{
//			// HH 6.8.14 - Each iteration we need to reset the testCoord so it is back at the current location
//			// of the vehicle (sensor)
//			testCoord.setTo(0,0);
//			testCoord.addIn(location);
//			newBearing = Utility.correctAngle(getDirection() + i); // HH 6.8.14 - Reset the bearing for this iteration
//			
//			amountAdd = new Double2D(Utility.xMovement(newBearing, rangeSensitivity), Utility.yMovement(newBearing, rangeSensitivity));
//			
//			// New Fault #11 (FAULT #21) - HH 28/7/14 - Force the angle loop to start half-way through
//			if (sim.getFault(11) == true) {
//				startRange = UGVViewingRange/2;
//				sim.setFault(11); // HH 13.11.14 Added
//			} 
//
//			// New Fault #12 (FAULT #22) - HH 28/7/14 - Force the angle loop to end half-way through
//			if (sim.getFault(12) == true) {
//				endRange = UGVViewingRange/2;
//				sim.setFault(12); // HH 13.11.14 Added
//			} 
//			
//			// New Fault #13 (FAULT #23) - HH 28/7/14 - Force the sensor to 'reduce' angular resolution (double iterator step)
//			if (sim.getFault(13) == true) {
//				rangeSensitivity = sensitivityForRoadTracking*2;
//				sim.setFault(13); // HH 13.11.14 Added
//			} 
//									
//			// HH 6.8.14 - we don't use j, this just ensures we run the loop the right num
//			for(double j = startRange; j < endRange; j += rangeSensitivity)
//			{
//				// keep adding the amountAdd on and seeing if the coordinate is still on the road, or whether it 
//				// is on a road marking.  There would likely be no way to tell the difference between nearside and
//				// offside road markings, so inaccurate to use this detail from the object model.  Centre markings
//				// would be different, so can use this detail to differentiate between centre and offside.
//				
//				// HH 6.8.14 - Adding in the first increment prior to the test, rather than after as no point
//				// testing the location the vehicle is already in
//				testCoord.addIn(amountAdd);
//				
//				// Make sure we are still on the road
//				if (((Road) road).inShape(new Double2D(testCoord)) == false)
//				{
//					break; // exit the for loop and try the next bearing.
//				}
//				
//				if (reqLine == genLineType.CENTRE) // Are we looking for a centre line?
//				{
//					if ((((Road) road).getLine(LineType.CENTRE)).contains(testCoord.x, testCoord.y))
//					{
//						if (((location.distance(testCoord.x, testCoord.y) > location.distance(RM)) && findNearest == false) || RM.x == -1) {
//							RM = new Double2D(testCoord.x, testCoord.y);
//						} else if ((location.distance(testCoord.x, testCoord.y) < location.distance(RM)) && findNearest == true) {
//							RM = new Double2D(testCoord.x, testCoord.y);
//						}
//					}
//				} else { // Check both sets of edge markings as the vision algorithms are unlikely to know which are which
//						 // as they would be the same colour/shape
//					if ((((Road) road).getLine(LineType.NWSIDE)).contains(testCoord.x, testCoord.y) ||
//						(((Road) road).getLine(LineType.SESIDE)).contains(testCoord.x, testCoord.y))
//					{
//						if (((location.distance(testCoord.x, testCoord.y) > location.distance(RM)) && findNearest == false) || RM.x == -1) {
//							RM = new Double2D(testCoord.x, testCoord.y);
//						} else if ((location.distance(testCoord.x, testCoord.y) < location.distance(RM)) && findNearest == true) {
//							RM = new Double2D(testCoord.x, testCoord.y);
//						}
//					}
//				}
//			}
//		}
//		
//		return RM; 
//	}	
//	
	

	
//	/**
//	 * HH 7.5.14 - Based on Car.alterCourse (Robert Lee)
//	 * 
//	 * Method which adds a Waypoint for the vehicle to travel via on it's path to
//	 * prevent it from leaving the road
//	 * 
//	 * @param roads All of the roads in the environment
//	 * @param location of the vehicle (me)
//	 * @param wpID id for the Waypoint
//	 * @param sim // HH 28.7.14 - added for fault insertion
//	 */
//	public void alterCourse(Bag roads, Double2D me, int wpID, Continuous2D environment, Double2D destination, COModel sim)
//	{
//		double resolution = 0.5;
//		MutableDouble2D coord = new MutableDouble2D(me);
//		Waypoint wp;
//		double xComponent;
//		double yComponent;
//	
//		// Work out which direction we would need to turn in to be closer to the actual destination ('real' target)
//		// code inspired by Car.setDirection (Robert Lee)
//		double idealDirection = Utility.calculateAngle(me, destination);
//		double reqDirection = Utility.correctAngle(idealDirection - getDirection());
//		int iMult = 0;
//		
//		if (reqDirection >= 0 && reqDirection < 180) {
//			iMult = 1;
//		} else if (reqDirection >= 0 && reqDirection < 180) {
//			iMult = -1;
//		} else {
//			// TO DO - Error condition here...
//		}
//		
//		// HH 28.7.14 - added new loop params to support fault insertion
//		double minTurn = 0;
//		double maxTurn = getStats().getCurrentMaxTurning() - 5;
//		double turnResolution = resolution;
//		
//		// FAULT #24 - HH 28/7/14 - Force the angle loop to start half-way through
//		if (sim.getFault(24) == true) {
//			minTurn = (getStats().getCurrentMaxTurning() - 5)/2;
//			sim.setFault(24); // HH 13.11.14 Added
//		} 
//
//		// FAULT #25 - HH 28/7/14 - Force the angle loop to end half-way through
//		if (sim.getFault(25) == true) {
//			maxTurn = (getStats().getCurrentMaxTurning() - 5)/2;
//			sim.setFault(25);
//		} 
//		
//		// FAULT #26 - HH 28/7/14 - Force the sensor to 'reduce' angular resolution (double iterator step)
//		if (sim.getFault(26) == true) {
//			turnResolution = resolution*2;
//			sim.setFault(26); // HH 13.11.14 Added
//		} 
//				
//		for(double i = minTurn; i < maxTurn; i += turnResolution) // HH - TO DO - Why do we -5 here?  To ensure we turn at least 5 degrees?
//		{		
//			
//			if (checkRoads(roads, Utility.correctAngle(getDirection() - (i*iMult)), sim) == true)
//			{
//				//then moving right gives a clear path
//				//set wp and return
//				
//				//first must find out where to put wp
//				xComponent = Utility.xMovement(Utility.correctAngle(getDirection() - (iMult*(i+5))), (UGVViewingRange / 1));
//				yComponent = Utility.yMovement(Utility.correctAngle(getDirection() - (iMult*(i+5))), (UGVViewingRange / 1));
//				coord.addIn(xComponent, yComponent);
//				wp = new Waypoint(wpID, getTargetID());
//				wp.setLocation(new Double2D(coord));
//				setTargetID(wpID);
//				environment.setObjectLocation(wp, new Double2D(coord));
//				return;
//				
//			} else if (checkRoads(roads, Utility.correctAngle(getDirection() + (i*iMult)), sim) == true) {
//				//then moving left gives a clear path
//				//set wp and return
//				
//				xComponent = Utility.xMovement(Utility.correctAngle(getDirection() + (iMult*(i+5))), (UGVViewingRange / 1));
//				yComponent = Utility.yMovement(Utility.correctAngle(getDirection() + (iMult*(i+5))), (UGVViewingRange / 1));
//				coord.addIn(xComponent, yComponent);
//				wp = new Waypoint(wpID, getTargetID());
//				wp.setLocation(new Double2D(coord));
//				setTargetID(wpID);
//				environment.setObjectLocation(wp, new Double2D(coord));
//				return;
//			}
//		}
//		
//		//no path that can be immediately turned onto is clear
//		//therefore see if it is possible for the car to see a clear path even
//		//if it can't be immediately turned onto
//		for(double i = (getStats().getCurrentMaxTurning()-5); i < (UGVViewingAngle / 2); i += resolution)
//		{
//			if (checkRoads(roads, Utility.correctAngle(getDirection() - i), sim) == true)
//			{
//				//then moving right gives a clear path
//				//set wp and return
//				
//				//first must find out where to put wp
//				
//				xComponent = Utility.xMovement(Utility.correctAngle(getDirection() - (getStats().getCurrentMaxTurning()+5)), (UGVViewingRange / 1));
//				yComponent = Utility.yMovement(Utility.correctAngle(getDirection() - (getStats().getCurrentMaxTurning()+5)), (UGVViewingRange / 1));
//				coord.addIn(xComponent, yComponent);
//				wp = new Waypoint(wpID, getTargetID());
//				wp.setLocation(new Double2D(coord));
//				setTargetID(wpID);
//				environment.setObjectLocation(wp, new Double2D(coord));
//				return;
//				
//			} else if (checkRoads(roads, Utility.correctAngle(getDirection() + i), sim) == true) {
//				//then moving left gives a clear path
//				//set wp and return
//				
//				xComponent = Utility.xMovement(Utility.correctAngle(getDirection() + (getStats().getCurrentMaxTurning()+5)), (UGVViewingRange / 1));
//				yComponent = Utility.yMovement(Utility.correctAngle(getDirection() + (getStats().getCurrentMaxTurning()+5)), (UGVViewingRange / 1));
//				coord.addIn(xComponent, yComponent);
//				wp = new Waypoint(wpID, getTargetID());
//				wp.setLocation(new Double2D(coord));
//				setTargetID(wpID);
//				environment.setObjectLocation(wp, new Double2D(coord));
//				return;
//			}
//		}
//	}
	
	/**
	 * HH 7.5.14 - Based on Car.alterCourse (Robert Lee)
	 * 
	 * Method which adds a Waypoint for the vehicle to travel via on it's path to
	 * prevent it from leaving the road - specific version for when UGV is to follow the road markings.  Needs to take
	 * into account when on the approach  to a junction, that should be prepared to stop, and then change direction
	 * (possibly including a UTurn) to enable movement closer to the target and/or to explore uncharted space.
	 * NOTE - This method should only be called if we are on the approach to a junction, otherwise the vehicle
	 * should maintain its current heading whilst making slight allowances if the vehicle has temporarily moved
	 * away from the nearside road marking.
	 * 
	 * @param roads All of the roads in the environment
	 * @param location of the vehicle (me)
	 * @param wpID id for the Waypoint
	 * @param environment so that we can add the new waypoint
	 * @param destination: either the target if we're in a junction approach, or the required position at the limit of the range, assuming current heading might not get vehicle there
	 */
	public void alterCourseRM(Bag roads, Double2D me, int wpID, Continuous2D environment, Double2D destination)
	{
//		double resolution = 0.5;
//		MutableDouble2D coord = new MutableDouble2D(me);
//		Waypoint wp;
//		double xComponent;
//		double yComponent;
	
		// Work out which direction we would need to turn in to be closer to the supplied destination (whether it be
		// the target, or an arbitrary future point to help with lane tracking) code inspired by Car.setDirection (Robert Lee)
		double idealDirection = Utility.calculateAngle(me, destination);

		// If the ideal direction and the current direction are within 5 degrees of one another, there is no point turning
		// yet, so exit this method.
		if (Math.abs(idealDirection - getDirection()) < 5) {
			return;
		}
		
		// [TODO] add code here to check the ideal direction against the internal mapping to see if it would allow us
		// to explore previously uncharted territory.
		
		//double reqDirection = correctAngle(idealDirection - getDirection());
		//int iMult = 0;
		
		
//		// [TODO] CHECK WHETHER THESE ARE THE RIGHT WAY AROUND
//		if (reqDirection >= 0 && reqDirection < 180) {
//			iMult = 1;
//		} else if (reqDirection >= 180 && reqDirection < 360) {
//			iMult = -1;
//		} else {
//			// TO DO - Error condition here...
//		}	
		
		// HH 8.7.14 [TODO] Need to analyse which junction arms are available for turning into, and determine
		// which one will bring the UGV closer to the target (or which ones haven't been explored yet)
		
		
		
		
// HH 8.7.14 Note - the code below will insert a waypoint at the limit of the UGV viewing range, which is not ideal if a
// rapid response is required e.g. to turn within a junction
// HH 19.6.14 [TODO] Removed this as may no longer be relevant, or may need reinstating in the future for obstacles etc.		
//		for(double i = 0; i < (getStats().getCurrentMaxTurning() - 5); i += resolution) // HH - TO DO - Why do we -5 here?  To ensure we turn at least 5 degrees?
//		{		
//			
//			if (checkRoads(roads, correctAngle(getDirection() - (i*iMult))) == true)
//			{
//				//then moving right gives a clear path
//				//set wp and return
//				
//				//first must find out where to put wp
//				xComponent = xMovement(correctAngle(getDirection() - (iMult*(i+5))), (UGVViewingRange / 1));
//				yComponent = yMovement(correctAngle(getDirection() - (iMult*(i+5))), (UGVViewingRange / 1));
//				coord.addIn(xComponent, yComponent);
//				wp = new Waypoint(wpID, getTargetID());
//				wp.setLocation(new Double2D(coord));
//				setTargetID(wpID);
//				environment.setObjectLocation(wp, new Double2D(coord));
//				return;
//				
//			} else if (checkRoads(roads, correctAngle(getDirection() + (i*iMult))) == true) {
//				//then moving left gives a clear path
//				//set wp and return
//				
//				xComponent = xMovement(correctAngle(getDirection() + (iMult*(i+5))), (UGVViewingRange / 1));
//				yComponent = yMovement(correctAngle(getDirection() + (iMult*(i+5))), (UGVViewingRange / 1));
//				coord.addIn(xComponent, yComponent);
//				wp = new Waypoint(wpID, getTargetID());
//				wp.setLocation(new Double2D(coord));
//				setTargetID(wpID);
//				environment.setObjectLocation(wp, new Double2D(coord));
//				return;
//			}
//		}
//		
//		//no path that can be immediately turned onto is clear
//		//therefore see if it is possible for the car to see a clear path even
//		//if it can't be immediately turned onto
//		for(double i = (getStats().getCurrentMaxTurning()-5); i < (UGVViewingAngle / 2); i += resolution)
//		{
//			if (checkRoads(roads, correctAngle(getDirection() - i)) == true)
//			{
//				//then moving right gives a clear path
//				//set wp and return
//				
//				//first must find out where to put wp
//				
//				xComponent = xMovement(correctAngle(getDirection() - (getStats().getCurrentMaxTurning()+5)), (UGVViewingRange / 1));
//				yComponent = yMovement(correctAngle(getDirection() - (getStats().getCurrentMaxTurning()+5)), (UGVViewingRange / 1));
//				coord.addIn(xComponent, yComponent);
//				wp = new Waypoint(wpID, getTargetID());
//				wp.setLocation(new Double2D(coord));
//				setTargetID(wpID);
//				environment.setObjectLocation(wp, new Double2D(coord));
//				return;
//				
//			} else if (checkRoads(roads, correctAngle(getDirection() + i)) == true) {
//				//then moving left gives a clear path
//				//set wp and return
//				
//				xComponent = xMovement(correctAngle(getDirection() + (getStats().getCurrentMaxTurning()+5)), (UGVViewingRange / 1));
//				yComponent = yMovement(correctAngle(getDirection() + (getStats().getCurrentMaxTurning()+5)), (UGVViewingRange / 1));
//				coord.addIn(xComponent, yComponent);
//				wp = new Waypoint(wpID, getTargetID());
//				wp.setLocation(new Double2D(coord));
//				setTargetID(wpID);
//				environment.setObjectLocation(wp, new Double2D(coord));
//				return;
//			}
//		}
	}
		
	/** 
	 * HH 10.7.14 - Work out a new waypoint location to allow a U-turn (based on current location and heading)
	 * should just be a 180 degree flip, but need to know whether this equates to a translation in the N, E, S
	 * or W direction.  Width of translation should be equivalent to roadWidth/2
	 * 
	 * @param me
	 * @param bearing
	 * @return Double2D with location of waypoint to allow a U-turn
	 */
	public Double2D getUTurn(Double2D me, double bearing)
	{
		UGV_Direction direction = Utility.getDirection(bearing);
		
		switch (direction) {
		
			// HH 14.7.14 Updated to align further from the kerb (taking road markings into account) so that match locations
		    // returned by calcRMOffset.
			// HH 15.7.14 Revised above to align central to the lane, independent of road markings (to prevent really tight Uturns
			case NORTH : 
				//return new Double2D(me.x+(Road.roadWidth/2 - (2*(Constants.ROADEDGEOFFSET + Constants.ROADEDGINGWIDTH))), me.y);
				return new Double2D(me.x+Road.roadWidth/2, me.y);
			case EAST : 
				//return new Double2D(me.x, me.y+(Road.roadWidth/2 - (2*(Constants.ROADEDGEOFFSET + Constants.ROADEDGINGWIDTH))));
				return new Double2D(me.x, me.y+Road.roadWidth/2);
			case SOUTH : 
				//return new Double2D(me.x-(Road.roadWidth/2 - (2*(Constants.ROADEDGEOFFSET + Constants.ROADEDGINGWIDTH))), me.y);
				return new Double2D(me.x-Road.roadWidth/2, me.y);
			case WEST : 
				//return new Double2D(me.x, me.y-(Road.roadWidth/2 - (2*(Constants.ROADEDGEOFFSET + Constants.ROADEDGINGWIDTH))));
				return new Double2D(me.x, me.y-Road.roadWidth/2);
		}		

		// [TODO] Add error condition here!!
		return new Double2D(0,0);
	}
	
	/** 
	 * HH 7.8.14 - Work out a new waypoint location for parked car overtake (based on current location and heading)
	 * should be based on known obstacle parameters, but need to know whether this equates to a translation in the 
	 * N, E, S or W direction. The passed param distancePt will be the intersection with the obstacle and in the case
	 * of the beginning of an overtake will be then minimum intersection, for the second stange of overtake will be the 
	 * maximum intersection, and for the final stage will be a 'dummy' intersection of (-1,-1) as no intersection 
	 * should be detected here and the constant offset of OBSTACLE_HEADWAY should be used instead
	 * 
	 * @param me
	 * @param bearing
	 * @param distance - can 
	 * @param isReturn - if this WP is to return the vehicle to the centre of the lane
	 * @return Double2D with location of waypoint to allow start of overtake
	 */
	public Double2D getOvertakeWP(Double2D me, double bearing, Double2D distancePt, OvertakeStage inStage)
	{
		UGV_Direction direction = Utility.getDirection(bearing);
		
		// Work out the offset that should be used into the lane (added to the current vehicle position within lane)
		double laneOffset = Constants.OBSTACLE_WIDTH + 0.5; // Pull out to avoid the obstacle - HH 18.11.14 Added an extra 0.5m buffer as obstacle is 0.325m from kerb
		
		if (inStage == OvertakeStage.OVERTAKE_PULLEDOUT) {
			laneOffset = 0; // Remain at same offset, to avoid the obstacle
		} else if (inStage == OvertakeStage.OVERTAKE_FINISH) {
			laneOffset = -(Constants.OBSTACLE_WIDTH + 0.5); // NOTE - this is a negative offset! Return to original lane, obstacle passed - HH 18.11.14 Added an extra 0.5m buffer as obstacle is 0.325m from kerb
		}
			
		double distanceOffset;
		
		// HH 15.12.14 Changed the code below to bring the waypoint slightly further away from the
		// edge of the obstacle
		double obsOffset = Constants.OBSTACLE_BUFFER; // HH 16.12.14 Added a variable here so we can adjust 17.12.14 -> Constant
		
		// TODO: HH 16.12.14 We want to set the waypoints so they are relative to the 
		// observed location of the UGV, rather than relative to the current location of the
		// UGV - this should help to mitigate against cascading errors caused by the UGV being offset from
		// the WP at the point when it 'reaches' the WP.
		
		switch (direction) {

		case NORTH : 
			distanceOffset = distancePt.y;
			
			if (inStage == OvertakeStage.OVERTAKE_FINISH) {
				distanceOffset = me.y - OBSTACLE_HEADWAY;
			}
			
			// HH 15.12.14 Add buffer
			if (inStage == OvertakeStage.OVERTAKE_START) {
				distanceOffset = distanceOffset + obsOffset; // Nearer to UGV
			} else if (inStage == OvertakeStage.OVERTAKE_PULLEDOUT) {
				distanceOffset = distanceOffset - obsOffset; // Further from UGV
			}
			
			return new Double2D(me.x + laneOffset, distanceOffset);
		case EAST : 

			distanceOffset = distancePt.x;
			
			if (inStage == OvertakeStage.OVERTAKE_FINISH) {
				distanceOffset = me.x + OBSTACLE_HEADWAY;
			}
			
			// HH 15.12.14 Add buffer
			if (inStage == OvertakeStage.OVERTAKE_START) {
				distanceOffset = distanceOffset - obsOffset; // Nearer to UGV
			} else if (inStage == OvertakeStage.OVERTAKE_PULLEDOUT) {
				distanceOffset = distanceOffset + obsOffset; // Further from UGV
			}
			
			return new Double2D(distanceOffset, me.y + laneOffset);
		case SOUTH : 

			distanceOffset = distancePt.y;
			
			if (inStage == OvertakeStage.OVERTAKE_FINISH) {
				distanceOffset = me.y + OBSTACLE_HEADWAY;
			}
			
			// HH 15.12.14 Add buffer
			if (inStage == OvertakeStage.OVERTAKE_START) {
				distanceOffset = distanceOffset - obsOffset; // Nearer to UGV
			} else if (inStage == OvertakeStage.OVERTAKE_PULLEDOUT) {
				distanceOffset = distanceOffset + obsOffset; // Further from UGV
			}
			
			return new Double2D(me.x - laneOffset, distanceOffset);
		case WEST : 

			distanceOffset = distancePt.x;
			
			if (inStage == OvertakeStage.OVERTAKE_FINISH) {
				distanceOffset = me.x - OBSTACLE_HEADWAY;
			}
			
			// HH 15.12.14 Add buffer
			if (inStage == OvertakeStage.OVERTAKE_START) {
				distanceOffset = distanceOffset + obsOffset; // Nearer to UGV
			} else if (inStage == OvertakeStage.OVERTAKE_PULLEDOUT) {
				distanceOffset = distanceOffset - obsOffset; // Further from UGV
			}
			
			return new Double2D(distanceOffset, me.y - laneOffset);
		}	
				
		// [TODO] Add error condition here!!
		return new Double2D(0,0);
	}
	
	private boolean checkWall()
	{
		Double2D me = this.location;
		
		if(me.x <= 0.2 )
		{
			//System.out.println("clash with the left wall!");
			return true;
		}
		else if(Constants.WorldXVal - me.x <= 0.2)
		{
			//System.out.println("clash with the right wall!");
			return true;
		}
		
		if (me.y <= 0.2)
		{
			//System.out.println("clash with the upper wall!");
			return true;
		}
		else if(Constants.WorldYVal - me.y <= 0.2)
		{
			//System.out.println("clash with the lower wall!");
			return true;
		}
		
		return false;
	}
	

	
	/** 
	 * HH 14.7.14 - Work out whether the supplied location is about to leave one of the road surfaces
	 * 
	 * @param Bag: roads collection
	 * @param Double2D: location to be checked
	 * @param double: bearing on which UGV is travelling
	 * @return boolean: true if offset location will leave the road surface
	 */
	private boolean nearlyOffRoad(Bag roads, Double2D location, double bearing)
	{
		Double2D offsetLocation = new Double2D(location.x, location.y);
		
		double offset = 2;
		
		UGV_Direction direction = Utility.getDirection(bearing);
		
		switch (direction) {
		
			case NORTH : 
				offsetLocation = new Double2D(location.x, (location.y - offset));
				break;
			case EAST : 
				offsetLocation = new Double2D((location.x + offset), location.y);
				break;
			case SOUTH : 
				offsetLocation = new Double2D(location.x, (location.y + offset));
				break;
			case WEST : 
				offsetLocation = new Double2D((location.x - offset), location.y);
				break;
		}	
		
		return !onRoad(roads, getShapeAtOffset(offsetLocation, Utility.getOrientation2D(bearing)));
	}
	
	/** 
	 * HH 15.7.14
	 * 
	 * @return the 2D array of information about junction that have been visited
	 */	
	public int[][] getJunctionHistory()
	{
		return junctionHistory;
	}
	
	/** 
	 * HH 15.7.14 - update the 2D array of information about junction that have been visited
	 */
	public void updateJunctionHistory(int idx, UGV_Direction direction)
	{
		if (idx <= junctionHistory.length && direction.ordinal() <= junctionHistory[idx].length )
		{
			junctionHistory[idx][direction.ordinal()] ++;
		}
	}
	
	//
	public boolean getTargetFound() {
		return targetFound;
	}
	
	/*
	 *  HH 6.8.14 - return the number of steps required to complete the manoeuvre, based
	 *  on various assumptions about obstacle length, headway and speed/acceleration.
	 */
	
	private int getAvgManoeuvreTime(double distanceToObstacle) {

		double manoeuvreLength = distanceToObstacle + Constants.OBSTACLE_LENGTH + Constants.OBSTACLE_HEADWAY; // Headway before and after obstacle is the same
		
		// We assume that the vehicle is able to complete the overtake unimpeded, and can therefore accelerate
		// from the current speed, at the maximum acceleration, up to the maximum speed of the vehicle.  
		
		double prevSpeed = 0;
		double currentSpeed = getSpeed();
		int noSteps = 0;
		double distance = 0;
		double acc = getStats().getCurrentMaxAccel();
		double maxSpeed = getStats().getCurrentMaxSpeed();
		
		while (distance < manoeuvreLength) {
			
			prevSpeed = currentSpeed;
			currentSpeed = Math.min(currentSpeed + acc, maxSpeed);
			distance = distance + ((prevSpeed + currentSpeed)/2);			
			noSteps ++; 
		}
		
				
		return noSteps;
	}
	
	/*
	 *  HH 6.8.14 - return the distance to an obstacle which is found in the current lane.  Search conducted at a 
	 *  range of up to 25m.  Search progresses in a similar way to the roadMarkings search, and looks for the closest
	 *  hit on an obstacle within a narrow range: Constants.UGVObsViewingAngle
	 *  HH 3.12.14 - removed input inRoad as the UGV cannot be allowed to 'query the road', it can only build up
	 *  an image of its surroundings by using sensors.
	 */
	
	private Double2D checkForObstacle(COModel sim, double bearing, Obstacle obstacle, boolean getMax) {
		
		//simple and dirty method which checks the coordinates between 0 and 
		//the obstacle viewing range away from the target in certain increments and see 
		//if they intersect with road markings
		MutableDouble2D testCoord = new MutableDouble2D();
		Double2D amountAdd = new Double2D();
		double reqDistance = UGVObsViewingRange;
		Double2D reqCoord = new Double2D(-1,-1);
		double distance;
		
		if (getMax == true) {
			reqDistance = 0;
		}
				
		// HH 13.8.14 Need to restrict the obstacle checks to those which are in the same lane as
		// the UGV, so need to know direction in order to restrict in method below
		UGV_Direction direction = Utility.getDirection(bearing);
		
		// For each angle that the sensor is able to view, turning in realistic increments
		double resolution = 0.5;
		double newBearing = 0.0;
		
		// HH 28.7.14 - Added variables for the loop to support fault insertion
		double startAngle = -(UGVObsViewingAngle/2);
		double endAngle = (UGVObsViewingAngle / 2);
		double startRange = 0;
		double endRange = UGVObsViewingRange;
		double rangeSensitivity = sensitivityForRoadTracking;
						
		// HH 20/8/14 - Use the road marking detection algorithm to find the centre of the road so that we
		// can make sure that we only detect obstacles on the same side of the road as the UGV's direction of
		// travel.
		
		// HH 3.12.14 Need to remove dependence on access to the Road object, should conduct a search instead
		//Double2D furthestLaneMarking = findRoadMarkings(inRoad, bearing, location, sim);
		// HH 4.12.14 Changed from Constants.genLineType.CENTRE to Constants.genLineType.NEARSIDE
		Double2D furthestLaneMarking = locateRoadMarkings_AllRoads(sim.roads, false, sim, Constants.genLineType.NEARSIDE, UGVViewingAngle , UGVViewingRange, sensitivityForRoadTracking);
		
		double centreOffset = Road.roadWidth/2 - Constants.ROADEDGINGWIDTH - Constants.ROADEDGEOFFSET; // Distance between edge line and centre
		
		for(double i = startAngle; i < endAngle; i += resolution)
		{
			// Reset the location that we start testing from to be the location of the UGV and set the bearing
			// that we are going to use for this iteration
			testCoord.setTo(0,0);
			testCoord.addIn(location);
			newBearing = Utility.correctAngle(bearing + i);
			
			// Construct the x an y increments for each iteration below
			amountAdd = new Double2D(Utility.xMovement(newBearing, rangeSensitivity), Utility.yMovement(newBearing, rangeSensitivity));
						
		    // NOTE - j is not actually used, it just ensures the correct number of iterations
			for(double j = startRange; j < endRange; j += rangeSensitivity){
												
				testCoord.addIn(amountAdd);  // move the test location outwards on the chosen bearing
				
				// HH 3.12.14 Ensure that the testCoord location is still on the road surface,
				// if not, we would expect that any vision algorithm would have noticed this
				// and stopped searching at this bearing (similar to stopping the search 
				// when we have found an obstacle (as we can't see through it!)
				if (sim.roadAtPoint(new Double2D(testCoord), sim.roads) == false) {
					break; // Don't search any further on this bearing
				}
				
				// HH 13.8.14 Ensure that the our test coordinate is in the same lane as the UGV
				boolean inLane = true;
				double centre;
				switch (direction) {
				
					case NORTH : {
						centre = furthestLaneMarking.x + centreOffset;
						if (testCoord.x > centre || testCoord.x < (centre - Road.roadWidth/2)) {
							inLane = false;
						}	
						break;
					}
					case SOUTH : {
						centre = furthestLaneMarking.x - centreOffset;
						if (testCoord.x < centre || testCoord.x > (centre + Road.roadWidth/2)) {
							inLane = false;
						}
						break;
					}
					case EAST : {
						centre = furthestLaneMarking.y + centreOffset;
						if (testCoord.y > centre || testCoord.y < (centre - Road.roadWidth/2)) {
							inLane = false;
						}
						break;
					}
					case WEST : {
						centre = furthestLaneMarking.y - centreOffset;
						if (testCoord.y < centre || testCoord.y > (centre + Road.roadWidth/2)) {
							inLane = false;
						}
						break;
					}
				}
				
				if (inLane == true) {
					// keep adding the amountAdd on and seeing if the coordinate is inside an obstacle
					if (((ParkedCar) obstacle).inShape(new Double2D(testCoord.x, testCoord.y)))
					{
						// Store the distance at which the testCoord has intersected
						distance = location.distance(testCoord.x, testCoord.y);

						if (getMax == true) {
							if (distance > reqDistance) {
								reqDistance = distance;
								reqCoord = new Double2D(testCoord.x, testCoord.y);
							}						
						} else {
							if (distance < reqDistance) {
								reqDistance = distance;
								reqCoord = new Double2D(testCoord.x, testCoord.y);
							}
						}

						// Exit the loop as we don't need to search any further as we've found an obstacle
						break;
					}
				}
			}
		}
		
		return reqCoord;
	}

	/**
	 * HH 7.8.14 - Based on Car.checkCourse (Robert Lee)
	 *  
	 * @param sim
	 * @param bearing
	 * @return distance to the obstacle
	 */
	private Double2D checkAllObstacles(COModel sim, double bearing, boolean getMax)
	{
		// Init to the values we would want for finding the min
		double reqDistance = UGVObsViewingRange + 1;
		double currentDistance = UGVObsViewingRange + 1;
		Double2D currentDistanceCoord; 
		Double2D reqCoord = new Double2D(-1,-1);
		
		if (getMax == true) {
			reqDistance = 0;
			currentDistance = 0;
		}
		
		// HH 3.12.14 - It's not really acceptable to 'query the road' to find out what its ID is,
		// and then query the Obstacle to find out which road it is on.  Instead we just restrict our search for 
		// obstacles within our line of sight so that the search terminates as soon as the search point
		// leaves the road, or reaches an obstacle.  If the search terminates when it reaches the road then it 
		// would be impossible to detect obstacles which are on a different road (unless the roads are closer 
		// than the resolution of the search, which shouldn't be the case).
		
//		// HH 13.8.14 - Work out which road the UGV is on, so that we can compare the id to that of the 
//		// obstacle - we are only interested in obstacles that are on the same road as the UGV
//		int roadId = -1;
//		Road thisRoad = null;
//		
//		for (int r = 0; r < sim.roads.size(); r++)
//		{
//			if (((Road)sim.roads.get(r)).inShape(location) == true) {
//				roadId = ((Road) sim.roads.get(r)).getID();
//				thisRoad = (Road) sim.roads.get(r);
//				break;
//			}
//		}
		
		// Look through all the obstacles and look for intersection
		for(int i = 0; i < sim.obstacles.size(); i++)
		{
			
//			if (((ParkedCar) sim.obstacles.get(i)).getRoadId() == roadId)
//			{
			//if (((Obstacle)sim.obstacles.get(i)).inShape(location) == true ) 
			//{
				currentDistanceCoord = checkForObstacle(sim, bearing, (Obstacle) sim.obstacles.get(i), getMax); // HH 3.12.14 Removed thisRoad param
				if (currentDistanceCoord.x > -1) {
					currentDistance = location.distance(currentDistanceCoord.x, currentDistanceCoord.y);

					if (getMax == true) {
						if ( currentDistance > reqDistance )
						{
							reqDistance = currentDistance;
							reqCoord = currentDistanceCoord;
						}					
					} else {
						if ( currentDistance < reqDistance )
						{
							reqDistance = currentDistance;
							reqCoord = currentDistanceCoord;
						}
					}
				}
			//}
			//}
		}
		
		return reqCoord; // TODO Need to do a check on return value as if this returns a value greater
							// than the maximum sensor range (for getMin) or 0 for (getMax) then it denotes 
							// *no obstacle*
	}

//	/**
//	 * HH 9.9.14 - Based on Car.checkCourse (Robert Lee)
//	 *  
//	 * @param roads
//	 * @param findNearest - whether we are interested in the closest or furthest RM found
//	 * @param sim // HH 28.7.14 - added for fault insertion
//	 * @param reqLine - which line are we searching for (nearside, offside, centre)
//	 * @return coordinates of the furthermost road marking detected.
//	 */
//	private Double2D locateRoadMarkings_AllRoads(Bag roads, boolean findNearest, COModel sim, genLineType reqLine)
//	{
//		Double2D currentXY = new Double2D(0,0);
//		Double2D requiredXY = new Double2D(0,0);
//
//		// Check all the roads as the UGV doesn't *know* which one it is on
//		for(int i = 0; i < roads.size(); i++)
//		{
////			// FAULT #14 - HH 28/7/14 - Exit the loop half-way through
////			if (sim.getFault(14) == true) {
////				if (i == (roads.size()/2)) {
////					break;
////				}
////			}
//			
//			currentXY = locateRoadMarkings((Road) roads.get(i), findNearest, sim, reqLine);
//
//			// See if the latest return value is a better match for what we are looking for
//			if ( (onMap(currentXY) == true) && ((location.distance(currentXY) > location.distance(requiredXY) && findNearest == false) ||
//			     (location.distance(currentXY) < location.distance(requiredXY) && findNearest == true) || (requiredXY.x == -1)) )
//			{
//				// check for an invalid return code
//				if (currentXY.x != -1) {
//					requiredXY = currentXY;
//				}
//			}
//
//		}
//		
//		return requiredXY;
//	}
	
//	/*
//	 *  HH 8.9.14 - return the distance to a moving obstacle which is found on the road ahead.  Search conducted at a 
//	 *  range of up to 100m.  Search progresses in a similar way to the roadMarkings search, and looks for the closest
//	 *  hit on an obstacle within a narrow range: Constants.UGVMovObsViewingAngle
//	 *  
//	 *  @param sim
//	 *  @param inCar - the vehicle that has been detected as being on the same road
//	 *  @param sensorLoc - for oncoming traffic, pass in the offside front corner location
//	 *  @param sameLane - true if we are looking for other moving vehicles ahead of us in the lane, false for oncoming vehicles
//	 */
//	
//	private Double2D checkForMovingObstacle(COModel sim, DumbCar inCar, Double2D sensorLoc, boolean sameLane) {
//		
//		//simple and dirty method which checks the coordinates between 0 and 
//		//the moving obstacle viewing range away from the target in certain increments and see 
//		//if they intersect with the supplied car (moving obs) 
//		MutableDouble2D testCoord = new MutableDouble2D();
//		Double2D amountAdd = new Double2D();
//		double reqDistance = UGVMovObsViewingRange;
//		Double2D reqCoord = new Double2D(-1,-1);
//		double distance;
//		
//		// HH 13.8.14 Need to restrict the obstacle checks to those which are in the same lane as
//		// the UGV, so need to know direction in order to restrict in method below
//		UGV_Direction direction = getDirection(getDirection()); // Returns a compass direction rather than angle
//		
//		// HH 9.9.14 Need to work out the bounds for the lane that we are searching in, by checking the
//		// road markings.  If we are looking for vehicles in the same lane, search should be bounded by 
//		// the nearside and centre lane markings; for oncoming vehicles, it will be the centre and offside 
//		// markings.  
//		Double2D leftBound; 
//		Double2D rightBound;
//		
//		if (sameLane == true) 
//		{
//			leftBound = locateRoadMarkings_AllRoads(sim.roads, true, sim, genLineType.NEARSIDE);
//			rightBound = locateRoadMarkings_AllRoads(sim.roads, true, sim, genLineType.CENTRE);
//		} else {
//			leftBound = locateRoadMarkings_AllRoads(sim.roads, true, sim, genLineType.CENTRE);
//			rightBound = locateRoadMarkings_AllRoads(sim.roads, true, sim, genLineType.OFFSIDE);
//		}
//		
//		// For each angle that the sensor is able to view, turning in realistic increments
//		double resolution = 0.5;
//		double newBearing = 0.0;
//		
//		// HH 28.7.14 - Added variables for the loop to support fault insertion
//		double startAngle = -(UGVMovObsViewingAngle/2);
//		double endAngle = (UGVMovObsViewingAngle / 2);
//		double startRange = 0;
//		double endRange = UGVMovObsViewingRange;
//		double rangeSensitivity = sensitivityForRoadTracking;
//				
////		// FAULT #17 - HH 28/7/14 - Force the angle loop to start half-way through
////		if (sim.getFault(17) == true) {
////			startAngle = 0;
////		} 
////
////		// FAULT #18 - HH 28/7/14 - Force the angle loop to end half-way through
////		if (sim.getFault(18) == true) {
////			endAngle = 0;
////		} 
////		
////		// FAULT #19 - HH 28/7/14 - Force the angle sensor to 'reduce' range resolution (double iterator step)
////		if (sim.getFault(19) == true) {
////			resolution = resolution*2;
////		} 
//				
//		for(double i = startAngle; i < endAngle; i += resolution)
//		{
//			// Reset the location that we start testing from to be the location of the UGV and set the bearing
//			// that we are going to use for this iteration
//			testCoord.setTo(0,0);
//			testCoord.addIn(location);
//			newBearing = correctAngle(getDirection() + i);
//			
//			// Construct the x an y increments for each iteration below
//			amountAdd = new Double2D(xMovement(newBearing, rangeSensitivity), yMovement(newBearing, rangeSensitivity));
//						
////			// FAULT #21 - HH 28/7/14 - Force the angle loop to start half-way through
////			if (sim.getFault(21) == true) {
////				startRange = UGVViewingRange/2;
////			} 
////
////			// FAULT #22 - HH 28/7/14 - Force the angle loop to end half-way through
////			if (sim.getFault(22) == true) {
////				endRange = UGVViewingRange/2;
////			} 
////			
////			// FAULT #23 - HH 28/7/14 - Force the sensor to 'reduce' angular resolution (double iterator step)
////			if (sim.getFault(23) == true) {
////				rangeSensitivity = sensitivityForRoadTracking*2;
////			} 
//						
//		    // NOTE - j is not actually used, it just ensures the correct number of iterations
//			for(double j = startRange; j < endRange; j += rangeSensitivity){
//												
//				testCoord.addIn(amountAdd);  // move the test location outwards on the chosen bearing
//				
//				// HH 13.8.14 Ensure that the our test coordinate is within our test bounds
//				boolean inLane = false;
//				
//				if (direction == UGV_Direction.NORTH || direction == UGV_Direction.SOUTH)
//				{
//					if (testCoord.x > leftBound.x && testCoord.x < rightBound.x) {
//						inLane = true;
//					}
//				} else {
//					if (testCoord.y > leftBound.y && testCoord.y < rightBound.y) {
//						inLane = true;
//					}
//				}
//								
//				if (inLane == true) {
//					// keep adding the amountAdd on and seeing if the coordinate is inside an obstacle
//					if (((DumbCar) inCar).inShape(new Double2D(testCoord.x, testCoord.y)))
//					{
//						// Store the distance at which the testCoord has intersected
//						distance = location.distance(testCoord.x, testCoord.y);
//
//						if (distance < reqDistance) {
//							reqDistance = distance;
//							reqCoord = new Double2D(testCoord.x, testCoord.y);
//						}
//
//						// Exit the loop as we don't need to search any further as we've found an obstacle
//						break;
//					}
//				}
//			}
//		}
//		
//		return reqCoord;
//	}

//	/**
//	 * HH 8.9.14 - Return the closest observed intersection with a moving obstacle
//	 *  
//	 * @param sim
//	 * @param bearing
//	 * @param sameLane
//	 * @return distance to the obstacle
//	 */
//	private Double2D checkAllMovingObstacles(COModel sim, boolean sameLane)
//	{
//		// Init to the values we would want for finding the min
//		double reqDistance = UGVMovObsViewingRange + 1;
//		double currentDistance = UGVMovObsViewingRange + 1;
//		
//		// Calculate the location of the sensor, assuming it is on the front offside corner of the vehicle
//		Double2D sensorLoc;
//		double xDispl = xMovement(correctAngle(getDirection() - 90), UGV_WIDTH/2);
//		double yDispl = yMovement(correctAngle(getDirection() - 90), UGV_WIDTH/2);
//		sensorLoc = new Double2D(location.x + xDispl, location.y + yDispl);
//		
//		Double2D currentDistanceCoord; 
//		Double2D reqCoord = new Double2D(-1,-1);
//				
//		DumbCar currentCar;
//		
//		// Look through all the moving obstacles (cars) and look for intersection
//		for(int i = 0; i < sim.cars.size(); i++)
//		{
////			// FAULT #14 - HH 28/7/14 - Exit the loop half-way through
////			if (sim.getFault(14) == true) {
////				if (i == (roads.size()/2)) {
////					break;
////				}
////			}
//			
//			// HH 8.9.14 - Work out which road the car is on, so that we can compare the ids 
//			// - we are only interested in cars that are on the same road as the UGV
//			currentCar = (DumbCar) sim.cars.get(i);
//			currentDistanceCoord = checkForMovingObstacle(sim, currentCar, sensorLoc, sameLane);
//			
//			if (currentDistanceCoord.x > -1) {
//				currentDistance = location.distance(currentDistanceCoord.x, currentDistanceCoord.y);
//
//				if ( currentDistance < reqDistance )
//				{
//					reqDistance = currentDistance;
//					reqCoord = currentDistanceCoord;
//				}
//			}
//
//		}
//		
//		return reqCoord; // Need to do a check on return value as if this returns a value greater
//							// than the maximum sensor range then it denotes *no obstacle*
//	}
	
	/** 
	 * HH 21.8.14 - Work out whether the UGV has driven past the waypoint (in the direction of travel) i.e.
	 * overshot it (because it couldn't turn fast enough etc) - log this in the InfoLog, but not as an Accident
	 * - an accident will be logged if the overshoot has resulted in a collision.
	 * 
	 * @param Double2D: waypoint location to be checked
	 * @param double: bearing on which UGV is travelling
	 * @return boolean: true if the current UGV location has overshot the waypoint, assuming it has 
	 *                  been travelling along the bearing
	 */
	private boolean overshotWaypoint(Double2D WPlocation, double bearing)
	{
		UGV_Direction direction = Utility.getDirection(bearing);
		
		switch (direction) {
		
			case NORTH : 
				if (location.y < WPlocation.y) {return true;}
				break;
			case EAST : 
				if (location.x > WPlocation.x) {return true;}
				break;
			case SOUTH : 
				if (location.y > WPlocation.y) {return true;}
				break;
			case WEST : 
				if (location.x < WPlocation.x) {return true;}
				break;
		}	
		
		return false;
	}

	// HH 25.8.14 - Returns true if UGV currently engaged in overtaking manouevre
	public boolean isOvertaking() {
		
		if (overtakeStage != OvertakeStage.NOT_OVERTAKING) {
			return true;
		} else {
			return false;
		}
	}
		
	// HH 16.12.14 - Returns true if UGV currently engaged in uturn manouevre
	public boolean isUTurning() {
		
		Bag everything = sim.environment.getAllObjects();
		Entity e;
		Entity eTarget = new Entity(-1, TOTHER); //this id for the target is illegal, to get ids one should use COModel.getNewID()
						
		// Find the target from the bag of all entities (is probably item 0)
		for(int i = 0; i < everything.size(); i++)
		{
			e = (Entity) everything.get(i);			
			if (e.getID() == this.getTargetID())
			{
				eTarget =  e;
			}
		}
		
		if (eTarget.getType() == TUTURNWP ) {
			return true;
		} else {
			return false;
		}
	}
	/**
	 * HH 24.9.14 - method which returns a (square) Rectangle2D representing the UGV (location is at front of vehicle)
	 */
	public Shape getShape()
	{
		// The location is the centre of this shape which we will assume is of the size given in
		// the Constants file for an obstacle
		double widthOffset = Constants.UGV_WIDTH/2;
			
		// NOTE - this needs to take into account the orientation of the moving obstacle as
		// this will affect which direction to apply the widthOffset and lengthOffset in (NB. Just 
		// using the UGV.getDirection static method for converting bearing to compass direction)
		//return new Rectangle2D.Double(location.x-widthOffset, location.y-widthOffset, Constants.UGV_WIDTH, Constants.UGV_WIDTH);
		
		// 24.9.14 Return a shape aligned with the oriented vehicles
		Rectangle2D.Double carRectangle = new Rectangle2D.Double();
		
		// HH 24.9.14 - Assume the basic shape is as it would appear when pointed along the x-axis, so this means some swapping around of width/length
		//carRectangle = new Rectangle2D.Double(location.x - widthOffset, location.y - widthOffset, Constants.UGV_WIDTH, Constants.UGV_WIDTH);		
		// HH 8.10.14 - Updated the above so that the UGV getShape returns an object with the 'location' at the front, rather than in centre
		carRectangle = new Rectangle2D.Double(location.x - Constants.UGV_WIDTH, location.y - widthOffset, Constants.UGV_WIDTH, Constants.UGV_WIDTH);		
		AffineTransform rotateTransform = AffineTransform.getRotateInstance(((Car) this).orientation2D(), location.x, location.y);
		Shape carShape = rotateTransform.createTransformedShape(carRectangle);
		
		return carShape;
	}	

	/**
	 * HH 13/10/14 - method which returns a (square) Rectangle2D representing the UGV and centred at supplied location
	 * and with supplied Orientation (which should be using the same reference frame as Orientation2D would) 
	 */
	public Shape getShapeAtOffset(Double2D inLocation, double inOrientation2D)
	{
		// The location is the centre of this shape which we will assume is of the size given in
		// the Constants file for an obstacle
		double widthOffset = Constants.UGV_WIDTH/2;
			
		// 24.9.14 Return a shape aligned with the oriented vehicles
		Rectangle2D.Double carRectangle = new Rectangle2D.Double();
		
		// Assume the basic shape is as it would appear when pointed along the x-axis, so this means some swapping around of width/length
		// - returns an object with the 'location' at the front, rather than in centre
		carRectangle = new Rectangle2D.Double(inLocation.x - Constants.UGV_WIDTH, inLocation.y - widthOffset, Constants.UGV_WIDTH, Constants.UGV_WIDTH);		
		AffineTransform rotateTransform = AffineTransform.getRotateInstance(inOrientation2D, inLocation.x, inLocation.y);
		Shape carShape = rotateTransform.createTransformedShape(carRectangle);
		
		return carShape;
	}	
	
	/**
	 * HH 24.9.14 method which returns true or false if a provided coordinate is in the shape
	 */
	public boolean inShape(Double2D coord)
	{
		Shape carShape = getShape();		
		return carShape.contains(coord.x, coord.y);
	}
	
	/**
	 * HH 12.12.14 - Check for any moving obstacles within a 3m radius of the centre of the UGV/Car
	 *  
	 * @param sim
	 * @return boolean
	 */
	protected boolean checkAllMovingObsCircle(COModel sim, Bag inCars, double inSensitivity)
	{
		// Init to the values we would want for finding the min
		double range = Constants.UGV_WIDTH/2 + 1; // Approx 1m around the edge of the UGV (closer at corners)
		boolean obsFound = false;
		
		// Assume sensor is located in the centre of the vehicle
		Car currentCar;
		
		// Look through all the moving obstacles (cars) and look for intersection
		for(int i = 0; i < inCars.size(); i++)
		{
			currentCar = (Car) inCars.get(i);
			if (currentCar.isActive == true && (currentCar.ID != this.ID))
			{
				obsFound = checkForMovingObsCircle(sim, currentCar, range, inSensitivity);

				if (obsFound == true)
				{
					return obsFound; // No need to keep searching
				}
			}
		}
		return obsFound; 
	}
	
	/*
	 *  HH 12.12.14 - Search around the centre of the UGV at the specified range using 
	 *  range sensitivity supplied, and default angle sensitivity.  Return true if a 
	 *  moving obstacle is found during this search
	 *  
	 *  @param sim
	 *  @param inCar - the vehicle that has been detected as being on the same road
	 *  @return boolean - is an obstacle found in range
	 */
	
	private boolean checkForMovingObsCircle(COModel sim, Car inCar, double inRange, double inSensitivity) {
		
		//simple and dirty method which checks the coordinates between 0 and 
		//the supplied range away from the target in certain increments and see 
		//if they intersect with the supplied car (moving obs) 
		MutableDouble2D testCoord = new MutableDouble2D();
		Double2D amountAdd = new Double2D();
				
		// For this search, we don't care whether the obstacle is in our lane, or whether it is in front of 
		// us or behind us. 
		
		// For each angle that the sensor is able to view, turning in realistic increments
		double resolution = 0.5;
			
		// Check all angles from South all the way around
		for(double i = 0; i < 360; i += resolution)
		{
			// Reset the location that we start testing from to be the location of the UGV and set the bearing
			// that we are going to use for this iteration
			testCoord.setTo(0,0);
			testCoord.addIn(location); // Sensor assumed to be at centre of vehicle for 360 deg scan
			
			// Construct the x an y increments for each iteration below
			amountAdd = new Double2D(Utility.xMovement(i, inSensitivity), Utility.yMovement(i, inSensitivity));
												
		    // NOTE - j is not actually used, it just ensures the correct number of iterations
			for(double j = 0; j <= inRange; j += inSensitivity){
												
				testCoord.addIn(amountAdd);  // move the test location outwards on the chosen bearing
											
				boolean isInShape = false;
				if (inCar.type == DUMBCAR) 
				{
					isInShape = ((DumbCar) inCar).inShape(new Double2D(testCoord.x, testCoord.y));
				} else { // must be a UGV
					isInShape = ((UGV) inCar).inShape(new Double2D(testCoord.x, testCoord.y));
				}
										
				if (isInShape == true)
				{
					return true; // Can exit as we have found an intersection
				}
			}
		}
		
		return false;
	}
	
	/*
	 * 12.12.14 HH CalcStoppingDistance method to determine the distance to the nearest vehicle in front
	 * 
	 * @return double distance to nearest vehicle in front
	 */

}


