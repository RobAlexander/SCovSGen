package modeling;

import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.Area;
import java.awt.geom.Rectangle2D;

import modeling.COModel.jctExitDirInfo;
import modeling.COModel.jctPairInfo;
import sim.engine.SimState;
import sim.field.continuous.Continuous2D;
import sim.util.Bag;
import sim.util.Double2D;
import sim.util.MutableDouble2D;

/**
 * Derived class created to provide automated road-driving functionality over existing Car class.
 * This vehicle uses road markings to determine its location on the road, and vision/lidar sensors to 
 * detect static and moving obstacles on the road ahead.
 * 
 * @author HH
 */
public class UGV extends Car {

	private static final long serialVersionUID = 1L;
	
	private final double sensitivityForRoadTracking = 0.5;
	private double UGVViewingRange = 10;
	private double UGVViewingAngle = 90;
	
	// Obstacle Detection Params (wide angle, short range)
	private double UGVObsViewingRange = 25;
	private double UGVObsViewingAngle = 180; // Wider range so can detect obstacle length better.
	
	// Moving Obstacle Detection Params (narrow angle, long range)
	private double UGVMovObsViewingRange = 100;
	private double UGVMovObsViewingAngle = 30; // We will assume that the sensor is placed on the offside front corner of the vehicle
											   // so this should provide sufficient search breadth to cover most of the road (>6m away)
	
	private int[][] junctionHistory; // Store visit count for each junction index and direction to show where we have already been
	private Entity finalTarget;
	private boolean targetFound = false;
	
	private OvertakeStage overtakeStage = OvertakeStage.NOT_OVERTAKING;
	
	// variable for tracking IN-RUN situation metrics
	private int prevJct = -1;
	private Double2D prevJctLoc;
	
	private int UTurnTargetDir = -1; // Store the target direction in degrees when we are in a UTurn so that we don't overshoot it
	
	/**
	 * Construct the UGV, initialising with the start direction/bearing, target ID, and various other
	 * elements to enable to the UGV to function correctly, including access to the COModel for fault
	 * seeding/tracking, and noJcts for setting up the junctionHistory array.
	 * @param idNo (int - unique identifier for UGV object)
	 * @param idTarget (int - unique identifier for the Target object which the UGV is aiming for)
	 * @param performance (CarPerformance - performance characteristics for UGV e.g. maxSpeed)
	 * @param bearing (double - initial direction in which the UGV will travel)
	 * @param noJcts (int - number of junctions in the simulation environment)
	 * @param sim (COModel - access to the simulation environment)
	 */
	public UGV(int idNo, int idTarget, CarPerformance performance, double bearing, int noJcts, COModel sim) {
		super(idNo, idTarget, performance, bearing, TUGV);
				
		// Create and clear the junctionHistory array.  This can be used to ensure that the UGV
		// explores the map space in a more systematic way, rather than always taking the same exit 
		// (e.g. to bring it closer to the Target), or compared to a completely random approach.
		// The junction selection mechanism is explained in detail in Junction.getJunctionExit() 
		int jIdx = 0;
		junctionHistory = new int[noJcts][UGV_Direction.values().length];
		
		for (int i = 0; i < noJcts; i++)
		{
			// Loop for compass directions, using correct Enum index, and init to false
			for (UGV_Direction j : UGV_Direction.values()) {
				
				jIdx = j.ordinal();
				junctionHistory[i][jIdx] = 0; // A zero indicates that this approach has not been visited
			}
		}
		
		finalTarget = new Entity(-1, TOTHER); // Invalid type, this field is set at the beginning of the step routine
		targetFound = false; // Used in logging to report whether the target has been found
	}
		
	/**
	 * Based on modeling.Car.step (Robert Lee) - functionality changed to force vehicle to restrict movement to
	 * remain on road network.
	 * (non-Javadoc)
	 * @see modeling.Car#step(sim.engine.SimState)
	 * @param state (SimState - access to the simulation environment)
	 */
	@Override
	public void step(SimState state)
	{
		// Set outside of the main loop as is used to call the dealWithTermination code below
		sim = (COModel) state;
		
		if(this.isActive == true)
		{
			resetSpeedParams(); // Reset these for this step
			
			Continuous2D environment = sim.environment;
			Double2D me = environment.getObjectLocation(this);
						
			MutableDouble2D sumForces = new MutableDouble2D(); // Record the changes to be made to the location of the car

			// Store the previous location now, before we do anything with it
			storePrevLoc(me);
			
			double moveV; // Vertical component of the cars movement
			double moveH; // Horizontal component of the cars movement
			
			// Get location of target
			Bag everything = environment.getAllObjects(); // This will get all of the objects in the world, then start filtering :)
			
			this.setStats(sim.getCarMaxSpeed(), sim.getCarMaxAcceleration(), sim.getCarMaxDecceleration(), sim.getCarMaxTurning());
	        
			Entity e;		
			Entity eTarget = new Entity(-1, TOTHER); // This id for the target is illegal
							
			// Find the target from the bag of all entities
			for(int i = 0; i < everything.size(); i++)
			{
				e = (Entity) everything.get(i);			
				if (e.getID() == this.getTargetID())
				{
					eTarget = e;
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
			
			// Check to see if we have recently started to leave a junction, and are just
			// waiting for the 'back-end' to vacate the junction so we can mark it as cleared.
			if (getJctID() > 0)
			{
				// Check that we have started to leave the junction (i.e. that front of UGV has left junction)
				if (sim.junctionAtPoint(me, sim.junctions) == 0 || sim.junctionAtPoint(me, sim.junctions) != this.getJctID())
				{
					// See if we have entirely left the junction
					if (sim.junctionAtArea(new Area(this.getShape()), sim.junctions) <= 0) // ...or in a different jct
					{
						// We've left the junction, so reset the junction occupancy so someone else can enter
						sim.unOccupyJunction(getJctID(), sim.junctions, this.getID());
						setJctID(0);
						
						UTurnTargetDir = -1; // Clear the targetDirection flag too
					}
				}
			}			
			
			// Work out the stopping distance for the vehicle based on speed and maxDeceleratione
			double stoppingDistance = getSpeed(); 
			double tempSpeed = getSpeed();
			
			while (tempSpeed > 0) {
				stoppingDistance = stoppingDistance + tempSpeed; 
				tempSpeed = tempSpeed - getStats().getCurrentMaxDecel(); // Being extra cautious, assume hasn't started to decelerate on this step
			}
			
			stoppingDistance = Math.max(stoppingDistance, Constants.MIN_STOPPINGDISTANCE); // Make sure we are at least above a minimum distance
						
			// Regardless of what we are doing, if we are travelling at a slow enough speed, and
			// moving looks likely to cause a collision, then we should stop immediately.  As the speed threshold
			// is set at the maxDeceleration, we aren't breaking any motion laws to just stop entirely.
			if (getSpeed() <= getStats().getCurrentMaxDecel())
			{
				// Check the area around the UGV to ensure that there is no overlap.  Method will be to 
				// search around the UGV shape in all directions and check for overlapping with any
				// Moving obstacles.  If any overlap is detected, the vehicle should stop immediately.  To try 
				// and simulate sensing appropriately, we will check outwards in a circle from the centre of the
				// UGV to a range of 3m from the centre.
				if (checkAllMovingObsCircle(sim, sim.cars, sensitivityForRoadTracking) == true)
				{
					emergencyStop(); // Actually need to set speed to zero to avoid crashes registering even 'tho we're not going to move
					return; // No need to continue with Step, we're just going to stay still!
				}
			}
			
			// Check whether we have just entered a junction, or junction approach.  If so, process the various 
			// options for decreasing speed, waiting at an occupied junction, or choosing a junction exit point.
			boolean inJunctionApproach = false;
			
			// Check to see whether we are already executing a turning manoeuvre, if so, 
			// don't need to check the junctions as it's immaterial until we have finished the turn.
			if (eTarget.getType() != TUTURNWP)
			{
				for(int i = 0; i < sim.junctions.size(); i++) 
				{
					// New Fault #7 - Exit the loop half-way through
					if (sim.getFault(7) == true) {
						if (i == (sim.junctions.size()/2)) {
							break;
						}
						sim.setFault(7);
					}
										
					// ARE WE INSIDE A JUNCTION (i.e. is the front of the UGV inside the junction)?
					if (((Junction) sim.junctions.get(i)).inShape(me)) {
								
						// Vehicle currently within the junction, ensure that we are checking whether we need a new waypoint to redirect
						// towards the destination
						jctExitDirInfo junctionDirInfo = ((Junction) sim.junctions.get(i)).getJunctionExit(finalTarget.getLocation(), this, i, sim);
						Double2D junctionWP = junctionDirInfo.exitWP;
						UTurnTargetDir = junctionDirInfo.direction; // Store the direction
						
						// Check on the return value, if we don't get a valid WP back, we haven't
						// succeeded in entering the junction.  We need to slow down to zero (perhaps log an emergency
						// stop if that exceeds the maximum deceleration of the vehicle), and we shouldn't set 
						// the inJunctionFlag.  ALSO, make sure that we can't execute any movements by following the 
						// methods which follow - may need to set a new mode for TWAIT.
						if (junctionWP.x == -1 && junctionWP.y == -1)
						{
							// Something has gone wrong, an exit has not been chosen - maybe the junction is
							// already occupied
							// TODO - do we need to set a flag, or stop the vehicle
							double overshoot = emergencyStop();
							
							// NOTE: An emergencyStop command will set the speed to zero immediately.  If the UGV
							// has not decelerate appropriately on the approach to the junction, this command may
							// result in it reducing speed at a rate faster than maxDeceleration.  This might be a
							// contravention of the vehicle physics that we are trying to replicate.
							if (overshoot > 0) {
								sim.infoLog.addLog("Step: " + sim.schedule.getSteps() + ", Car: " + this.getID() + " would not have stopped in time, excess speed = " + 
										overshoot + " in junction " + ((Junction)sim.junctions.get(i)).getID() + 
							           ", at speed: " + this.getSpeed() + ", bearing: " + this.getDirection() + ".");
							}
							
							startWaiting(); // Go into waiting mode
							
							// To try and see why vehicles are entering occupied junctions
							sim.infoLog.addLog("Step: " + sim.schedule.getSteps() + ", UGV is waiting at junction with the following...");
							
						// The turning WP location has been chosen successfully, now create the actual WP, and complete
						// housekeeping associated with the junctionArray/IN-RUN situation metrics
						} else {
							// Make sure the WP is on the road surface
							if (ptOnRoad(sim.roads, junctionWP) == true) 
							{
								// Execute generic method to create and return the new waypoint
								eTarget = createWaypoint(junctionWP, sim, me, TUTURNWP, false, null); //set eTarget to be new WP
								
								// As we are entering a junction, compute the metrics from the stretch of road we've
								// just passed along...
								// Store the combination of prevJct and thisJct so that we can use it to calculate IN_RUN
								// Situation Metrics.  Note that in contrast to the junctionHistory array, the indexing
								// here does use the ID of the junctions.  This is because the junctions are amongst the
								// first entities to be added to the simulation, and therefore they have the lowest ID
								// numbers.  Roads are also added at the same time, so IDs are likely to be allocated to 
								// Roads/Junctions in turn and as a result, the junctionArray will be sparsely populated.
								
								// Get the ID of the junction that we are currently looping through; get it's location
								int currentJct = ((Junction)sim.junctions.get(i)).getID();
								Double2D currentLoc = ((Junction)sim.junctions.get(i)).getLocation();
								
								// If we've got a valid previous junction, store the locations of the previous
								// and current junctions in the junctionArray and update the previous junction
								// information for the next iteration.  In COModel.HgetIRJunctionSep(), this array
								// is used to calculate the distribution (in terms of length) of the sections of 
								// road that are traversed by the UGV.  It does not track how many times each road
								// section is traversed, only that is was completed at least once.
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
							
							goSlow(); // We're in a junction now, so vote for a slow speed.
						}

						// To try and work out why vehicles are leaving the road during turns, create
						// a log of the speed, location, and bearing of the UGV when it enters a junction
						sim.infoLog.addLog("Step: " + sim.schedule.getSteps() + ", UGV (" + this.getID() + ") = (" + me.x + "," + me.y + "), bearing = " +
								this.getDirection() + " : " + Utility.getDirection(this.getDirection()) + ", speed = " + this.getSpeed() + 
								". Attempting to enter Junction #" + ((Junction)sim.junctions.get(i)).getID() + 
								", currently occupied by ID #" + ((Junction)sim.junctions.get(i)).getOccupierID() + ".");
						
					// ARE WE INSIDE A JUNCTION APPROACH
					} else if (((Junction) sim.junctions.get(i)).inApproach(me)) {
						// Vehicle currently within the junction approach, slow down and maintain current direction
						goSlow(); 
						inJunctionApproach = true;
					}
				}
			}
			
			// The next thing we need to check is that we are not coming to the edge of the map, or the edge of
			// the road.  In either of these cases, we want to execute a U-turn.  This can be achieved in the same way as a U-turn
			// within a junction i.e. by inserting a waypoint at the same location in the adjacent lane (opposite direction) after
			// slowing the vehicle down almost to a stop (to allow it to make the tight turn).
			
			// Firstly make sure we are not about to reach the target, as that can confuse everything due to strange bearings etc.
			if (me.distance(finalTarget.getLocation()) >= 3 && eTarget.getType() != TUTURNWP && isWaiting() == false) // Check we haven't already set a WP to follow
			{
				// Are we about to leave the road, or hit the wall
				if (((nearlyOffRoad(sim.roads, me, this.getDirection()) == true) || (checkWallClose(this.getDirection()) == true)))
				{
					// The vehicle is going to leave the road or hit the wall if it remains on this bearing so slow down
					// to prepare for manoeuvre
					goSlow();

					// We only want to execute a Uturn if we are about to leave the road, or hit the wall
					if (checkWall() == true || nearlyOffRoad(sim.roads, me, this.getDirection()) == true) 
					{
						// We've run out of road or map
						Double2D uTurnWP = getUTurn(me, this.getDirection());

						// Create and return the new waypoint
						eTarget = createWaypoint(uTurnWP, sim, me, TUTURNWP, false, null); // Set eTarget to be new WP
					}
				}
			} 
			
			// Before we set a straight ahead, we need to check for any static obstacles on the path 
			// in front of us.  Only do this if we are not in a junction or a junction approach (as the map 
			// generator has been configured to prevent obstacles being added in these locations - similar to 
			// rules of the road and not parking near or in junctions).  Method looks for the closest obstacle
			// detection and the farthest obstacle detection within range.
			
			// TODO this might be considered a naive implementation as it won't respond very well to 
			// situations where other drivers have 'broken the rules'.
			// TODO *** REBUILD THIS OVERTAKING CODE ***  This code does not work properly, and during the code
			// commenting, many errors and inconsistencies were discovered.  One big issue is in the way that WP
			// are 'collected' from as far as 1.25m away (depending on speed) so the location of the UGV as it 
			// collects a WP is not a reliable estimate of a suitable offset from the obstacle e.g. for the 
			// overtake.  It might be better to be dynamically building up a picture of where the obstacles are
			// and what gaps (if any) there are between them.  Perhaps this should be executed once we are in
			// Overtake_Start as we'll have a better view.  
			
			if (eTarget.getType() != TUTURNWP && isWaiting() == false) { 
				
				// Need these outside of the if statement below
				Double2D maxDistanceToObsCoord;
				Double2D minDistanceToObsCoord;
				
				// New Fault #18 - A seedable fault to replicate a bug found in the code
				if (sim.getFault(18) == true) {
					// Look for an obstacle in front and evaluate the distance
					maxDistanceToObsCoord = this.checkAllObstacles(sim, this.getDirection(), true, 0); 
					minDistanceToObsCoord = this.checkAllObstacles(sim, this.getDirection(), false, 0); 
					sim.setFault(18);
				} else {
					// Try this with the /desired direction of travel/ instead as we can't see obstacles properly once
					// we have pulled out into the overtake as we are not looking in the direction of travel
					// anymore - (but our sensor could be)
					maxDistanceToObsCoord = this.checkAllObstacles(sim, Utility.getDirectionDeg(this.getDirection()), true, 0); 
					minDistanceToObsCoord = this.checkAllObstacles(sim, Utility.getDirectionDeg(this.getDirection()), false, 0); 
				}

				// If the maxDistanceToObs is going to suggest a large separation (such that the UGV would pull back in between
				// obstacles) we should do a quick check to make sure we don't have three in a row.  Limit search to ObsLen*2.5
				// (plus headway and speed) and see if anything is detected at close to this limit
				
				// TODO - There are potentially a few issues with this code.  checkAllObstacles is not designed to return locations
				// that are further away than the range (sometimes it can end up being very slightly outside the range), and so it
				// doesn't make sense to be testing for this.  Also, the range used for distToMiddleObs is not
				// actually all that different from UGBObsViewingRange anyway 7+12.5+2.5(max) = 22m cf 25m  Needs to be tested!
				// So... this code is *rarely* executed, and is unlikely to be telling us whether we have 'three in a row'
				boolean threeInARow = false;
				if (maxDistanceToObsCoord.x != -1 && me.distance(maxDistanceToObsCoord) > UGVObsViewingRange) {
					Double2D distToMiddleObs = this.checkAllObstacles(sim, Utility.getDirectionDeg(this.getDirection()), true, (Constants.OBSTACLE_HEADWAY + getSpeed() + Constants.OBSTACLE_LENGTH*2.5));
					if (distToMiddleObs.x != -1 && me.distance(distToMiddleObs) > (Constants.OBSTACLE_LENGTH + Constants.OBSTACLE_HEADWAY + getSpeed())) {
						threeInARow = true; // TODO - Check whether this code is ever executed!
					}
				}
				
				// Look for a moving obstacle in the next lane
				Double2D minDistanceToMovObsCoord = this.checkAllMovingObstacles(sim, sim.cars, false, UGVMovObsViewingAngle, 
						                                                         UGVMovObsViewingRange, sensitivityForRoadTracking);			
								
				// Check to see if we are already in a parked car manoeuvre
				if (eTarget.getType() == TPARKEDCAR) {				
				
					// Check to see if we are close to the waypoint (adjusted to account for 2.5m/step speed) i.e.
					// should we 'reach it' on this step?  ...or did we already pass it?
					if (me.distance(eTarget.getLocation()) <= (Math.max(1, getSpeed()/2)) || 
						overshotWaypoint(eTarget.getLocation(), getDirection()) == true) {
					
						// Work out whether this is the first waypoint or the second one?
						
						// Include check for where we have extended the START stage but may have passed the end of the Obs
						if (overtakeStage == OvertakeStage.OVERTAKE_START && maxDistanceToObsCoord.x != -1) {

							// This must be the first waypoint as we can still see the back of the 
							// parked vehicle with the sensor, so add the second waypoint and remove this one
							Double2D pCarWP = new Double2D(-1,-1);  // This will be set below
							
							// Need to check whether we are seeing the back of this vehicle, or some point
							// on a subsequent parked car.  For simplicity, just use some rough boundary conditions
							// We'll enter this if-statement when this obstacle looks like it's on its own, or 
							// when another obstacle is detected within the viewing range (unless fault seeded)
							// TODO - check the threeInARow code is functioning as required 
							if ((me.distance(maxDistanceToObsCoord) <= (Constants.OBSTACLE_LENGTH + 1) || threeInARow == true ||
								((me.distance(maxDistanceToObsCoord) <= UGVObsViewingRange && sim.getFault(16) == false))) && maxDistanceToObsCoord.x != -1) {
								
								// Use the location of the START waypoint to determine the location of the new 'pulled-out' waypoint
								// The offset within the lane of the current WP is used for the lane offset of the new WP, however
								// the farthest intersection with the obstacle is used for the distance into the lane.
								pCarWP = getOvertakeWP(eTarget.getLocation(), getDirection(), maxDistanceToObsCoord, OvertakeStage.OVERTAKE_PULLEDOUT);
								
								// If it possible that we haven't seen the end of the parked platoon, we shouldn't change
								// the overtake stage, we do this by looking for obstacle detections that are very close to the end of
								// our viewing range.
								// TODO - This might not be very a very sensible check as we are not taking into account the
								// possibility that there may be a large gap with no parked cars between the current one, and
								// the one that is detected at the limit - we would really want to pull back in here, but
								// instead we are remaining in Overtake_Start and setting a Overtake_PulledOut WP ahead
								if (Math.abs(me.distance(maxDistanceToObsCoord) - UGVObsViewingRange) < 2) {
									// Let's assume that there is more platoon outside of our viewing range, so we'll
									// leave the stage at _START so that we can execute this code again; log what we're
									// doing
									sim.infoLog.addLog("Step: " + sim.schedule.getSteps() + ", UGV = (" + me.x + "," + me.y + "), bearing = " +
											this.getDirection() + " : " + Utility.getDirection(this.getDirection()) + ", speed = " + this.getSpeed() + 
											". Remaining in overtake stage: START; obstacles may extend past range limit.");
									
								// We've just set a pulled out WP, so set our state to PulledOut
								} else {
									overtakeStage = OvertakeStage.OVERTAKE_PULLEDOUT;
								
									// Log to try and work out why vehicles are leaving the road during overtakes
									sim.infoLog.addLog("Step: " + sim.schedule.getSteps() + ", UGV = (" + me.x + "," + me.y + "), bearing = " +
										this.getDirection() + " : " + Utility.getDirection(this.getDirection()) + ", speed = " + this.getSpeed() + 
										". Entering overtake stage: PULLED OUT.");
								}
							
							// New Fault #16 - seeded fault to replace a bug that was found
							} else if (me.distance(maxDistanceToObsCoord) <= UGVObsViewingRange && sim.getFault(16) == true && maxDistanceToObsCoord.x != -1) {
								
								// Use the location of the START waypoint to determine the location of the new waypoint, but say
								// that we are creating a new start WP (and so displacing further into the opposite lane)
								pCarWP = getOvertakeWP(eTarget.getLocation(), getDirection(), maxDistanceToObsCoord, OvertakeStage.OVERTAKE_START);
								
								// Try and work out why vehicles are leaving the road during overtakes
								sim.infoLog.addLog("Step: " + sim.schedule.getSteps() + ", UGV = (" + me.x + "," + me.y + "), bearing = " +
										this.getDirection() + " : " + Utility.getDirection(this.getDirection()) + ", speed = " + this.getSpeed() + 
										". Extending overtake stage: START, another obstacle detected ahead.");
								
								// NOTE - keep overtakeStage as OVERTAKE_START
								sim.setFault(16); // Update fault called
								
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
								
								// The location of the START waypoint determines the offset from the kerb for the new waypoint, but
								// the tempObstaclePt above determines its displacement down the lane so this should leave to a
								// standard overtake manoeuvre.
								// TODO - Note that this is not very flexible to a vehicle parked on a curve in the road, or for obstacles of 
								// different lengths as it assumes a fixed obstacle length.
								pCarWP = getOvertakeWP(eTarget.getLocation(), getDirection(), tempObstaclePt, OvertakeStage.OVERTAKE_PULLEDOUT);
								overtakeStage = OvertakeStage.OVERTAKE_PULLEDOUT;
								
								// Try and work out why vehicles are leaving the road during overtakes
								sim.infoLog.addLog("Step: " + sim.schedule.getSteps() + ", UGV = (" + me.x + "," + me.y + "), bearing = " +
										this.getDirection() + " : " + Utility.getDirection(this.getDirection()) + ", speed = " + this.getSpeed() + 
										". Entering overtake stage: PULLED OUT, but obstacle detected ahead.");
							}
														
							// Create new WP as selected location, and point eTarget to this new WP
							eTarget = createWaypoint(pCarWP, sim, me, TPARKEDCAR, true, eTarget);			
							
						// If we are in Overtake_PulledOut, or we've managed to pass the end of the Obs without entering OVERTAKE_PULLEDOUT
						} else if (overtakeStage == OvertakeStage.OVERTAKE_PULLEDOUT || (overtakeStage == OvertakeStage.OVERTAKE_START && 
								                                                         maxDistanceToObsCoord.x == -1)) {
							
							// We may have finished the overtaking manoeuvre, but need to make sure we are tracking back to 
							// the appropriate offset from the kerb.
							// TODO - Here we are using the current location of the vehicle, rather than the current
							// WP to determine where to set the next WP (after offsetting back towards the kerb, and allowing
							// for the obstacle_headway distance after the obstacle) - this can be problematic if the UGV
							// has tracked too closely to the obstacle.  As previously mentioned, it would be better to build
							// up a dynamic image of the environment, including the road markings and obstacle outlines, and then
							// plot a sensibly path.  Should be some code below which will take the vehicle back to the right
							// offset from the kerb when it is no longer in an overtake.
							Double2D pCarWP = getOvertakeWP(me, getDirection(), new Double2D(Constants.OBSTACLE_HEADWAY, Constants.OBSTACLE_HEADWAY), OvertakeStage.OVERTAKE_FINISH);
							eTarget = createWaypoint(pCarWP, sim, me, TPARKEDCAR, true, eTarget); //set eTarget to be new WP		

							// Try and work out why vehicles are leaving the road during overtakes
							sim.infoLog.addLog("Step: " + sim.schedule.getSteps() + ", UGV = (" + me.x + "," + me.y + "), bearing = " +
									this.getDirection() + " : " + Utility.getDirection(this.getDirection()) + ", speed = " + this.getSpeed() + 
									". Entering overtake stage: FINISH.");
							
							overtakeStage = OvertakeStage.OVERTAKE_FINISH;
							
							// Get the UGV to slow down as it pulls back in, as it needs to re-evaluate surroundings
							goSlow();
							
						} else if (overtakeStage == OvertakeStage.OVERTAKE_FINISH) {

							// Try and work out why vehicles are leaving the road during overtakes
							sim.infoLog.addLog("Step: " + sim.schedule.getSteps() + ", UGV = (" + me.x + "," + me.y + "), bearing = " +
									this.getDirection() + " : " + Utility.getDirection(this.getDirection()) + ", speed = " + this.getSpeed() + 
									". Entering overtake stage: NOT OVERTAKING.");
							
							overtakeStage = OvertakeStage.NOT_OVERTAKING;
							
							// 'Eat' the waypoint and get back the 'real' target!
							setTargetID(((Waypoint) eTarget).getNextPoint());
							environment.remove(eTarget);
						}
					
					// New Fault #17 - forget to keep turning towards the WPs when we are executing an 
					// overtake (this can prevent the UGV from achieving the right trajectory)
					} else if (sim.getFault(17) == false) {
						// We want to make sure that the UGV is continuing to turn towards its WP
						setDirection(me, eTarget.getLocation());
						
						sim.infoLog.addLog("Step: " + sim.schedule.getSteps() + ", UGV = (" + me.x + "," + me.y + "), bearing = " +
								this.getDirection() + " : " + Utility.getDirection(this.getDirection()) + ", speed = " + this.getSpeed() + 
								". Continuing in current overtaking stage: " + overtakeStage + "."); 
						
						// Ensure the UGV continues to slow down as it pulls back in, as it needs to re-evaluate surroundings
						if (overtakeStage == OvertakeStage.OVERTAKE_FINISH)
						{
							goSlow();
						}
						
					} else {
						sim.setFault(17); // Call above must not have happened, so log the fault
					}
				} 
				
				// If we are not overtaking at the moment, then look for any new obstacles ahead.
				// Separated this out so can also be run by a vehicle pulling back in after an overtake
				if (eTarget.getType() != TPARKEDCAR || overtakeStage == OvertakeStage.NOT_OVERTAKING) 
				{
					
					// If an obstacle has been detected within the viewing range
					if (minDistanceToObsCoord.x > 0 && me.distance(minDistanceToObsCoord) <= UGVObsViewingRange) 
					{
						// See if we can overtake yet? (this will depend on the presence of an oncoming vehicle - we
						// checked for this earlier), we don't start the manoeuvre until we are at the right distance from the object
						// Need to take into account that we need the oncoming vehicle to pass the 
						// UGV before the UGV starts its manoeuvre to ensure that there is
						// no collision.  Previously we have actually just been checking to ensure that the
						// UGV completes its manoeuvre before the oncoming car reaches the UGV starting
						// location - this is a bit meaningless, and could easily lead to a collision
						
						// Work out the distance to the closest oncoming vehicle
						double minDistanceToMovObs = UGVMovObsViewingRange * 2;
						if (minDistanceToMovObsCoord.x > 0) // Make sure that we've actually found an obstacle in range before we use the value
						{
							minDistanceToMovObs = location.distance(minDistanceToMovObsCoord);
						}
						
						// Work out the distance between the oncoming car, and the point at which the UGV
						// should have pulled in after the manoeuvre
						// TODO Note that this only really works if the static obstacle is wholly within the
						// viewing range of the UGV sensor, otherwise we don't reall know how far the overtake
						// distance needs to be.
						double distMovObsToEndOvertake = UGVMovObsViewingRange * 2;
						if (maxDistanceToObsCoord.x > 0) {
							distMovObsToEndOvertake = minDistanceToMovObs - me.distance(maxDistanceToObsCoord) - Constants.OBSTACLE_HEADWAY;
						} else {
							// We'll just have to estimate this distance based on the minimum distance (which we've already
							// checked is a valid value).
							distMovObsToEndOvertake = minDistanceToMovObs - me.distance(minDistanceToObsCoord) - Constants.OBSTACLE_LENGTH - Constants.OBSTACLE_HEADWAY;
						}
						
						// Work out how long the oncoming vehicle will take to reach the point where the overtake ends;
						// Work out how long the UGV is likely to take to reach the end of the overtake.  
						
						// If the UGV is going to be quicker, then we can overtake.
						if ((distMovObsToEndOvertake / Car.CAR_SPEED) > (getAvgManoeuvreTime(me.distance(maxDistanceToObsCoord))))
						{ 
							// Make sure we are close enough to the parked car to begin the manoeuvre
							if (me.distance(minDistanceToObsCoord) <= (Constants.OBSTACLE_HEADWAY + getSpeed())) 
							{
								// Insert a waypoint adjacent to the start of the obstacle, but offset from the kerb by the 
								// required amount, using the current lane location of the UGV as a guide
								Double2D pCarWP = getOvertakeWP(me, getDirection(), minDistanceToObsCoord, OvertakeStage.OVERTAKE_START);
								eTarget = createWaypoint(pCarWP, sim, me, TPARKEDCAR, true, eTarget); //set eTarget to be new WP	
								
								// Try and work out why vehicles are leaving the road during overtakes
								sim.infoLog.addLog("Step: " + sim.schedule.getSteps() + ", UGV = (" + me.x + "," + me.y + "), bearing = " +
										this.getDirection() + " : " + Utility.getDirection(this.getDirection()) + ", speed = " + this.getSpeed() + 
										". Entering overtake stage: START.");
								
								overtakeStage = OvertakeStage.OVERTAKE_START; // Change the stage so we know we've started the O/T
							}
						
						// The oncoming car is too close for us to overtake so we will have to wait
						} else {
							
							// Check to see if we are at the point where we have to actually stop to be able to complete
							// the manoeuvre safely
							if (me.distance(minDistanceToObsCoord) <= (Constants.OBSTACLE_HEADWAY + getSpeed()))
							{
								double overshoot = emergencyStop(); // NOTE as previous use of this method that may contravene laws of physics!
								
								if (overshoot > 0) {
									sim.infoLog.addLog("Step: " + sim.schedule.getSteps() + ", Car: " + this.getID() + " would not have stopped in time for overtake, excess speed = " + 
											overshoot + 
								           ", at speed: " + this.getSpeed() + ", bearing: " + this.getDirection() + ".");
								}
								
								// Make sure that we ignore any votes to speed up
								goSlowStop();
								
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
						
			// This should be a catch-all, and if we don't already have a turning WP set, we should try to 
			// set a 'straight ahead' one by searching for road markings.
			if (eTarget.getType() != TUTURNWP && (eTarget.getType() != TPARKEDCAR || overtakeStage == OvertakeStage.NOT_OVERTAKING) &&
					isWaiting() == false) { 
			
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
			}
			
			// Regardless of what is going on - if there is another moving vehicle on the road ahead, and within our 
			// stopping distance, then we need to slow down.
			// TODO: Maybe make this so that we slow down if we are going faster than the car in front - so maybe keep track of the 
			// distance to the car in front but should be allowed to keep travelling at the speed of the car in front, with a 
			// separation of about 2m if we are e.g. in a junction approach, or travelling at low speeds.
			// The calculation is outside of the if condition so that we can test for default return value
			
			// Change the viewing angle used if we are in a junction
			double tempViewAngle = UGVMovObsViewingAngle;
			if (eTarget.getType() == TUTURNWP) {
				tempViewAngle = 180; 
			} else {
				tempViewAngle = 45; 
			}
			
			Double2D tempMovObsLoc = checkAllMovingObstacles(sim, sim.cars, true, tempViewAngle, UGVMovObsViewingRange, sensitivityForRoadTracking);
			
			if (location.distance(tempMovObsLoc) < stoppingDistance && tempMovObsLoc.x != -1)
			{
				goSlowStop();
			} else {
				goFaster(false); // This is just a vote to speed up, in case the vehicle has got stuck
			}
			
			// We need to ensure that when a vehicle is supposed to be waiting at a junction, it must
			// not be allowed to *creep* forward 
			if (isWaiting() == true)
			{
				// This was a fault that was found during testing so it has been inserted as a seeded fault
				// so if the fault is active, we don't slow down in waiting mode, but in most cases (when fault inactive)
				// the UGV will slow to a stop in waiting mode.
				
				// New Fault #15 - Don't enforce the slow down to zero condition (can cause creep through jct)
				if (sim.getFault(15) == true) {
					// We're in the fault condition, so don't slow the vehicle down.
					sim.setFault(15);
				} else {
					goSlowStop(); // Make sure that we actually slow down to 0
				}
			}
			
			// TODO - Fix this, bit of a botch to enforce termination as waypoint locations mean that the vehicle
			// is missing the target, in order to stay on the road so we'll check to see if we are near to the
			// 'actual' target, and if so, we'll throw away the waypoint chain.
			
			// A little pre-processing so we don't get a null ptr exception in the loop
			double tempDist = 0;
			if (eTarget.getID() == -1)
			{
				tempDist = 4; // Set higher than the threshold below
				
				// Let's at least set the target to point to the final target
				eTarget = finalTarget;
				setTargetID(finalTarget.getID()); 
				
			} else {
				tempDist = me.distance(eTarget.getLocation());
			}
			
			if (me.distance( (finalTarget).getLocation()) < 4.5) 
			{
				// We don't really want vehicles straying towards targets in the other lane as 
				// can produce some weird behaviour.  However, we'll retain this as a seedable fault
				
				// New Fault #14 - Forget to check if the target is in the same lane as the UGV
				if (sim.getFault(14) == true || 
					(checkSameLane(sim, finalTarget.getLocation(), UGVViewingRange, UGVViewingAngle, sensitivityForRoadTracking, getDirection())))
				{
					eTarget = finalTarget;
					setTargetID(finalTarget.getID()); // If we don't set the ID too then we end up setting tempDist to 4 on next iter
					setDirection(me, eTarget.getLocation());
					
					// Added this test in case vehicle is added next to a waypoint
					if (getSpeed() >= 1) {
						goSlow(); 
					} else if (getSpeed() < 1) {
						goFaster(false); 
					}	
					
					if (sim.getFault(14) == true) {
						sim.setFault(14); // If the fault was set, record that!
					}
				}
			// If we are not near the target, and we are not turning, or in a junction/junction Approach
			// Or we are taking part in an overtaking manoeuvre (not waiting) And we are not in waiting mode at
			// a junction, then it's fine to request a tentative speed up.
			} else if (((tempDist > 3 && eTarget.getType() != TUTURNWP && inJunctionApproach == false) ||
					   (eTarget.getType() == TPARKEDCAR && overtakeStage != OvertakeStage.WAIT)) && isWaiting() == false) { 

				goFaster(false);

			// As long as we are not waiting at a junction
			} else if (isWaiting() == false) {
									
				// For regular waypoints, and junction manoeuvres, make sure we slow down if
				// necessary, and keep turning towards the waypoint (limited by the desired direction of 
				// travel, in the case of a junction turn)
				if (eTarget.getType() == TWAYPOINT || eTarget.getType() == TUTURNWP) 
				{	
					if (eTarget.getType() == TUTURNWP) {
						goSlow();
						setDirection(me, eTarget.getLocation(), this.UTurnTargetDir);
					} else {
						setDirection(me, eTarget.getLocation());
					}
				}
			} 
			
			// If we are really close to our target, whether it is a waypoint or a real target, we may want to
			// finish the run, or 'eat' the target and get the next one.
			if (tempDist < 1.5)
			{
				if (eTarget.getID() == -1)
				{
					// TODO flag an error as -1 is an illegal id so at this point it can only be that there isn't
					//an existing target for the car
					this.isActive = false;
					
				} else {
					if (eTarget.getType() == TTARGET)
					{
						// Check to make sure the Target is in the same lane as the vehicle before we 'eat' the target
						// double inRange, double inAngle, double inSensitivity, double inBearing
						if (checkSameLane(sim, eTarget.location, UGVViewingRange, UGVViewingAngle, sensitivityForRoadTracking, getDirection()) == true) 
						{
							this.isActive = false;
							targetFound = true;
						}
						
					// For regular Waypoints, 'eat' the waypoint and get the next one
					} else if (eTarget.getType() == TWAYPOINT || eTarget.getType() == TUTURNWP) {
						// Get rid of wp and get new target ID
						setTargetID(((Waypoint) eTarget).getNextPoint());
						environment.remove(eTarget);
					}
				}			
			}		
						
			// Call the operations to calculate how much the car moves in the x
			// and y directions.

			// Request a slow down if we think we are going to overshoot the waypoint during an overtake
			if (getSpeed() >= tempDist && eTarget.getType() == TPARKEDCAR)
			{
				goSlow(); // Request a slow down before we do the speed calcs and move the UGV
			}
			
			// Sort out all the speed requirements, actually changes the speed of the vehicle
			doSpeedCalcs();			
			
			// Original methods for determining movement, work out displacement in each direction,
			// add to the current location, and then 'move' the UGV by changing its location.
			moveV = Utility.yMovement(getDirection(), getSpeed());
			moveH = Utility.xMovement(getDirection(), getSpeed());
			
			sumForces.zero();
			sumForces.addIn(new Double2D(moveH, moveV));	
	        sumForces.addIn(me);
			sim.environment.setObjectLocation(this, new Double2D(sumForces));
			this.setLocation( new Double2D(sumForces));
						
			location = new Double2D(sumForces); // TODO - Suspect this should be removed as probably just repeats the command above	
		}
		
		// Check to see if there are actually any agents left, or should we stop
		if(sim!=null)
		{
			sim.dealWithTermination();
		}
    }
		
	/** 
	 * Based on Car.checkWall (Robert Lee)
	 * 
	 *  Check the direction in which the vehicle is travelling and reports whether the 
	 *  vehicle is getting close (within UGVViewingRange) to the wall.
	 * @param double bearing (double - the direction the UGV is pointing in)
	 * @return boolean (true if getting too close to the wall (i.e. within vision))
	 */
	private boolean checkWallClose(double bearing)
	{
		Double2D me = this.location;
		UGV_Direction myDirection = Utility.getDirection(bearing);
		
		if(me.x <= UGVViewingRange && myDirection == UGV_Direction.WEST)
		{
			return true;
		}
		else if(Constants.WorldXVal - me.x <= UGVViewingRange && myDirection == UGV_Direction.EAST)
		{
			return true;
		}
		
		if (me.y <= UGVViewingRange && myDirection == UGV_Direction.NORTH)
		{
			return true;
		}
		else if(Constants.WorldYVal- me.y <= UGVViewingRange && myDirection == UGV_Direction.SOUTH)
		{
			return true;
		}
		
		return false;
	}
				
	/** 
	 * Work out a new waypoint location to allow a U-turn (based on current location and heading)
	 * should just be a 180 degree flip, but need to know whether this equates to a translation in the N, E, S
	 * or W direction.  Width of translation should be equivalent to roadWidth/2
	 * @param me (Double2D - current location of UGV)
	 * @param bearing (double - direction UGV is pointing in)
	 * @return Double2D (return location of waypoint to allow a U-turn)
	 */
	public Double2D getUTurn(Double2D me, double bearing)
	{
		UGV_Direction direction = Utility.getDirection(bearing);
		
		switch (direction) 
		{
			case NORTH : 
				return new Double2D(me.x+Road.roadWidth/2, me.y);
			case EAST : 
				return new Double2D(me.x, me.y+Road.roadWidth/2);
			case SOUTH : 
				return new Double2D(me.x-Road.roadWidth/2, me.y);
			case WEST : 
				return new Double2D(me.x, me.y-Road.roadWidth/2);
		}		

		// [TODO] Add error condition here!!
		return new Double2D(0,0);
	}
	
	/** 
	 * Work out a new waypoint location for parked car overtake (based on current location and heading)
	 * should be based on known obstacle parameters, but need to know whether this equates to a translation in the 
	 * N, E, S or W direction. The passed param distancePt will be the intersection with the obstacle and in the case
	 * of the beginning of an overtake will be then minimum intersection, for the second stage of overtake will be the 
	 * maximum intersection, and for the final stage will be a 'dummy' intersection of (-1,-1) as no intersection 
	 * should be detected here and the constant offset of OBSTACLE_HEADWAY should be used instead
	 * 
	 * @param me (Double2D - location of UGV)
	 * @param bearing (double - direction in which the UGV is pointing)
	 * @param distancePt (Double2D - coordinates of location we will use to help us site the waypoint) 
	 * @param isStage (OvertakeStage - which stage of the overtake are we in, determines relative location of waypoint)
	 * @return Double2D (location of waypoint to direct path of overtake)
	 */
	public Double2D getOvertakeWP(Double2D me, double bearing, Double2D distancePt, OvertakeStage inStage)
	{
		UGV_Direction direction = Utility.getDirection(bearing);
		
		// For the default case (OVERTAKE_START)
		// Work out the offset that should be used into the lane (added to the current vehicle position within lane)
		double laneOffset = Constants.OBSTACLE_WIDTH + 0.5; // Pull out to avoid the obstacle - 0.325m from kerb
		
		if (inStage == OvertakeStage.OVERTAKE_PULLEDOUT) {
			laneOffset = 0; // Remain at same offset, to avoid the obstacle
		} else if (inStage == OvertakeStage.OVERTAKE_FINISH) {
			laneOffset = -(Constants.OBSTACLE_WIDTH + 0.5); // NOTE - this is a negative offset! Return to original lane, obstacle passed
		}
			
		double distanceOffset;
		
		// Bring the waypoint slightly further away from the edge of the obstacle
		double obsOffset = Constants.OBSTACLE_BUFFER; 
		
		// We want to set the waypoints so they are relative to the 
		// desired location of the UGV, rather than relative to the current location of the
		// UGV - this should help to mitigate against cascading errors caused by the UGV being offset from
		// the WP at the point when it 'reaches' the WP.
		
		switch (direction) {

		case NORTH : 
			distanceOffset = distancePt.y;
			
			// Offset in direction of travel if required
			if (inStage == OvertakeStage.OVERTAKE_FINISH) {
				distanceOffset = me.y - OBSTACLE_HEADWAY; // Allow a buffer zone for pulling back in after O/T
			} else if (inStage == OvertakeStage.OVERTAKE_START) {
				distanceOffset = distanceOffset + obsOffset; // Nearer to UGV, further from Obstacle
			} else if (inStage == OvertakeStage.OVERTAKE_PULLEDOUT) {
				distanceOffset = distanceOffset - obsOffset; // Further from UGV, further from Obstacle
			}
			
			return new Double2D(me.x + laneOffset, distanceOffset); // Return the WP location
		case EAST : 

			distanceOffset = distancePt.x;
			
			// Offset in direction of travel if required
			if (inStage == OvertakeStage.OVERTAKE_FINISH) {
				distanceOffset = me.x + OBSTACLE_HEADWAY; // Allow a buffer zone for pulling back in after O/T
			} else if (inStage == OvertakeStage.OVERTAKE_START) {
				distanceOffset = distanceOffset - obsOffset; // Nearer to UGV, further from Obstacle
			} else if (inStage == OvertakeStage.OVERTAKE_PULLEDOUT) {
				distanceOffset = distanceOffset + obsOffset; // Further from UGV, further from Obstacle
			}
			
			return new Double2D(distanceOffset, me.y + laneOffset); // Return the WP location
		case SOUTH : 

			distanceOffset = distancePt.y;
			
			if (inStage == OvertakeStage.OVERTAKE_FINISH) {
				distanceOffset = me.y + OBSTACLE_HEADWAY; // Allow a buffer zone for pulling back in after O/T
			} else if (inStage == OvertakeStage.OVERTAKE_START) {
				distanceOffset = distanceOffset - obsOffset; // Nearer to UGV, further from Obstacle
			} else if (inStage == OvertakeStage.OVERTAKE_PULLEDOUT) {
				distanceOffset = distanceOffset + obsOffset; // Further from UGV, further from Obstacle
			}
			
			return new Double2D(me.x - laneOffset, distanceOffset); // Return the WP location
		case WEST : 

			distanceOffset = distancePt.x;
			
			if (inStage == OvertakeStage.OVERTAKE_FINISH) {
				distanceOffset = me.x - OBSTACLE_HEADWAY; // Allow a buffer zone for pulling back in after O/T
			} else if (inStage == OvertakeStage.OVERTAKE_START) {
				distanceOffset = distanceOffset + obsOffset; // Nearer to UGV, further from Obstacle
			} else if (inStage == OvertakeStage.OVERTAKE_PULLEDOUT) {
				distanceOffset = distanceOffset - obsOffset; // Further from UGV, further from Obstacle
			}
			
			return new Double2D(distanceOffset, me.y - laneOffset); // Return the WP location
		}	
				
		// [TODO] Add error condition here!!
		return new Double2D(0,0);
	}
	
	/**
	 * This method checks to see if the UGV has collided with the wall.  This is deemed to be the 
	 * case if the centre front of the vehicle comes within 20cm of any of the boundaries of the
	 * map.  In practice, this would mean overlapping the 'wall' by up to 180cm, depending upon the
	 * location and orientation of the UGV.  TODO review the relevance of this method, and also whether
	 * it is still appropriate now that the 'location' of the UGV is centre-front, rather than the middle
	 * of the vehicle?
	 * @return boolean (return true if the UGV is deemed too close to the wall, false otherwise)
	 */
	private boolean checkWall()
	{
		Double2D me = this.location;
		
		if(me.x <= 0.2 )
		{
			return true;
		}
		else if (Constants.WorldXVal - me.x <= 0.2)
		{
			return true;
		}
		
		if (me.y <= 0.2)
		{
			return true;
		}
		else if (Constants.WorldYVal - me.y <= 0.2)
		{
			return true;
		}
		
		return false;
	}
	
	/** 
	 * Work out whether the supplied location is about to leave one of the road surfaces
	 * if it was to continue on current bearing for 2m.
	 * @param roads (Bag - collection of roads in the simulation environment)
	 * @param location (Double2D - location to be checked)
	 * @param bearing (double - direction in which UGV is pointing)
	 * @return boolean (true if offset location will leave the road surface)
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
	 * This method returns the junction history array belonging to the UGV.
	 * @return int[][] (the 2D array of information about junction that have been visited)
	 */	
	public int[][] getJunctionHistory()
	{
		return junctionHistory;
	}
	
	/** 
	 * Update the 2D junction history array with information about a junction that has just been visited
	 * - note that the array is indexed by the Bag.get(idx) index, rather than the junction id, in order
	 * to reduce the size of the array.  This is only acceptable because junctions are static in the model, 
	 * they are not created or destroyed (as this could change the order in the bag).
	 * @param idx (int - the index used by Bag.get(idx) to retrieve the junction from junctions)
	 * @param direction (UGV_Direction - the compass direction for the junction exit we are visiting)
	 */
	public void updateJunctionHistory(int idx, UGV_Direction direction)
	{
		if (idx <= junctionHistory.length && direction.ordinal() <= junctionHistory[idx].length )
		{
			junctionHistory[idx][direction.ordinal()] ++; // Increase the count of visits to this exit
		}
	}
	
	/**
	 * This method returns the 'targetFound' field, so that the log methods can report whether the target was reached.
	 * @return boolean (true if the target was reached, false otherwise)
	 */
	public boolean getTargetFound() {
		return targetFound;
	}
	
	/**
	 *  Return the estimated number of steps required to complete the overtake manoeuvre, based
	 *  on various assumptions about obstacle length, headway and speed/acceleration.
	 *  @param distanceToObstacle (double - the distance to the closest part of the obstacle)
	 *  @return int (return the estimated number of steps/iterations to safely pass the object)
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
	
	/**
	 *  Return the coordinates of the nearest/farthest intersection with the supplied obstacle if it is found in the current lane.  
	 *  Search conducted at a range of up to 25m (default) or a lower supplied value (inRangeLimit).  Search progresses in 
	 *  a similar way to the roadMarkings search, and looks for the closest/farthest hit on an obstacle within a narrow range: 
	 *  Constants.UGVObsViewingAngle.  At each angle that is iterated through, the search begins close to the UGV sensor and
	 *  radiates outwards until it leaves the road, reaches an obstacle, or exceeds the range limit; this method is only
	 *  searching for obstacles on the same road and in the same lane.  Note: the coordinates returned are not necessarily
	 *  the closest/farthest 'edge' of the obstacle, as the search resolution means that the intersection can be detected
	 *  'part way through' (inside) an object. Once this first intersection is detected, the search will stop, so the 
	 *  method will not return any points which are further inside the obstacle (at least not at this bearing).
	 *  @param sim (COModel - access to the simulation environment)
	 *  @param bearing (double - the 'centre' direction for the sensor, should generally be the direction of the lane)
	 *  @param obstacle (Obstacle - the obstacle we wish to check for detection by the sensor)
	 *  @param getMax (boolean - true to return the farthest obstacle detection, false to return the nearest)
	 *  @param inRangeLimit (double - the range (distance) at which we want to stop the search, must be < UGVObsViewingRange
	 *                       or it will be ignored; set to 0 to use the default max range)
	 *  @return Double2D (return the coordinates of the farthest/nearest obstacle detection)
	 */
	private Double2D checkForObstacle(COModel sim, double bearing, Obstacle obstacle, boolean getMax, double inRangeLimit) {
		
		MutableDouble2D testCoord = new MutableDouble2D(); // The sensor test location
		Double2D amountAdd = new Double2D(); // Increment to replicate a sensor checking along a vector
		
		// If a range limit is supplied, update the internal variables
		double currentViewingRange = UGVObsViewingRange; // Default limit to viewing range
		if (inRangeLimit != 0 && inRangeLimit < UGVObsViewingRange) {
			currentViewingRange = inRangeLimit; // Custom supplied range limit
		}
		
		Double2D reqCoord = new Double2D(-1,-1);
		double distance;
		double reqDistance = UGVObsViewingRange; // as we want to to find an object closer than this...
		
		if (getMax == true) {
			reqDistance = 0; // as we want to find an obstacle further away than this..
		}
				
		// Need to restrict the obstacle checks to those which are in the same lane as
		// the UGV, so need to know own lane direction in order to restrict in method below
		UGV_Direction direction = Utility.getDirection(bearing);
		
		// For each angle that the sensor is able to view, turning in realistic increments
		double resolution = 0.5; // amount in degrees to increment the angle
		double newBearing = 0.0; // stores the current bearing that the sensor is searching on
		
		// Variables for the loop to support fault insertion
		double startAngle = -(UGVObsViewingAngle/2); // start to the left of centre
		double endAngle = (UGVObsViewingAngle / 2); // end to the right of centre
		double startRange = 0; // start from the sensor location
		double endRange = currentViewingRange; // end at the limit of the viewing range
		double rangeSensitivity = sensitivityForRoadTracking; // amount in metres to increment the sensor range
						
		// Use the road marking detection algorithm to find the nearside lane marking so that we can use it to work
		// out the centre line, and make sure that we only detect obstacles on the same side of the road as the UGV's 
		// direction of travel.
		// TODO Consider whether it would be more efficient (and sensible) to use findNearest here instead - we're only 
		// using this method to help us work out the location of the centre line so that we can check whether the 
		// obstacle is in the same lane as us
		Double2D furthestLaneMarking = locateRoadMarkings_AllRoads(sim.roads, false, sim, Constants.genLineType.NEARSIDE, 
				                                                   UGVViewingAngle , UGVViewingRange, sensitivityForRoadTracking);
		
		double centreOffset = Road.roadWidth/2 - Constants.ROADEDGINGWIDTH - Constants.ROADEDGEOFFSET; // Distance between edge line and centre
		
		// Loop for each angle (sweeping to the full distance range - see internal loop)
		for(double i = startAngle; i < endAngle; i += resolution)
		{
			// Reset the location that we start testing from to be the location of the UGV and set the bearing
			// that we are going to use for this iteration
			testCoord.setTo(0,0);
			testCoord.addIn(location); // Assumes that the sensor is in the front-centre of the vehicle
			newBearing = Utility.correctAngle(bearing + i); // Set the sensor angle
			
			// Construct the x an y increments for each iteration below
			amountAdd = new Double2D(Utility.xMovement(newBearing, rangeSensitivity), Utility.yMovement(newBearing, rangeSensitivity));
						
		    // Loop to full range of sensor vision at current angle
			// NOTE - j is not actually used, it just ensures the correct number of iterations
			for(double j = startRange; j < endRange; j += rangeSensitivity){
												
				testCoord.addIn(amountAdd);  // move the test location outwards on the chosen bearing
				
				// Ensure that the testCoord location is still on the road surface,
				// if not, we would expect that any vision algorithm would have noticed this
				// and stopped searching at this bearing (similar to stopping the search 
				// when we have found an obstacle (as we can't see through it!)
				if (sim.roadAtPoint(new Double2D(testCoord), sim.roads) == false) {
					break; // Don't search any further on this bearing
				}
				
				// Ensure that the test coordinate is in the same lane as the UGV
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
				
				// If it is in the same lane...
				if (inLane == true) {
					
					// See if the test coordinate is inside an obstacle
					if (((ParkedCar) obstacle).inShape(new Double2D(testCoord.x, testCoord.y)))
					{
						// Calculate the distance (from the sensor) at which the testCoord has intersected
						distance = location.distance(testCoord.x, testCoord.y);

						// See whether we have improved on the max/min distance that we are looking for
						// and store a new 'current best value' if we have
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

						// Exit the loop as we don't need to search any further on this bearing as we've found an obstacle
						break;
					}
				}
			}
		}
		
		return reqCoord;
	}

	/**
	 * Return the coordinates of the nearest/farthest intersection with any obstacle that is found in the current lane.  
	 * Search conducted at a range of up to 25m (default) or a lower supplied value (inRangeLimit).  Search progresses in 
	 * a similar way to the roadMarkings search, and looks for the closest/farthest hit on an obstacle within a narrow range: 
	 * Constants.UGVObsViewingAngle.  At each angle that is iterated through, the search begins close to the UGV sensor and
	 * radiates outwards until it leaves the road, reaches an obstacle, or exceeds the range limit; this method is only
	 * searching for obstacles on the same road and in the same lane. See above for more detail
	 * @param sim (COModel - access to the simulation environment)
	 * @param bearing (double - the 'centre' direction for the sensor, should generally be the direction of the lane)
	 * @param getMax (boolean - true to return the farthest point on an obstacle, false for the closest)
	 * @param inLimitRange (double - double - the range (distance) at which we want to stop the search, must be < UGVObsViewingRange
	 *                       or it will be ignored; set to 0 to use the default max range)
	 * @return Double2D (the coordinates of the farthest/nearest intersection with the obstacle)
	 */
	private Double2D checkAllObstacles(COModel sim, double bearing, boolean getMax, double inLimitRange)
	{
		// Init to the values we would want for finding the minimum distance to the obstacle
		double reqDistance = UGVObsViewingRange + 1; // want to find values smaller than this
		double currentDistance = UGVObsViewingRange + 1; // want to find values smaller than this
		Double2D currentDistanceCoord; 
		Double2D reqCoord = new Double2D(-1,-1);
		
		if (getMax == true) {
			reqDistance = 0; // want to find values bigger than this
			currentDistance = 0; // want to find values bigger than this
		}
		
		// It's not really acceptable to 'query the road' to find out what its ID is,
		// and then query the Obstacle to find out which road it is on.  Instead we just restrict our search for 
		// obstacles within our line of sight so that the search terminates as soon as the search point
		// leaves the road, or reaches an obstacle.  If the search terminates when it reaches the road then it 
		// would be impossible to detect obstacles which are on a different road (unless the roads are closer 
		// than the resolution of the search, which shouldn't be the case).
				
		// Look through all the obstacles and look for intersection
		for(int i = 0; i < sim.obstacles.size(); i++)
		{
			currentDistanceCoord = checkForObstacle(sim, bearing, (Obstacle) sim.obstacles.get(i), getMax, inLimitRange); 
			
			// If we have got a valid return value (i.e. an obstacle was found) the calculate the distance to that
			// obstacle
			if (currentDistanceCoord.x > -1) {
				currentDistance = location.distance(currentDistanceCoord.x, currentDistanceCoord.y);

				// If we are looking for the maximum distance, and the current one is greater than the one we 
				// have found so far, update the stored distance and coords
				if (getMax == true) {
					if ( currentDistance > reqDistance )
					{
						reqDistance = currentDistance;
						reqCoord = currentDistanceCoord;
					}					
				
				// If we are looking for the minimum distance, and the current one is shorter than the one we 
				// have found so far, update the stored distance and coords
				} else {
					if ( currentDistance < reqDistance )
					{
						reqDistance = currentDistance;
						reqCoord = currentDistanceCoord;
					}
				}
			}
		}
		
		return reqCoord; // TODO Need to do a check on return value as if this returns a value greater
						 // than the maximum sensor range (for getMin) or 0 for (getMax) then it denotes 
						 // *no obstacle*
	}
	
	/** 
	 * Work out whether the UGV has driven past the waypoint (in the direction of travel) i.e.
	 * overshot it (because it couldn't turn fast enough etc) - log this in the InfoLog, but not as an Accident
	 * - an accident will be logged if the overshoot has resulted in a collision.
	 * 
	 * @param WPlocation (Double2D - waypoint location to be checked)
	 * @param bearing (double - bearing in which the UGV is pointing)
	 * @return boolean  (true if the current UGV location has overshot the waypoint, assuming it has 
	 *                  been travelling along the bearing)
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

	/**
	 * Returns true if UGV currently engaged in overtaking manoeuvre.
	 * TODO - May want to do some testing to make sure that the UGV start mode
	 * is initialised to OvertakeStage.NOT_OVERTAKING in the constructor, and 
	 * reset at appropriate points through the simulation.
	 * @return boolean (return true if UGV is overtaking, else false)
	 */
	public boolean isOvertaking() {
		
		if (overtakeStage != OvertakeStage.NOT_OVERTAKING) {
			return true;
		} else {
			return false;
		}
	}
		
	/**
	 * Returns true if UGV currently engaged in uturn manouevre
	 * @return boolean (returns true if the UGV is currently turning, else false)
	 */
	public boolean isUTurning() {
		
		Bag everything = sim.environment.getAllObjects();
		Entity e;
		Entity eTarget = new Entity(-1, TOTHER); // This id for the target is illegal, to get ids one should use COModel.getNewID()
						
		// Find the target from the bag of all entities
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
	 * Method which returns a Shape object representing the UGV (location is at centre-front of vehicle)
	 * @return Shape (return a Shape object representing the vehicle footprint)
	 */
	public Shape getShape()
	{
		// The location is the front-centre of this shape which we will assume is of the size given in
		// the Constants file for an obstacle
		double widthOffset = Constants.UGV_WIDTH/2;
			
		// Return a shape aligned with the oriented vehicles
		Rectangle2D.Double carRectangle = new Rectangle2D.Double();
		
		// Assume the basic shape is as it would appear when pointed along the x-axis, so this means some swapping around of width/length
		// returns an object with the 'location' at the centre-front
		carRectangle = new Rectangle2D.Double(location.x - Constants.UGV_WIDTH, location.y - widthOffset, Constants.UGV_WIDTH, Constants.UGV_WIDTH);		
		AffineTransform rotateTransform = AffineTransform.getRotateInstance(((Car) this).orientation2D(), location.x, location.y);
		Shape carShape = rotateTransform.createTransformedShape(carRectangle);
		
		return carShape;
	}	

	/**
	 * Method which returns a Shape object representing the UGV (i.e. same size and shape) and centred at 
	 * the supplied location, using the supplied Orientation2D to mimic the UGV orientation.   It is called 
	 * by nearlyOffRoad which displaces the vehicle 2m in the direction of travel, in an attempt to see whether 
	 * it is about to leave the road.  This method provides the displaced shape.
	 * @param inLocation (Double2D - the 'new' centre-front location of the UGV )
	 * @param inOrientation2D (double - the orientation2D direction of travel of the UGV)
	 * @return Shape (returns a Shape object to represent the displaced UGV footprint)
	 */
	public Shape getShapeAtOffset(Double2D inLocation, double inOrientation2D)
	{
		// The location is the displaced front- centre of this shape which we will assume is of the size 
		// given in the Constants file for an obstacle
		double widthOffset = Constants.UGV_WIDTH/2;
			
		// Return a shape aligned with the oriented vehicles
		Rectangle2D.Double carRectangle = new Rectangle2D.Double();
		
		// Assume the basic shape is as it would appear when pointed along the x-axis, so this means some swapping around of width/length
		// - returns an object with the 'location' at the front-centre
		carRectangle = new Rectangle2D.Double(inLocation.x - Constants.UGV_WIDTH, inLocation.y - widthOffset, Constants.UGV_WIDTH, Constants.UGV_WIDTH);		
		AffineTransform rotateTransform = AffineTransform.getRotateInstance(inOrientation2D, inLocation.x, inLocation.y);
		Shape carShape = rotateTransform.createTransformedShape(carRectangle);
		
		return carShape;
	}	
	
	/**
	 * Method which returns true or false depending on whether a provided coordinate is in the shape
	 * @param coord (Double2D - coordinates of location to test for intersection with UGV shape)
	 * @return boolean (returns true if there is an intersection, false otherwise)
	 */
	public boolean inShape(Double2D coord)
	{
		Shape carShape = getShape();		
		return carShape.contains(coord.x, coord.y);
	}
	
	/**
	 * Check for any moving obstacles within a 1.5m radius of the centre of the UGV/Car
	 * TODO Check that this really is a 1.5m radius around the centre, and not using the location
	 * which is now at the front of the UGV
	 * @param sim (COModel - access to the simulation environment)
	 * @param inCars (Bag - access to the cars in the simulation)
	 * @param inSensitivity (double - range resolution to use to search)
	 * @return boolean (returns true if an obstacle is found in range, false otherwise)
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
					return obsFound; // No need to keep searching or we'll overwrite obsFound!
				}
			}
		}
		return obsFound; 
	}
	
	/**
	 *  Search around the centre of the UGV at the specified range using 
	 *  range sensitivity supplied, and default angle sensitivity.  Return true if a 
	 *  moving obstacle is found during this search.  NOTE: Make sure this isn't called on the
	 *  UGV itself as it will always return true - exclude on the calling loop (see above)
	 *  @param sim (COModel - access to the simulation environment)
	 *  @param inCar (Car - the vehicle that hwe want to check for proximity)
	 *  @param inRange (double - the distance to which we want the search to be carried out)
	 *  @param inSensitivity (double - the resolution of the search i.e. distance increments)
	 *  @return boolean (true if an obstacle is found in range, false otherwise)
	 */
	private boolean checkForMovingObsCircle(COModel sim, Car inCar, double inRange, double inSensitivity) {
		
		// Simple and dirty method which checks the coordinates between 0 and 
		// the supplied range away from the chosen sensor location in certain increments and see 
		// if they intersect with the supplied car (moving obs) 
		MutableDouble2D testCoord = new MutableDouble2D();
		Double2D amountAdd = new Double2D();
				
		// For this search, we don't care whether the obstacle is in our lane, or whether it is in front of 
		// us or behind us. 
		
		// Work out the 'centre' of the UGV, as the 'location' is actually the front-centre of the UGV.  We
		// need to know which direction the UGV is pointing in so that we can offset appropriately. NOTE: this
		// will only be an approximation if the UGV is turning or overtaking.
		UGV_Direction direction = Utility.getDirection(getDirection());
		Double2D centreLoc = new Double2D(location.x, location.y);
		
		switch (direction) {
		
			case NORTH : 
				centreLoc = new Double2D(location.x, location.y + Constants.OBSTACLE_WIDTH/2);
				break;
			case EAST : 
				centreLoc = new Double2D(location.x - Constants.OBSTACLE_WIDTH/2, location.y);
				break;
			case SOUTH : 
				centreLoc = new Double2D(location.x, location.y - Constants.OBSTACLE_WIDTH/2);
				break;
			case WEST : 
				centreLoc = new Double2D(location.x + Constants.OBSTACLE_WIDTH/2, location.y);
				break;
		}	
		
		// For each angle that the sensor is able to view, turning in realistic increments
		double resolution = 0.5;
			
		// Check all angles from South all the way around
		for(double i = 0; i < 360; i += resolution)
		{
			// Reset the location that we start testing from to be the location of CENTRE of the UGV and set 
			// the bearing that we are going to use for this iteration
			testCoord.setTo(0,0);
			testCoord.addIn(centreLoc); // Sensor assumed to be at centre of vehicle for 360 deg scan
			
			// Construct the x and y increments for each iteration below
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
}


