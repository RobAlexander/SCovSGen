package modeling;
//MASON imports
import java.awt.Shape;
import java.awt.geom.Area;
import java.awt.geom.Rectangle2D;

import sim.engine.SimState;
import sim.portrayal.Oriented2D;
import sim.util.*;

/**
 * This class provides the basic Car functionality/behaviour that is shared between the UGV and the
 * DumbCar, and could form for basis for other future vehicle objects.
 *
 * @author Robert Lee/hh940
 */
public abstract class Car extends Entity implements Oriented2D
{
	private static final long serialVersionUID = 1L;
	
	//parameters for car movement
	private double direction = 0; //will be a value between 0(inc) and 360(exc)
	private double speed = 0; //the speed the vehicle is travelling at
	private CarPerformance performance;//the set performance for the car;
	public boolean isActive= true;
	
	//parameters for navigation
	private int targetID;
	
	//parameters for sensors
	private double viewingRange = 10; //how many units in front of the car it can see obstacles
	private double viewingAngle = 90; //this is the angle for the viewing in front of the car, viewingAngle / 2 in both directions from right in front of the car

	protected COModel sim;
	
	// Keeps track of the previous location so we can work out if the vehicle has crossed over any
	// boundary lines etc during its motion (this is important when the vehicle is travelling quickly as infractions 
	// might occur between two steps and would remain undetected if we just look at the location of the origin point 
	// and the destination point during any given time step).
	private Double2D prevLoc;

	// For junction priority
	private int jctID = 0;
	private boolean isWaiting = false;
	
	// For controlling speed increase/decrease so we can't 'over-accelerate'
	private int voteReallySlow = 0;
	private int voteSlow = 0;
	private int voteSpeedUp = 0;
	private int voteSlowStop = 0; // Added due to vehicles being allowed to accelerate with above methods
	private double stoppingDistance = Constants.WorldXVal*2; // Added so we can try to stop vehicles colliding. ('max' result indicates no veh ahead)
	
	/**
	 * Constructor for Car class
	 * @param int idNo (unique identifier)
	 * @param int idTarget (unique identifier for Target)
	 * @param CarPerformance (performance characteristics for car e.g. max speed)
	 */
	public Car(int idNo, int idTarget, CarPerformance performance)
	{
		super(idNo, TCAR);
		this.targetID = idTarget;
		this.performance = performance;
	}

	/**
	 * Additional constructor for Car class, allows provision of an initial direction/bearing
	 * and (sub)Type in case we want to call this from a child object.
	 * @param int idNo (unique identifier)
	 * @param int idTarget (unique identifier for Target)
	 * @param CarPerformance (performance characteristics for car e.g. max speed)
	 * @param double initialBearing (initial direction for vehicle)
	 * @param int inType (allows provision of a subtype in place of TCAR e.g. TUGV)
	 */
	public Car(int idNo, int idTarget, CarPerformance performance, double initialBearing, int inType)
	{
		super(idNo, inType);
		this.targetID = idTarget;
		this.performance = performance;
		this.direction = initialBearing; // [TODO] include some checking on the input range
	}
	
	/**
	 * Abstract placeholder method for step() to force subclasses to provide their own implementation;
	 * unlikely that child classes would share the detailed movement control that is defined in step().
	 * @param SimState state (access to the environment and other objects in the simulation)
	 */
	public abstract void step(SimState state);
	
	/**
	 * Returns the current orientation of the object in radians (required to support
	 * OrientedPortrayal2D.
	 * @return double (current orientation of the object in radians)
	 */
	public double orientation2D()
	{
		// For some reason, the orientation of the vehicle seems to be displayed 
		// relative to zero degrees along the increasing x axis.  As a result, we need to flip 
		// the compass over the 45/225 degree bisection to convert between the two systems.
		return Math.toRadians(Utility.correctAngle(90-this.direction));
	}

	/**
	 * Set the junction id; required for junction priority.
	 * @param int inJctID (unique identifier for junction currently occupied)
	 */
	public void setJctID(int inJctID)
	{
		jctID = inJctID;
	}
	
	/**
	 * Get the junction id; required for junction priority.
	 * @return int (unique identifier for junction currently occupied)
	 */
	public int getJctID()
	{
		return jctID;
	}
 
	/**
	 * Get the value of isWaiting; required for junction priority.
	 * @return boolean (whether the vehicle is waiting at a junction)
	 */
	public boolean isWaiting()
	{
		return isWaiting;
	}
	
	/**
	 * Set the value of isWaiting to true to record that the vehicle is waiting at a
	 * junction; required for junction priority.
	 */	
	public void startWaiting()
	{
		isWaiting = true;
	}

	/**
	 * Set the value of isWaiting to false to record that the vehicle is no longer 
	 * waiting at a junction; required for junction priority.
	 */	
	public void stopWaiting()
	{
		isWaiting = false;
	}
	
    /**
	 * Record the current stopping distance as the supplied separation (as calculated 
	 * by findImminentCrash); previous distance is compared to current distance on next step
	 * in step() method.
	 * @param double inCurrentSep (distance to closest DumbCar/UGV)
	 */
	public void setStoppingDistance(double inCurrentSep)
	{
		stoppingDistance = inCurrentSep;
	}
	
    /**
	 * Return the previous stopping distance for comparison to current distance on next step
	 * in step() method.
	 * @return double (distance to closest DumbCar/UGV [on previous step])
	 */
	public double getStoppingDistance()
	{
		return stoppingDistance;
	}	

	/**
	 * A method which turns the car towards the direction of the target point.
	 * @param loc the location of the car
	 * @param targ the target location for the car
	 */
	protected void setDirection(Double2D loc, Double2D targ) 
	{
		//first the ideal bearing for the car to get to it's target must be calculated
		double idealDirection = Utility.calculateAngle(loc, targ);
		
		//now based on the ideal bearing for the car to get to it's position it
		//must be determined if the car needs to be changed from the bearing it's
		//on at all
		if (idealDirection != direction)
		{
			//then the course that the car is on needs correcting
			//check if it would be quicker to turn left or right
			double delta = idealDirection - direction;
			if(delta>0)
			{
				if(delta <= 180)
				{
					turnLeft(delta);
				} else if (delta >180 )	{
					turnRight(360 - delta);
				}
			} else {
				if (delta >= -180)
				{
					turnRight(-delta);
				} else {
					turnLeft(360+delta);
				}
			}
		}		
	}

	/**
	 * Method changes the direction of the Car so that moving on that bearing will bring it closer
	 * to the supplied target location/waypoint.  Additional parameter targetDir is used to prevent 
	 * the UGV from turning further than the direction of travel of the lane it is turning into.
	 * @param Double2D loc (the location of the car)
	 * @param Double2D targ (the target location/waypoint the car is travelling towards)
	 * @param double targetDir (direction of the lane the car is turning into)
	 */
	protected void setDirection(Double2D loc, Double2D targ, double targetDir)
	{
		double idealDirection = Utility.calculateAngle(loc, targ);
		
		// Make sure that this 'idealDirection' isn't actually an overshoot
		double currentToIdeal = Utility.correctAngle(idealDirection - direction);
		double currentToTarget = Utility.correctAngle(targetDir - direction);
		
		// If the angular distance to the idealDirection is further, then we should stop at the targetDirection
		if (currentToIdeal > currentToTarget && currentToTarget <= this.performance.getCurrentMaxTurning() && 
			sim.roadAtPoint(this.location, sim.roads) && targetDir != -1) {
			idealDirection = targetDir;
		}
		
		//now based on the ideal bearing for the car to get to it's position it
		//must be determined if the car needs to be changed from the bearing it's
		//on at all
		if (idealDirection != direction)
		{
			//then the course that the car is on needs correcting
			//check if it would be quicker to turn left or right
			double delta = idealDirection - direction;
			if(delta>0)
			{
				if(delta <= 180)
				{
					turnLeft(delta);
				} else if (delta >180 )	{
					turnRight(360 - delta);
				}
			} else {
				if (delta >= -180)
				{
					turnRight(-delta);
				} else {
					turnLeft(360+delta);
				}
			}
		}		
	}
	
	/**
	 * A method which turns the car to the left by the amount specified in theta,
	 * up to a maximum of getCurrentMaxTurning() degrees.
	 * @param double theta (angle the car would like to turn through)
	 */
	private void turnLeft(double theta)
	{
		if(theta <= this.performance.getCurrentMaxTurning())
		{
			direction += theta;
		} else {
			direction += this.performance.getCurrentMaxTurning();
		}
		
		this.direction = Utility.correctAngle(direction); // Adjust to correct scale 0 to <360
	}
	
	/**
	 * A method which turns the car to the right by the amount specified in theta,
	 * up to a maximum of getCurrentMaxTurning() degrees.
	 * @param double theta (angle the car would like to turn through)
	 */
	private void turnRight(double theta)
	{
		if(theta <= this.performance.getCurrentMaxTurning())
		{
			direction -= theta;
		} else {
			direction -= this.performance.getCurrentMaxTurning();
		}
		
		this.direction = Utility.correctAngle(direction); // Adjust to correct scale 0 to <360
	}
		
	/**
	 * A method which increases/decreases the speed of the vehicle as much as possible 
	 * (limited by getCurrentMaxAccel()/getCurrentMaxDecel() on each step until it reaches 
	 * a defined maximum/minimum speed (zero).
	 * @param boolean accelerate (true to accelerate, false to decelerate)
	 */
	protected void changeSpeed(boolean accelerate)
	{
		if (accelerate == true)
		{
			//the car is accelerating
			if (speed <= performance.getCurrentMaxSpeed())
			{
				//then continue to speed up
				if ((speed + performance.getCurrentMaxAccel()) < performance.getCurrentMaxSpeed())
				{
					speed += performance.getCurrentMaxAccel();
				} else {
					speed = performance.getCurrentMaxSpeed();
				}
			} else if (speed > performance.getCurrentMaxSpeed()) {
				//prevent car travelling over maximum speed
				changeSpeed(DECELERATE);
			}
		} else {
			//then the car is to decelerate
			speed -= performance.getCurrentMaxDecel();
		
			if (speed < 0)
			{
				//stop the car moving at a minus speed when it's trying to slow down
				//reverse will have to be implemented separately
				speed = 0;
			}
		}
	}
	
	/**
	 * A method which decreases the speed immediately to zero, and reports whether this manouevre
	 * is actually possible by returning the speed at which the vehicle should be travelling, 
	 * following one step at maximum deceleration.  NOTE: this method may contravene the laws of physics
	 * as it brings the vehicle to a stop immediately, ignoring deceleration constraints.
	 * @return double (speed the vehicle would be travelling had it braked with getCurrentMaxDecel())
	 */
	protected double emergencyStop() 
	{
		double speedRemainder = speed - performance.getCurrentMaxDecel();
		speed = 0;
		return speedRemainder;
	}	

	/**
	 * Returns the performance statistics currently attributed to the vehicle.
	 * @return CarPerformance (current performance configuration of vehicle e.g. maxSpeed)
	 */	
	public CarPerformance getStats() {
		return performance;
	}

	/**
	 * Sets the performance statistics currently attributed to the vehicle.
	 * @param double maxCarSpeed (maximum speed achievable by vehicle)
	 * @param double maxCarAcceleration (maximum increase in speed in one simulation step)
	 * @param double maxCarDeceleration (maximum decrease in speed in one simulation step)
	 * @param double maxCarTurning (maximum turn achievable; in one simulation step)
	 */	
	public void setStats(double maxCarSpeed, double maxCarAcceleration, double maxCarDeceleration, double maxCarTurning) {
		this.performance.setCurrentMaxSpeed(maxCarSpeed);
		this.performance.setCurrentMaxAcceleration(maxCarAcceleration);
		this.performance.setCurrentMaxDeceleration(maxCarDeceleration);
		this.performance.setCurrentMaxTurning(maxCarTurning);
	}
	
	// Get/setters for use by the child classes
	public int getTargetID() { return targetID;}
	public double getDirection() { return direction; }
	public double getSpeed() { return speed; }
	
	public void setTargetID(int newID) {
		targetID = newID; // TO DO - Add some range checking here...
	}
		
	/*
	 * Store location of vehicle at previous time step.
	 * @param Double2D inPrevLoc (previous location of vehicle)
	 */
	protected void storePrevLoc(Double2D inPrevLoc) {
		prevLoc = inPrevLoc;
	}
	
	public Double2D getPrevLoc() { return prevLoc; }

	/** 
	 * Work out whether the supplied location is on the Map i.e. within the boundaries
	 * of the simulation environment.
	 * @param Double2D location (location to be checked)
	 * @return boolean (true if supplied location is on the map)
	 */
	protected boolean onMap(Double2D location)
	{
		if (location.x > Constants.WorldXVal || location.x < 0 || location.y > Constants.WorldYVal || location.y < 0)
		{
			return false;
		} else {
			return true;
		}
	}
	
	/** 
	 * Work out whether the supplied Shape is on the Map i.e. within the boundaries
	 * of the simulation environment.
	 * @param Shape inShape (Shape to be checked (all must be off map for false return value))
	 * @return boolean (true if at least some of inShape is on the map)
	 */
	protected boolean onMap(Shape inShape)
	{
		// Construct a Rectangle2D.Double which represents the whole map, then test for intersection with 
		// the supplied inShape
		Shape mapShape = (Shape) new Rectangle2D.Double(0, 0, Constants.WorldXVal, Constants.WorldYVal);
		Area mapArea = new Area(mapShape);
		mapArea.intersect(new Area(inShape));
		
		return !mapArea.isEmpty();
	}
	
	/** 
	 * Work out whether the supplied Shape is on the Map and return the Area of shape
	 * overlapping the map.
	 * @param Shape in Shape (Shape to be overlapped with map)
	 * @return Area (returns an Area object which is the overlap between inShape and the map)
	 */
	protected Area getAreaOnMap(Shape inShape)
	{
		// Construct a Rectangle2D.Double which represents the whole map, then test for intersection with 
		// the supplied inShape
		Shape mapShape = (Shape) new Rectangle2D.Double(0, 0, Constants.WorldXVal, Constants.WorldYVal);
		Area mapArea = new Area(mapShape);
		mapArea.intersect(new Area(inShape));
		
		return mapArea;
	}
	
	/** 
	 * Work out whether the supplied Shape is on the Road and return the Area of shape 
	 * overlapping the road
	 * @param Bag roads (all roads in the simulation environment)
	 * @param Shape inShape (shape to be checked for intersection with the road surface)
	 * @return Area (Area of inShape intersecting with Road surface (may be Empty))
	 */
	protected Area getAreaOnRoad(Bag roads, Shape inShape)
	{
		// First make sure that the point is not located outside of the map (as the roads do extend
		// further than the map).
		if (onMap(inShape) == false)
		{
			return new Area(); // Return an empty Area
		}
		
		// Build an Area shape that consists of all the Roads
		Area allRoads = new Area();
		Shape currentRoad;
		
		for(int i = 0; i < roads.size(); i++)
		{
			currentRoad = (Shape) ((Road) roads.get(i)).getSurface(); 
			allRoads.add(new Area(currentRoad));
		}		
		
		// Constrain the road by the area of the map just in case we have ended up with roads outside of the map.
		Area mapArea = new Area((Shape) new Rectangle2D.Double(0, 0, Constants.WorldXVal, Constants.WorldYVal));
		allRoads.intersect(mapArea);
		
		allRoads.intersect(new Area(inShape)); // Intersect the on-road area with the supplied Shape
		return allRoads; // Return intersection of Shape and allRoads
	}
	
	/** 
	 * Return the Area of a supplied Shape which does not intersect with a Road
	 * @param Bag roads (roads collection)
	 * @param Shape inShape (shape to be checked for intersection with the non-road surfaces)
	 * @return Area (Area of inShape intersecting with non-road surface (may be Empty))
	 */
	protected Area getAreaNotOnRoad(Bag roads, Shape inShape)
	{
		// First make sure that the point is not located outside of the map (as the roads do extend
		// further than the map.
		if (onMap(inShape) == false)
		{
			return new Area(); // Return an empty Area
		}
		
		// Build an Area shape that consists of all the Roads
		Area allRoads = new Area();
		Shape currentRoad;
		
		for(int i = 0; i < roads.size(); i++)
		{
			currentRoad = (Shape) ((Road) roads.get(i)).getSurface(); 
			allRoads.add(new Area(currentRoad));
		}		
		
		// Create a mapArea which is the the XOR of the entire map and all roads.  NOTE: We've already checked
		// that inShape is within the map bounds, so it doesn't matter that the XOR will result in any roads
		// which are outside of the map being retained.
		Area mapArea = new Area((Shape) new Rectangle2D.Double(0, 0, Constants.WorldXVal, Constants.WorldYVal));
		mapArea.exclusiveOr(allRoads);
		
		mapArea.intersect(new Area(inShape)); // Intersect the off-road area with the supplied Shape
		return mapArea; // Return intersection of Shape and allRoads
	}
		
	/** 
	 * Test supplied Shape object for intersection with the road network on the simulation map,
	 * return true if any part of the shape overlaps with the road. 
	 * @param Bag roads (all roads in the simulation)
	 * @param Shape inShape (shape to be checked for intersection with the road surface)
	 * @return boolean (true if supplied inShape overlaps any road surface)
	 */
	protected boolean onRoad(Bag roads, Shape inShape)
	{
		// First make sure that the point is not located outside of the map (as the roads do extend
		// further than the map.
		if (onMap(inShape) == false)
		{
			return false;
		}
		
		// Build an Area shape that consists of all the Roads
		Area allRoads = new Area();
		Shape currentRoad;
		
		for(int i = 0; i < roads.size(); i++)
		{
			currentRoad = (Shape) ((Road) roads.get(i)).getSurface(); 
			allRoads.add(new Area(currentRoad));
		}		

		// Test for intersection with the road.  Something is only off-road if it 
		// is entirely off the road i.e. no intersection with the road network at all.  Still need to constrain the
		// road by the area of the map just in case we have ended up with roads outside of the map.
		Area mapArea = new Area((Shape) new Rectangle2D.Double(0, 0, Constants.WorldXVal, Constants.WorldYVal));
		allRoads.intersect(mapArea); // Constrain the roads
		
		allRoads.intersect(new Area(inShape)); // Intersect the on-road area with the supplied Shape
		
		return !allRoads.isEmpty(); // If this is empty => no intersection so totally off-road
	}
	
	/** 
	 * Test a Double2D for intersection with the road surface, return true if the point
	 * is on the road.
	 * @param Bag roads (all roads on the simulation map)
	 * @param Double2D inLocation (coordinates of location to be checked)
	 * @return boolean (true if location intersects with any road surface)
	 */
	protected boolean ptOnRoad(Bag roads, Double2D inLocation)
	{
		// First make sure that the point is not located outside of the map (as the roads do extend
		// further than the map.
		if (onMap(inLocation) == false)
		{
			return false;
		}
		
		boolean foundOnRoad = false;
		
		for(int i = 0; i < roads.size(); i++)
		{
			if (((Road) roads.get(i)).inShape(inLocation) == true)
			{
				foundOnRoad = true;
				break;
			}
		}
		
		return foundOnRoad;
	}
	
	/*
	 * Resolve any conflicting speed requirements, giving priority to the (safety critical) braking,
	 * and ensuring that a single call to doSpeedCalcs will result in a single ACCELERATE or DECELERATE
	 * command, even if there have been multiple requests to speedUp or slowDown in the step routine.
	 * NOTE: There is nothing to prevent this method from being called twice from within Step, or any
	 * other routine, so it is possible for this to happen.  Other methods will allow votes to be cast 
	 * for changes of speed, this method compares the votes, prioritising braking requests, and 
	 * executes the change of speed request.
	 * 
	 * In order of precedence: 
	 * voteSlowStop - guarantees deceleration
	 * voteReallySlow - generally causes deceleration, except at very low speeds
	 * voteSlow && voteSpeedUp - opposite commands, so try to keep speed around a steady 0.5.
	 * voteSpeedUp != 0 - guarantees acceleration 
	 */
	protected void doSpeedCalcs()
	{
		// Slow/stop guarantees a slow down (used if we detect a moving obstacle ahead)
		if (voteSlowStop > 0) {
			changeSpeed(DECELERATE);
		} else if (voteReallySlow > 0) {
			// Adjust here to change performance while cornering, this method aims to 
			// keep the speed close to one which can stop completely within 1-2 simulation
			// steps.
			final double MinSpeed = sim.getCarMaxDecceleration();
			
			if (getSpeed() < MinSpeed) {
				changeSpeed(ACCELERATE);
				
				// New Fault #5 - Repeat the command to increase speed again
				if (sim.getFault(5) == true && this.getType() == TUGV) { // Check for UGV
					changeSpeed(ACCELERATE);
					sim.setFault(5);
				}
			} else if (getSpeed() >= MinSpeed*2) {
				changeSpeed(DECELERATE);
				
				// New Fault #6 - Repeat the command to reduce speed again
				if (sim.getFault(6) == true && this.getType() == TUGV) { // Check for UGV
					changeSpeed(DECELERATE);
					sim.setFault(6);
				}
			}
		} else if (voteSlow > 0 && voteSpeedUp >= 0) { // Force a speed up under certain conditions
			// Adjust here to change performance while cornering, this method is similar
			// to the one above, but aims to keeps the speed a little bit higher.
			final double MinSpeed = 0.5;
			
			if (getSpeed() < MinSpeed) {
				changeSpeed(ACCELERATE);
			} else if (getSpeed() >= MinSpeed) {
				changeSpeed(DECELERATE);
			}
		} else if (voteSpeedUp > 0 || voteSpeedUp == -1) { // Force a speed up
			changeSpeed(ACCELERATE);
		}
	}
	
	/** 
	 * Vote for vehicle to decelerate during speed change.
	 */
	protected void goSlow()
	{
		voteSlow++;
	}

	/** 
	 * Vote for vehicle to speed up during speed change.  This request can be made stronger by supplying 
	 * the parameter 'true', which will cause the doSpeedCalcs method to ignore any simple voteSlow 
	 * requests (voteSlowStop/voteReallySlow still take priority though).
	 * @param boolean force (if true, indicate that speed up should be prioritised if possible, perhaps to avoid collision)
	 */
	protected void goFaster(boolean force) // HH 24.9 14 - If force is set to true then vehicle is required to prioritise speeding up 
	{
		if (force == true || voteSpeedUp < 0)
		{
			voteSpeedUp = -1; // This is a flag to indicate that the vehicle *must* speed up - probably to avoid a collision
		} else {
			voteSpeedUp++;
		}
	} 

	/** 
	 * Vote for a guaranteed slow down; this takes priority over all other speed change requests.
	 */
	protected void goSlowStop()
	{
		voteSlowStop++;
	} 
	
	/** 
	 * Modified version of goSlow() for approaching junctions and parked cars, this has priority
	 * over goSlow, but also tries to prevent vehicles from getting stuck at speed = 0.
	 */
	protected void goReallySlow()
	{
		voteReallySlow++;
	}
	
	/** 
	 * Reset all the speed parameters back to zero.
	 */
	protected void resetSpeedParams()
	{
		voteReallySlow = 0;
		voteSlow = 0;
		voteSpeedUp = 0;
		voteSlowStop = 0; 
	}
	
	/** 
	 * Create a new waypoint with a unique id. If isReplacement is true, then if the next WP is not a target 
	 * it can be replaced by copying its next point to the new WP, and removing it from the simulation. If the
	 * next WP is a target, or isReplacement is false, the new WP is inserted in front i.e. its next point is
	 * set to point to the currentTarget.  Method ends with some housekeeping to set up the Waypoint properly
	 * in the simulation environment, and finally the vehicle is turned in the direction of the new Waypoint
	 * NOTE: It is possible that some execution paths will allow setDirection to be called a second time from
	 * the step routine, and this could result in the vehicle turning further than maxCarTurning during one
	 * simulation step, which could contravene the physics of the system.	  
	 * @param Double2D WPlocation (coordinates of location for new WP)
	 * @param COModel sim (access to simulation environment)
	 * @param Double2D location (current location)
	 * @param boolean isReplacement (true = should replace existing and steal its next point)
	 * @param Entity eTarget_current (current WP so we don't have to search through all entities to find it)
	 * @return Waypoint (the waypoint that is created)
	 */
	protected Waypoint createWaypoint(Double2D WPlocation, COModel sim, Double2D location, int type, boolean isReplacement, Entity eTarget_current)
	{
		int wpID = sim.getNewID();
		
		// New Fault #0 - Overwrite the previous id
		if (sim.getFault(0) == true && this.getType() == TUGV) {
			wpID --;
			sim.setFault(0);
		}
		
		// If this new WP is to replace the previous one, we need to find out which
		// next point it was using so that we can copy it to the new WP
		Waypoint wp;
		if (isReplacement == true)
		{
			// We can't cast the incoming argument for eTarget_current to a Waypoint, as it is possible for this
			// to be a Target, instead of a Waypoint (so we wouldn't be able to call the getNextPoint method on it anyway)
			int nextP = 0;
			if (eTarget_current.getType() == Constants.TTARGET)
			{
				nextP = eTarget_current.getID(); // Retain the target for our next point
			} else if (eTarget_current.getType() == Constants.TWAYPOINT) { // Check the type is Waypoint before cast or might get exception
				nextP = ((Waypoint) eTarget_current).getNextPoint();
				sim.environment.remove(eTarget_current); // Now that we are done with the old point, remove it.
			}
			
			wp = new Waypoint(wpID, nextP, type); // Use the current next point as new one (replace current WP), unless current WP is a target.
		} else {
			wp = new Waypoint(wpID, getTargetID(), type); // Regular case, using the current targetID as the next point (add WP on way to current WP)
		}
				
		Double2D tempLoc = new Double2D(WPlocation.x, WPlocation.y); // Copy the desired location
		
		// New Faults #1,2,3,4 - Displace the location for the WP	
		if (sim.getFault(1) == true && this.getType() == TUGV) {
			tempLoc = new Double2D(WPlocation.x+1, WPlocation.y+1);
			sim.setFault(1);
		} else if (sim.getFault(2) == true && this.getType() == TUGV) {
			tempLoc = new Double2D(WPlocation.x+1, WPlocation.y-1);
			sim.setFault(2);
		} else if (sim.getFault(3) == true && this.getType() == TUGV) {
			tempLoc = new Double2D(WPlocation.x-1, WPlocation.y+1);
			sim.setFault(3);
		} else if (sim.getFault(4) == true && this.getType() == TUGV) {
			tempLoc = new Double2D(WPlocation.x-1, WPlocation.y-1);
			sim.setFault(4);
		}
				
		wp.setLocation(tempLoc);
		setTargetID(wpID);
		sim.environment.setObjectLocation(wp, tempLoc);
		setDirection(location, tempLoc); // Turn the vehicle towards the new WP
		return wp; // return WP so we can set eTarget to point to it
	}
	
	/*
	 *  Using a search method that is based on a lidar type sensor operation, test various locations
	 *  at increasing distance from the sensor location (search resolution defined by inSensitivity), 
	 *  and then sweeping with an angular search pattern defined by inAngle (moving from inAngle/2 to 
	 *  left of UGV direction to inAngle/2 to the right) at an angular resolution of 0.5.  At each
	 *  angle (search direction), the search continues until the test location leaves the road, or the
	 *  limit of the search.  The search checks that the test location is inside the lane that we are 
	 *  interested in (between lane markings dependent on sameLane parameter), and then checks for 
	 *  intersection with the inCar object.  If an intersection occurs, calculate the distance from the
	 *  centre-front of the UGV to the test location.  If it is closer than any previous obstacle, record the
	 *  test location, and the distance, so that repeated search will allow us to find the closest 
	 *  obstacle.  The coordinates of the closest test location to receive a 'hit' on the obstacle are
	 *  returned at the end of the method call.  
	 *  @param COModel sim (access to the simulation environment)
	 *  @param Car inCar (the vehicle that we want to test whether it is in range)
	 *  @param Double2D sensorLoc (location of sensor for oncoming traffic, pass in the offside front corner location of UGV)
	 *  @param boolean sameLane (true if we are looking for other moving vehicles ahead of us in the lane, false for oncoming vehicles)
	 *  @param double inAngle (angular range to use for search, relative to UGV direction checks +/- inAngle/2)
	 *  @param double inRange (how far ahead of the UGV should be searched)
	 *  @param double inSensitivity (increment to use in x and y directions when moving test location further from sensor)
	 *  @return Double2D (coordinates of the closest location where the sensor detected a vehicle)
	 */
	private Double2D checkForMovingObstacle(COModel sim, Car inCar, Double2D sensorLoc, boolean sameLane, double inAngle, double inRange, double inSensitivity) {
		
		// Simple and dirty method which checks the coordinates between 0 and 
		// the moving obstacle viewing range away from the target in certain increments and see 
		// if they intersect with the supplied car (moving obs) 
		MutableDouble2D testCoord = new MutableDouble2D();
		Double2D amountAdd = new Double2D();
		double reqDistance = inRange;
		Double2D reqCoord = new Double2D(-1,-1);
		double distance;
		
		// Restrict the obstacle checks to those which are in the same lane as
		// the UGV, so need to know direction in order to restrict in method below
		UGV_Direction direction = Utility.getDirection(getDirection()); // Returns a compass direction rather than angle
		
		// Work out the bounds for the lane that we are searching in, by checking the
		// road markings.  If we are looking for vehicles in the same lane, search should be bounded by 
		// the nearside and centre lane markings; for oncoming vehicles, it will be the centre and offside 
		// markings.  
		Double2D leftBound; 
		Double2D rightBound;
		
		if (sameLane == true) 
		{
			leftBound = locateRoadMarkings_AllRoads(sim.roads, true, sim, genLineType.NEARSIDE, viewingAngle, viewingRange, inSensitivity);
			rightBound = locateRoadMarkings_AllRoads(sim.roads, true, sim, genLineType.CENTRE, viewingAngle, viewingRange, inSensitivity);
		} else {
			leftBound = locateRoadMarkings_AllRoads(sim.roads, true, sim, genLineType.CENTRE, viewingAngle, viewingRange, inSensitivity);
			rightBound = locateRoadMarkings_AllRoads(sim.roads, true, sim, genLineType.OFFSIDE, viewingAngle, viewingRange, inSensitivity);
		}
		
		// For each angle that the sensor is able to view, turning in realistic increments
		double resolution = 0.5;
		double newBearing = 0.0;
		
		// Variables for the loop to support fault insertion
		double startAngle = -(inAngle/2);
		double endAngle = (inAngle / 2);
		double startRange = 0;
		double endRange = inRange;
		double rangeSensitivity = inSensitivity;
								
		for(double i = startAngle; i <= endAngle; i += resolution)
		{
			// Reset the location that we start testing from and set the bearing
			// that we are going to use for this iteration
			testCoord.setTo(0,0);
			testCoord.addIn(sensorLoc); // Displaced to model sensor on front of vehicle
			newBearing = Utility.correctAngle(getDirection() + i);
			
			// Construct the x an y increments for each iteration below
			amountAdd = new Double2D(Utility.xMovement(newBearing, rangeSensitivity), Utility.yMovement(newBearing, rangeSensitivity));
						
		    // NOTE - j is not actually used, it just ensures the correct number of iterations
			for(double j = startRange; j <= endRange; j += rangeSensitivity){
												
				testCoord.addIn(amountAdd);  // move the test location outwards on the chosen bearing
				
				// Ensure that our test coordinate is between us and the edge of the road,
				// as soon as we hit a point that is no longer on the road surface then we should 
				// discontinue our search.
				if (sim.roadAtPoint(new Double2D(testCoord), sim.roads) == false) {
					break; // Try searching again at the next bearing.
				}
								
				// Ensure that the our test coordinate is within our test bounds
				boolean inLane = false;
				
				if (direction == UGV_Direction.NORTH || direction == UGV_Direction.SOUTH)
				{
					if ((testCoord.x > leftBound.x && testCoord.x < rightBound.x) || (testCoord.x < leftBound.x && testCoord.x > rightBound.x)) {
						inLane = true;
					}
				} else {
					if ((testCoord.y > leftBound.y && testCoord.y < rightBound.y) || (testCoord.y < leftBound.y && testCoord.y > rightBound.y)) {
						inLane = true;
					}
				}
								
				if (inLane == true) {
					// Find out which class of car we have so we can call the right inShape method
					boolean isInShape = false;
					if (inCar.type == DUMBCAR) 
					{
						isInShape = ((DumbCar) inCar).inShape(new Double2D(testCoord.x, testCoord.y));
					} else { // must be a UGV
						isInShape = ((UGV) inCar).inShape(new Double2D(testCoord.x, testCoord.y));
					}
										
					// keep adding the amountAdd on and seeing if the coordinate is inside an obstacle
					if (isInShape == true)
					{
						// Store the distance at which the testCoord has intersected
						distance = location.distance(testCoord.x, testCoord.y);

						if (distance < reqDistance) {
							reqDistance = distance;
							reqCoord = new Double2D(testCoord.x, testCoord.y);
						}
					}
				}
			}
		}
		
		return reqCoord;
	}
	
	/**
	 * Loop through the set of all moving obstacles (DumbCars) and check to see whether any
	 * would be found by a simulated sensor on the front right of the vehicle with angle,
	 * range and range sensitivity provided as arguments.  Can also specify whether we are looking 
	 * for vehicles in the sameLane, or oncoming vehicles.  Record and return the coordinates of 
	 * the closest 'hit'.
	 * @param COModel sim (access to the simulation environment)
	 * @param Bag inCars (access to all the cars in the simulation)
	 * @param boolean sameLane (true indicates search for vehicle in lane ahead, false means oncoming vehicle)
	 * @param double inAngle (angular range to use for search, relative to UGV direction checks +/- inAngle/2)
	 * @param double inRange (how far ahead of the UGV should be searched)
	 * @param double inSensitivity (increment to use in x and y directions when moving test location further from sensor)
	 * @return Double2D (coordinates of the closest location where the sensor detected a vehicle)
	 */
	protected Double2D checkAllMovingObstacles(COModel sim, Bag inCars, boolean sameLane, double inAngle, double inRange, double inSensitivity)
	{
		// Init to the values we would want for finding the min
		double reqDistance = inRange + 1;
		double currentDistance = inRange + 1;
		
		// Calculate the location of the sensor, assuming it is on the front offside corner of the vehicle
		// and that Car.location returns the coordinates of the centre-front of the vehicle.  Note that we
		// make use of the movement methods here as the displacement is the same as if we were to move the
		// vehicle to that location.  A movement to the front right, from front-centre is equivalent to moving 
		// at -90 degrees to the direction of travel, at a speed of UGV_WIDTH/2 (distance travelled in one step.
		Double2D sensorLoc;
		double xDispl = Utility.xMovement(Utility.correctAngle(getDirection() - 90), UGV_WIDTH/2);
		double yDispl = Utility.yMovement(Utility.correctAngle(getDirection() - 90), UGV_WIDTH/2);
		sensorLoc = new Double2D(location.x + xDispl, location.y + yDispl);
		
		Double2D currentDistanceCoord; 
		Double2D reqCoord = new Double2D(-1,-1);
				
		Car currentCar;
		
		// Look through all the moving obstacles (cars) and look for intersection
		for(int i = 0; i < inCars.size(); i++)
		{		
			// Make sure the car is active, and that it is not the same vehicle that we are 'in'
			currentCar = (Car) inCars.get(i);
			if (currentCar.isActive == true && (currentCar.ID != this.ID))
			{
				currentDistanceCoord = checkForMovingObstacle(sim, currentCar, sensorLoc, sameLane, inAngle, inRange, inSensitivity);

				// If a 'hit' was detected, check if it was nearer than any previous result, and if so, 
				// store the distance, and the coordinates of the 'hit'.
				if (currentDistanceCoord.x > -1) {
					currentDistance = location.distance(currentDistanceCoord.x, currentDistanceCoord.y);

					if ( currentDistance < reqDistance )
					{
						reqDistance = currentDistance;
						reqCoord = currentDistanceCoord;
					}
				}
			}

		}
		
		return reqCoord; // Need to do a check on return value as if this returns a value greater
							// than the maximum sensor range then it denotes *no obstacle*
	}

	/**
	 * HH 9.9.14 - Based on Car.checkCourse (Robert Lee)
	 *  
	 * @param roads
	 * @param findNearest - whether we are interested in the closest or furthest RM found
	 * @param sim // HH 28.7.14 - added for fault insertion
	 * @param reqLine - which line are we searching for (nearside, offside, centre)
	 * @return coordinates of the furthermost road marking detected.
	 * 
	 * HH 3.12.14 Changed to Protected
	 */
	protected Double2D locateRoadMarkings_AllRoads(Bag roads, boolean findNearest, COModel sim, genLineType reqLine, double inAngle, double inRange, double inSensitivity)
	{
		Double2D currentXY = new Double2D(0,0);
		
		// HH 4.12.14 Initialisation matters here or the conditions below won't evaluate correctly
		Double2D requiredXY;
		if (findNearest == false) {
			// Want to find the largest value, so the closest one would be the current location
			requiredXY = location;
		} else {
			// Want to find the furthest value, and can't be any further away than fictional point 2x max X/Y
			requiredXY = new Double2D((2 * Constants.WorldXVal),(2 * Constants.WorldYVal));
		}
		
		// Check all the roads as the UGV doesn't *know* which one it is on
		for(int i = 0; i < roads.size(); i++)
		{		
			currentXY = locateRoadMarkings((Road) roads.get(i), findNearest, sim, reqLine, inAngle, inRange, inSensitivity);

			// See if the latest return value is a better match for what we are looking for
			if ( (onMap(currentXY) == true) && ((location.distance(currentXY) > location.distance(requiredXY) && findNearest == false) ||
			     (location.distance(currentXY) < location.distance(requiredXY) && findNearest == true) || (requiredXY.x == -1)) )
			{
				// check for an invalid return code
				if (currentXY.x != -1) {
					requiredXY = currentXY;
				}
			}

		}
		
		return requiredXY;
	}
	
	/** 
	 * HH 9.9.14 - Based on Car.onCourse (Robert Lee), and some code in Car.alterCourse (Robert Lee)
	 * New version of findRoadMarkings which is less of a 'cheat' as it moves left and right from the
	 * current bearing until it reaches a road marking (or exceeds range).  It will only continue to look
	 * for as long as it detects either road surface, or a line, so this should be more aligned with image
	 * processing methods for finding the lines on the roads.  This should restrict the algorithm to finding
	 * lines that are on the same road as the UGV without needing 'special' access to the road that the
	 * UGV is currently on as was done in the previous method.
	 * 
	 * @param road // passed in from the Bag of all roads (assume we will be in a loop searching all of them)
	 * @param findNearest // true = return the nearest point found; false = return the furthest
	 * @param sim // supports failure insertion
	 * @param reqLine - which line are we searching for (nearside, offside, centre)
	 * @param inAngle - viewing angle of vehicle
	 * @param inRange - viewing range of vehicle
	 * @param inSensitivity - increment to be used for range increases
	 * @return the coordinate location of road markings that are detected within the range of the vehicle sensor
	 */
	private Double2D locateRoadMarkings(Road road, boolean findNearest, COModel sim, genLineType reqLine, double inAngle, double inRange, double inSensitivity)
	{
		//simple and dirty method which checks the coordinates between 0 and 
		//the viewing range away from the target in certain increments and see 
		//if they intersect with road markings
		MutableDouble2D testCoord = new MutableDouble2D();
		Double2D amountAdd = new Double2D();
		testCoord.addIn(location);
		
		Double2D RM = new Double2D(-1, -1); // Default value
		
		// For each angle that the sensor is able to view, turning in realistic increments
		double resolution = 0.5;
		double newBearing = 0.0;
		
		// Set defaults (appropriate for centre line and offside line)
		double startAngle = 0; // HH 23.9.14 - Swapped these over
		double endAngle = -(inAngle/2); // HH 23.9.14 - Swapped these over
		double startRange = 0;
		double endRange = inRange;
		double rangeSensitivity = inSensitivity;		
		
		// Alter the search params for nearside search
		if (reqLine == genLineType.NEARSIDE) 
		{
			startAngle = (inAngle / 2); // HH 23.9.14 - Swapped these over
			endAngle = 0; // HH 23.9.14 - Swapped these over
		}
		
		// New Fault #8 (FAULT #17) - HH 28/7/14 - Force the angle loop to start half-way through
		if (sim.getFault(8) == true && this.getType() == TUGV) { // HH 29.1.15 Added check for type
			startAngle = 0;
			sim.setFault(8); // HH 13.11.14 Added
		} 

		// New Fault #9 (FAULT #18) - HH 28/7/14 - Force the angle loop to end half-way through
		if (sim.getFault(9) == true && this.getType() == TUGV) { // HH 29.1.15 Added check for type
			endAngle = 0;
			sim.setFault(9); // HH 13.11.14 Added
		} 
		
		// New Fault #10 (FAULT #19) - HH 28/7/14 - Force the angle sensor to 'reduce' range resolution (double iterator step)
		if (sim.getFault(10) == true && this.getType() == TUGV) { // HH 29.1.15 Added check for type
			resolution = resolution*2;
			sim.setFault(10); // HH 13.11.14 Added
		}  
				
		// Check the viewable range at each angle		
		for(double i = startAngle; i >= endAngle; i -= resolution)
		{
			// HH 6.8.14 - Each iteration we need to reset the testCoord so it is back at the current location
			// of the vehicle (sensor)
			testCoord.setTo(0,0);
			testCoord.addIn(location);
			//newBearing = Utility.correctAngle(getDirection() + i); // HH 6.8.14 - Reset the bearing for this iteration
			newBearing = Utility.correctAngle(Utility.getDirectionDeg(getDirection()) + i); // HH 8.12.14 - Replaced with the direction of desired movement, rather than pointing
			
			amountAdd = new Double2D(Utility.xMovement(newBearing, rangeSensitivity), Utility.yMovement(newBearing, rangeSensitivity));
			
			// New Fault #11 (FAULT #21) - HH 28/7/14 - Force the angle loop to start half-way through
			if (sim.getFault(11) == true && this.getType() == TUGV) { // HH 29.1.15 Added check for type
				startRange = inRange/2;
				sim.setFault(11); // HH 13.11.14 Added
			} 

			// New Fault #12 (FAULT #22) - HH 28/7/14 - Force the angle loop to end half-way through
			if (sim.getFault(12) == true && this.getType() == TUGV) { // HH 29.1.15 Added check for type
				endRange = inRange/2;
				sim.setFault(12); // HH 13.11.14 Added
			} 
			
			// New Fault #13 (FAULT #23) - HH 28/7/14 - Force the sensor to 'reduce' angular resolution (double iterator step)
			if (sim.getFault(13) == true && this.getType() == TUGV) { // HH 29.1.15 Added check for type
				rangeSensitivity = inSensitivity*2;
				sim.setFault(13); // HH 13.11.14 Added
			} 
						
			// HH 6.8.14 - we don't use j, this just ensures we run the loop the right num
			for(double j = startRange; j <= endRange; j += rangeSensitivity)
			{
				// keep adding the amountAdd on and seeing if the coordinate is still on the road, or whether it 
				// is on a road marking.  There would likely be no way to tell the difference between nearside and
				// offside road markings, so inaccurate to use this detail from the object model.  Centre markings
				// would be different, so can use this detail to differentiate between centre and offside.
				
				// HH 6.8.14 - Adding in the first increment prior to the test, rather than after as no point
				// testing the location the vehicle is already in
				testCoord.addIn(amountAdd);
				
				// Make sure we are still on the road
				if (((Road) road).inShape(new Double2D(testCoord)) == false)
				{
					break; // exit the for loop and try the next bearing.
				}
				
				if (reqLine == genLineType.CENTRE) // Are we looking for a centre line?
				{
					if ((((Road) road).getLine(LineType.CENTRE)).contains(testCoord.x, testCoord.y))
					{
						if (((location.distance(testCoord.x, testCoord.y) > location.distance(RM)) && findNearest == false) || RM.x == -1) {
							RM = new Double2D(testCoord.x, testCoord.y);
						} else if ((location.distance(testCoord.x, testCoord.y) < location.distance(RM)) && findNearest == true) {
							RM = new Double2D(testCoord.x, testCoord.y);
						}
					}
				} else { // Check both sets of edge markings as the vision algorithms are unlikely to know which are which
						 // as they would be the same colour/shape
					if ((((Road) road).getLine(LineType.NWSIDE)).contains(testCoord.x, testCoord.y) ||
						(((Road) road).getLine(LineType.SESIDE)).contains(testCoord.x, testCoord.y))
					{
						if (((location.distance(testCoord.x, testCoord.y) > location.distance(RM)) && findNearest == false) || RM.x == -1) {
							RM = new Double2D(testCoord.x, testCoord.y);
						} else if ((location.distance(testCoord.x, testCoord.y) < location.distance(RM)) && findNearest == true) {
							RM = new Double2D(testCoord.x, testCoord.y);
						}
					}
				}
			}
		}
		
		return RM; 
	}	
	
	/*
	 *  HH 4.12.14 Method to determine (without cheating) whether the point supplied
	 *  is in the same lane as the vehicle.  We use the lane marking sensors to find
	 *  the lane boundaries and then compare these to the location at which the 
	 *  vehicle has been detected.
	 */
	protected boolean checkSameLane(COModel sim, Double2D testLoc, double inRange, double inAngle, double inSensitivity, double inBearing) {
		
		// Need to restrict the obstacle checks to those which are in the same lane as
		// the UGV, so need to know direction in order to restrict in method below
		UGV_Direction direction = Utility.getDirection(inBearing);
		
		// Use the road marking detection algorithm to find the centre of the road so that we can make 
		// sure that we only detect obstacles on the same side of the road as the UGV's direction of travel.		
		Double2D furthestLaneMarking = locateRoadMarkings_AllRoads(sim.roads, false, sim, Constants.genLineType.NEARSIDE, inAngle , inRange, inSensitivity);

		double centreOffset = Road.roadWidth/2 - Constants.ROADEDGINGWIDTH - Constants.ROADEDGEOFFSET; // Distance between edge line and centre
		boolean inLane = true;
		double centre;
		
		switch (direction) {

			case NORTH : {
				centre = furthestLaneMarking.x + centreOffset;
				if (testLoc.x > centre || testLoc.x < (centre - Road.roadWidth/2)) {
					inLane = false;
				}	
				break;
			}
			case SOUTH : {
				centre = furthestLaneMarking.x - centreOffset;
				if (testLoc.x < centre || testLoc.x > (centre + Road.roadWidth/2)) {
					inLane = false;
				}
				break;
			}
			case EAST : {
				centre = furthestLaneMarking.y + centreOffset;
				if (testLoc.y > centre || testLoc.y < (centre - Road.roadWidth/2)) {
					inLane = false;
				}
				break;
			}
			case WEST : {
				centre = furthestLaneMarking.y - centreOffset;
				if (testLoc.y < centre || testLoc.y > (centre + Road.roadWidth/2)) {
					inLane = false;
				}
				break;
			}
		}
			
		return inLane;
	}
	
	/*
	 * HH 24.9.14 - Return the distance between the location of this vehicle, and the supplied coordinates.  
	 * The distance will be enhanced with a sign (+/-) to indicate whether the point is likely to be behind
	 * the current location (-), or in front (+).  This method is crude and may not always be accurate.
	 * NOTE: Returns WorldX*2 if inCoord is (-1,-1)  
	 */
	public double calcDistance(Double2D inCoord)
	{
		// HH 29.9.14 - New method will just return the distance, which can then be compared to any previous
		// distance for the purposes of checking whether the vehicle is getting too close to the one ahead

		if (inCoord.x == -1)
		{
			return Constants.WorldXVal*2; // Don't try and calculate the distance to (-1,-1), it's meaningless!!!
		}
		
		double retVal = 1; // we're going to use this as a multiplier, so this is the default +ve case
			
		return (retVal * location.distance(inCoord));
	}
	
	/* 
	 * HH 24.9.14 - Abstract method so don't have to cast to run from subclasses
	 */
	public abstract Shape getShape();
}
