package modeling;
//MASON imports
import java.awt.Shape;
import java.awt.geom.Area;
import java.awt.geom.Rectangle2D;

import modeling.Constants.LineType;
import modeling.Constants.UGV_Direction;
import modeling.Constants.genLineType;
import sim.engine.*;
import sim.field.continuous.*;
import sim.portrayal.Oriented2D;
import sim.util.*;

/**
 *
 * @author Robert Lee
 */
public abstract class Car extends Entity implements Oriented2D
{
	//parameters for car movement
	private double direction = 0; //will be a value between 0(inc) and 360(exc)
	private double speed = 0; //the speed the vehicle is travelling at
	private CarPerformance performance;//the set performance for the car;
	public boolean isActive= true;
	//private static int noActiveCars = COModel.noCars;
	
	//parameters for navigation
	private int targetID;
	
	//parameters for sensors
	private double viewingRange = 10; //how many units in front of the car it can see obstacles
	private double viewingAngle = 90; //this is the angle for the viewing in front of the car, viewingAngle / 2 in both directions from right in front of the car
	private final double sensitivityForCollisions = 0.5; //this is used to see if the car will collide with obstacles on it's current heading
	
	//parameters for recording information about sim
	private double distanceToDanger = Double.MAX_VALUE; //records the closest distance to danger experienced by the car

	protected COModel sim; // HH 26.8.14 private => protected
	
	// HH 16.7.14 Keep track of the previous location so we can work out if the vehicle has crossed over any
	// boundary lines etc during it's motion (this is important when the vehicle is travelling quickly as infractions 
	// might occur between two steps and would remain undetected if we just look at the location of the origin point 
	// and the destination point during any given time step).
	private Double2D prevLoc;

	// HH 4.9.14 - For junction priority
	private int jctID = 0;
	private boolean isWaiting = false;
	
	// HH 22.9.14 - For controlling speed increase/decrease so we can't 'over-accelerate'
	private int voteReallySlow = 0;
	private int voteSlow = 0;
	private int voteSpeedUp = 0;
	private int voteSlowStop = 0; // HH 23.9.14 - Added due to vehicles being allowed to accelerate with above methods
	private double stoppingDistance = Constants.WorldXVal*2; // HH 25.9.14 Added so we can try to stop vehicles colliding. ('max' result indicates no veh ahead)
	
	public Car(int idNo, int idTarget, CarPerformance performance)
	{
		super(idNo, TCAR);
		this.targetID = idTarget;
		this.performance = performance;
	}

	// HH 18.6.14 - created another constructor to allow provision of an initial direction/bearing
	// HH 25.8.15 - added type in case we want to call from a derived class
	public Car(int idNo, int idTarget, CarPerformance performance, double initialBearing, int inType)
	{
		super(idNo, inType);
		this.targetID = idTarget;
		this.performance = performance;
		this.direction = initialBearing; // [TODO] include some checking on the input range
	}
	// HH - end
	
	/*
	 * HH 17.9.14 - returns the current orientation of the object in radians (required to support
	 * OrientedPortrayal2D
	 */
	public double orientation2D()
	{
		
		// HH 18.9.14 - For some reason, the orientation of the vehicle seems to be displayed 
		// relative to zero degrees along the increasing x axis.  As a result, we need to flip 
		// the compass over the 45/225 degree bisection to convert between the two systems.
		return Math.toRadians(Utility.correctAngle(90-this.direction));
	}


	
	/**
	 * Method which calculated the stopping distance of the car at a particular speed
	 * 
	 * @param speed Value of speed which the stopping distance is calculated for
	 * @return the stopping distance of the vehicle at that speed
	 */
//	private double stoppingDistance(double speed)
//	{
//		double dist = 0;
//		double s = speed;
//		
//		s -= performace.getCurrentMaxDecel();
//		
//		while (s > 0)
//		{
//			//loop until stopped
//			dist += s;
//			s -= performace.getCurrentMaxDecel();
//		}
//		
//		return dist;
//	}
	

	@Override
	public void step(SimState state)
	{
		if(this.isActive == true)
		{
			sim = (COModel) state;
			Continuous2D environment = sim.environment;
			
			Double2D me = environment.getObjectLocation(this);
			MutableDouble2D sumForces = new MutableDouble2D(); //used to record the changes to be made to the location of the car

			// HH 16/7/14 - Store the previous location now, before we do anything with it
			storePrevLoc(me);
			// HH - end
			
			Double2D targetCoor= me;
			double moveV; //verticle component of the cars movement
			double moveH; //horizontal component of the cars movement
			
			//get location of target
			Bag everything = environment.getAllObjects(); //this will get all of the objects in the world, then start filtering :)
			Bag obstacles = new Bag();
			
			this.performance = new CarPerformance(sim.getCarMaxSpeed(),sim.getCarMaxAcceleration(), sim.getCarMaxDecceleration(), sim.getCarMaxTurning());
			//System.out.println("Car.step is called, car"+this.getID()+"'s coordinate: ("+ me.x+" , "+me.y+")");
	        
			Entity e;
			
			Entity eTarget = new Entity(-1, TOTHER); //this id for the target is illegal, to get ids one should use COModel.getNewID()
					
			for(int i = 0; i < everything.size(); i++)
			{
				e = (Entity) everything.get(i);			
				if (e.getID() == targetID)
				{
					eTarget =  e;
					targetCoor = eTarget.getLocation();
				} else if (e.getType() == TCIROBSTACLE) {
					obstacles.add(e);
				}
			}
			
			dealWithTerrain();
					
			//see if on course to target
			if (direction == Utility.calculateAngle(me, targetCoor))
			{
				if (checkCourse(obstacles, direction)) 
				{
					//only check to see if the car is to hit something if it onto it's
					//target course, in the case that it isn't on course then it may turn out of
					//the way of things in it's current path
					int wpID =sim.getNewID();
					alterCourse(obstacles, me, wpID);						
				}
			}
			
			if (me.distance(targetCoor) > 3)
			{
				goFaster(false); // HH 22.9.14 - Replaced the below
				//changeSpeed(ACCELERATE);
				setDirection(me, targetCoor);
			} else {
				if (eTarget.getType() == TTARGET)
				{
					goSlow(); // HH 22.9.14 - Replaced the below
					//changeSpeed(DECELERATE);
				}
			} 
			
			if (me.distance(targetCoor) < 1) {
				
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
						//System.out.println("Car"+this.getID()+"arrived at destination, the closest to danger the car got was " + Double.toString(distanceToDanger));
						//System.out.println("Ending Sim at destination the closest to danger the car got was " + Double.toString(distanceToDanger));
						//sim.schedule.clear();
						this.isActive = false;
						
					} else if (eTarget.getType() == TWAYPOINT) {
						//get rid of wp and get new target ID
						//System.out.println("Car"+this.getID()+"gets a new target and removing waypoint");
						targetID = ((Waypoint) eTarget).getNextPoint();
						environment.remove(eTarget);
					}
				}			
			}		
			
			//call the operations to calculate how much the car moves in the x
			//and y directions.
			moveV = Utility.yMovement(direction, speed);
			moveH = Utility.xMovement(direction, speed);
			
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
				proximityToDanger(obstacles, location);
//			}
			
			
		}
		if(sim!=null)
		{
			sim.dealWithTermination();
		}
    }
	
	// HH 4.9.14 - Junction priority stuff
	public void setJctID(int inJctID)
	{
		jctID = inJctID;
	}

	public int getJctID()
	{
		return jctID;
	}
    
	public boolean isWaiting()
	{
		return isWaiting;
	}
	
	public void startWaiting()
	{
		isWaiting = true;
	}
	
	public void stopWaiting()
	{
		isWaiting = false;
	}
	// HH end
	
    /*
     * HH 25.9.14 - Set the previous separation distance
     */
	public void setStoppingDistance(double inCurrentSep)
	{
		stoppingDistance = inCurrentSep;
	}
	
    /*
     * HH 25.9.14 - Get the previous separation distance
     */
	public double getStoppingDistance()
	{
		return stoppingDistance;
	}	

	//**************************************************************************
	//methods for setting the speed and direction of the car and for moving the car
	
	/**
	 * A method which moves the car in the direction of the target point.
	 * 
	 * @param loc the location of the car
	 * @param targ the target location for the car
	 */
	protected void setDirection(Double2D loc, Double2D targ) // HH 7.5.14 - changed this from private to protected
	{
		double idealDirection = Utility.calculateAngle(loc, targ);
		
		//first the ideal bearing for the car to get to it's target must be calculated
		//System.out.println("x: " + Double.toString(loc.x) + " y: " + Double.toString(loc.y));

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
				}

				else if (delta >180 )
				{
					turnRight(360 - delta);
				}
				
			}
			else
			{
				if (delta >= -180)
				{
					turnRight(-delta);
				}
				else
				{
					turnLeft(360+delta);
				}
			}
			
		}		
	}

	/**
	 * HH 22.12.14 - Updated method to prevent the UGV from turning further than the target direction of
	 * travel. Method changes the direction of the Car so that moving on that bearing will bring it closer
	 * to the supplied target.
	 * 
	 * @param loc the location of the car
	 * @param targ the target location for the car
	 * @param targetDir target direction for the car
	 */
	protected void setDirection(Double2D loc, Double2D targ, double targetDir)
	{
		double idealDirection = Utility.calculateAngle(loc, targ);
		
		// HH 22.12.14 Need to make sure that this 'idealDirection' isn't actually an overshoot
		double currentToIdeal = Utility.correctAngle(idealDirection - direction);
		double currentToTarget = Utility.correctAngle(targetDir - direction);
		
		// HH 22.12.14 If the angular distance is further to the idealDirection then we should stop at the targetDirection
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
				}

				else if (delta >180 )
				{
					turnRight(360 - delta);
				}
				
			}
			else
			{
				if (delta >= -180)
				{
					turnRight(-delta);
				}
				else
				{
					turnLeft(360+delta);
				}
			}
			
		}		
	}
	
	
	/**
	 * A method which turns the car to the left towards a given bearing.
	 * 
	 * @param bearing the bearing the car is turning onto
	 */
	private void turnLeft(double theta)
	{
		if(theta <= this.performance.getCurrentMaxTurning())
		{
			direction += theta;
		}
		else
		{
			direction += this.performance.getCurrentMaxTurning();
		}
		this.direction = Utility.correctAngle(direction);
	}
	
	
	/**
	 * A method which turns the car to the right towards a given bearing.
	 * 
	 * @param bearing the bearing the car is turning onto
	 */
	
	private void turnRight(double theta)
	{
		if(theta <= this.performance.getCurrentMaxTurning())
		{
			direction -= theta;
		}
		else
		{
			direction -= this.performance.getCurrentMaxTurning();
		}
		
		this.direction = Utility.correctAngle(direction);
	}
	
	
//	private void turnRight(double bearing)
//	{
//		if (direction > bearing) {bearing += 360;} //correct cases wrapping around 360
//		
//		if (bearing > (direction + performace.getCurrentMaxTurning()))
//		{
//			//the target bearing is more right than can be turned to in one step
//			direction += performace.getCurrentMaxTurning();
//		} else {
//			direction = bearing;
//		}
//		
//		direction = correctBearing(direction);
//	}
		

	
	
	/**
	 * A method which increases the speed of the vehicle as much as possible until
	 * it reaches a defined maximum speed.
	 */
	protected void changeSpeed(boolean accelerate) // HH 7.5.14 - changed this from private to protected
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
				//prevent car travelling over maximum speed - useful if car moves into terrain which lowers max speed
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
	 * following one step at maximum deceleration.
	 */
	protected double emergencyStop() 
	{
		double speedRemainder = speed - performance.getCurrentMaxDecel();
		speed = 0;
		return speedRemainder;
	}	
	//**************************************************************************
	//methods for viewing and then dodging other Entities
	
	/**
	 * Detects if the car has hit any of the obstacles in the Bag passed to it
	 * 
	 * @param obstacles a bag containing all of the obstacles in the environment
	 */
	private boolean detectCollision(Bag obstacles)
	{
		return sim.obstacleAtPoint(location, obstacles);
	}
	
	
	/**
	 * A method which measures how far away the closest obstacle to the car is
	 * 
	 * @param obstacles 
	 */
	private void proximityToDanger(Bag obstacles, Double2D coord)
	{
		double check;
		
		for (int i = 0; i < obstacles.size(); i++)
		{
			check = ((Obstacle) obstacles.get(i)).obstacleToPoint(coord);
			if (check < distanceToDanger)
			{
				distanceToDanger = check;
			}
		}
		
	}
	
	
	/**
	 * Checks to see if the car from it's current position and heading can see the
	 * Entity it is passed.
	 * 
	 * @param e
	 * @return value which reflects if the car can see the entity or not.
	 */
//	private boolean canSee(Entity e)
//	{
//		Double2D eLocation = e.getLocation();
//		double lowerLimit;
//		double upperLimit;
//		double bearingToObstacle; //this only needs calculating if the target is within range of the car
//
//		if (myLocation.distance(eLocation) < viewingRange)
//		{
//			//the entity is within the range of things that can be seen by the car
//			
//			//must calculate bearing to obstacle
//			bearingToObstacle = calculateBearing(myLocation, eLocation);
//			lowerLimit = direction - (viewingAngle / 2);
//			upperLimit = direction + (viewingAngle / 2);
//			
//			//must work out if the edges of the viewing angle will cross the 0/360 line
//			if ((upperLimit > 360) && (((lowerLimit < bearingToObstacle) && (bearingToObstacle < 360)) || 
//					((0 < bearingToObstacle) && (bearingToObstacle < (upperLimit - 360)))))
//			{
//				return true;
//			} else if ((lowerLimit < 0) && (((0 < bearingToObstacle) && (bearingToObstacle < upperLimit)) || 
//					(((lowerLimit + 360) < bearingToObstacle) && (bearingToObstacle < 360)))) {
//				return true;
//			} else if ((lowerLimit < bearingToObstacle) && (bearingToObstacle < upperLimit)) {
//				return true;
//			}			
//			//if within the limits then the target can be seen by the car
//		}
//		
//		return false;
//	}
	
	
	/**
	 * Method which adds a Waypoint for the vehicle to travel via on it's path to
	 * prevent it from hitting an obstacle in it's way
	 * 
	 * @param obstacles All of the obstacles in the environment
	 * @param me location of the vehicle
	 * @param wpID id for the Waypoint
	 */
	public void alterCourse(Bag obstacles, Double2D me, int wpID)
	{
		double resolution = 0.5;
		MutableDouble2D coord = new MutableDouble2D(me);
		Waypoint wp;
		double xComponent;
		double yComponent;
	
		
		for(double i = 0; i < (performance.getCurrentMaxTurning() - 5); i += resolution)
		{
			if (checkCourse(obstacles, Utility.correctAngle(direction - i)) == false)
			{
				//then moving right gives a clear path
				//set wp and return
				
				//first must find out where to put wp
				xComponent = Utility.xMovement(Utility.correctAngle(direction - (i+5)), (viewingRange / 1));
				yComponent = Utility.yMovement(Utility.correctAngle(direction - (i+5)), (viewingRange / 1));
				coord.addIn(xComponent, yComponent);
				wp = new Waypoint(wpID, targetID);
				wp.setLocation(new Double2D(coord));
				targetID = wpID;
				sim.environment.setObjectLocation(wp, new Double2D(coord));
				return;
				
			} else if (checkCourse(obstacles, Utility.correctAngle(direction + i)) == false) {
				//then moving left gives a clear path
				//set wp and return
				
				xComponent = Utility.xMovement(Utility.correctAngle(direction + (i+5)), (viewingRange / 1));
				yComponent = Utility.yMovement(Utility.correctAngle(direction + (i+5)), (viewingRange / 1));
				coord.addIn(xComponent, yComponent);
				wp = new Waypoint(wpID, targetID);
				wp.setLocation(new Double2D(coord));
				targetID = wpID;
				sim.environment.setObjectLocation(wp, new Double2D(coord));
				return;
			}
		}
		
		//no path that can be immediately turned onto is clear
		//therefore see if it is possible for the car to see a clear path even
		//if it can't be immediately turned onto
		for(double i = (performance.getCurrentMaxTurning()-5); i < (viewingAngle / 2); i += resolution)
		{
			if (checkCourse(obstacles, Utility.correctAngle(direction - i)) == false)
			{
				//then moving right gives a clear path
				//set wp and return
				
				//first must find out where to put wp
				
				xComponent = Utility.xMovement(Utility.correctAngle(direction - (performance.getCurrentMaxTurning()+5)), (viewingRange / 1));
				yComponent = Utility.yMovement(Utility.correctAngle(direction - (performance.getCurrentMaxTurning()+5)), (viewingRange / 1));
				coord.addIn(xComponent, yComponent);
				wp = new Waypoint(wpID, targetID);
				wp.setLocation(new Double2D(coord));
				targetID = wpID;
				sim.environment.setObjectLocation(wp, new Double2D(coord));
				return;
				
			} else if (checkCourse(obstacles, Utility.correctAngle(direction + i)) == false) {
				//then moving left gives a clear path
				//set wp and return
				
				xComponent = Utility.xMovement(Utility.correctAngle(direction + (performance.getCurrentMaxTurning()+5)), (viewingRange / 1));
				yComponent = Utility.yMovement(Utility.correctAngle(direction + (performance.getCurrentMaxTurning()+5)), (viewingRange / 1));
				coord.addIn(xComponent, yComponent);
				wp = new Waypoint(wpID, targetID);
				wp.setLocation(new Double2D(coord));
				targetID = wpID;
				sim.environment.setObjectLocation(wp, new Double2D(coord));
				return;
			}
		}
	}


	//this method tests a course to see if any of the obstacles in the bag will be hit
	//by the car if it moves from it's position on the bearing provided
	/**
	 * 
	 * @param obstacles
	 * @param bearing
	 * @return true if going to hit something in obstacles, false if not
	 */
	private boolean checkCourse(Bag obstacles, double bearing)
	{
		for(int i = 0; i < obstacles.size(); i++)
		{
			if (onCourse((Obstacle) obstacles.get(i), bearing))
			{
				return true;
			}
		}
		
		return false;
	}
	
	
	/** this method will analyse an obstacle and will see if the car will hit it
	 *  on the course specified as bearing
	 * 
	 * @param o
	 * @param bearing
	 * @return true if going to hit o on provided course (as far as it can see) false if not
	 */
	private boolean onCourse(Obstacle o, double bearing)
	{
		//simple and dirty method which checks the coordinates between 0 and 
		//the viewing range away from the target in certain increments and see 
		//if they're in the obstacle
		MutableDouble2D testCoord = new MutableDouble2D();
		Double2D amountAdd = new Double2D(Utility.xMovement(bearing, sensitivityForCollisions), Utility.yMovement(bearing, sensitivityForCollisions));
		testCoord.addIn(location);
		
		for(double i = 0; i < viewingRange; i += sensitivityForCollisions)
		{
			//keep adding the amountAdd on and seeing if the coordinate is in the obstacle o
			//going to need to change obstacles to be a subset of entities now so that one 
			//can use the inShape with all of them
			if (o.inShape(new Double2D(testCoord)))
			{
				return true; //the testing doesn't need to continue if it would hit the obstacle at one point
			}
			testCoord.addIn(amountAdd);
			
		}
		
		return false; //the car does not hit the obstacle at any point it can see on it's current course
	}
	
	
	/**
	 * A method which checks the terrain that the car is currently in and then
	 * changes it's properties as required based on the terrain it is currently
	 * in
	 */
	protected void dealWithTerrain() // H 26.8.14 - Changed from private to protected
	{
		// TODO HH - 14.10.14 TO DO - Instead of using location here, we (may) need to use the centre of the 
		// area which overlaps with the road, so:
		// - test for intersection of 'location' with map
		// - if that fails, try an approximation to the centre of the overlap area (max-min/2 in each direction) and 
		//   test for intersection.
		// - if that fails to intersect, then get the max(x+y) and subtract 0.1m in either direction and try that
		// - if that fails to intersect, reduce the 0.1m and try again... repeat
		Double2D testLoc = location;
		
		if (!onMap(testLoc))
		{
			// Try an approximation to the centre of the area of the car which appears on the map,
			// but make sure that the point falls within the shape boundary
			Area carOnMap = getAreaOnMap(getShape());
			testLoc = Utility.getAreaCentre(carOnMap);
			if (!(onMap(testLoc) && carOnMap.contains(testLoc.x, testLoc.y)))
			{
				// Find a 'corner' of the car which has the largest x,y coordinates
				testLoc = Utility.getAreaMaxPt(carOnMap);
				
				if (!carOnMap.contains(testLoc.x, testLoc.y))
				{
					System.out.println("Car.dealWithTerrain: Max Pt not within Car Shape.");
				}
			}
			 
		}
		
		// HH 15.10.14 Now we know we are on the map, check the terrain!
		int type = sim.terrainAtPoint(testLoc); 
		
		switch (type)
		{
			case NORMAL:
				//normal terrain
				performance.reset();
				break;
			case GRAVEL:
				//driving on gravel
				performance.reset();
				performance.setCurrentMaxSpeed(performance.getCurrentMaxSpeed() / 2);
				break;
			case ICE:
				//driving on ice
				performance.reset();
				performance.setCurrentMaxAcceleration(performance.getCurrentMaxAccel() / 2);
				performance.setCurrentMaxDeceleration(performance.getCurrentMaxDecel() / 2);
				performance.setCurrentMaxTurning(performance.getCurrentMaxTurning() / 2);				
				break;
			default:
				performance.reset();
				break;
		}
	}
	
	private boolean checkWall()  
	{
		Double2D me = sim.environment.getObjectLocation(this);
		
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
		else if(Constants.WorldYVal- me.y <= 0.2)
		{
			//System.out.println("clash with the lower wall!");
			return true;
		}
		
		return false;
	}


	public CarPerformance getStats() {
		return performance;
	}

    // TO DO - check whether this has the desired behaviour (may just set all to 0.0)
	public void setStats(CarPerformance stats) {
		this.performance = stats;
	}
	
	// HH 7.5.14 - Added new method as not sure that above method works
	public void setStats(double maxCarSpeed, double maxCarAcceleration, double maxCarDeceleration, double maxCarTurning) {
		this.performance.setCurrentMaxSpeed(maxCarSpeed);
		this.performance.setCurrentMaxAcceleration(maxCarAcceleration);
		this.performance.setCurrentMaxDeceleration(maxCarDeceleration);
		this.performance.setCurrentMaxTurning(maxCarTurning);
	}
	
	/* 
	 * HH 7.5.14 - Added get/set for the targetID/direction/speed for use by the child class UGV
	 */
	public int getTargetID() {
		return targetID;
	}
	
	public void setTargetID(int newID) {
		targetID = newID; // TO DO - Add some range checking here...
	}
	
	public double getDirection() {
		return direction;
	}
		
	public double getSpeed() {
		return speed;
	}
	// HH - end
	
	// HH 16.7.14 - Store previous location
	protected void storePrevLoc(Double2D inPrevLoc) {
		prevLoc = inPrevLoc;
	}
	
	public Double2D getPrevLoc() {
		return prevLoc;
	}
	// HH - end


	/** 
	 * HH 14.7.14 - Work out whether the supplied location is on the Map
	 * HH 26.8.14 - Moved from UGV to Car so can be used by DumbCar; private => protected
	 * 
	 * @param Double2D: location to be checked
	 * @return boolean: true if offset location is on the map
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
	 * HH 8.10.14 - Work out whether the supplied Shape is on the Map
	 * 
	 * @param Shape: Shape to be checked (all must be off map for false return value)
	 * @return boolean: true if inShape is on the map
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
	 * HH 15.10.14 - Work out whether the supplied Shape is on the Map and return the Area of shape
	 * overlapping the map
	 * 
	 * @param Shape: Shape to be checked
	 * @return Area: returns an Area object which is the overlap between inShape and the map
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
	 * HH 15.10.14 - Work out whether the supplied Shape is on the Road and return the Area of shape 
	 * overlapping the road
	 * 
	 * @param Bag: roads collection
	 * @param Shape: shape to be checked for intersection with the road surface
	 * @return Area: Area of inShape intersecting with Road surface (may be Empty)
	 */
	protected Area getAreaOnRoad(Bag roads, Shape inShape)
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
		
		// Constrain the road by the area of the map just in case we have ended up with roads outside of the map.
		Area mapArea = new Area((Shape) new Rectangle2D.Double(0, 0, Constants.WorldXVal, Constants.WorldYVal));
		allRoads.intersect(mapArea);
		
		allRoads.intersect(new Area(inShape)); // Intersect the on-road area with the supplied Shape
		return allRoads; // Return intersection of Shape and allRoads
	}
	
	// TODO  - See below, various methods which will be required to determine the severity of various crashes
	
	/** 
	 * HH 15.10.14 - Return the Area of a supplied Shape which does not intersect with a Road
	 * 
	 * @param Bag: roads collection
	 * @param Shape: shape to be checked for intersection with the road surface
	 * @return Area: Area of inShape intersecting with Road surface (may be Empty)
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
		
		// Create a mapArea whichi s the the XOR of the entire map and all roads.
		Area mapArea = new Area((Shape) new Rectangle2D.Double(0, 0, Constants.WorldXVal, Constants.WorldYVal));
		mapArea.exclusiveOr(allRoads);
		
		mapArea.intersect(new Area(inShape)); // Intersect the off-road area with the supplied Shape
		return mapArea; // Return intersection of Shape and allRoads
	}
	
	
	// TODO - Add a method for returning the intersection Area of a supplied Shape with the bag of moving
	// obstacles
	
	// TODO - Add a method for returning the intersection Area of a supplied Shape with the bag of static 
	// obstacles
	
	/** 
	 * HH 14.7.14 - Work out whether the supplied location is contained within any of the road surfaces
	 * HH 26.8.14 - Moved from UGV to Car so can be used by DumbCar; private => protected
	 * HH 8.10.14 - Updated so that it is a Shape object which is tested for intersection, rather than a Double2D
	 * 
	 * @param Bag: roads collection
	 * @param Shape: shape to be checked for intersection with the road surface
	 * @return boolean: true if location is on any road surface
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

		
		// HH 14.10.14 - Updated so that we test for intersection with the road.  Something is only off-road if it 
		// is entirely off the road i.e. no intersection with the road network at all.  Still need to constrain the
		// road by the area of the map just in case we have ended up with roads outside of the map.
		Area mapArea = new Area((Shape) new Rectangle2D.Double(0, 0, Constants.WorldXVal, Constants.WorldYVal));
		allRoads.intersect(mapArea); // Constrain the roads
		
		allRoads.intersect(new Area(inShape)); // Intersect the on-road area with the supplied Shape
		
		return !allRoads.isEmpty(); // If this is empty => no intersection so totally off-road
		
		// HH 14.10.14 - Older code
//		// Create a map constrained by the boundaries of the world, but not including any of the road surfaces
//		Area mapArea = new Area((Shape) new Rectangle2D.Double(0, 0, Constants.WorldXVal, Constants.WorldYVal));
//		mapArea.subtract(allRoads);
//		
//		// Intersect the off-road area, with the supplied Shape
//		mapArea.intersect(new Area(inShape));		
// 
//		return mapArea.isEmpty();
	}
	
	/** 
	 * HH 13.10.14 - Test a Double2D for intersection with the road surface
	 * 
	 * @param Bag: roads collection
	 * @param Double2D: location to be checked
	 * @return boolean: true if location is on any road surface
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
	 * HH 22.9.14 - Resolve any conflicting speed requirements, giving priority to the 
	 * (safety critical) braking
	 */
	protected void doSpeedCalcs()
	{
		// HH 23.9.14 - Added slow/stop to guarantee a slow down if we detect a moving obstacle ahead
		if (voteSlowStop > 0) {
			changeSpeed(DECELERATE);
		} else if (voteReallySlow > 0) {
			// Adjust here to change performance while cornering 
			final double MinSpeed = sim.getCarMaxDecceleration(); // HH 31/7/14 - changed from 0.2
			final double MaxSpeed = sim.getCarMaxDecceleration();
			
			if (getSpeed() < MinSpeed) {
				changeSpeed(ACCELERATE);
				
				// New Fault #5 (Fault #10) - 22/7/14 - Repeat the command to increase speed again
				if (sim.getFault(5) == true) {
					changeSpeed(ACCELERATE);
					sim.setFault(5);
				}
			} else if (getSpeed() >= MaxSpeed*2) {
				changeSpeed(DECELERATE);
				
				// New Fault #6 (Fault #11) - 22/7/14 - Repeat the command to reduce speed again
				if (sim.getFault(6) == true) {
					changeSpeed(DECELERATE);
					sim.setFault(6);
				}
			}
		} else if (voteSlow > 0 && voteSpeedUp >= 0) { // HH 24.9.14 - force a speed up under certain conditions
			// Adjust here to change performance while cornering 
			final double MinSpeed = 0.5; // HH 31/7/14 - changed from 0.2
			final double MaxSpeed = 0.5;
			
			if (getSpeed() < MinSpeed) {
				changeSpeed(ACCELERATE);
			} else if (getSpeed() >= MaxSpeed) {
				changeSpeed(DECELERATE);
			}
		} else if (voteSpeedUp > 0 || voteSpeedUp == -1) { // HH 24.9.14 - Allow for the negative flag
			changeSpeed(ACCELERATE);
		}
	}
	
	/** 
	 * HH 15.7.14 - adjust the speed for junction/end of map/road
	 * HH 26.8.14 - Moved from UGV to Car so can be used by DumbCar; private => protected
	 */
	protected void goSlow()
	{
		voteSlow++;
	}

	/** 
	 * HH 22.9.14 - vote for a speed up
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
	 * HH 23.9.14 - vote for a guaranteed slow down
	 */
	protected void goSlowStop()
	{
		voteSlowStop++;
	} 
	
	/** 
	 * HH 9.9.14 - Modified version of goSlow() for approaching junctions and parked cars
	 */
	protected void goReallySlow()
	{
		voteReallySlow++;
	}
	
	/** 

	 * HH 22.9.14 - Reset all the speed params
	 */
	protected void resetSpeedParams()
	{
		voteReallySlow = 0;
		voteSlow = 0;
		voteSpeedUp = 0;
		voteSlowStop = 0; // HH 23.9.14 Added new param for guaranteeing slow down
	}
	
	/** 
	 * HH 15.7.14 - Create a new waypoint, and do all the associated manipulation
	 * HH 26.8.14 - Moved from UGV to Car so can be used by DumbCar; private => protected,
	 *              ALSO changed faults so only occur for type == UGV
	 * HH 21.11.14 - Added params so that we can use the WP creation as a replacement for the
	 * 			     existing WP, rather than inserting it between the UGV and the current WP
	 * 
	 * @param Double2D: location for new WP
	 * @param sim: COModel parameter
	 * @param Double2D: current location
	 * @param boolean: isReplacement HH 21.11.14 [true = should replace existing and steal its next point]
	 * @param Waypoint: eTarget_current HH 21.11.14 [current WP so we don't have to search through all entities to find it]
	 * @return Waypoint: the waypoint that is created
	 */
	protected Waypoint createWaypoint(Double2D WPlocation, COModel sim, Double2D location, int type, boolean isReplacement, Entity eTarget_current)
	{
		int wpID = sim.getNewID();
		
		// New Fault #0 (Fault #5) - 22/7/14 - Overwrite the previous id
		if (sim.getFault(0) == true && this.getType() == TUGV) {
			wpID --;
			sim.setFault(0);
		}
		
		// HH 21.11.14 If this new WP is to replace the previous one, we need to find out which
		// next point it was using so that we can copy it to the new WP
		Waypoint wp;
		if (isReplacement == true)
		{
			// HH 21.11.14 - We can't cast the incoming argument for eTarget_current to a Waypoint, as it is possible for this
			// to be a Target, instead of a Waypoint (so we wouldn't be able to call the getNextPoint method on it anyway)
			int nextP = 0;
			if (eTarget_current.getType() == Constants.TTARGET)
			{
				nextP = eTarget_current.getID();
			} else if (eTarget_current.getType() == Constants.TWAYPOINT) { // HH 27.11.14 Check the type is Waypoint before cast or might get exception
				nextP = ((Waypoint) eTarget_current).getNextPoint();
				sim.environment.remove(eTarget_current); // Now that we are done with the old point, remove it.
			}
			
			wp = new Waypoint(wpID, nextP, type); // Use the current next point as the new next point (replace current WP)
		} else {
			wp = new Waypoint(wpID, getTargetID(), type); // Regular case, using the current targetID as the next point (add WP on way to current WP)
		}
				
		Double2D tempLoc = new Double2D(WPlocation.x, WPlocation.y); // copy the desired location
		
		// New Faults #1,2,3,4 (Faults #6,7,8,9) - 22/7/14 - Displace the location for the WP	
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
		setDirection(location, tempLoc);
		return wp; //return wp so we can set eTarget to point to it
	}
	
	/*
	 *  HH 8.9.14 - return the distance to a moving obstacle which is found on the road ahead.  Search conducted at a 
	 *  range of up to 100m.  Search progresses in a similar way to the roadMarkings search, and looks for the closest
	 *  hit on an obstacle within a narrow range: Constants.UGVMovObsViewingAngle
	 *  
	 *  @param sim
	 *  @param inCar - the vehicle that has been detected as being on the same road
	 *  @param sensorLoc - for oncoming traffic, pass in the offside front corner location
	 *  @param sameLane - true if we are looking for other moving vehicles ahead of us in the lane, false for oncoming vehicles
	 */
	
	private Double2D checkForMovingObstacle(COModel sim, Car inCar, Double2D sensorLoc, boolean sameLane, double inAngle, double inRange, double inSensitivity) {
		
		//simple and dirty method which checks the coordinates between 0 and 
		//the moving obstacle viewing range away from the target in certain increments and see 
		//if they intersect with the supplied car (moving obs) 
		MutableDouble2D testCoord = new MutableDouble2D();
		Double2D amountAdd = new Double2D();
		double reqDistance = inRange;
		Double2D reqCoord = new Double2D(-1,-1);
		double distance;
		
		// HH 13.8.14 Need to restrict the obstacle checks to those which are in the same lane as
		// the UGV, so need to know direction in order to restrict in method below
		UGV_Direction direction = Utility.getDirection(getDirection()); // Returns a compass direction rather than angle
		
		// HH 9.9.14 Need to work out the bounds for the lane that we are searching in, by checking the
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
		
		// HH 28.7.14 - Added variables for the loop to support fault insertion
		double startAngle = -(inAngle/2);
		double endAngle = (inAngle / 2);
		double startRange = 0;
		double endRange = inRange;
		double rangeSensitivity = inSensitivity;
				
//		// FAULT #17 - HH 28/7/14 - Force the angle loop to start half-way through
//		if (sim.getFault(17) == true) {
//			startAngle = 0;
//		} 
//
//		// FAULT #18 - HH 28/7/14 - Force the angle loop to end half-way through
//		if (sim.getFault(18) == true) {
//			endAngle = 0;
//		} 
//		
//		// FAULT #19 - HH 28/7/14 - Force the angle sensor to 'reduce' range resolution (double iterator step)
//		if (sim.getFault(19) == true) {
//			resolution = resolution*2;
//		} 
				
		for(double i = startAngle; i <= endAngle; i += resolution)
		{
			// Reset the location that we start testing from to be the location of the UGV and set the bearing
			// that we are going to use for this iteration
			testCoord.setTo(0,0);
			testCoord.addIn(sensorLoc); // HH 25.9.14 Displaced to model sensor on front of vehicle
			newBearing = Utility.correctAngle(getDirection() + i);
			
			// Construct the x an y increments for each iteration below
			amountAdd = new Double2D(Utility.xMovement(newBearing, rangeSensitivity), Utility.yMovement(newBearing, rangeSensitivity));
						
//			// FAULT #21 - HH 28/7/14 - Force the angle loop to start half-way through
//			if (sim.getFault(21) == true) {
//				startRange = UGVViewingRange/2;
//			} 
//
//			// FAULT #22 - HH 28/7/14 - Force the angle loop to end half-way through
//			if (sim.getFault(22) == true) {
//				endRange = UGVViewingRange/2;
//			} 
//			
//			// FAULT #23 - HH 28/7/14 - Force the sensor to 'reduce' angular resolution (double iterator step)
//			if (sim.getFault(23) == true) {
//				rangeSensitivity = sensitivityForRoadTracking*2;
//			} 
						
		    // NOTE - j is not actually used, it just ensures the correct number of iterations
			for(double j = startRange; j <= endRange; j += rangeSensitivity){
												
				testCoord.addIn(amountAdd);  // move the test location outwards on the chosen bearing
				
				// HH 4.12.14 Ensure that our test coordinate is between us and the edge of the road,
				// as soon as we hit a point that is no longer on the road surface then we should 
				// discontinue our search.
				if (sim.roadAtPoint(new Double2D(testCoord), sim.roads) == false) {
					break; // Try searching again at the next bearing.
				}
								
				// HH 13.8.14 Ensure that the our test coordinate is within our test bounds
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
					// HH 24.9.14 - Find out which class of car we have so we can call the right inShape method
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

						// Exit the loop as we don't need to search any further as we've found an obstacle
						break;
					}
				}
			}
		}
		
		return reqCoord;
	}
	
	/**
	 * HH 8.9.14 - Return the closest observed intersection with a moving obstacle
	 *  
	 * @param sim
	 * @param bearing
	 * @param sameLane
	 * @return distance to the obstacle
	 */
	protected Double2D checkAllMovingObstacles(COModel sim, Bag inCars, boolean sameLane, double inAngle, double inRange, double inSensitivity)
	{
		// Init to the values we would want for finding the min
		double reqDistance = inRange + 1;
		double currentDistance = inRange + 1;
		
		// Calculate the location of the sensor, assuming it is on the front offside corner of the vehicle
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
//			// FAULT #14 - HH 28/7/14 - Exit the loop half-way through
//			if (sim.getFault(14) == true) {
//				if (i == (roads.size()/2)) {
//					break;
//				}
//			}
			
			// HH 8.9.14 - Work out which road the car is on, so that we can compare the ids 
			// - we are only interested in cars that are on the same road as the UGV
			currentCar = (Car) inCars.get(i);
			if (currentCar.isActive == true && (currentCar.ID != this.ID))
			{
				currentDistanceCoord = checkForMovingObstacle(sim, currentCar, sensorLoc, sameLane, inAngle, inRange, inSensitivity);

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
//			// FAULT #14 - HH 28/7/14 - Exit the loop half-way through
//			if (sim.getFault(14) == true) {
//				if (i == (roads.size()/2)) {
//					break;
//				}
//			}
			
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
		if (sim.getFault(8) == true) {
			startAngle = 0;
			sim.setFault(8); // HH 13.11.14 Added
		} 

		// New Fault #9 (FAULT #18) - HH 28/7/14 - Force the angle loop to end half-way through
		if (sim.getFault(9) == true) {
			endAngle = 0;
			sim.setFault(9); // HH 13.11.14 Added
		} 
		
		// New Fault #10 (FAULT #19) - HH 28/7/14 - Force the angle sensor to 'reduce' range resolution (double iterator step)
		if (sim.getFault(10) == true) {
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
			if (sim.getFault(11) == true) {
				startRange = inRange/2;
				sim.setFault(11); // HH 13.11.14 Added
			} 

			// New Fault #12 (FAULT #22) - HH 28/7/14 - Force the angle loop to end half-way through
			if (sim.getFault(12) == true) {
				endRange = inRange/2;
				sim.setFault(12); // HH 13.11.14 Added
			} 
			
			// New Fault #13 (FAULT #23) - HH 28/7/14 - Force the sensor to 'reduce' angular resolution (double iterator step)
			if (sim.getFault(13) == true) {
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
		
		// HH 29.9.14 This method didn't really work properly as the intersection point cannot reliably tell you 
		// which vehicle is in front when they are overlapping.		
//		double bearing = getDirection();
//		
//		if (bearing >= 315 && bearing < 45) {
//			if (inCoord.y < location.y)
//			{
//				retVal = -1;
//			}
//		} else if (bearing >= 45 && bearing < 135) {
//			if (inCoord.x < location.x)
//			{
//				retVal = -1;
//			}	
//		} else if (bearing >= 135 && bearing < 225) {
//			if (inCoord.y > location.y)
//			{
//				retVal = -1;
//			}	
//		} else if (bearing >= 225 && bearing < 315) {
//			if (inCoord.x > location.x)
//			{
//				retVal = -1;
//			}	
//		}
//		
		return (retVal * location.distance(inCoord));
	}
	
	/* 
	 * HH 24.9.14 - Abstract method so don't have to cast to run from subclasses
	 */
	public abstract Shape getShape();
}
