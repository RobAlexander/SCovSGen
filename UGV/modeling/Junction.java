package modeling;
import java.awt.geom.Path2D;
import java.awt.geom.Rectangle2D;

import modeling.COModel.jctExitDirInfo;
import modeling.COModel.jctExitInfo;
import sim.util.*;

/**
 * This class will describe an intersection, or junction which can be of type T or X, and
 * which is used for enforcing 'special' driving behaviour in the vicinity of a junction
 * e.g. slow down to stop, 3/4 way stop behaviour.  Vehicles will need to be aware that they
 * are approaching a junction in order to adapt their behaviour accordingly.  The junctions
 * will be stored as 'point' objects corresponding to the centre of the junction, they will
 * be drawn as squares (rectangles with a length and width equal to Road.roadWidth), and they 
 * will have methods for representing themselves as a small square like the above, or a larger
 * square which will include the junction approaches where vehicles may need to start to slow
 * down.
 *
 * @author hh940
 */
public class Junction extends Entity
{	
	public static final int jctApproachLen = (20 + (int)Math.ceil(CAR_SPEED));  // HH - 30/7/14 updated in line with new car performance (45km/hr to 0 in 20m) 
	                                                          // HH 27.8.14 Added car speed to increase buffer as assumes starts to decel at 20m (varies due to speed)
	
	//private int direction;
	private double[] lengthDir = new double[T_WEST+1];
	
	// HH 3.9.14 Implementing 4 Way Stop
	private boolean occupied = false;
	private long occupiedTime = 0; // store the timestep at which the junction became occupied
	
	// HH 23.12.14 - Store the ID of the vehicle who has occupied the junction (used to validate unOccupy requests)
	private long occupierID = 0;
	
//	public Junction(int idNo, double x, double y, int inDirection)
//	{
//		super(idNo, TTJUNCTION);
//		this.setLocation(new Double2D(x,y));
//		
//		if (inDirection >= T_NORTH && inDirection <= T_WEST ) {
//			this.direction = inDirection;
//		} else {
//			this.direction = -1;
//			// TO DO - Raise an error here.
//		}
//	}
	
	public Junction(int idNo, double x, double y, double nDir, double eDir, double sDir, double wDir)
	{
		super(idNo, TTJUNCTION);
		this.setLocation(new Double2D(x,y));
		
		this.lengthDir[T_NORTH] = Math.min(nDir, jctApproachLen);
		this.lengthDir[T_EAST] = Math.min(eDir, jctApproachLen);
		this.lengthDir[T_SOUTH] = Math.min(sDir, jctApproachLen);
		this.lengthDir[T_WEST] = Math.min(wDir, jctApproachLen);
	}
	
	
	/**
	 * Method returns true if coord intersects with the junction object (when extended by required width), false otherwise
	 */
	public boolean inShape(Double2D coord)
	{
		// Construct a rectangle that is the size of the whole road area (rather than just the centre line)
		Rectangle2D.Double junctionArea = getJunctionArea();
		
		// TODO - DOes this method need to be updated?
		return junctionArea.contains(coord.x, coord.y);
	}	
	
	/**
	 * Method returns true if coord intersects with the junction or junction approach (when extended by required width), false otherwise
	 */
	public boolean inApproach(Double2D coord)
	{
		// Construct a rectangle that is the size of the whole road area (rather than just the centre line)
		Path2D.Double junctionApproach= getJunctionApproach();
		
		// TODO - DOes this method need to be updated?
		return junctionApproach.contains(coord.x, coord.y);
	}
	
	/**
	 * HH - 18.8.14 Method returns true if coord intersects with the junction or junction exit - analoguous to approach (when extended by 
	 * required width), false otherwise
	 */
	public boolean inExit(Double2D coord)
	{
		// Construct a rectangle that is the size of the whole road area (rather than just the centre line)
		Path2D.Double junctionExit = getJunctionExit();
		
		// TODO - DOes this method need to be updated?
		return junctionExit.contains(coord.x, coord.y);
	}
	
	/**
	 *  Method returns a Rectangle2D which represents the internal junction area (rather than just the centre point)
	 */
	public Rectangle2D.Double getJunctionArea()
	{
		// Calculate the coordinates of the upper left corner of the rectangle
		return new Rectangle2D.Double((location.x-(Road.roadWidth/2)), (location.y-(Road.roadWidth/2)), Road.roadWidth, Road.roadWidth);	
	}
	
	/**
	 *  Method returns a Path2D which represents the junction approaches (rather than just the centre point)
	 *  
	 *  NOTE: This can only handle T-junctions at present (or cross-roads if the direction is set to -1), 
	 *  and in the case where a road is joined very close to the end of another road, the resulting junction is 
	 *  simply a right-angle corner.  Currently, this will still draw as a T-junction.  TO DO - Could solve this
	 *  by storing 2 directions to ignore, but more complicated to pass to constructor as need to check how much 
	 *  road is left either side of each newly created junction.
	 *  
	 *  HH - Modified on 9.7.14 so that only include the actual approach lanes, rather than the lanes that are leaving the junction
	 */	
	public Path2D.Double getJunctionApproach()
	{
		Path2D.Double junctionApproach = new Path2D.Double();
		
		double x = location.x;
		double y = location.y;
		double halfWidth = Road.roadWidth/2;
		
		junctionApproach.moveTo(x+halfWidth,  y-halfWidth); // start location is inner top right so present regardless of T-junction type
		
		if (lengthDir[T_EAST] > 0) {
			junctionApproach.lineTo(x+halfWidth, y);
			junctionApproach.lineTo(x+lengthDir[T_EAST], y);
			junctionApproach.lineTo(x+lengthDir[T_EAST], y+halfWidth);
		}
		
		junctionApproach.lineTo(x+halfWidth, y+halfWidth);
		
		if (lengthDir[T_SOUTH] > 0) {
			junctionApproach.lineTo(x, y+halfWidth);
			junctionApproach.lineTo(x, y+lengthDir[T_SOUTH]);
			junctionApproach.lineTo(x-halfWidth, y+lengthDir[T_SOUTH]);
		}
		
		junctionApproach.lineTo(x-halfWidth, y+halfWidth);
		
		if (lengthDir[T_WEST] > 0) {
			junctionApproach.lineTo(x-halfWidth, y);
			junctionApproach.lineTo(x-lengthDir[T_WEST], y);
			junctionApproach.lineTo(x-lengthDir[T_WEST], y-halfWidth);
		}
		
		junctionApproach.lineTo(x-halfWidth, y-halfWidth);
		
		if (lengthDir[T_NORTH] > 0) {
			junctionApproach.lineTo(x, y-halfWidth);
			junctionApproach.lineTo(x, y-lengthDir[T_NORTH]); 
			junctionApproach.lineTo(x+halfWidth, y-lengthDir[T_NORTH]);
		}
			
		junctionApproach.closePath(); // Close the cross up
	
		return junctionApproach;
	}
	
	/**
	 *  Method returns a Path2D which represents the junction exits (rather than just the centre point)
	 *  
	 *  NOTE: This can only handle T-junctions at present (or cross-roads if the direction is set to -1), 
	 *  and in the case where a road is joined very close to the end of another road, the resulting junction is 
	 *  simply a right-angle corner.  Currently, this will still draw as a T-junction.  TO DO - Could solve this
	 *  by storing 2 directions to ignore, but more complicated to pass to constructor as need to check how much 
	 *  road is left either side of each newly created junction.

	 */	
	public Path2D.Double getJunctionExit()
	{
		Path2D.Double junctionExit = new Path2D.Double();
		
		double x = location.x;
		double y = location.y;
		double halfWidth = Road.roadWidth/2;
		
		junctionExit.moveTo(x+halfWidth,  y-halfWidth); // start location is inner top right so present regardless of T-junction type
		
		if (lengthDir[T_EAST] > 0) {
			junctionExit.lineTo(x+lengthDir[T_EAST], y-halfWidth);
			junctionExit.lineTo(x+lengthDir[T_EAST], y);
			junctionExit.lineTo(x+halfWidth, y);
		}
		
		junctionExit.lineTo(x+halfWidth, y+halfWidth);
		
		if (lengthDir[T_SOUTH] > 0) {
			junctionExit.lineTo(x+halfWidth, y+lengthDir[T_SOUTH]);
			junctionExit.lineTo(x, y+lengthDir[T_SOUTH]);
			junctionExit.lineTo(x, y+halfWidth);
		}
		
		junctionExit.lineTo(x-halfWidth, y+halfWidth);
		
		if (lengthDir[T_WEST] > 0) {
			junctionExit.lineTo(x-lengthDir[T_WEST], y+halfWidth);
			junctionExit.lineTo(x-lengthDir[T_WEST], y);
			junctionExit.lineTo(x-halfWidth, y);
		}
		
		junctionExit.lineTo(x-halfWidth, y-halfWidth);
		
		if (lengthDir[T_NORTH] > 0) {
			junctionExit.lineTo(x-halfWidth, y-lengthDir[T_NORTH]); 
			junctionExit.lineTo(x, y-lengthDir[T_NORTH]);
			junctionExit.lineTo(x, y-halfWidth);
		}
			
		junctionExit.closePath(); // Close the cross up
	
		return junctionExit;
	}
	
	/**
	 *  Method returns a Double2D which represents the centre of the egress lane from the junction which 
	 *  should bring the vehicle closer to the target.  This can return the adjacent (and opposite direction)
	 *  lane if a U-turn is deemed to be the most appropriate movement.  
	 *  
	 *  15/7/14 - Updates so that location returned is slightly outside of the junction so that 
	 *  upon 'collection' by the UGV, the UGV is no longer in the junction.
	 *  ALSO checks the junction history to see whether the exit has already been checked
	 *  i) on the first iteration, will ignore any exits that have already been visited
	 *  ii) on future iterations, exits that have already been visited *may* still be selected
	 *  but with 50% probability; loop continues until an exit is found.
	 *  
	 *  16/7/14 - Updated so that junction history now stores a count of the number of times an exit
	 *  has been chosen so that the probability used on future iterations (as ii) will reduce the
	 *  more times it has been chosen
	 *  
	 *  22/12/13 - Updated so return type now includes the target direction so that we can prevent the
	 *  UGV from overshooting the target direction of travel.
	 */	
	//public Double2D getJunctionExit(Double2D target, UGV theUGV, int idx, COModel sim)
	public jctExitDirInfo getJunctionExit(Double2D target, UGV theUGV, int idx, COModel sim)
	{
		// HH 3.9.14 - Implementing 4-way stops (not necessarily fair when more than one car is waiting
		// as this method does not take into account the length of time that a car has been waiting, it is 
		// just the car/UGV which calls this method first following the junction becoming unoccupied that 
		// will be given the chance to move.)
		
		// Firstly need to check to see if the junction is already in use (or there is no point
		// going any further)
		if (occupied) 
		{	
			// Check the time that the occupied flag was set and if it has exceeded a maximum time, then clear 
			// the occupied flag and allow the method to continue below.  
			if (sim.schedule.getSteps() > (occupiedTime + 80)) // HH 16.12.14 increased from 50 steps to 120 17.12.14 Down to 80 steps
			{
				occupierID = 0; // 23.12.14 New var
				occupied = false;
				occupiedTime = 0;
			} else {
				return new jctExitDirInfo(new Double2D(-1, -1), 999); // HH 18.12.14 Doesn't appear that anyone uses the ID returned!
				//return new Double2D(-1, this.ID); // Error Code to indicate junction occupied, returns jctID
			}			
		}
		
		// HH 23.12.14 Make sure that we aren't still occupying a different junction
		if (theUGV.getJctID() != this.ID)
		{
			sim.unOccupyJunction(theUGV.getJctID(), sim.junctions, theUGV.ID);
		}
		
		// HH 3.9.14 Implementing 4-way stops, we've got here, so the junction is now occupied!
		occupied = true;
		occupiedTime = sim.schedule.getSteps(); // Timestamp of some kind here to facilitate timeout clearing of flag
		
		// HH 23.12.14 New var
		occupierID = theUGV.getID();
		
		// HH 16.10.14 - Updated so doesn't confuse idx and ID
		theUGV.setJctID(this.ID);
		//theUGV.setJctID(idx);
		theUGV.stopWaiting(); // Doesn't matter if the vehicle wasn't actually waiting or not, call this anyway
		
		double x = location.x;
		double y = location.y;
		double laneWidth = Road.roadWidth/2;
		final double offset = 1; // HH 15.7.14 offset to move location outside of junction
		
		int [][] junctionHistory = theUGV.getJunctionHistory();
		
		Double2D exitCoords = new Double2D(-1,-1);
		Double2D tempCoords = new Double2D(0,0);
		int loopCount = 0;
		UGV_Direction selected = UGV_Direction.NORTH;
		
		while (exitCoords.x == -1 && exitCoords.y == -1 && loopCount < 1000) {
		
			if (lengthDir[T_EAST] > 0 && ((junctionHistory[idx][UGV_Direction.EAST.ordinal()] == 0) || (loopCount > 0 && (sim.random.nextDouble() < (Math.pow(0.8, (junctionHistory[idx][UGV_Direction.EAST.ordinal()]))))))) 
			{
				tempCoords = new Double2D(x+laneWidth, y-(laneWidth/2));
				if ((exitCoords.x == -1 && exitCoords.y == -1) || (exitCoords.distance(target) > tempCoords.distance(target))) {
					exitCoords = new Double2D(tempCoords.x + offset, tempCoords.y);
					selected = UGV_Direction.EAST;
				}
			}

			if (lengthDir[T_SOUTH] > 0 && ((junctionHistory[idx][UGV_Direction.SOUTH.ordinal()] == 0) || (loopCount > 0 && (sim.random.nextDouble() < (Math.pow(0.8, (junctionHistory[idx][UGV_Direction.SOUTH.ordinal()]))))))) 
			{
				tempCoords = new Double2D(x+(laneWidth/2), y+laneWidth);
				if ((exitCoords.x == -1 && exitCoords.y == -1) || (exitCoords.distance(target) > tempCoords.distance(target))) {
					exitCoords = new Double2D(tempCoords.x, tempCoords.y + offset);
					selected = UGV_Direction.SOUTH;
				}
			}

			if (lengthDir[T_WEST] > 0 && ((junctionHistory[idx][UGV_Direction.WEST.ordinal()] == 0) || (loopCount > 0 && (sim.random.nextDouble() < (Math.pow(0.8, (junctionHistory[idx][UGV_Direction.WEST.ordinal()]))))))) 
			{
				tempCoords = new Double2D(x-laneWidth, y+(laneWidth/2));
				if ((exitCoords.x == -1 && exitCoords.y == -1) || (exitCoords.distance(target) > tempCoords.distance(target))) {
					exitCoords = new Double2D(tempCoords.x - offset, tempCoords.y);
					selected = UGV_Direction.WEST;
				}
			}

			if (lengthDir[T_NORTH] > 0 && ((junctionHistory[idx][UGV_Direction.NORTH.ordinal()] == 0) || (loopCount > 0 && (sim.random.nextDouble() < (Math.pow(0.8, (junctionHistory[idx][UGV_Direction.NORTH.ordinal()]))))))) 
			{
				tempCoords = new Double2D(x-(laneWidth/2), y-laneWidth);
				if ((exitCoords.x == -1 && exitCoords.y == -1) || (exitCoords.distance(target) > tempCoords.distance(target))) {
					exitCoords = new Double2D(tempCoords.x, tempCoords.y - offset);
					selected = UGV_Direction.NORTH;
				}
			}
			
			loopCount ++;
		}
		
		// HH 15.7.14 - Check for a valid return value and if so, update the junctionHistory array
		if (exitCoords.x != -1)
		{
			theUGV.updateJunctionHistory(idx, selected);
		}
		
		return new jctExitDirInfo(exitCoords, Utility.getDirectionDeg(selected));
	}
	
	/**
	 *  Method returns a Double2D which represents the centre of the egress lane from the junction which 
	 *  is chosen at random.  This can return the adjacent (and opposite direction) for a U-turn.  
	 *  
	 *  26.8.14 - The location returned is slightly outside of the junction so that 
	 *  upon 'collection' by the Car, the Car is no longer in the junction.
	 *  Loop continues until a valid exit is chosen.  Where the junction is actually a dead end, the
	 *  vehicle is permitted to continue straight ahead and leave the road network.
	 */	
	public jctExitInfo getRandomExit(DumbCar theCar, int idx, COModel sim)
	{
		// HH 3.9.14 - Implementing 4-way stops
		// Firstly need to check to see if the junction is already in use (or there is no point
		// going any further
		if (occupied) {
			
			// Check the time that the occupied flag was set and if it has exceeded a maximum time, then clear 
			// the occupied flag and allow the method to continue below.  
			if (sim.schedule.getSteps() > (occupiedTime + 30)) // HH 16.12.14 Changed from 20 steps to 120, 17.12.14 Down to 30 steps
			{
				occupierID = 0; // 23.12.14 New var
				occupied = false;
				occupiedTime = 0;
			} else {
				return new jctExitInfo(new Double2D(-1, -1), this.ID); // Error Code to indicate junction occupied, returns jctID HH 18.12.14 Updated to separate parts of return value
			}
		}
		
		// HH 23.12.14 Make sure that we aren't still occupying a different junction
		if (theCar.getJctID() != this.ID)
		{
			sim.unOccupyJunction(theCar.getJctID(), sim.junctions, theCar.ID);
		}
				
		// HH 3.9.14 Implementing 4-way stops, we've got here, so the junction is now occupied!
		occupied = true;
		occupiedTime = sim.schedule.getSteps(); // Timestamp of some kind here to facilitate timeout clearing of flag
		
		// HH 23.12.14 New var
		occupierID = theCar.getID();
		
		// HH 16.10.14 Updated so doesn't confuse ID and idx
		theCar.setJctID(this.ID);
		//theCar.setJctID(idx);
		theCar.stopWaiting(); // Doesn't matter if the vehicle wasn't actually waiting or not, call this anyway
		
		double x = location.x;
		double y = location.y;
		double laneWidth = Road.roadWidth/2;
		final double offset = 1; // HH 15.7.14 offset to move location outside of junction
		
		Double2D exitCoords = new Double2D(-1,-1);
		Double2D tempCoords = new Double2D(0,0);
		int loopCount = 0;
		
		// HH 24.11.14 Actually need to know which direction the Car is facing in order to
		// ensure that it can leave the junction at a dead-end, but that it cannot 
		// perform a u-turn to exit from a dead-end at which it has only just joined.
		UGV_Direction carDir = Utility.getDirection(theCar.getDirection());
		
		// Work out if this is a dead-end (used to permit the dead-end to be used as an exit by a DumbCar)
		UGV_Direction selected = UGV_Direction.NORTH;
		int dirCount = 0;
		if (lengthDir[T_EAST] > 0) {
			dirCount++;
			selected = UGV_Direction.EAST;
		}
		
		if (lengthDir[T_SOUTH] > 0) {
			dirCount++;
			selected = UGV_Direction.SOUTH;
		}
		
		if (lengthDir[T_WEST] > 0) {
			dirCount++;
			selected = UGV_Direction.WEST;
		}
		
		if (lengthDir[T_NORTH] > 0) {
			dirCount++;
			selected = UGV_Direction.NORTH;
		}		
				
		// Loop until we have chosen a direction that is actually present
		while (exitCoords.x == -1 && exitCoords.y == -1 && loopCount < 1000) {
		
			// Choose a number at random, will specify the nth available direction (in the ordering)
			int dir = sim.random.nextInt(4) + 1; // there will always be a minimum of 1 direction
			
			// HH 16.10.14 - To fix a bug, with newly entering vehicles doing immediate UTurns, swapped
			// the && selected == UGV_Direction.NORTH and && selected == UGV_Direction.SOUTH in the code
			// below. 24.11.14 - This was not really a bug fix, it probably introduced a different bug and
			// only half-addressed the perceived one.
			if ((lengthDir[T_EAST] > 0 || (dirCount == 1 && lengthDir[T_EAST] == 0 && carDir == UGV_Direction.EAST)) && dir == 1) {
				tempCoords = new Double2D(x+laneWidth, y-(laneWidth/2));
				if (exitCoords.x == -1 && exitCoords.y == -1) {
					exitCoords = new Double2D(tempCoords.x + offset, tempCoords.y);
				}
			} else if ((lengthDir[T_SOUTH] > 0 || (dirCount == 1 && lengthDir[T_SOUTH] == 0 & carDir == UGV_Direction.SOUTH)) && dir == 2) {
				tempCoords = new Double2D(x+(laneWidth/2), y+laneWidth);
				if (exitCoords.x == -1 && exitCoords.y == -1) {
					exitCoords = new Double2D(tempCoords.x, tempCoords.y + offset);
				}
			} else if ((lengthDir[T_WEST] > 0 || (dirCount == 1 && lengthDir[T_WEST] == 0 && carDir == UGV_Direction.WEST)) && dir == 3) {
				tempCoords = new Double2D(x-laneWidth, y+(laneWidth/2));
				if (exitCoords.x == -1 && exitCoords.y == -1) {
					exitCoords = new Double2D(tempCoords.x - offset, tempCoords.y);
				}
			} else if ((lengthDir[T_NORTH] > 0 || (dirCount == 1 && lengthDir[T_NORTH] == 0 && carDir == UGV_Direction.NORTH)) && dir == 4) {
				tempCoords = new Double2D(x-(laneWidth/2), y-laneWidth);
				if (exitCoords.x == -1 && exitCoords.y == -1) {
					exitCoords = new Double2D(tempCoords.x, tempCoords.y - offset);
				}
			}
			
			loopCount ++;
		}
				
		return new jctExitInfo(exitCoords, this.ID);
	}
	
	// HH 23.12.14 Update to include a check on the id of the vehicle that is trying to clear the lock on the junction
	public void unOccupy(long inID)
	{
		if (occupierID == inID) {
			occupied = false;
			occupiedTime = 0;
			occupierID = 0;
		}
	}
	
	// HH 23.12.14 For logging
	public long getOccupierID() {
		return occupierID;
	}
	
	public boolean isDeadEnd()
	{
		int dirCount = 0;
		if (lengthDir[T_EAST] > 0) {
			dirCount++;
		}
		
		if (lengthDir[T_SOUTH] > 0) {
			dirCount++;
		}
		
		if (lengthDir[T_WEST] > 0) {
			dirCount++;
		}
		
		if (lengthDir[T_NORTH] > 0) {
			dirCount++;
		}	
		
		if (dirCount == 1)
		{
			return true;
		} else {
			return false;
		}
	}
	
	/*
	 * HH 9.9.14 getDeadEndEntry() - Return the coordinate pair representing the 
	 * entry point that should be used to add a new vehicle to the network.
	 */
	public Double2D getDeadEndEntry()
	{
		double x = location.x;
		double y = location.y;
		double laneWidth = Road.roadWidth/2 - 0.1; // HH 30.9.14 Introduced a small offset to prevent vehicles being added just outside net
		
		// HH 29.9.14 - Add the vehicles at the outside edge of the junction so that
		// they can choose a new direction
		if (lengthDir[T_EAST] > 0) {
			return new Double2D(x-laneWidth, y-(laneWidth/2));
		}
		
		if (lengthDir[T_SOUTH] > 0) {
			return new Double2D(x+(laneWidth/2), y-laneWidth);
		}
		
		if (lengthDir[T_WEST] > 0) {
			return new Double2D(x+laneWidth, y+(laneWidth/2));
		}
		
		if (lengthDir[T_NORTH] > 0) {
			return new Double2D(x-(laneWidth/2), y+laneWidth);
		}	
		
		// HH 29.9.14 - Old code which adds vehicles at the interface between the junction and the road
//		if (lengthDir[T_EAST] > 0) {
//			return new Double2D(x+laneWidth, y-(laneWidth/2));
//		}
//		
//		if (lengthDir[T_SOUTH] > 0) {
//			return new Double2D(x+(laneWidth/2), y+laneWidth);
//		}
//		
//		if (lengthDir[T_WEST] > 0) {
//			return new Double2D(x-laneWidth, y+(laneWidth/2));
//		}
//		
//		if (lengthDir[T_NORTH] > 0) {
//			return new Double2D(x-(laneWidth/2), y-laneWidth);
//		}	
		
		return new Double2D(-1,-1); // Shouldn't ever get here...
	}
	
}
