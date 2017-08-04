package modeling;
import java.awt.geom.Path2D;
import java.awt.geom.Rectangle2D;

import modeling.COModel.jctExitDirInfo;
import modeling.COModel.jctExitInfo;
import sim.util.*;

/**
 * This class will describe an intersection, or junction which can be of type T or X, and
 * which is used for enforcing 'special' driving behaviour in the vicinity of a junction
 * e.g. slow down to stop, 3 or 4 way stop behaviour.  Vehicles will need to be aware that they
 * are approaching a junction in order to adapt their behaviour accordingly.  The junctions
 * will be stored as 'point' objects corresponding to the centre of the junction, they will
 * be drawn as squares (rectangles with a length and width equal to Road.roadWidth), and they 
 * will have methods for representing themselves as a small square like the above, or a larger
 * polygon which will include the junction approaches where vehicles may need to start to slow
 * down.
 *
 * @author hh940
 */
public class Junction extends Entity
{	
	private static final long serialVersionUID = 1L;

	// Car speed is added to increase buffer as assumes starts to decel at 20m (varies due to speed)
	public static final int jctApproachLen = (20 + (int)Math.ceil(CAR_SPEED));  // Supposed to reflect car performance (45km/hr to 0 in 20m) 
	                                                          
	private double[] lengthDir = new double[T_WEST+1];
	
	// Implementing 4 Way Stop
	private boolean occupied = false;
	private long occupiedTime = 0; // store the timestep at which the junction became occupied
	
	// Store the ID of the vehicle who has occupied the junction (used to validate unOccupy requests)
	private long occupierID = 0;
	
	/**
	 * Constructor.  Create the Entity, set its location, and populated the lengthDir array with either
	 * the length of road extending in each direction from the junction, or jctApproachLen if the road
	 * extends beyond the junction approach distance.
	 * @param idNo (int - unique identifier assigned to junction)
	 * @param x (double - x coordinate of junction centre)
	 * @param y (double - y coordinate of junction centre)
	 * @param nDir (double - length of road extending from junction in NB direction; limited to jctApproachLen)
	 * @param eDir (double - length of road extending from junction in EB direction; limited to jctApproachLen)
	 * @param sDir (double - length of road extending from junction in SB direction; limited to jctApproachLen)
	 * @param wDir (double - length of road extending from junction in WB direction; limited to jctApproachLen)
	 */
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
	 * @param coord (Double2D - coordinates of location we want to check for intersection with the junction)
	 * @return boolean (returns true if the location is inside the junction)
	 */
	public boolean inShape(Double2D coord)
	{
		// Construct a rectangle that is the size of the whole road area (rather than just the centre line)
		// and check whether it contains the supplied coordinates.
		Rectangle2D.Double junctionArea = getJunctionArea();
		return Utility.betterContains(junctionArea, coord);
	}	
	
	/**
	 * Method returns true if coord intersects with the junction or junction approach (when constructed at required dimensions), false otherwise
	 * @param coord (Double2D - coordinates of location we want to check for intersection with the junction approach)
	 * @return boolean (returns true if the location is inside the junction approach)
	 */
	public boolean inApproach(Double2D coord)
	{
		// Construct a Path2D that is the size of the whole junction approach area
		Path2D.Double junctionApproach = getJunctionApproach();
		return junctionApproach.contains(coord.x, coord.y);
	}
	
	/**
	 * Method returns true if coord intersects with the junction or junction exit - analoguous to approach (when extended by 
	 * required width), false otherwise
	 * @param coord (Double2D - coordinates of location we want to check for intersection with the junction exit)
	 * @return boolean (returns true if the location is inside the junction exit)
	 */
	public boolean inExit(Double2D coord)
	{
		// Construct a Path2D that is the size of the whole junction exit area
		Path2D.Double junctionExit = getJunctionExit();
		return junctionExit.contains(coord.x, coord.y);
	}
	
	/**
	 *  Method returns a Rectangle2D which represents the internal junction area (rather than just the centre point)
	 *  @return Rectangle2D.Double (returns a Rectangle2D which represents the shape of the junction area)
	 */
	public Rectangle2D.Double getJunctionArea()
	{
		// Calculate the coordinates of the upper left corner of the rectangle
		return new Rectangle2D.Double((location.x-(Road.roadWidth/2)), (location.y-(Road.roadWidth/2)), Road.roadWidth, Road.roadWidth);	
	}
	
	/**
	 *  Method returns a Path2D which represents the junction approaches (rather than just the centre point)
	 *  The lengths of each junction arm are used to determine whether to construct a junction approach
	 *  in each direction.  If the arm length is zero, the path is not extended to include a junction approach
	 *  in that direction.  The junction approach length is limited to jctApproachLen or the length of the 
	 *  junction arm (if that is shorter).  Only includes the actual approach lanes, rather than including the 
	 *  lanes that are leaving the junction.
	 *  @return Path2D.Double (return a Path2D.Double which represents the area of the junction and junction approaches)
	 */	
	public Path2D.Double getJunctionApproach()
	{
		Path2D.Double junctionApproach = new Path2D.Double();
		
		double x = location.x;
		double y = location.y;
		double halfWidth = Road.roadWidth/2;
		
		junctionApproach.moveTo(x+halfWidth,  y-halfWidth); // start location is inner top right so present regardless of T-junction type
		
		// Draw around the EB junction approach if it exists
		if (lengthDir[T_EAST] > 0) {
			junctionApproach.lineTo(x+halfWidth, y);
			junctionApproach.lineTo(x+lengthDir[T_EAST], y);
			junctionApproach.lineTo(x+lengthDir[T_EAST], y+halfWidth);
		}
		
		// Either complete the EB approach, or omit the EB arm if the above was not executed
		junctionApproach.lineTo(x+halfWidth, y+halfWidth);
		
		// Draw around the SB junction approach if it exists
		if (lengthDir[T_SOUTH] > 0) {
			junctionApproach.lineTo(x, y+halfWidth);
			junctionApproach.lineTo(x, y+lengthDir[T_SOUTH]);
			junctionApproach.lineTo(x-halfWidth, y+lengthDir[T_SOUTH]);
		}
		
		// Either complete the SB approach, or omit the SB arm if the above was not executed
		junctionApproach.lineTo(x-halfWidth, y+halfWidth);
		
		// Draw around the WB junction approach if it exists
		if (lengthDir[T_WEST] > 0) {
			junctionApproach.lineTo(x-halfWidth, y);
			junctionApproach.lineTo(x-lengthDir[T_WEST], y);
			junctionApproach.lineTo(x-lengthDir[T_WEST], y-halfWidth);
		}
		
		// Either complete the WB approach, or omit the WB arm if the above was not executed
		junctionApproach.lineTo(x-halfWidth, y-halfWidth);
		
		// Draw around the NB junction approach if it exists
		if (lengthDir[T_NORTH] > 0) {
			junctionApproach.lineTo(x, y-halfWidth);
			junctionApproach.lineTo(x, y-lengthDir[T_NORTH]); 
			junctionApproach.lineTo(x+halfWidth, y-lengthDir[T_NORTH]);
		}
			
		// Either complete the NB approach, or omit the NB arm if the above was not executed
		junctionApproach.closePath(); // Close the 'cross' up
	
		return junctionApproach;
	}
	
	/**
	 *  Method returns a Path2D which represents the junction exits (rather than just the centre point)
	 *  The lengths of each junction arm are used to determine whether to construct a junction exit
	 *  in each direction.  If the arm length is zero, the path is not extended to include a junction exit
	 *  in that direction.  The junction exit length is limited to jctApproachLen or the length of the 
	 *  junction arm (if that is shorter).  Only includes the actual exit lanes, rather than including the 
	 *  lanes that are entering the junction.
	 *  @return Path2D.Double (return a Path2D.Double which represents the area of the junction and junction exits)
	 */	
	public Path2D.Double getJunctionExit()
	{
		Path2D.Double junctionExit = new Path2D.Double();
		
		double x = location.x;
		double y = location.y;
		double halfWidth = Road.roadWidth/2;
		
		junctionExit.moveTo(x+halfWidth,  y-halfWidth); // start location is inner top right so present regardless of T-junction type
		
		// Draw around the EB junction approach if it exists
		if (lengthDir[T_EAST] > 0) {
			junctionExit.lineTo(x+lengthDir[T_EAST], y-halfWidth);
			junctionExit.lineTo(x+lengthDir[T_EAST], y);
			junctionExit.lineTo(x+halfWidth, y);
		}
		
		// Either complete the EB approach, or omit the EB arm if the above was not executed
		junctionExit.lineTo(x+halfWidth, y+halfWidth);
		
		// Draw around the SB junction approach if it exists
		if (lengthDir[T_SOUTH] > 0) {
			junctionExit.lineTo(x+halfWidth, y+lengthDir[T_SOUTH]);
			junctionExit.lineTo(x, y+lengthDir[T_SOUTH]);
			junctionExit.lineTo(x, y+halfWidth);
		}
		
		// Either complete the SB approach, or omit the SB arm if the above was not executed
		junctionExit.lineTo(x-halfWidth, y+halfWidth);
		
		// Draw around the WB junction approach if it exists
		if (lengthDir[T_WEST] > 0) {
			junctionExit.lineTo(x-lengthDir[T_WEST], y+halfWidth);
			junctionExit.lineTo(x-lengthDir[T_WEST], y);
			junctionExit.lineTo(x-halfWidth, y);
		}
		
		// Either complete the WB approach, or omit the WB arm if the above was not executed
		junctionExit.lineTo(x-halfWidth, y-halfWidth);
		
		// Draw around the NB junction approach if it exists
		if (lengthDir[T_NORTH] > 0) {
			junctionExit.lineTo(x-halfWidth, y-lengthDir[T_NORTH]); 
			junctionExit.lineTo(x, y-lengthDir[T_NORTH]);
			junctionExit.lineTo(x, y-halfWidth);
		}
			
		// Either complete the NB approach, or omit the NB arm if the above was not executed
		junctionExit.closePath(); // Close the cross up
	
		return junctionExit;
	}
	
	/**
	 *  Method returns a Double2D which represents the centre of the egress lane from the junction which 
	 *  should bring the vehicle closer to the target.  This can return the adjacent (and opposite direction)
	 *  lane if a U-turn is deemed to be the most appropriate movement.  The location returned is slightly 
	 *  outside of the junction so that upon 'collection' by the UGV, the UGV is no longer in the junction.
	 *  ALSO checks the junction history to see whether the exit has already been visited.
	 *  i) on the first iteration, junctions which have not been visited yet are chosen in preference
	 *  to those which have been visited, and those which bring the UGV closer (crow-flies) to the target
	 *  are prioritised. This happens each time the UGV approaches the junction until all exits have been
	 *  visited at least once, see below: 
	 *  ii) on future iterations, exits that have already been visited *may* still be selected
	 *  but with reducing probability (the more times they have been visited, the less likely they are to
	 *  be chosen again.  If more than one exit is identified as a possibility during an iteration, those 
	 *  which bring the UGV closer (crow-flies) to the target are prioritised; loop continues until an exit 
	 *  is found.  
	 *  
	 *  The junction history stores a count of the number of times an exit
	 *  has been chosen so that the probability used on future iterations (as ii) will reduce the
	 *  more times it has been chosen.  Return type includes the target direction so that we can prevent the
	 *  UGV from overshooting the target direction of travel.
	 *  @param target (Double2D - coordinates of the UGV's target)
	 *  @param theUGV (UGV - the UGV which has arrived at the junction)
	 *  @param idx (int - index used to access this Junction in the Junctions Bag)
	 *  @param sim (COModel - access to the simulation environment)
	 *  @return jctExitDirInfo (return the location and bearing that the UGV should be aiming for)
	 */	
	public jctExitDirInfo getJunctionExit(Double2D target, UGV theUGV, int idx, COModel sim)
	{
		// This is not a great implementation of 4-way stops (as not necessarily fair when more than one car 
		// is waiting as this method does not take into account the length of time that a car has been waiting, it is 
		// just the car/UGV which calls this method first following the junction becoming unoccupied that 
		// will be given the chance to move.)
		
		// Firstly need to check to see if the junction is already in use (or there is no point
		// going any further)
		if (occupied) 
		{	
			// Check the time that the occupied flag was set and if it has exceeded a maximum time, then clear 
			// the occupied flag and allow the method to continue below.  
			if (sim.schedule.getSteps() > (occupiedTime + 80)) // Timeout after 80s
			{
				occupierID = 0;
				occupied = false;
				occupiedTime = 0;
			} else {
				return new jctExitDirInfo(new Double2D(-1, -1), 999); // Return error code, junction is legitimately occupied
			}			
		}
		
		// Make sure that the UGV isn't still occupying a different junction; if it is, clear that junction first
		if (theUGV.getJctID() != this.ID)
		{
			sim.unOccupyJunction(theUGV.getJctID(), sim.junctions, theUGV.ID);
		}
		
		// We've got here, so this junction should now be occupied!
		occupied = true;
		occupiedTime = sim.schedule.getSteps(); // Timestamp of some kind here to facilitate timeout clearing of flag
		
		occupierID = theUGV.getID(); // Store the ID of the UGV occupying the junction
		
		// Make sure we don't confuse idx and ID
		theUGV.setJctID(this.ID);
		theUGV.stopWaiting(); // Doesn't matter if the vehicle wasn't actually waiting or not, call this anyway
		
		double x = location.x;
		double y = location.y;
		double laneWidth = Road.roadWidth/2;
		final double offset = 1.5; // Offset to move location outside of junction, distance is set a little 
		                           // further so that WP doesn't get 'eaten' prematurely
		
		int [][] junctionHistory = theUGV.getJunctionHistory(); // Read in the junction history array from the UGV
		
		Double2D exitCoords = new Double2D(-1,-1);
		Double2D tempCoords = new Double2D(0,0);
		int loopCount = 0;
		UGV_Direction selected = UGV_Direction.NORTH;
		
		// Loop until we find valid exit coordinates, or we reach the loop timeout at 1000 steps, note that we
		// use the supplied idx parameter, which is the Bag.get(idx).  This is what we use in the junctionHistory
		// array for identifying the junction as it keeps the array size to be the number of junctions, rather
		// than a substantially larger array if we wanted to index by junction identifier.  NOTE: because we do not
		// change the elements in the Junctions bag, the locations (idx) in the Bag should remain constant. If we 
		// were adding/removing junctions, then we would no longer be able to guarantee the items would be in the 
		// same places (see sim.util.Bag in MASON documentation).
		while (exitCoords.x == -1 && exitCoords.y == -1 && loopCount < 1000) {
		
			// Check the EB exit. If we have not visited this direction at this junction yet, or there
			// are no junctions which haven't been visited and a biased coin flip selects this one.
			if (lengthDir[T_EAST] > 0 && ((junctionHistory[idx][UGV_Direction.EAST.ordinal()] == 0) || 
			     (loopCount > 0 && (sim.random.nextDouble() < (Math.pow(0.8, (junctionHistory[idx][UGV_Direction.EAST.ordinal()]))))))) 
			{
				tempCoords = new Double2D(x+laneWidth, y-(laneWidth/2)); // Mid-point of exit lane
				
				// If we haven't found a valid exit yet, or this exit location will bring us closer to our
				// target than the one we have previously selected, then update our exitCoords to this new
				// exit location (plus an offset into the lane).  Set this direction as selected.
				if ((exitCoords.x == -1 && exitCoords.y == -1) || (exitCoords.distance(target) > tempCoords.distance(target))) {
					exitCoords = new Double2D(tempCoords.x + offset, tempCoords.y);
					selected = UGV_Direction.EAST;
				}
			}

			// Check the SB exit. If we have not visited this direction at this junction yet, or there
			// are no junctions which haven't been visited and a biased coin flip selects this one.
			if (lengthDir[T_SOUTH] > 0 && ((junctionHistory[idx][UGV_Direction.SOUTH.ordinal()] == 0) || 
				  (loopCount > 0 && (sim.random.nextDouble() < (Math.pow(0.8, (junctionHistory[idx][UGV_Direction.SOUTH.ordinal()]))))))) 
			{
				tempCoords = new Double2D(x+(laneWidth/2), y+laneWidth); // Mid-point of exit lane

				// If we haven't found a valid exit yet, or this exit location will bring us closer to our
				// target than the one we have previously selected, then update our exitCoords to this new
				// exit location (plus an offset into the lane).  Set this direction as selected.
				if ((exitCoords.x == -1 && exitCoords.y == -1) || (exitCoords.distance(target) > tempCoords.distance(target))) {
					exitCoords = new Double2D(tempCoords.x, tempCoords.y + offset);
					selected = UGV_Direction.SOUTH;
				}
			}

			// Check the WB exit. If we have not visited this direction at this junction yet, or there
			// are no junctions which haven't been visited and a biased coin flip selects this one.
			if (lengthDir[T_WEST] > 0 && ((junctionHistory[idx][UGV_Direction.WEST.ordinal()] == 0) || 
				  (loopCount > 0 && (sim.random.nextDouble() < (Math.pow(0.8, (junctionHistory[idx][UGV_Direction.WEST.ordinal()]))))))) 
			{
				tempCoords = new Double2D(x-laneWidth, y+(laneWidth/2)); // Mid-point of exit lane

				// If we haven't found a valid exit yet, or this exit location will bring us closer to our
				// target than the one we have previously selected, then update our exitCoords to this new
				// exit location (plus an offset into the lane).  Set this direction as selected.
				if ((exitCoords.x == -1 && exitCoords.y == -1) || (exitCoords.distance(target) > tempCoords.distance(target))) {
					exitCoords = new Double2D(tempCoords.x - offset, tempCoords.y);
					selected = UGV_Direction.WEST;
				}
			}

			// Check the NB exit. If we have not visited this direction at this junction yet, or there
			// are no junctions which haven't been visited and a biased coin flip selects this one.
			if (lengthDir[T_NORTH] > 0 && ((junctionHistory[idx][UGV_Direction.NORTH.ordinal()] == 0) || 
				  (loopCount > 0 && (sim.random.nextDouble() < (Math.pow(0.8, (junctionHistory[idx][UGV_Direction.NORTH.ordinal()]))))))) 
			{
				tempCoords = new Double2D(x-(laneWidth/2), y-laneWidth); // Mid-point of exit lane

				// If we haven't found a valid exit yet, or this exit location will bring us closer to our
				// target than the one we have previously selected, then update our exitCoords to this new
				// exit location (plus an offset into the lane).  Set this direction as selected.
				if ((exitCoords.x == -1 && exitCoords.y == -1) || (exitCoords.distance(target) > tempCoords.distance(target))) {
					exitCoords = new Double2D(tempCoords.x, tempCoords.y - offset);
					selected = UGV_Direction.NORTH;
				}
			}
			
			loopCount ++;
		}
		
		// Check for a valid return value and if so, update the junctionHistory array so that we 
		// know we've just visited this junction exit (stores a count of the number of visits)
		if (exitCoords.x != -1)
		{
			theUGV.updateJunctionHistory(idx, selected);
		}
		
		return new jctExitDirInfo(exitCoords, Utility.getDirectionDeg(selected)); // Return the location and bearing we are aiming for
	}
	
	/**
	 *  Method returns a Double2D which represents the centre of the egress lane from the junction.  
	 *  This is chosen at random, and can return the adjacent (and opposite direction) for a U-turn.  
	 *  The location returned is slightly outside of the junction so that 
	 *  upon 'collection' by the Car, the Car is no longer in the junction.
	 *  Loop continues until a valid exit is chosen.  Where the junction is actually a dead end, the
	 *  vehicle is permitted to continue straight ahead and leave the road network.
	 *  @param theCar (DumbCar - the Car which has arrived at the junction)
	 *  @param idx (int - index used to access this Junction in the Junctions Bag)
	 *  @param sim (COModel - access to the simulation environment)
	 *  @return jctExitInfo (return the location and bearing that the DumbCar should be aiming for)
	 */	
	public jctExitInfo getRandomExit(DumbCar theCar, int idx, COModel sim)
	{
		// Firstly need to check to see if the junction is already in use (or there is no point
		// going any further
		if (occupied) {
			
			// Check the time that the occupied flag was set and if it has exceeded a maximum time, then clear 
			// the occupied flag and allow the method to continue below.  
			if (sim.schedule.getSteps() > (occupiedTime + 30)) // Reset junction occupancy to zero after 30 steps (more aggressive than UGV)
			{
				occupierID = 0;
				occupied = false;
				occupiedTime = 0;
			} else {
				return new jctExitInfo(new Double2D(-1, -1), this.ID); // Error Code to indicate junction occupied
			}
		}
		
		// Make sure that the Car isn't still occupying a different junction
		if (theCar.getJctID() != this.ID)
		{
			sim.unOccupyJunction(theCar.getJctID(), sim.junctions, theCar.ID);
		}
				
		// We've reached this point, so the junction is now occupied!
		occupied = true;
		occupiedTime = sim.schedule.getSteps(); // Timestamp of some kind here to facilitate timeout clearing of flag
		
		occupierID = theCar.getID(); // Store the ID of the DumbCar occupying the junction
		
		// Make sure we dnn't confuse ID and idx
		theCar.setJctID(this.ID);
		theCar.stopWaiting(); // Doesn't matter if the vehicle wasn't actually waiting or not, call this anyway
		
		double x = location.x;
		double y = location.y;
		double laneWidth = Road.roadWidth/2;
		final double offset = 1; // Offset to move location outside of junction
		
		Double2D exitCoords = new Double2D(-1,-1);
		Double2D tempCoords = new Double2D(0,0);
		int loopCount = 0;
		
		// Actually need to know which direction the Car is facing in order to
		// ensure that it can leave the junction at a dead-end, but that it cannot 
		// perform a u-turn to exit from a dead-end at which it has only just joined.
		UGV_Direction carDir = Utility.getDirection(theCar.getDirection());
		
		// Work out if this is a dead-end (used to permit the dead-end to be used as an exit by a DumbCar)
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
				
		// Loop until we have chosen a direction that is actually present
		while (exitCoords.x == -1 && exitCoords.y == -1 && loopCount < 1000) {
		
			// Choose a number at random, will specify the nth available direction (in the ordering)
			// - this is used to effect the random choice, although it has to correspond to the 
			// direction chosen being available (that's why we need lots of iterations, to 
			// maximise our chances of finding a match)
			int dir = sim.random.nextInt(4) + 1; // there will always be a minimum of 1 direction
			
			// If we've chosen dir=1, and there is an EB arm, or this is a EB dead-end which the vehicle is heading towards, choose EB
			if ((lengthDir[T_EAST] > 0 || (dirCount == 1 && lengthDir[T_EAST] == 0 && carDir == UGV_Direction.EAST)) && dir == 1) 
			{
				tempCoords = new Double2D(x+laneWidth, y-(laneWidth/2)); // Mid-point of exit lane
				if (exitCoords.x == -1 && exitCoords.y == -1) {
					exitCoords = new Double2D(tempCoords.x + offset, tempCoords.y);
				}
			// If we've chosen dir=2, and there is an SB arm, or this is a SB dead-end which the vehicle is heading towards, choose SB	
			} else if ((lengthDir[T_SOUTH] > 0 || (dirCount == 1 && lengthDir[T_SOUTH] == 0 & carDir == UGV_Direction.SOUTH)) && dir == 2) 
			{
				tempCoords = new Double2D(x+(laneWidth/2), y+laneWidth); // Mid-point of exit lane
				if (exitCoords.x == -1 && exitCoords.y == -1) {
					exitCoords = new Double2D(tempCoords.x, tempCoords.y + offset);
				}
			// If we've chosen dir=3, and there is an WB arm, or this is an WB dead-end which the vehicle is heading towards, choose WB	
			} else if ((lengthDir[T_WEST] > 0 || (dirCount == 1 && lengthDir[T_WEST] == 0 && carDir == UGV_Direction.WEST)) && dir == 3) 
			{
				tempCoords = new Double2D(x-laneWidth, y+(laneWidth/2)); // Mid-point of exit lane
				if (exitCoords.x == -1 && exitCoords.y == -1) {
					exitCoords = new Double2D(tempCoords.x - offset, tempCoords.y);
				}
			// If we've chosen dir=4, and there is an NB arm, or this is a NB dead-end which the vehicle is heading towards, choose NB
			} else if ((lengthDir[T_NORTH] > 0 || (dirCount == 1 && lengthDir[T_NORTH] == 0 && carDir == UGV_Direction.NORTH)) && dir == 4) 
			{
				tempCoords = new Double2D(x-(laneWidth/2), y-laneWidth); // Mid-point of exit lane
				if (exitCoords.x == -1 && exitCoords.y == -1) {
					exitCoords = new Double2D(tempCoords.x, tempCoords.y - offset);
				}
			}
			
			loopCount ++;
		}
				
		return new jctExitInfo(exitCoords, this.ID); // Return the offset exit location and the ID of the junction
	}
	
	/** 
	 * Unoccupy the junction: includes a check on the id of the vehicle that is trying to clear the lock on the junction
	 * to make sure it's the vehicle that we think is in the junction.
	 * @param inID (long - ID of the vehicle trying to unoccupy the junction)
	 */
	public void unOccupy(long inID)
	{
		if (occupierID == inID) {
			occupied = false;
			occupiedTime = 0;
			occupierID = 0;
		}
	}
	
	/**
	 * Return the ID of the vehicle occupying the junction (for logging purposes)
	 * @return long (return the ID of the vehicle occupying the junction)
	 */
	public long getOccupierID() {
		return occupierID;
	}
	
	/**
	 * This method iterates through the junction arm length array and counts any non-zero length
	 * junction arms.  If there is only one, this signifies a dead-end and true is returned.  Any
	 * other count implies no dead-end and false is returned.
	 * @return boolean (return true if the junction is a dead-end)
	 */
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
	
	/**
	 * Return the coordinate pair representing the entry point that should be used to add a new 
	 * DumbCar vehicle to the network in a dead-end
	 * @return Double2D (coordinates of location (just inside junction) where the new DumbCar should be 
	 * added to the network)
	 */
	public Double2D getDeadEndEntry()
	{
		double x = location.x;
		double y = location.y;
		double laneWidth = Road.roadWidth/2 - 0.1; // A small offset to prevent vehicles being added just outside net
		
		// Add the vehicles at the outside edge of the junction so that
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
				
		return new Double2D(-1,-1); // Shouldn't ever get here...
	}
}
