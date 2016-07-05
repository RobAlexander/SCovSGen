/**
 * 
 */
package modeling;

import java.awt.geom.Area;
import java.awt.geom.PathIterator;
import java.awt.geom.Rectangle2D;
import java.util.Calendar;

import modeling.Constants.UGV_Direction;
import sim.util.Double2D;

/**
 * @author hh940
 * 
 * Class of static methods that were removed from other classes to make them
 * appear more accessible, generic, and more sensibly grouped.
 *
 */
public class Utility {

	/**
	 * Constructor - unnecessary as we only use the static methods.
	 */
	public Utility() {
		// TODO Auto-generated constructor stub
	}
	
	/**
	 * As the basic 'contains' method does not include points which are located *on* the eastern and
	 * southern boundaries of the rectangular shape, this method does.
	 * @param testRect (Rectangle2D.Double - the rectangle we are testing for containing the point)
	 * @param testPt (Double2D - the coordinates we are testing for intersection with the rectangle)
	 * @return boolean (return true if testPt is contained within testRect)
	 */
	public static boolean betterContains(Rectangle2D.Double testRect, Double2D testPt)
	{
		// Simple case
		if (testRect.contains(testPt.x, testPt.y) == true)
		{
			return true;
		}
		
		// Otherwise see if the point is on the eastern boundary
		double xE = testRect.x + testRect.width;
		double yE1 = testRect.y;
		double yE2 = testRect.y + testRect.height;
		
		if (testPt.x == xE && testPt.y >= yE1 && testPt.y <= yE2)
		{
			return true;
		}
		
		// Otherwise see if the point is on the southern boundary
		double xS1 = testRect.x;
		double xS2 = xE;
		double yS = yE2;
		
		if (testPt.y == yS && testPt.x >= xS1 && testPt.x <= xS2)
		{
			return true;
		}		
		
		return false; // if we get here, it's definitely not contained
	}
	
	// ***** Seven static direction methods (below) moved from UGV/Car class *****
	
	/** 
	 * Work out which compass direction the vehicle is pointing in, based on its heading
	 * @param bearing (double - the direction the vehicle is pointing in)
	 * @return UGV_Direction (either NORTH, EAST, SOUTH, or WEST to indicate approximate direction of vehicle)
	 */
	public static UGV_Direction getDirection(double bearing)
	{
		if (correctAngle(bearing) >= 315 || correctAngle(bearing) < 45)	{
			return UGV_Direction.SOUTH;
		} else if (correctAngle(bearing) < 135) {
			return UGV_Direction.EAST;
		} else if (correctAngle(bearing) < 225) {
			return UGV_Direction.NORTH;
		} else { // must be between 225 and 315 
			return UGV_Direction.WEST;
		}
	}

	/** 
	 * Work out which compass direction the vehicle is pointing in, based on its heading,
	 * but return the degrees-equivalent for this compass direction
	 * @param bearing (double - the direction the vehicle is pointing in)
	 * @return int (either NORTH (180), EAST (90), SOUTH (0), or WEST (270) to indicate 
	 *              approximate direction of vehicle)
	 */
	public static int getDirectionDeg(double bearing)
	{
		if (correctAngle(bearing) >= 315 || correctAngle(bearing) < 45)	{
			return 0;
		} else if (correctAngle(bearing) < 135) {
			return 90;
		} else if (correctAngle(bearing) < 225) {
			return 180;
		} else { // must be between 225 and 315 
			return 270;
		}
	}

	/**
	 * Return 1 for N or E; 2 for S or W
	 * Designed to compliment Road.getLane which returns similar coded directions.
	 * Used to check that the target is on the side of the road which aligns with the
	 * direction of travel of the UGV (for clarity when the UGV is performing an O/T).
	 * @param bearing (double - the direction the vehicle is pointing in)
	 * @return int (return 1 for N or E; 2 for S or W)
	 */
	public static int getDirectionInt(double bearing)
	{
		if (correctAngle(bearing) >= 315 || correctAngle(bearing) < 45)	{
			return 2; // South
		} else if (correctAngle(bearing) < 135) {
			return 1; // East
		} else if (correctAngle(bearing) < 225) {
			return 1; // North
		} else { // must be between 225 and 315 
			return 2; // West
		}
	}
	
	/** 
	 * Return the degrees-equivalent for this compass direction 
	 * @param UGV_Direction bearing (either NORTH, EAST, SOUTH, or WEST to indicate approximate direction of vehicle)
	 * @return int (angular equivalent of bearing)
	 */
	public static int getDirectionDeg(UGV_Direction bearing)
	{
		if (bearing == UGV_Direction.SOUTH)	{
			return 0;
		} else if (bearing == UGV_Direction.EAST) {
			return 90;
		} else if (bearing == UGV_Direction.NORTH) {
			return 180;
		} else { // must be between 225 and 315 
			return 270;
		}
	}
	
	/** 
	 * A method which changes a bearing to be in the range of 0 (inclusive) to 360 (exclusive)
	 * @param b (double - the bearing to be corrected)
	 * @return double (a bearing equivalent to b which has been converted to be in the correct range)
	 */
	protected static double correctAngle(double b)
	{
		if (b >= 360)
		{
			return (b - 360);
		}
		
		if (b < 0)
		{
			return (b + 360);
		}
		
		return b;
	}
	
	/**
	 * Returns the current orientation of the object in radians (as per
	 * OrientedPortrayal2D) for the supplied bearing
	 * @param inBearing (double - the direction the vehicle is pointing in)
	 * @return double (the orientation2D of the vehicle in radians)
	 */
	public static double getOrientation2D(double inBearing)
	{
		// For some reason, the orientation of the vehicle seems to be displayed 
		// relative to zero degrees along the increasing x axis.  As a result, we need to flip 
		// the compass over the 45/225 degree bisection to convert between the two systems.
		return Math.toRadians(correctAngle(90-inBearing));
	}
	
	/**
	 * Calculates the bearing the vehicle should be travelling on to move directly
	 * from a location to another.
	 * TODO Might want to insert some test routines for this as the maths is a little
	 * counter-intuitive
	 * @param point1 (Double2D - coordinates of start point)
	 * @param point2 (Double2D - coordinates of end point)
	 * @return double (return the bearing to travel from point1 to point2)
	 */
	public static double calculateAngle(Double2D point1, Double2D point2) 
	{
		Double2D vector = point2.subtract(point1); // find difference between points
		double angle;
		
		if(vector.y != 0)
		{
			angle = Math.toDegrees(Math.atan(vector.x / vector.y));
			
			// Deal with signs cancelling etc., to ensure that the angle returned is relative to the
			// increasing y-axis
			if(vector.x > 0)
			{
				if (vector.y < 0) 
				{	
					angle += 180;
				} 
			} else {
				if (vector.y < 0) 
				{	
					angle += 180;
				} else {
					angle += 360;
				}
			}
			
		} else { // vector.y == 0 
			
			// The car is either in line with the target horizontally or vertically
			if (vector.x > 0)
			{
			    angle = 90;			    
			} else {
				angle = 270;
			}
		}
		
		return angle;
	}

    /**
     * A function which based on the direction the car is facing and the speed it
	 * is travelling at returns a value for how much the x position should change in one step. 
	 * TODO - Insert some test scripts for this of standard cases and boundary conditions
	 * @param angle (double - direction in which the car is pointing)
	 * @param speed (double - the speed of the vehicle)
     * @return double (the change in x coordinate of the car in the world)
     */
	public static double xMovement(double angle, double speed) 
	{
		double xChange;
		
		if (angle <= 90) 
		{
			xChange = (speed * Math.sin(Math.toRadians(angle)));
		} else if (angle <= 180) {
			xChange = (speed * Math.sin(Math.toRadians(180 - angle)));
		} else if (angle <= 270) {
			xChange = (-1 * speed * Math.cos(Math.toRadians(270 - angle)));
		} else {
			xChange = (-1 * speed * Math.sin(Math.toRadians(360 - angle)));
		}	
		return xChange;
    }
	
	/**
	 * The y axis equivalent of the xMovement method
	 * @param angle (double - direction in which the car is pointing)
	 * @param speed (double - the speed of the vehicle)
	 * @return double (the change in y coordinate of the car in the world)
	 */
	public static double yMovement(double angle, double speed)
	{
		double yChange;
		if (angle <= 90) 
		{
			yChange = (speed * Math.cos(Math.toRadians(angle)));
		} else if (angle <= 180) {
			yChange = (-1 * speed * Math.cos(Math.toRadians(180 - angle)));
		} else if (angle <= 270) {
			yChange = (-1 * speed * Math.sin(Math.toRadians(270 - angle)));
		} else {
			yChange = (speed * Math.cos(Math.toRadians(360 - angle)));
		}	
		return yChange;
    }
	
	/**
	 * This method converts the current time to a string for use in the log files, etc.
	 * TODO: this method might benefit from some refinement and testing, it's possible that
	 * it muddles up AM/PM.
	 * @return String (the current time represented as a string)
	 */
	public static String timeToString() {
		
		Calendar timeNow = Calendar.getInstance();
		String timeString = "";
		
		if (timeNow.get(Calendar.HOUR) == 0) {
			if (timeNow.get(Calendar.AM_PM) == Calendar.AM) {
				timeString += "00:";
			} else {
				timeString += "12:";
			}
		} else if (timeNow.get(Calendar.HOUR) < 10) {
			timeString += "0";
			timeString += timeNow.get(Calendar.HOUR);
			timeString += ":";
		} else {
			timeString += timeNow.get(Calendar.HOUR);
			timeString += ":";
		}
		
		if (timeNow.get(Calendar.MINUTE) < 10) {
			timeString += "0";
			timeString += timeNow.get(Calendar.MINUTE);
			timeString += ":";
		} else {
			timeString += timeNow.get(Calendar.MINUTE);
			timeString += ":";
		}		

		if (timeNow.get(Calendar.SECOND) < 10) {
			timeString += "0";
			timeString += timeNow.get(Calendar.SECOND);
			timeString += ":";
		} else {
			timeString += timeNow.get(Calendar.SECOND);
			timeString += ":";
		}
		
		timeString += timeNow.get(Calendar.MILLISECOND);
		
		return timeString;
	}
}
