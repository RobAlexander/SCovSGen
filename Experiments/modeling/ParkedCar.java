package modeling;

import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.Area;
import java.awt.geom.Rectangle2D;

import sim.portrayal.Oriented2D;
import sim.util.Double2D;

public class ParkedCar extends Obstacle implements Oriented2D {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	
	private int roadId;
	
	// HH 2.10.14 - Updated to pass in the direction
	public ParkedCar(int idNo, int typeNo, double inDirection, int inRoadId) {
		super(idNo, typeNo, inDirection);
		roadId = inRoadId; // HH 13.8.14 - Added to allow check for whether UGV and Parked Car are on same road
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

	/**
	 * HH 2.10.14 method which returns true or false if a provided area intersects with the
	 * shape
	 */
	public boolean inShape(Shape inShape)
	{
		Shape carShape = getShape();	
		Area carArea = new Area (carShape);
		carArea.intersect(new Area(inShape));
		return !carArea.isEmpty();
	}	
	
	/**
	 * method which returns the id of the road on which the Obstacle is located
	 */
	public int getRoadId()
	{
		return roadId;
	}
	
	/**
	 * HH 2.10.14 - Updated to use same method as DumbCar to return a Shape object representing the parked car obstacle and centred at location
	 */
	public Shape getShape()
	{
		// The location is the centre of this shape which we will assume is of the size given in
		// the Constants file for an obstacle
		double widthOffset = Constants.OBSTACLE_WIDTH/2;
		double lengthOffset = Constants.OBSTACLE_LENGTH/2;
		
		// 24.9.14 Return a shape aligned with the oriented vehicles
		Rectangle2D.Double carRectangle = new Rectangle2D.Double();
		
		// HH 24.9.14 - Assume the basic shape is as it would appear when pointed along the x-axis, so this means some swapping around of width/length
		carRectangle = new Rectangle2D.Double(location.x - lengthOffset, location.y - widthOffset, Constants.OBSTACLE_LENGTH, Constants.OBSTACLE_WIDTH);		
		AffineTransform rotateTransform = AffineTransform.getRotateInstance(orientation2D(), location.x, location.y);
		Shape carShape = rotateTransform.createTransformedShape(carRectangle);
		
		return carShape;
	}


	
// HH 2.10.14 - Replaced with method above
//	public Rectangle2D.Double getShape()
//	{
//		// The location is the centre of this shape which we will assume is of the size given in
//		// the Constants file
//		double widthOffset = Constants.OBSTACLE_WIDTH/2;
//		double lengthOffset = Constants.OBSTACLE_LENGTH/2;
//		Rectangle2D.Double carShape;
//		
//		// NOTE - this needs to take into account the orientation of the lane on which the obstacle is positioned as
//		// this will affect which direction to apply the widthOffset and lengthOffset in
//		if (isNS == true) {
//			carShape = new Rectangle2D.Double(location.x-widthOffset, location.y-lengthOffset, 
//					Constants.OBSTACLE_WIDTH, Constants.OBSTACLE_LENGTH);
//		} else {
//			carShape = new Rectangle2D.Double(location.x-lengthOffset, location.y-widthOffset, 
//					Constants.OBSTACLE_LENGTH, Constants.OBSTACLE_WIDTH);
//		}
//		
//		return carShape;
//	}	
	
}
