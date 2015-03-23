package modeling;

import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.Area;
import java.awt.geom.Rectangle2D;

import sim.portrayal.Oriented2D;
import sim.util.Double2D;

public class ParkedCar extends Obstacle implements Oriented2D {

	private static final long serialVersionUID = 1L;
	
	private int roadId;
	
	/**
	 * Updated to pass in the direction
	 * @param int idNo ()
	 * @param int typeNo ()
	 * @param double inDirection ()
	 * @param int inRoadId ()
	 */
	public ParkedCar(int idNo, int typeNo, double inDirection, int inRoadId) {
		super(idNo, typeNo, inDirection);
		roadId = inRoadId; // HH 13.8.14 - Added to allow check for whether UGV and Parked Car are on same road
	}
	
	/**
	 * method which returns true or false if a provided coordinate is in the shape
	 * would have to be overwritten when implemented
	 * @param Double2D coord ()
	 * @return boolean ()
	 */
	public boolean inShape(Double2D coord)
	{
		Shape carShape = getShape();		
		return carShape.contains(coord.x, coord.y);
	}	

	/**
	 * HH 2.10.14 method which returns true or false if a provided area intersects with the
	 * shape
	 * @param Shape inShape ()
	 * @return boolean ()
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
	 * @return int ()
	 */
	public int getRoadId()
	{
		return roadId;
	}
	
	/**
	 * Updated to use same method as DumbCar to return a Shape object representing the parked car 
	 * obstacle and centred at location
	 * @return Shape ()
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
}
