package modeling;

import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.Area;
import java.awt.geom.Rectangle2D;

import sim.portrayal.Oriented2D;
import sim.util.Double2D;

/**
 * @author hh940
 * 
 * This class represents ParkedCar obstacles.  These are initialised when the map is generated
 * and they do not change.  They are located at a fixed offset from the kerb and have fixed,
 * constant dimensions.
 */
public class ParkedCar extends Obstacle implements Oriented2D {

	private static final long serialVersionUID = 1L;
	
	private int roadId; // Store the ID of the Road the ParkedCar is located on
	
	/**
	 * Constructor.  Construct the underlying Obstacle object, and store the supplied Road ID
	 * @param idNo (int - unique identifier for the ParkedCar object)
	 * @param typeNo (int - type identifier for the ParkedCar)
	 * @param inDirection (double - direction in which the ParkedCar is pointing, 0 if the obstacle is oriented N/S or 90 if it is oriented E/W)
	 * @param inRoadId (int - ID for the road the ParkedCar is added on)
	 */
	public ParkedCar(int idNo, int typeNo, double inDirection, int inRoadId) {
		super(idNo, typeNo, inDirection);
		roadId = inRoadId; // To allow check for whether UGV and Parked Car are on same road
	}
	
	/**
	 * Method which returns true or false if a provided coordinate is in the shape
	 * @param coord (Double2D - coordinates of location we want to check for intersection with ParkedCar)
	 * @return boolean (returns true if the coordinate intersects with the object)
	 */
	public boolean inShape(Double2D coord)
	{
		Shape carShape = getShape();		
		return carShape.contains(coord.x, coord.y);
	}	

	/**
	 * Method which returns true or false if a provided Shape object intersects with the
	 * ParkedCar object
	 * @param inShape (Shape - footprint of the object we are checking for intersection)
	 * @return boolean (returns true if the supplied shape intersects with the object)
	 */
	public boolean inShape(Shape inShape)
	{
		Shape carShape = getShape();	
		Area carArea = new Area (carShape);
		carArea.intersect(new Area(inShape));
		return !carArea.isEmpty();
	}	
	
	/**
	 * method which returns the ID of the road on which the Obstacle is located
	 * @return int (return ID of the Road on which the ParkedCar is located)
	 */
	public int getRoadId()
	{
		return roadId;
	}
	
	/**
	 * Uses same method as DumbCar to return a Shape object representing the parked car 
	 * obstacle and centred at location
	 * @return Shape (returns Shape object representing the footprint of the ParkedCar)
	 */
	public Shape getShape()
	{
		// The location is the centre of this shape which we will assume is of the size given in
		// the Constants file for an obstacle
		double widthOffset = Constants.OBSTACLE_WIDTH/2;
		double lengthOffset = Constants.OBSTACLE_LENGTH/2;
		
		// Create a shape aligned with the oriented vehicles, and pivoted around the centre of the shape
		// to lie pointing in the appropriate direction
		Rectangle2D.Double carRectangle = new Rectangle2D.Double();
		
		// Assume the basic shape is as it would appear when pointed along the x-axis, so this means some swapping around of width/length
		carRectangle = new Rectangle2D.Double(location.x - lengthOffset, location.y - widthOffset, Constants.OBSTACLE_LENGTH, Constants.OBSTACLE_WIDTH);		
		AffineTransform rotateTransform = AffineTransform.getRotateInstance(orientation2D(), location.x, location.y);
		Shape carShape = rotateTransform.createTransformedShape(carRectangle);
		
		return carShape;
	}
}
