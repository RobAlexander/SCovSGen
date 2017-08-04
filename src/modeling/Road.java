package modeling;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;

import modeling.Constants.*;
import sim.engine.*;
import sim.util.*;

/**
 * Class to describe Road objects.  These are static objects, represented by a line (extended by a fixed width) and with
 * Road Markings to create lanes at fixed locations. Roads have a direction, typically 0 for N/S and 90 for E/W, and a
 * length (which can vary between roads, but is fixed once set).
 * 
 * @author hh940
 */

public class Road extends Line2D.Double implements Steppable 
{
	private static final long serialVersionUID = 1L;
	
	protected final int ID;
	protected final int type;	
	protected boolean isSchedulable;
	public static final double roadWidth = 6.0;
	private double roadLength;
	private double direction; // This can range from 0 (incl) to 180 (excl) where 0 is equivalent to N/S and 90 to E/W
	
	/** 
	 * Constructor.  Store the supplied unique ID and typeNo of the road, create an appropriate line, given the 
	 * start and end coordinates, and calculate the road length and direction.
	 * @param idNo (int - unique identifier for the Road object)
	 * @param typeNo (int - type identifier for the Road object )
	 * @param start (Double2D - location for the start of the road)
	 * @param end (Double2D - location for the end of the road)
	 */
	public Road(int idNo, int typeNo, Double2D start, Double2D end)
	{
		ID = idNo;
		type = typeNo;
		this.setLine(start.x, start.y, end.x, end.y);
		
		// calculate length
		if (start.x == end.x) {
			// Vertical line
			roadLength = Math.abs(start.y-end.y);
			setDirection(0); // N/S orientation
			
		} else if (start.y == end.y) {
			// Horizontal line
			roadLength = Math.abs(start.x-end.x);
			setDirection(90); // E/W orientation
		} else {
			// TODO - Raise an error here
		}
	}	
		
	/**
	 * This method returns the direction in which the road is aligned - 0 for N/S or 90 for E/W
	 * @return double (the direction: 0 for N/S or 90 for E/W)
	 */
	public double getDirection() {
		return direction;
	}

	/**
	 * This method sets the direction of the road - 0 for N/S or 90 for E/W
	 * @param direction (double - the direction to set, 0 for N/S or 90 for E/W)
	 */
	public void setDirection(double direction) {
		this.direction = direction; // TODO - May want to consider some range checking on the input
	}

	/**
	 * This method allows a routine to be defined which will execute on every step/iteration.  This is unused
	 * at present, but may be necessary if we want to implement any 'smart' road features such as sensing or
	 * communication with Cars.
	 * @param state (SimState - access to the simulation environment)
	 */
	@Override
	public void step(SimState state) {
		// TODO Auto-generated method stub
	}
	
	/**
	 * This method returns whether the Road object is schedulable
	 * @return boolean (returns true if the Road object is schedulable)
	 */
	public boolean isSchedulable() {
		return isSchedulable;
	}
	
	/**
	 * This method returns the type identifier for this Road object
	 * @return int (returns the type identifier for this Road object)
	 */
	public int getType() {return type;}
	
	/**
	 * This method returns the unique identifier for this Road object
	 * @return int (returns the unique identifier for this Road object)
	 */
	public int getID() {return ID;}
	
	/**
	 * This method returns the length of the Road
	 * @return double (returns the length of the Road)
	 */
	public double getLength() {return roadLength;}
	
	/** 
	 * Return whether the Road is in a North/South orientation.
	 * TODO: It may not be appropriate to call this method when we have reverted to non-grid road layout
	 * @return boolean (return true if the road is orientated N/S, or false otherwise)
	 */
	public boolean getIsNS() 
	{
		if (direction == 0)
		{
			return true;
		} else {
			return false;
		}
	}

	/**
	 * This method returns a string identifier for the Road object, currently this consists of the just the 
	 * unique identifier for the Road object.
	 * @return String ()
	 */
	public String toString() 
	{
		return "" + getID();
	}
	
	/**
	 * Method returns true if the supplied location intersects with the road object (when extended by required width), false otherwise
	 * @param coord (Double2D - coordinates of location to test for intersection with the Road object)
	 * @return boolean (returns true if the supplied location intersects with the footprint of the Road)
	 */
	public boolean inShape(Double2D coord)
	{
		// Construct a rectangle that is the size of the whole road area (rather than just the centre line)
		Rectangle2D.Double roadSurface = getSurface();
		
		// This 'contains' method is better than the standard one which doesn't include points which are on the 
		// eastern and southern boundaries of the shape
		return Utility.betterContains(roadSurface, coord);
	}
		
	/**
	 * Method returns true if the supplied rectangle intersects with the road object (when extended by required width), false otherwise
	 * @param inRectangle (Rectangle2D.Double - rectangle to test for intersection with the Road object)
	 * @return boolean (returns true if the supplied rectangle intersects with the footprint of the Road)
	 */
	public boolean inShape(Rectangle2D.Double inRectangle)
	{
		// Construct a rectangle that is the size of the whole road area (rather than just the centre line)
		Rectangle2D.Double roadSurface = getSurface();
		
		return roadSurface.intersects(inRectangle);
	}	
	
	/**
	 *  Method returns a Rectangle2D which represents the whole road area (rather than just the centre line)
	 *  @return Rectangle2D.Double (returns a rectangle object representing the footprint of the Road)
	 */
	public Rectangle2D.Double getSurface()
	{
		double tlx; // top left x
		double tly; // top left y
		
		Rectangle2D.Double roadSurface = new Rectangle2D.Double();
		
		// Calculate the coordinates of the upper left corner of the rectangle
		if (x1 == x2) {
			// Vertical line
			tlx = x1 - roadWidth/2;
			tly = Math.min(y1, y2);
			roadSurface = new Rectangle2D.Double(tlx, tly, roadWidth, roadLength);
		} else if (y1 == y2) {
			// Horizontal line
			tly = y1 - roadWidth/2;
			tlx = Math.min(x1, x2);
			roadSurface = new Rectangle2D.Double(tlx, tly, roadLength, roadWidth);
		} else {
			// TODO - Raise an error here
		}
		
		return roadSurface;
	}
	
	/**
	 *  Method returns a Rectangle2D which represents the painted lines on the specified location on the
	 *  road.  Assumes a line thickness of 10cm at the sides at a fixed offset from the kerb, and 10cm in 
	 *  the centre.  For now, assume continuous single line in both cases (this should be adjusted in 
	 *  time to allow for broken lines at junctions, and broken lines in the centre (warning/standard) - 
	 *  see Traffic Signs Manual, Ch 5, 2003.
	 *  @param inLineType (LineType - NESIDE / CENTRE / SWSIDE)
	 *  @return Rectangle2D.Double (returns rectangle object representing the footprint of the selected 
	 *  line type on the Road)
	 */
	public Rectangle2D.Double getLine(LineType inLineType)
	{
		double tlx;
		double tly;
		
		Rectangle2D.Double paintedLine = new Rectangle2D.Double();
		
		switch (inLineType) {
			case NESIDE : {
		
				// Calculate the coordinates of the upper left corner of the rectangle
				if (x1 == x2) {
					// Vertical line in NB carriageway
					tlx = x1 - roadWidth/2 + Constants.ROADEDGEOFFSET;
					tly = Math.min(y1, y2);
					paintedLine = new Rectangle2D.Double(tlx, tly, Constants.ROADEDGINGWIDTH, roadLength);
				} else if (y1 == y2) {
					// Horizontal line in EB carriageway
					tly = y1 - roadWidth/2 + Constants.ROADEDGEOFFSET;
					tlx = Math.min(x1, x2);
					paintedLine = new Rectangle2D.Double(tlx, tly, roadLength, Constants.ROADEDGINGWIDTH);
				} else {
					// TODO - Raise an error here
				}
				break;
			}
			case SWSIDE : {
				// Calculate the coordinates of the upper left corner of the rectangle
				if (x1 == x2) {
					// Vertical line in SB carriageway
					tlx = x1 + roadWidth/2 - Constants.ROADEDGEOFFSET - Constants.ROADEDGINGWIDTH;
					tly = Math.min(y1, y2);
					paintedLine = new Rectangle2D.Double(tlx, tly, Constants.ROADEDGINGWIDTH, roadLength);
				} else if (y1 == y2) {
					// Horizontal line in WB carriageway
					tly = y1 + roadWidth/2 - Constants.ROADEDGEOFFSET - Constants.ROADEDGINGWIDTH;
					tlx = Math.min(x1, x2);
					paintedLine = new Rectangle2D.Double(tlx, tly, roadLength, Constants.ROADEDGINGWIDTH);
				} else {
					// TODO - Raise an error here
				}				
				break;
			}
			case CENTRE : {
				// Calculate the coordinates of the upper left corner of the rectangle
				if (x1 == x2) {
					// Vertical line
					tlx = x1 - Constants.ROADEDGINGWIDTH/2;
					tly = Math.min(y1, y2);
					paintedLine = new Rectangle2D.Double(tlx, tly, Constants.ROADEDGINGWIDTH, roadLength);
				} else if (y1 == y2) {
					// Horizontal line
					tly = y1 - Constants.ROADEDGINGWIDTH/2;
					tlx = Math.min(x1, x2);
					paintedLine = new Rectangle2D.Double(tlx, tly, roadLength, Constants.ROADEDGINGWIDTH);
				} else {
					// TODO - Raise an error here
				}				
				break;
			}
		}
		
		return paintedLine;
	}
	
	/**
	 *  Method returns a Line2D.Double which represents the innermost edge of the painted lines on the specified location on the
	 *  road.  Assumes a line thickness of 10cm at the sides.  For now, assume continuous single
	 *  line in both cases (this should be adjusted in time to allow for broken lines at junctions, and broken lines in 
	 *  the centre (warning/standard) - see workbook for details (or Traffic Signs Manual, Ch 5, 2003).
	 *  
	 *  @param inLineType (LineType - type identifier for line required: NEARSIDE/OFFSIDE)
	 *  @return Line2D.Double (returns a line representing the inside edge of the required road marking)
	 */
	public Line2D.Double getThinLine(LineType inLineType)
	{
		double tlx;
		double tly;
		
		Line2D.Double paintedLine = new Line2D.Double();
		
		switch (inLineType) {
			case CENTRE : {
				paintedLine = new Line2D.Double(-1, -1, -1, -1); // Return inappropriate values to cascade the error
				break;
			}
			case NESIDE : {
		
				// Calculate the coordinates of the upper left corner of the rectangle
				if (x1 == x2) {
					// Vertical line in NB carriageway
					tlx = x1 - roadWidth/2 + Constants.ROADEDGEOFFSET + Constants.ROADEDGINGWIDTH;
					tly = Math.min(y1, y2);
					paintedLine = new Line2D.Double(tlx, tly, tlx, tly + roadLength);
				} else if (y1 == y2) {
					// Horizontal line in EB carriageway
					tly = y1 - roadWidth/2 + Constants.ROADEDGEOFFSET + Constants.ROADEDGINGWIDTH;
					tlx = Math.min(x1, x2);
					paintedLine = new Line2D.Double(tlx, tly, tlx + roadLength, tly);
				} else {
					// TODO - Raise an error here
				}
				break;
			}
			case SWSIDE : {
				// Calculate the coordinates of the upper left corner of the rectangle
				if (x1 == x2) {
					// Vertical line in SB carriageway
					tlx = x1 + roadWidth/2 - Constants.ROADEDGEOFFSET - Constants.ROADEDGINGWIDTH;
					tly = Math.min(y1, y2);
					paintedLine = new Line2D.Double(tlx, tly, tlx, tly + roadLength);
				} else if (y1 == y2) {
					// Horizontal line in WB carriageway
					tly = y1 + roadWidth/2 - Constants.ROADEDGEOFFSET - Constants.ROADEDGINGWIDTH;
					tlx = Math.min(x1, x2);
					paintedLine = new Line2D.Double(tlx, tly, tlx + roadLength, tly);
				} else {
					// TODO - Raise an error here
				}				
				break;
			}
		default:
			break;
		}
		
		return paintedLine;
	}
	
	/*
	 * This method returns 1 for N or E; 2 for S or W
	 * @param coord (Double2D - coordinates of location on the Road at which we want to check the lane direction)
	 * @return int (return 1 if we are in the NB or EB lane, return 2 if we are in the SB or WB lane)
	 */
	public int getLane(Double2D coord)
	{
		int retVal = 0;
		
		if (getIsNS() == true) 
		{
			if (coord.x >= x1)
			{
				retVal = 2;
			} else {
				retVal = 1;
			}	
		} else {
			if (coord.y >= y1)
			{
				retVal = 2;
			} else {
				retVal = 1;
			}				
		}
				
		return retVal;
	}
}
