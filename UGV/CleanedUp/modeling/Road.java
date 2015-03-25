package modeling;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;

import modeling.Constants.*;
import sim.engine.*;
import sim.util.*;

/**
 * Class to describe road objects
 * 
 * @author hh940
 *
 */

public class Road extends Line2D.Double implements Steppable 
{
	private static final long serialVersionUID = 1L;
	
	protected final int ID;
	protected final int type;	
	protected boolean isSchedulable;
	public static final double roadWidth = 6.0;
	private double roadLength;
	private double direction; // HH 2.10.14 This can range from 0 (incl) to 180 (excl) where 0 is equivalent to N/S and 90 to E/W
	
	/** 
	 * Constructor.
	 * @param idNo (int - )
	 * @param typeNo (int - )
	 * @param start (Double2D - )
	 * @param end (Double2D - )
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
			setDirection(0); // HH 2.10.14 Replaced isNS = true
			
		} else if (start.y == end.y) {
			// Horizontal line
			roadLength = Math.abs(start.x-end.x);
			setDirection(90); // HH 2.10.14 Replaced isNS = false
		} else {
			// TODO - Raise an error here
		}
	}	
		
	/**
	 * This method...
	 * @return double (the direction)
	 */
	public double getDirection() {
		return direction;
	}

	/**
	 * This method...
	 * @param direction (double - the direction to set)
	 */
	public void setDirection(double direction) {
		this.direction = direction;
	}

	/**
	 * This method...
	 * @param state (SimState - )
	 */
	@Override
	public void step(SimState state) {
		// TODO Auto-generated method stub
	}
	
	/**
	 * This method...
	 * @return boolean ()
	 */
	public boolean isSchedulable() {
		return isSchedulable;
	}
	
	/**
	 * This method...
	 * @return int ()
	 */
	public int getType() {return type;}
	
	/**
	 * This method...
	 * @return int ()
	 */
	public int getID() {return ID;}
	
	/**
	 * This method...
	 * @return double ()
	 */
	public double getLength() {return roadLength;}
	
	/** 
	 * HH 2.10.14 - Updated to use direction field
	 * 
	 * TODO: It may not be appropriate to call this method when we have reverted to non-grid road layout
	 * @return boolean ()
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
	 * This method...
	 * @return String ()
	 */
	public String toString() 
	{
		return "" + getID();
	}
	
	/**
	 * Method returns true if coord intersects with the road object (when extended by required width), false otherwise
	 * @param coord (Double2D - )
	 * @return boolean ()
	 */
	public boolean inShape(Double2D coord)
	{
		// Construct a rectangle that is the size of the whole road area (rather than just the centre line)
		Rectangle2D.Double roadSurface = getSurface();
		
		// HH 4.11.14 - Replace old contains function as doesn't include points which are on the 
		// eastern and southern boundaries of the shape
		return Utility.betterContains(roadSurface, coord);
	}
		
	/**
	 * Method returns true if rectangle intersects with the road object (when extended by required width), false otherwise
	 * @param inRectangle (Rectangle2D.Double - )
	 * @return boolean ()
	 */
	public boolean inShape(Rectangle2D.Double inRectangle)
	{
		// Construct a rectangle that is the size of the whole road area (rather than just the centre line)
		Rectangle2D.Double roadSurface = getSurface();
		
		return roadSurface.intersects(inRectangle);
	}	
	
	/**
	 *  Method returns a Rectangle2D which represents the whole road area (rather than just the centre line)
	 *  @return Rectangle2D.Double ()
	 */
	public Rectangle2D.Double getSurface()
	{
		double tlx;
		double tly;
		
		Rectangle2D.Double roadSurface = new Rectangle2D.Double();
		
		// Calculate the coordinates of the upper left corner of the rectangle
		if (x1 == x2) {
			// Vertical line
			tlx = x1 - roadWidth/2;
			tly = Math.min(y1, y2);
			roadLength = Math.abs(y1-y2);
			roadSurface = new Rectangle2D.Double(tlx, tly, roadWidth, roadLength);
		} else if (y1 == y2) {
			// Horizontal line
			tly = y1 - roadWidth/2;
			tlx = Math.min(x1, x2);
			roadLength = Math.abs(x1-x2);
			roadSurface = new Rectangle2D.Double(tlx, tly, roadLength, roadWidth);
		} else {
			// TO DO - Raise an error here
		}
		
		return roadSurface;
	}
	
	/**
	 *  HH 16/6/14 Method returns a Rectangle2D which represents the painted lines on the specified location on the
	 *  road.  Assumes a line thickness of 10cm at the sides, and 10cm in the centre.  For now, assume continuous single
	 *  line in both cases (this should be adjusted in time to allow for broken lines at junctions, and broken lines in 
	 *  the centre (warning/standard) - see workbook for details (or Traffic Signs Manual, Ch 5, 2003).
	 *  @param inLineType (LineType - NESIDE / CENTRE / SWSIDE)
	 *  @return Rectangle2D.Double ()
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
					// Vertical line
					tlx = x1 - roadWidth/2 + Constants.ROADEDGEOFFSET;
					tly = Math.min(y1, y2);
					roadLength = Math.abs(y1-y2);
					paintedLine = new Rectangle2D.Double(tlx, tly, Constants.ROADEDGINGWIDTH, roadLength);
				} else if (y1 == y2) {
					// Horizontal line
					tly = y1 - roadWidth/2 + Constants.ROADEDGEOFFSET;
					tlx = Math.min(x1, x2);
					roadLength = Math.abs(x1-x2);
					paintedLine = new Rectangle2D.Double(tlx, tly, roadLength, Constants.ROADEDGINGWIDTH);
				} else {
					// TODO - Raise an error here
				}
				break;
			}
			case SWSIDE : {
				// Calculate the coordinates of the upper left corner of the rectangle
				if (x1 == x2) {
					// Vertical line
					tlx = x1 + roadWidth/2 - Constants.ROADEDGEOFFSET - Constants.ROADEDGINGWIDTH;
					tly = Math.min(y1, y2);
					roadLength = Math.abs(y1-y2);
					paintedLine = new Rectangle2D.Double(tlx, tly, Constants.ROADEDGINGWIDTH, roadLength);
				} else if (y1 == y2) {
					// Horizontal line
					tly = y1 + roadWidth/2 - Constants.ROADEDGEOFFSET - Constants.ROADEDGINGWIDTH;
					tlx = Math.min(x1, x2);
					roadLength = Math.abs(x1-x2);
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
					roadLength = Math.abs(y1-y2);
					paintedLine = new Rectangle2D.Double(tlx, tly, Constants.ROADEDGINGWIDTH, roadLength);
				} else if (y1 == y2) {
					// Horizontal line
					tly = y1 - Constants.ROADEDGINGWIDTH/2;
					tlx = Math.min(x1, x2);
					roadLength = Math.abs(x1-x2);
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
	 *  HH 18/6/14 Method returns a Line2D.Double which represents the innermost edge of the painted lines on the specified location on the
	 *  road.  Assumes a line thickness of 10cm at the sides.  For now, assume continuous single
	 *  line in both cases (this should be adjusted in time to allow for broken lines at junctions, and broken lines in 
	 *  the centre (warning/standard) - see workbook for details (or Traffic Signs Manual, Ch 5, 2003).
	 *  
	 *  argument: nearside / offside
	 *  @param inLineType (LineType - )
	 *  @return Line2D.Double ()
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
					// Vertical line
					tlx = x1 - roadWidth/2 + Constants.ROADEDGEOFFSET + Constants.ROADEDGINGWIDTH;
					tly = Math.min(y1, y2);
					roadLength = Math.abs(y1-y2);
					paintedLine = new Line2D.Double(tlx, tly, tlx, tly + roadLength);
				} else if (y1 == y2) {
					// Horizontal line
					tly = y1 - roadWidth/2 + Constants.ROADEDGEOFFSET + Constants.ROADEDGINGWIDTH;
					tlx = Math.min(x1, x2);
					roadLength = Math.abs(x1-x2);
					paintedLine = new Line2D.Double(tlx, tly, tlx + roadLength, tly);
				} else {
					// TODO - Raise an error here
				}
				break;
			}
			case SWSIDE : {
				// Calculate the coordinates of the upper left corner of the rectangle
				if (x1 == x2) {
					// Vertical line
					tlx = x1 + roadWidth/2 - Constants.ROADEDGEOFFSET - Constants.ROADEDGINGWIDTH;
					tly = Math.min(y1, y2);
					roadLength = Math.abs(y1-y2);
					paintedLine = new Line2D.Double(tlx, tly, tlx, tly + roadLength);
				} else if (y1 == y2) {
					// Horizontal line
					tly = y1 + roadWidth/2 - Constants.ROADEDGEOFFSET - Constants.ROADEDGINGWIDTH;
					tlx = Math.min(x1, x2);
					roadLength = Math.abs(x1-x2);
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
	 * @param coord (Double2D - )
	 * @return int ()
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
