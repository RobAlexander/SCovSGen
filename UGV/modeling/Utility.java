/**
 * 
 */
package modeling;

import java.awt.geom.Area;
import java.awt.geom.PathIterator;
import java.awt.geom.Rectangle2D;

import sim.util.Double2D;

/**
 * @author Heather
 *
 */
public class Utility {

	/**
	 * 
	 */
	public Utility() {
		// TODO Auto-generated constructor stub
	}
	
	/**
	 * HH 15.10.14 utility method to return the 'centre' of an area - as defined by (max+min)/2 for
	 * both x and y directions/	 * 	 
	 * */
	public static Double2D getAreaCentre(Area inArea)
	{
		double xMax = -1;
		double yMax = -1;
		double xMin = -1;
		double yMin = -1;
		double[] results = new double[6];
		int retVal;
		
		PathIterator tempIterator = inArea.getPathIterator(null);
		
		while (!tempIterator.isDone())
		{
			retVal = tempIterator.currentSegment(results);
			
			// See what type of segment we have, and look at the appropriate coords
			switch (retVal)
			{
			case PathIterator.SEG_MOVETO :
		    {
		        // Compare the values that we have just found to the current max and min
                if ((results[0] >= 0 && results[0] < xMin) || xMin < 0) {xMin = results[0];}
                if ((results[0] >= 0 && results[0] > xMax) || xMax < 0) {xMax = results[0];}
                if ((results[1] >= 0 && results[1] < yMin) || yMin < 0) {yMin = results[1];}
                if ((results[1] >= 0 && results[1] > yMax) || yMax < 0) {yMax = results[1];}
                 
		        tempIterator.next(); // Next point
                break;
		    }

		case PathIterator.SEG_LINETO :
			    {
			        // Compare the values that we have just found to the current max and min
                    if ((results[0] >= 0 && results[0] < xMin) || xMin < 0) {xMin = results[0];}
                    if ((results[0] >= 0 && results[0] > xMax) || xMax < 0) {xMax = results[0];}
                    if ((results[1] >= 0 && results[1] < yMin) || yMin < 0) {yMin = results[1];}
                    if ((results[1] >= 0 && results[1] > yMax) || yMax < 0) {yMax = results[1];}
                     
			        tempIterator.next(); // Next point
                    break;
			    }

			case PathIterator.SEG_CLOSE :   
			    {
			        tempIterator.next(); // Next point
			        break;	
			    }
			default :    
			    {
			    	System.out.println("Utility.getAreaCentre: Unknown PathIterator code:  " + retVal + ".");
			    	break;
			    }
			}
		}
		
		return new Double2D((xMax+xMin)/2, (yMax+yMin)/2);
	}

	/**
	 * HH 15.10.14 utility method to return the 'largest' vertex of an area - as defined by Max(x+y) for
	 * all points (x,y)	 
	 * */
	public static Double2D getAreaMaxPt(Area inArea)
	{
		double xMax = -1;
		double yMax = -1;
		double maxTotal = 0;
		double[] results = new double[6];
		int retVal;
		
		PathIterator tempIterator = inArea.getPathIterator(null);
		
		while (!tempIterator.isDone())
		{
			retVal = tempIterator.currentSegment(results);
			
			// See what type of segment we have, and look at the appropriate coords
			switch (retVal)
			{
			case PathIterator.SEG_MOVETO :
		    {
		    	if (maxTotal == 0 && results[0] >= 0 && results[1] >= 0)
		    	{
		    		xMax = results[0];
		    		yMax = results[1];
		    		maxTotal = xMax + yMax;
		    	}
		    	else if (results[0] >= 0 && results[1] >= 0)
		    	{
		    		if (results[0] + results[1] > maxTotal)
		    		{
		    			xMax = results[0];
			    		yMax = results[1];
			    		maxTotal = xMax + yMax;
		    		}
		    	}
                
		        tempIterator.next(); // Next point
		    	break;
		    }

		case PathIterator.SEG_LINETO :
			    {
			    	if (maxTotal == 0 && results[0] >= 0 && results[1] >= 0)
			    	{
			    		xMax = results[0];
			    		yMax = results[1];
			    		maxTotal = xMax + yMax;
			    	}
			    	else if (results[0] >= 0 && results[1] >= 0)
			    	{
			    		if (results[0] + results[1] > maxTotal)
			    		{
			    			xMax = results[0];
				    		yMax = results[1];
				    		maxTotal = xMax + yMax;
			    		}
			    	}
                    
			        tempIterator.next(); // Next point
			    	break;
			    }

			case PathIterator.SEG_CLOSE :   
			    {
			    	tempIterator.next(); // Next point
			    	break;	
			    }
			default :    
		        {
		    	    System.out.println("Utility.getAreaMaxPt: Unknown PathIterator code:  " + retVal + ".");
		    	    break;
		        }
			}
		}
		
		return new Double2D(xMax, yMax);
	}
	
	// HH 4.11.14 - As the basic contains method does not include points which are located *on* the eastern and
	//southern boundaries of the rectangular shape
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
	
}
