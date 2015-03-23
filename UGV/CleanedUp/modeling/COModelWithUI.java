package modeling;
import sim.display.*;
import sim.engine.*;
import sim.portrayal.DrawInfo2D;
import sim.portrayal.Inspector;
import sim.portrayal.continuous.*;
import sim.portrayal.simple.*;

import javax.swing.*;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.Rectangle2D;
import java.security.SecureRandom;

import sim.portrayal.grid.FastValueGridPortrayal2D;


/**
 * A class for running a simulation with a UI, run to see a simulation with a UI
 * showing it running.
 * 
 * @author Robert Lee
 */
public class COModelWithUI extends GUIState
{	
	protected COModelBuilder sBuilder; // = new COModelBuilder((COModel) state);
	
	public Display2D display;
	public JFrame displayFrame;
	ContinuousPortrayal2D environmentPortrayal = new ContinuousPortrayal2D();
	FastValueGridPortrayal2D obstaclesPortrayal = new FastValueGridPortrayal2D("Obstacle", true);  // immutable
	FastValueGridPortrayal2D terrainPortrayal = new FastValueGridPortrayal2D("Terrain", true);  // immutable
	FastValueGridPortrayal2D wallPortrayal = new FastValueGridPortrayal2D("Wall", true);  // immutable
	
	// HH 30/4/14 - Roads portrayal
	FastValueGridPortrayal2D roadsPortrayal = new FastValueGridPortrayal2D("Roads", true); // immutable    
	FastValueGridPortrayal2D junctionsPortrayal = new FastValueGridPortrayal2D("Junctions", true); // immutable
	FastValueGridPortrayal2D jctApproachPortrayal = new FastValueGridPortrayal2D("Junction Approaches", true); // immutable
	// HH 17.6.14 - Road markings portrayal 
	FastValueGridPortrayal2D roadMarkingPortrayal = new FastValueGridPortrayal2D("Road Markings", true); // immutable
   
	/** HH 29.1.15 - Set arguments to 0, True if want a 'normal' standard run
	 * 
	 * @param double percentageFaults ()
	 * @param boolean inWantRandomFaults ()
	 */
    public COModelWithUI(double percentageFaults, boolean inWantRandomFaults) 
    { 
    	//super(new COModel( System.nanoTime(), Constants.WorldXVal, Constants.WorldYVal, true, 0, 0)); // HH 22/7/14 Added percentage faults parameter to end
    	// HH 3.9.14 Changed seed method as concerned that integer casting of long value that occurs later could 
    	//result in lots of seeds having MAX_VALUE
    	super(new COModel( new SecureRandom().nextInt(), Constants.WorldXVal, Constants.WorldYVal, true, percentageFaults, 0, inWantRandomFaults));     	
    	System.out.println("COModelWithUI is being called!"+ "it's state(model)is: "+ state.toString());
    	sBuilder = new COModelBuilder((COModel) state);
    }
    
    /**
     * This method...
     * @param SimState state ()
     */
    public COModelWithUI(SimState state) {super(state); }    
    
    /**
     * This method...
     * @return String ()
     */
    public static String getName() { return "Robot-Testing-Sim"; } //[TODO] rename this
   
    /**
     * This method...
     */
    public void start()
	{
		System.out.println("COModelWithUI.start is called  "+ sBuilder.sim);
		sBuilder.sim.reset();
		
		// HH 30/4/14 Updated to add roads to simulations
		sBuilder.generateSimulation();//((COModel) state).noObstacles,((COModel) state).noCars); // HH 31/7/14 Updated 
		// HH end
		
		super.start();
		setupPortrayals();	
		
    	System.out.println("Just started the simulation: Free =" + Runtime.getRuntime().freeMemory());
    	System.out.println("Just started the simulation: Total =" + Runtime.getRuntime().totalMemory());
    	System.out.println("Just started the simulation: MAX = " + Runtime.getRuntime().maxMemory());
	}

	/**
	 * I do not know if this method is required by MASON at all, and the simulation
	 * with UI appears to run correctly even when it is removed, however all of 
	 * the example simulations that MASON comes with include a load method in the
	 * with UI class so I have done as well even though I have not found a reason
	 * as to if it is important to have one.
	 * @param SimState state ()
	 */
	public void load(SimState state)
	{
		sBuilder.sim.reset();
		sBuilder.generateSimulation();// HH 31/7/14 Updated
		
		super.load(state);
		setupPortrayals();
	}
	
	/**
	 * A method which sets up the portrayals of the different layers in the UI,
	 * this is where details of the simulation are coloured and set to different
	 * parts of the UI
	 */
	public void setupPortrayals()
	{		
		COModel simulation = (COModel) state;
		
		// HH 6.11.14 Moved the background images portrayals to be drawn first to try and improve appearance of vehicles driving
		// over the Junction Approaches - they currently change colour due to the transparency of the approaches
		
		// HH 30/4/14 Roads portrayal
		roadsPortrayal.setField(simulation.roadMap);
		roadsPortrayal.setMap(new sim.util.gui.SimpleColorMap(
				0,
				Constants.DUALTWOWAY,
				new Color(0,0,0,0),
				new Color(0,0,255,255)
				));

		// draw the junction approaches (ordering here not important though)
		jctApproachPortrayal.setField(simulation.jctApproachMap);
		jctApproachPortrayal.setMap(new sim.util.gui.SimpleColorMap(
				0,
				1,
				new Color(0,0,0,0),
				new Color(102,51,0,50) // slightly opaque
				));
		
		// draw the junctions
		junctionsPortrayal.setField(simulation.junctionMap);
		junctionsPortrayal.setMap(new sim.util.gui.SimpleColorMap(
				0,
				1,
				new Color(0,0,0,0),
				new Color(102,51,0,60) // slightly opaque
				));
		
		// HH - 17.6.14 - add the road markings
		roadMarkingPortrayal.setField(simulation.roadMarkingMap);
		roadMarkingPortrayal.setMap(new sim.util.gui.SimpleColorMap(
				0,
				Constants.WHITERPAINT,
				new Color(0,0,0,0),
				new Color(255,255,255,255)
				));
		
		obstaclesPortrayal.setField(simulation.obstacleMap);
		obstaclesPortrayal.setMap(new sim.util.gui.SimpleColorMap(
				0,
				1,
				new Color(0,0,0,0),
				new Color(0,0,255,255)
				));
        
		wallPortrayal.setField(simulation.wallMap);
		wallPortrayal.setMap(new sim.util.gui.SimpleColorMap(
				0,
				1,
				new Color(0,0,0,0),
				new Color(255,0,0,255)
				));		
				
		// HH 6.11.14 - Portrayals for DumbCar i.e. the Moving Obstacles
		// tell the portrayals what to portray and how to portray them
		environmentPortrayal.setField( simulation.environment );	
		environmentPortrayal.setPortrayalForClass(DumbCar.class, new OrientedPortrayal2D( new LabelledPortrayal2D( new RectanglePortrayal2D(Constants.OBSTACLE_WIDTH * 6)
		{
			/**
			 * 
			 */
			private static final long serialVersionUID = 1L;

			public void draw(Object object, Graphics2D graphics, DrawInfo2D info)
			{
				if(((Car)object).isActive==true)
				{
					graphics.setPaint(new Color(0, 255, 0, 255)); // Green
				}
				else
				{
					graphics.setPaint(new Color(0,0,0));
				}
				
				// HH 18.9.14 Create a rectangle object that we can rotate to point in the right direction
				Rectangle2D.Double carRectangle = new Rectangle2D.Double();
				
				// HH 8.10.14 - Changed so location is the front of the vehicle, rather than in the centre
				// previously: (info.draw.x - ((Constants.OBSTACLE_LENGTH/2)*info.draw.width))
				carRectangle = new Rectangle2D.Double((info.draw.x - ((Constants.OBSTACLE_LENGTH)*info.draw.width)), (info.draw.y - ((Constants.OBSTACLE_WIDTH/2)*info.draw.height)), 
				(info.draw.width*Constants.OBSTACLE_LENGTH), 
				(info.draw.height*Constants.OBSTACLE_LENGTH*(Constants.OBSTACLE_WIDTH/Constants.OBSTACLE_LENGTH)));
								
				AffineTransform rotateTransform = AffineTransform.getRotateInstance(((Car) object).orientation2D(), ((Car)object).getLocation().x*6, ((Car)object).getLocation().y*6);
				Shape carShape = rotateTransform.createTransformedShape(carRectangle);
				graphics.draw(carShape);
				graphics.fill(carShape); // HH 6.11.14 Added to make Shape more visible

			}
		}, null, new Color(0, 0, 0), false), 0, 2)
		
		);
		
		// HH 7.5.14 - Added portrayal for UGVs
		// tell the portrayals what to portray and how to portray them
		environmentPortrayal.setField( simulation.environment );	
		environmentPortrayal.setPortrayalForClass(UGV.class, new OrientedPortrayal2D(new RectanglePortrayal2D(Constants.UGV_WIDTH)
		{
			/**
			 * 
			 */
			private static final long serialVersionUID = 1L;

			public void draw(Object object, Graphics2D graphics, DrawInfo2D info)
			{
				if(((UGV)object).isActive==true)
				{
					graphics.setPaint(new Color(255, 128, 0, 255)); // Orange
				}
				else
				{
					graphics.setPaint(new Color(0,0,0));
				}
				
				AffineTransform rotateTransform = AffineTransform.getRotateInstance(((Car) object).orientation2D(), ((Car)object).getLocation().x*6, ((Car)object).getLocation().y*6);
				Shape carShape = rotateTransform.createTransformedShape(new Rectangle2D.Double((info.draw.x - ((Constants.UGV_WIDTH)*info.draw.width)), (info.draw.y - ((Constants.UGV_WIDTH/2)*info.draw.height)), 
						(info.draw.width*Constants.UGV_WIDTH), (info.draw.height*Constants.UGV_WIDTH)));
				graphics.draw(carShape);
				graphics.fill(carShape); // HH 6.11.14 Add fill
			}
		}, 0, 2)
		
		);
		// HH - end

		// HH 18.9.14 - Added portrayal for Waypoints (in order to make them less obstrusive in the display)
		// tell the portrayals what to portray and how to portray them
		environmentPortrayal.setField( simulation.environment );	
		environmentPortrayal.setPortrayalForClass(Waypoint.class, (new RectanglePortrayal2D(0.3)
		{
			/**
			 * 
			 */
			private static final long serialVersionUID = 1L;

			public void draw(Object object, Graphics2D graphics, DrawInfo2D info)
			{
				paint = new Color(255, 255, 0, 255); // Yellow
												
			    super.draw(object, graphics, info);
			}
		})
		
		);
		// HH - end
		
		// HH 7.5.14 - Added portrayal for Failures
		// tell the portrayals what to portray and how to portray them
		environmentPortrayal.setField( simulation.environment );	
		environmentPortrayal.setPortrayalForClass(Failure.class, new RectanglePortrayal2D(0.75) // HH 22.12.14 Increased size from 0.5
		{
			/**
			 * 
			 */
			private static final long serialVersionUID = 1L;

			public void draw(Object object, Graphics2D graphics, DrawInfo2D info)
			{
				paint = new Color(0, 0, 0, 255); // HH 22.12.14 Changed from red to black							
			    super.draw(object, graphics, info);
			}
		});
		// HH - end

		// HH 26.8.14 - Added portrayal for Crashes
		// tell the portrayals what to portray and how to portray them
		environmentPortrayal.setField( simulation.environment );	
		environmentPortrayal.setPortrayalForClass(Crash.class, new RectanglePortrayal2D(1)
		{
			/**
			 * 
			 */
			private static final long serialVersionUID = 1L;

			public void draw(Object object, Graphics2D graphics, DrawInfo2D info)
			{
				paint = new Color(255, 0, 0, 255);							
			    super.draw(object, graphics, info);
			}
		});
		// HH - end
		
		environmentPortrayal.setPortrayalForClass(Target.class, new LabelledPortrayal2D( new HexagonalPortrayal2D()
		{
			/**
			 * 
			 */
			private static final long serialVersionUID = 1L;

			public void draw(Object object, Graphics2D graphics, DrawInfo2D info)
			{
				paint = new Color(0, 0, 0, 255); // Black			
			    super.draw(object, graphics, info);
			}
		}, "T", new Color(0, 0, 0), false) 
				
		);
		
		// reschedule the displayer
		display.reset();
		// redraw the display
		display.repaint();
	}
	
	/**
	 * This method...
	 * @param Controller c ()
	 */
    public void init(Controller c)
        {
        super.init(c);

        // make the displayer
        display = new Display2D((6*Constants.WorldXVal),(6*Constants.WorldYVal),this); // HH 30.7.14 Constants.WorldXVal and WorldYVal From (600x600)
        // turn off clipping
        display.setClipping(false);

        displayFrame = display.createFrame();
        displayFrame.setTitle("Environment Display");
        c.registerFrame(displayFrame);   // register the frame so it appears in the "Display" list
        displayFrame.setVisible(true);
		
		//adding the different layers to the display
		display.attach(terrainPortrayal,"Terrain");
		display.attach(obstaclesPortrayal,"Obstacles");	
        display.attach(environmentPortrayal, "Environment" );
        display.attach(wallPortrayal,"Wall");
        
        // HH 30/4/14 Add road portrayals (in right order)
        display.attach(roadsPortrayal, "Roads");
        display.attach(jctApproachPortrayal, "Junction Approaches");
        display.attach(junctionsPortrayal, "Junctions");
        // HH 17.6.14 Add road markings
        display.attach(roadMarkingPortrayal, "Road Markings");
        // HH end
        
        System.out.println("COModelWithUI.init is called!");
        }
    
    /**
     * This method...
     */
    public void quit()
    {
        super.quit();

        if (displayFrame!=null) displayFrame.dispose();
        displayFrame = null;
        display = null;
    }
    
    /**
     * This method...
     * @return Object ()
     */
    public Object getSimulationInspectedObject(){return state;}
    
    /**
     * This method..
     * @return Inspector ()
     */
    public Inspector getInspector()
    {
    	Inspector i = super.getInspector();
    	i.setVolatile(true);
    	return i;
    }   
}
