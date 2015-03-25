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
 * A class for running a simulation with a UI visualisation. This contains 'portrayals' which seem to
 * be synonymous with layers.  In general these are dicretised grids on which we can represent static
 * elements of the model, such as roads, junctions and obstacles.  A continuous portrayal is used to 
 * capture other moving elements of the simulation.   
 * 
 * @author Robert Lee/hh940
 */
public class COModelWithUI extends GUIState
{	
	protected COModelBuilder sBuilder;
	
	public Display2D display;
	public JFrame displayFrame;
	ContinuousPortrayal2D environmentPortrayal = new ContinuousPortrayal2D();
	FastValueGridPortrayal2D obstaclesPortrayal = new FastValueGridPortrayal2D("Obstacle", true);  // immutable
	FastValueGridPortrayal2D terrainPortrayal = new FastValueGridPortrayal2D("Terrain", true);  // immutable
	FastValueGridPortrayal2D wallPortrayal = new FastValueGridPortrayal2D("Wall", true);  // immutable
	
	// Roads/Junction/Road Marking portrayals
	FastValueGridPortrayal2D roadsPortrayal = new FastValueGridPortrayal2D("Roads", true); // immutable    
	FastValueGridPortrayal2D junctionsPortrayal = new FastValueGridPortrayal2D("Junctions", true); // immutable
	FastValueGridPortrayal2D jctApproachPortrayal = new FastValueGridPortrayal2D("Junction Approaches", true); // immutable
	FastValueGridPortrayal2D roadMarkingPortrayal = new FastValueGridPortrayal2D("Road Markings", true); // immutable
   
	/** 
	 * Create a new simulation, complete with a UI from which the internal and external random seeds can be
	 * defined to repeat an earlier run.  Alternatively, run repeatedly with different random seeds to see a
	 * variety of different maps and behaviour.  Set arguments to 0, True if want a 'normal' standard run
	 * without any faults inserted.
	 * 
	 * @param percentageFaults (double - value of 0..1 (incl) to set % of faults to be injected 
	 *                                   into the model OR index of single fault to be injected)
	 * @param inWantRandomFaults (boolean - true if faults should be activated at random at supplied frequency, 
	 *                                      false to specify one active fault)
	 */
    public COModelWithUI(double percentageFaults, boolean inWantRandomFaults) 
    { 
    	super(new COModel(new SecureRandom().nextInt(), Constants.WorldXVal, Constants.WorldYVal, true, percentageFaults, 0, inWantRandomFaults));     	
    	System.out.println("COModelWithUI is being called!"+ "it's state(model)is: "+ state.toString());
    	sBuilder = new COModelBuilder((COModel) state);
    }
        
    /**
     * This method actually runs the simulation, first resetting it, and then generating the map using the 
     * external and internal random seeds that are supplied in the UI - where they are not present e.g. for
     * the external seed, then they will be added during the generateSimulation method.  In addition to 
     * generating the map and starting the simulation, this method also sets up the portrayals so that the
     * map and simulation can be visualised.
     */
    public void start()
	{
		System.out.println("COModelWithUI.start is called  "+ sBuilder.sim);
		sBuilder.sim.reset();
		
		sBuilder.generateSimulation();
		
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
	 * @param state (SimState - the simulation state)
	 */
	public void load(SimState state)
	{
		sBuilder.sim.reset();
		sBuilder.generateSimulation();
		
		super.load(state);
		setupPortrayals();
	}
	
	/**
	 * A method which sets up the portrayals of the different layers in the UI,
	 * this is where details of the simulation are given specific colours/shapes
	 * so that they can be visualised in the UI.  Static objects such as the roads 
	 * are given their own discrete grid at an appropriate resolution, while moving
	 * objects such as Cars, or point objects such as the Target are added to the
	 * continuous and mutable grid environmentPortrayal.
	 */
	public void setupPortrayals()
	{		
		COModel simulation = (COModel) state;
		
		// Background images portrayals e.g. roads should be drawn first to try and improve appearance of vehicles driving
		// over e.g. the Junction Approaches - otherwise vehicles will change colour due to the transparency of the approaches
		
		// Roads portrayal
		roadsPortrayal.setField(simulation.roadMap);
		roadsPortrayal.setMap(new sim.util.gui.SimpleColorMap(
				0,
				Constants.DUALTWOWAY,
				new Color(0,0,0,0),
				new Color(0,0,255,255) // Light grey
				));

		// Draw the junction approaches (ordering here not important though)
		jctApproachPortrayal.setField(simulation.jctApproachMap);
		jctApproachPortrayal.setMap(new sim.util.gui.SimpleColorMap(
				0,
				1,
				new Color(0,0,0,0),
				new Color(102,51,0,50) // slightly opaque brown/beige
				));
		
		// Draw the junctions
		junctionsPortrayal.setField(simulation.junctionMap);
		junctionsPortrayal.setMap(new sim.util.gui.SimpleColorMap(
				0,
				1,
				new Color(0,0,0,0),
				new Color(102,51,0,60) // slightly darker opaque brown beige
				));
		
		// Add the road markings (these are on a higher resolution grid) and are
		// not always visible unless you zoom in.
		roadMarkingPortrayal.setField(simulation.roadMarkingMap);
		roadMarkingPortrayal.setMap(new sim.util.gui.SimpleColorMap(
				0,
				Constants.WHITERPAINT,
				new Color(0,0,0,0),
				new Color(255,255,255,255) // White
				));
		
		// Add the Parked Car obstacles
		obstaclesPortrayal.setField(simulation.obstacleMap);
		obstaclesPortrayal.setMap(new sim.util.gui.SimpleColorMap(
				0,
				1,
				new Color(0,0,0,0),
				new Color(0,0,255,255) // Blue
				));
        
		// Add the walls - appears as a boundary around the map
		wallPortrayal.setField(simulation.wallMap);
		wallPortrayal.setMap(new sim.util.gui.SimpleColorMap(
				0,
				1,
				new Color(0,0,0,0),
				new Color(255,0,0,255) // Red
				));		
				
		// Portrayals for DumbCar i.e. the Moving Obstacles
		// tell the portrayals what to portray and how to portray them
		environmentPortrayal.setField(simulation.environment);	
		environmentPortrayal.setPortrayalForClass(DumbCar.class, new OrientedPortrayal2D( new LabelledPortrayal2D( new RectanglePortrayal2D(Constants.OBSTACLE_WIDTH * 6)
		{
			/**
			 * Green rectangles, labelled with ID, and displaying orientation (highlighted with red line pointing in direction of travel)
			 * TODO: This implementation does not scale well if the zoom level is changed in the user-interface; vehicles are displaced
			 * to inaccurate locations on the map. 
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
					graphics.setPaint(new Color(0,0,0)); // Black if inactive (shouldn't happen)
				}

				// Create a rectangle object that we can rotate to point in the right direction, note that the
				// location of the vehicle is taken to mean the centre front of the vehicle
				Rectangle2D.Double carRectangle = new Rectangle2D.Double();
				carRectangle = new Rectangle2D.Double(
						(info.draw.x - ((Constants.OBSTACLE_LENGTH)*info.draw.width)), (info.draw.y - ((Constants.OBSTACLE_WIDTH/2)*info.draw.height)), 
						(info.draw.width*Constants.OBSTACLE_LENGTH), 
						(info.draw.height*Constants.OBSTACLE_LENGTH*(Constants.OBSTACLE_WIDTH/Constants.OBSTACLE_LENGTH)));

				AffineTransform rotateTransform = AffineTransform.getRotateInstance(((Car) object).orientation2D(), 
						((Car)object).getLocation().x*6, ((Car)object).getLocation().y*6);

				Shape carShape = rotateTransform.createTransformedShape(carRectangle);
				graphics.draw(carShape);
				graphics.fill(carShape);
			}
		}, null, new Color(0, 0, 0), false), 0, 2)
		
		); // end setPortrayalForClass DumbCar
		
		// Portrayal for UGVs
		// tell the portrayals what to portray and how to portray them
		environmentPortrayal.setField( simulation.environment );	
		environmentPortrayal.setPortrayalForClass(UGV.class, new OrientedPortrayal2D(new RectanglePortrayal2D(Constants.UGV_WIDTH)
		{
			/**
			 * Orange squares, displaying orientation (highlighted with red line pointing in direction of travel)
			 * TODO: This implementation does not scale well if the zoom level is changed in the user-interface; vehicles are displaced
			 * to inaccurate locations on the map. 
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
					graphics.setPaint(new Color(0,0,0)); // Black (for when reaches Target)
				}
				
				AffineTransform rotateTransform = AffineTransform.getRotateInstance(((Car) object).orientation2D(), 
						((Car)object).getLocation().x*6, ((Car)object).getLocation().y*6);
				
				Shape carShape = rotateTransform.createTransformedShape(new Rectangle2D.Double((info.draw.x - ((Constants.UGV_WIDTH)*info.draw.width)), 
						(info.draw.y - ((Constants.UGV_WIDTH/2)*info.draw.height)), 
						(info.draw.width*Constants.UGV_WIDTH), (info.draw.height*Constants.UGV_WIDTH)));
				
				graphics.draw(carShape);
				graphics.fill(carShape); 
			}
		}, 0, 2)
		
		); // end setPortrayalForClass UGV

		// Portrayal for Waypoints (in order to make them less intrusive in the display)
		// tell the portrayals what to portray and how to portray them
		environmentPortrayal.setField( simulation.environment );	
		environmentPortrayal.setPortrayalForClass(Waypoint.class, (new RectanglePortrayal2D(0.3)
		{
			/**
			 * Yellow points - very small rectangles
			 */
			private static final long serialVersionUID = 1L;

			public void draw(Object object, Graphics2D graphics, DrawInfo2D info)
			{
				paint = new Color(255, 255, 0, 255); // Yellow								
			    super.draw(object, graphics, info);
			}
		})
		
		); // end setPortrayalForClass Waypoint
		
		// Portrayal for Failures
		// tell the portrayals what to portray and how to portray them
		environmentPortrayal.setField( simulation.environment );	
		environmentPortrayal.setPortrayalForClass(Failure.class, new RectanglePortrayal2D(0.75)
		{
			/**
			 * Black points - small rectangles
			 */
			private static final long serialVersionUID = 1L;

			public void draw(Object object, Graphics2D graphics, DrawInfo2D info)
			{
				paint = new Color(0, 0, 0, 255); // Black							
			    super.draw(object, graphics, info);
			}
		}); // end setPortrayalForClass Failure
		

		// Portrayal for Crashes
		// tell the portrayals what to portray and how to portray them
		environmentPortrayal.setField( simulation.environment );	
		environmentPortrayal.setPortrayalForClass(Crash.class, new RectanglePortrayal2D(1)
		{
			/**
			 * Red points - small rectangles
			 */
			private static final long serialVersionUID = 1L;

			public void draw(Object object, Graphics2D graphics, DrawInfo2D info)
			{
				paint = new Color(255, 0, 0, 255);	// Red						
			    super.draw(object, graphics, info);
			}
		}); // end setPortrayalForClass Crashes
		
		environmentPortrayal.setPortrayalForClass(Target.class, new LabelledPortrayal2D( new HexagonalPortrayal2D()
		{
			/**
			 * Black points - small hexagon labelled with a 'T'
			 */
			private static final long serialVersionUID = 1L;

			public void draw(Object object, Graphics2D graphics, DrawInfo2D info)
			{
				paint = new Color(0, 0, 0, 255); // Black			
			    super.draw(object, graphics, info);
			}
		}, "T", new Color(0, 0, 0), false) 
				
		); // end setPortrayalForClass Target
		
		// reschedule the displayer
		display.reset();
		// redraw the display
		display.repaint();
	}
	
	/**
	 * This method sets up the display and attaches all the portrayals/layers
	 * ready for visualisation
	 * @param c (Controller - controls windows and visualisation)
	 */
    public void init(Controller c)
    {
        super.init(c);

        // Make the displayer
        display = new Display2D((6*Constants.WorldXVal),(6*Constants.WorldYVal),this); // HH 30.7.14 Constants.WorldXVal and WorldYVal From (600x600)
        // Turn off clipping
        display.setClipping(false);

        displayFrame = display.createFrame();
        displayFrame.setTitle("Environment Display");
        c.registerFrame(displayFrame);   // Register the frame so it appears in the "Display" list
        displayFrame.setVisible(true);
		
		// Adding the different layers to the display
		display.attach(terrainPortrayal,"Terrain");
		display.attach(obstaclesPortrayal,"Obstacles");	
        display.attach(environmentPortrayal, "Environment" );
        display.attach(wallPortrayal,"Wall");
        
        // Add road portrayals (in right order)
        display.attach(roadsPortrayal, "Roads");
        display.attach(jctApproachPortrayal, "Junction Approaches");
        display.attach(junctionsPortrayal, "Junctions");
        // Add road markings
        display.attach(roadMarkingPortrayal, "Road Markings");
        
        System.out.println("COModelWithUI.init is called!");
    }
    
    /**
     * This method removes the UI.
     */
    public void quit()
    {
        super.quit();

        if (displayFrame!=null) displayFrame.dispose();
        displayFrame = null;
        display = null;
    }
    
    /**
     * This method returns the SimState object.
     * @return Object (the SimState object)
     */
    public Object getSimulationInspectedObject() {return state;}
    
    /**
     * This method returns the Inspector object.
     * @return Inspector (the Inspector object)
     */
    public Inspector getInspector()
    {
    	Inspector i = super.getInspector();
    	i.setVolatile(true);
    	return i;
    }   
}
