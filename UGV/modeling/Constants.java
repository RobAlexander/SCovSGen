package modeling;
/**
 *
 * @author Robert Lee
 */
public interface Constants
{
	//give all values stored names in all caps to fit with C style #define-s in a
	//header file
	//Entity Types
	public static final int TOTHER = 0; //a placeholder - save 0 for entities which aren't mentioned elsewhere
	public static final int TSTOPPER = 1; //the type constant for the stopper class
	public static final int TCAR = 2; //the type constant of a car
	public static final int TTARGET = 3; //the type contant of a target
	public static final int TWAYPOINT = 4; //the type constant of a waypoint
	public static final int TCIROBSTACLE = 5; //the type constant of an obstacle
	public static final int TWALL = 6; //the type constant of a wall
	public static final int TUTURNWP = 7; //the type constant for a U-turn waypoint - HH 10.7.14
	public static final int TFAILURE = 8; //the type constant for the location of a failure - HH 15.7.14
	public static final int TPARKEDCAR = 9; //the type constant for the location of a parked car (obstacle) - HH 6.8.14
	public static final int TUGV = 10; //type constant for UGV (used for selective error reporting) - HH 25.8.14
	public static final int DUMBCAR = 11; //type constant for a car obstacle - HH 25.8.14
	public static final int TWAITING = 12; // type constant for a car/UGV that is waiting for a junction to become free
	
	//Terrain types
	public static final int NORMAL = 0;
	public static final int ICE = 1;
	public static final int GRAVEL = 2;
	
	//Movement Constants
	public static final boolean ACCELERATE = true;
	public static final boolean DECELERATE = false;
	
	public static enum AccidentType
	{
		CLASHWITHOBSTACLE,
		CLASHWITHWALL,
		CLASHWITHOTHERCAR,
		LEAVEROAD,
		CROSSCENTRELINE,
		CROSS_NW_LINE,
		CROSS_SE_LINE,
		TIMEOUT;
	}
	
	// HH 22/7/14 - Faults
	public static final int MAX_FAULTS = 19;
	// HH end
	
	// HH 30/4/14 - Road
	public static final int NOTROAD = 0;
	public static final int SINGLEONEWAY = 1;
	public static final int SINGLETWOWAY = 2;
	public static final int DUALONEWAY = 3;
	public static final int DUALTWOWAY = 4;
	
	public static final int NOJUNCTION = 0;
	public static final int TTJUNCTION = 1;
	public static final int TXJUNCTION = 2;
	
	public static final int T_NORTH = 0; // Where the junction is a T, we need to know which arm is not used
	public static final int T_EAST = 1;
	public static final int T_SOUTH = 2;
	public static final int T_WEST= 3;
	
	// HH 16/6/14
	public static enum LineType
	{
		SESIDE,
		CENTRE,
		NWSIDE
	}
	
	// HH 9.9.14 - For when we want to be explicit relative to current location/heading
	public static enum genLineType
	{
		NEARSIDE,
		CENTRE,
		OFFSIDE
	}
	
	public static final double ROADEDGINGWIDTH = 0.1; // 10cm in metres - width of road edging
	public static final double CENTRELANEWIDTH = 0.1; // 10cm in metres - width of centre line
	public static final double ROADEDGEOFFSET = 0.225; // 22.5cm in metres - distance between marking and edge of road
	
	// HH 17.6.14
	public static final int NOPAINT = 0;
	public static final int WHITEPAINT = 1; 
	public static final int WHITERPAINT = 2;
	
	// HH 18.6.14 - direction in which vehicle is facing (used for working out nearside/offside relations)
	public static enum UGV_Direction
	{
		NORTH,
		EAST,
		SOUTH,
		WEST	
	}
	
	// HH 30.7.14 World Size / Model Params
	public static final double WorldXVal = 200;
	public static final double WorldYVal = 100;
	public static final int MIN_JUNCTIONS = 5;
	public static final int MAX_JUNCTIONS = 25;
	
	// HH 6.8.14 Obstacles/Moving Obstacles Params
	public static final double CAR_SPEED = 2.5; // this is m/step and is equivalent to a speed of 45km/hr under the assumption of 5 steps/s
	public static final double OBSTACLE_HEADWAY = 5; // an overtaking vehicle should stop 5m ahead of the obstacle to allow for manoeuvre
	public static final double OBSTACLE_LENGTH = 5; // assume that obstacles will be 5m long (but don't pull back in front until passed object)
	public static final double OBSTACLE_WIDTH = 2; // assume that obstacles will be 5m long (but don't pull back in front until passed object)
	public static final double UGV_WIDTH = 2; // HH 8.9.14
	
	public static final int MIN_OBSTACLES = 5;
	public static final int MAX_OBSTACLES = 10;
	
	// HH 13.8.14 More obstacles params
	public static enum OvertakeStage
	{
		OVERTAKE_START,
		OVERTAKE_PULLEDOUT,
		OVERTAKE_FINISH,
		NOT_OVERTAKING,
		WAIT // Used when we have to wait for an oncoming vehicle
	}
	
	// HH 26.8.14 Obstacle Car params
	public static final int MIN_CARS = 5;
	public static final int MAX_CARS = 20;
	
	public static final int MAX_ITERATIONS = 50; // Used for controlling loop when adding cars/obstacles
	// HH end
	
	// HH 1.9.14 Number of different Maps/Networks to generate on a given run
	public static final int NO_RANDOM_RUNS = 20;
	
}
