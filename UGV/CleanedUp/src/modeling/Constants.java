package modeling;

/**
 *
 * @author Robert Lee
 */
public interface Constants
{
	//give all values stored names in all caps to fit with C style #define-s in a header file
	// Entity Types
	public static final int TOTHER = 0; // a placeholder - save 0 for entities which aren't mentioned elsewhere
	public static final int TSTOPPER = 1; // the type constant for the stopper class
	public static final int TCAR = 2; // the type constant of a car
	public static final int TTARGET = 3; // the type contsant of a target
	public static final int TWAYPOINT = 4; // the type constant of a waypoint
	public static final int TCIROBSTACLE = 5; // the type constant of an obstacle
	public static final int TWALL = 6; // the type constant of a wall
	public static final int TUTURNWP = 7; // the type constant for a U-turn waypoint 
	public static final int TFAILURE = 8; // the type constant for the location of a failure
	public static final int TPARKEDCAR = 9; // the type constant for the location of a parked car (obstacle)
	public static final int TUGV = 10; // type constant for UGV (used for selective error reporting)
	public static final int DUMBCAR = 11; // type constant for a car obstacle
	public static final int TWAITING = 12; // type constant for a car/UGV that is waiting for a junction to become free
	
	// Movement Constants
	public static final boolean ACCELERATE = true;
	public static final boolean DECELERATE = false;
	
	// Types of Accident
	public static enum AccidentType
	{
		CLASHWITHOBSTACLE,
		CLASHWITHWALL,
		CLASHWITHOTHERCAR,
		LEAVEROAD,
		CROSSCENTRELINE,
		CROSS_NE_LINE,
		CROSS_SW_LINE,
		TIMEOUT;
	}
	
	// Faults
	public static final int MAX_FAULTS = 19;
	
	// Road Constants
	public static final int NOTROAD = 0;
	public static final int SINGLEONEWAY = 1;
	public static final int SINGLETWOWAY = 2;
	public static final int DUALONEWAY = 3;
	public static final int DUALTWOWAY = 4;
	
	// Junction Constants
	public static final int NOJUNCTION = 0;
	public static final int TTJUNCTION = 1;
	public static final int TXJUNCTION = 2;
	
	public static final int T_NORTH = 0; // Where the junction is a T, we need to know which arm is not used
	public static final int T_EAST = 1;
	public static final int T_SOUTH = 2;
	public static final int T_WEST= 3;
	
	// Road Marking Constants
	public static enum LineType
	{
		NESIDE, // represents a decrease in either x or y compared to the centre of the road
		CENTRE,
		SWSIDE // represents an increase in either x or y compared to the centre of the road
	}
	
	// For when we want to be explicit relative to current location/heading
	public static enum genLineType
	{
		NEARSIDE, // The same side of the road as the vehicle, or nearer to the kerb than the vehicle
		CENTRE,
		OFFSIDE // The opposite side of the road, or further from the kerb than the vehicle
	}
	
	public static final double ROADEDGINGWIDTH = 0.1; // 10cm in metres - width of road edging
	public static final double CENTRELANEWIDTH = 0.1; // 10cm in metres - width of centre line
	public static final double ROADEDGEOFFSET = 0.225; // 22.5cm in metres - distance between marking and edge of road
	
	public static final int NOPAINT = 0;
	public static final int WHITEPAINT = 1; 
	public static final int WHITERPAINT = 2; // Can be used in place of 'double' lines to denote a different line type
	
	// Compass direction in which vehicle is facing (used for working out nearside/offside relations)
	public static enum UGV_Direction
	{
		NORTH,
		EAST,
		SOUTH,
		WEST	
	}
	
	// World Size / Model Params
	public static final double WorldXVal = 200;
	public static final double WorldYVal = 100;
	public static final int MIN_JUNCTIONS = 5;
	public static final int MAX_JUNCTIONS = 25;
	
	// HH Obstacles/Moving Obstacles Params
	public static final double CAR_SPEED = 2.5; // this is m/step and is equivalent to a speed of 45km/hr under the assumption of 5 steps/s
	public static final double OBSTACLE_HEADWAY = 7; // an overtaking vehicle should stop 7m ahead of the obstacle to allow for manoeuvre
	public static final double OBSTACLE_LENGTH = 5; // assume that obstacles will be 5m long (but don't pull back in front until passed object)
	public static final double OBSTACLE_WIDTH = 2; // assume that obstacles will be 5m long (but don't pull back in front until passed object)
	public static final double UGV_WIDTH = 2; 
	public static final double OBSTACLE_BUFFER = 2; // Also used when generating network to prevent Obs being added too close
	
	public static enum OvertakeStage
	{
		OVERTAKE_START,
		OVERTAKE_PULLEDOUT,
		OVERTAKE_FINISH,
		NOT_OVERTAKING,
		WAIT // Used when we have to wait for an oncoming vehicle
	}
	
	// Minimum stopping distance to try and encourage UGV to be slightly less cautious
	public static final double MIN_STOPPINGDISTANCE = OBSTACLE_LENGTH; // Stay 1 car length behind
	
	public static final int MIN_OBSTACLES = 5;
	public static final int MAX_OBSTACLES = 10;
	
	// Moving Obstacle Car Constants
	public static final int MIN_CARS = 5;
	public static final int MAX_CARS = 20;
	
	public static final int MAX_ITERATIONS = 50; // Used for controlling loop when adding cars/obstacles
		
	// Number of different Maps/Networks to generate on a given run
	public static final int NO_RANDOM_RUNS = 20;
	
	// File path for the output files
	public static final String outFilePath = "D:\\hh940\\UGVResultsBackup\\TEMP\\"; 
	
	// Situation Coverage Constants
	
	// Used as a return value from the method which tests candidate external random seeds by 
	// generating the map and calculating specified (coverage) criteria
	public class coverageCriteriaInfo {
		public double distTargetToObs;
		public double distUGVToTarget;
		public double distPrevJctToTarget;
				
		public coverageCriteriaInfo ()
		{
			distTargetToObs = 0;
			distUGVToTarget = 0;
			distPrevJctToTarget = 0;
		}
		
		public coverageCriteriaInfo (double inDistTargetToObs, double inDistUGVToTarget, double inDistPrevJctToTarget) {
			distTargetToObs = inDistTargetToObs;
			distUGVToTarget = inDistUGVToTarget;
			distPrevJctToTarget = inDistPrevJctToTarget;
		}
	}
	
	// Constants used for calculating situation coverage
	public static final int NO_CATEGORIES = 6;
	public static final int NO_BOXES = NO_CATEGORIES*NO_CATEGORIES*NO_CATEGORIES;
	public static final int REQ_COV_COUNT = 1; // limit on the number of seeds that need to cover each 'box'
}
