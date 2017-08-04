package testing;

import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;

import sim.util.Double2D;
import modeling.COModel;
import modeling.CarPerformance;
import modeling.COModel.initialInfo;
import modeling.Constants;
import modeling.Constants.LineType;
import modeling.Road;
import modeling.Target;
import modeling.UGV;

public class UnitTestingExample {

	public UnitTestingExample() {
		// TODO Auto-generated constructor stub
	}

	public static void main(String[] args) {
		
		// TODO - enhance the code below so that it checks the return values are what would be expected and only reports
		// when the return values deviate from those required.
		
		// TODO Generate a COModel which contains individually defined roads running in 
		// predefined directions so that we can test some of the methods to ensure that
		// the return values are consistent with what would be expected
		COModel testModel = new COModel(0, 100, 100, false, 0, 1, true);
		
		// Construct a N/S road in the middle of the map, from 20m to 80m
		testModel.roads.add(new Road(testModel.getNewID(), Constants.SINGLETWOWAY, new Double2D(50,20), new Double2D(50,80)));
		
		// Construct a E/W road beside the other one, from 60m to 90m in the middle of the map
		testModel.roads.add(new Road(testModel.getNewID(), Constants.SINGLETWOWAY, new Double2D(60,50), new Double2D(90,50)));
		
		// Test some of the Road.getLine and Road.getThinLine methods to see if they are correct
		for (int i=0; i<testModel.roads.size(); i++)
		{
			System.out.println("Testing on Road = "+ ((Road)testModel.roads.get(i)).toString() + ".");
			
			Rectangle2D.Double tempR = ((Road)testModel.roads.get(i)).getLine(LineType.CENTRE);
			System.out.println("Test of getLine(CENTRE) = "+ tempR.x + ", " + tempR.y + ", " + tempR.width +  ", " + tempR.height + ".");
		
			tempR = ((Road)testModel.roads.get(i)).getLine(LineType.NESIDE);
			System.out.println("Test of getLine(NESIDE) = "+ tempR.x + ", " + tempR.y + ", " + tempR.width +  ", " + tempR.height + ".");
		
			tempR = ((Road)testModel.roads.get(i)).getLine(LineType.SWSIDE);
			System.out.println("Test of getLine(SWSIDE) = "+ tempR.x + ", " + tempR.y + ", " + tempR.width +  ", " + tempR.height + ".");
			
			Line2D.Double tempL = ((Road)testModel.roads.get(i)).getThinLine(LineType.CENTRE);
			System.out.println("Test of getThinLine(CENTRE) = "+ tempL.x1 + ", " + tempL.y1 + ", " + tempL.x2 +  ", " + tempL.y2 + ".");
		
			tempL = ((Road)testModel.roads.get(i)).getThinLine(LineType.NESIDE);
			System.out.println("Test of getThinLine(NESIDE) = "+ tempL.x1 + ", " + tempL.y1 + ", " + tempL.x2 +  ", " + tempL.y2 + ".");
		
			tempL = ((Road)testModel.roads.get(i)).getThinLine(LineType.SWSIDE);
			System.out.println("Test of getThinLine(SWSIDE) = "+ tempL.x1 + ", " + tempL.y1 + ", " + tempL.x2 +  ", " + tempL.y2 + ".");
		}
		
		// Add different test 'start locations' and check snapToLane
		Double2D testLoc = new Double2D(48, 50); // NB carriageway
		initialInfo retVal = testModel.snapToLane(testLoc.x, testLoc.y);
		System.out.println("Test of testModel NB finished with : startBearing = "+ retVal.startBearing + ", location = " + retVal.startLoc + ".");
		testLoc = new Double2D(51, 50); // SB carriageway
		retVal = testModel.snapToLane(testLoc.x, testLoc.y);
		System.out.println("Test of testModel SB finished with : startBearing = "+ retVal.startBearing + ", location = " + retVal.startLoc + ".");
		testLoc = new Double2D(70, 47.1); // EB carriageway, close to kerb
		retVal = testModel.snapToLane(testLoc.x, testLoc.y);
		System.out.println("Test of testModel EB finished with : startBearing = "+ retVal.startBearing + ", location = " + retVal.startLoc + ".");
		testLoc = new Double2D(60, 52.95); // WB carriageway, very close to kerb
		retVal = testModel.snapToLane(testLoc.x, testLoc.y);
		System.out.println("Test of testModel WB finished with : startBearing = "+ retVal.startBearing + ", location = " + retVal.startLoc + ".");
		
		// Add a UGV and test out a few Target locations to check HgetMinTargetKerbSeparation
		int targetID = testModel.getNewID();
		int UGVID = testModel.getNewID();
		UGV u = new UGV(UGVID, targetID, testModel.carStats, 0, testModel.junctions.size(), testModel);
		u.setLocation(new Double2D(50,50));
		testModel.ugvs.add(u);
		Target t = new Target(targetID);
		t.setLocation(new Double2D(47.5,50));
		testModel.allEntities.add(t);
		
		double d = testModel.HgetMinTargetKerbSeparation();
		System.out.println("Test of HgetMinTargetKerbSeparation NB finished with : " + d + ".");
		t.setLocation(new Double2D(51,50));
		d = testModel.HgetMinTargetKerbSeparation();
		System.out.println("Test of HgetMinTargetKerbSeparation SB finished with : " + d + ".");
		t.setLocation(new Double2D(75,48.5));
		d = testModel.HgetMinTargetKerbSeparation();
		System.out.println("Test of HgetMinTargetKerbSeparation EB finished with : " + d + ".");
		t.setLocation(new Double2D(75,52.5));
		d = testModel.HgetMinTargetKerbSeparation();
		System.out.println("Test of HgetMinTargetKerbSeparation WB finished with : " + d + ".");
		
		// Repeat the above test, but for HgetMinTargetCentreSeparation
		t.setLocation(new Double2D(47.5,50));
		d = testModel.HgetMinTargetCentreSeparation();
		System.out.println("Test of HgetMinTargetCentreSeparation NB finished with : " + d + ".");
		t.setLocation(new Double2D(51,50));
		d = testModel.HgetMinTargetCentreSeparation();
		System.out.println("Test of HgetMinTargetCentreSeparation SB finished with : " + d + ".");
		t.setLocation(new Double2D(75,48.5));
		d = testModel.HgetMinTargetCentreSeparation();
		System.out.println("Test of HgetMinTargetCentreSeparation EB finished with : " + d + ".");
		t.setLocation(new Double2D(75,52.5));
		d = testModel.HgetMinTargetCentreSeparation();
		System.out.println("Test of HgetMinTargetCentreSeparation WB finished with : " + d + ".");		
		
		// Repeat the above test, but for HgetUGVTargetRoadSeparation
		t.setLocation(new Double2D(47.5,50));
		d = testModel.HgetUGVTargetRoadSeparation();
		System.out.println("Test of HgetUGVTargetRoadSeparation NB finished with : " + d + ".");
		t.setLocation(new Double2D(51,50));
		d = testModel.HgetUGVTargetRoadSeparation();
		System.out.println("Test of HgetUGVTargetRoadSeparation SB finished with : " + d + ".");
		t.setLocation(new Double2D(75,48.5));
		d = testModel.HgetUGVTargetRoadSeparation();
		System.out.println("Test of HgetUGVTargetRoadSeparation EB finished with : " + d + ".");
		t.setLocation(new Double2D(75,52.5));
		d = testModel.HgetUGVTargetRoadSeparation();
		System.out.println("Test of HgetUGVTargetRoadSeparation WB finished with : " + d + ".");			
		
	}

}
