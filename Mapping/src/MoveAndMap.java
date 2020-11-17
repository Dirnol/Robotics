import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;

import javax.xml.stream.XMLStreamException;

import lejos.hardware.Button;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.GyroscopeAdapter;
import lejos.robotics.RangeFinderAdapter;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.geometry.Line;
import lejos.robotics.geometry.Rectangle;
import lejos.robotics.localization.CompassPoseProvider;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.mapping.EV3NavigationModel;
import lejos.robotics.mapping.LineMap;
import lejos.robotics.mapping.OccupancyGridMap;
import lejos.robotics.mapping.SVGMapLoader;
import lejos.robotics.navigation.DestinationUnreachableException;
import lejos.robotics.navigation.Move.MoveType;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Pose;
import lejos.robotics.navigation.Waypoint;
import lejos.robotics.objectdetection.Feature;
import lejos.robotics.objectdetection.FeatureDetector;
import lejos.robotics.objectdetection.FeatureListener;
import lejos.robotics.objectdetection.RangeFeatureDetector;
import lejos.robotics.pathfinding.Path;
import lejos.robotics.pathfinding.ShortestPathFinder;
import lejos.utility.Delay;
import lejos.utility.GyroDirectionFinder;

public class MoveAndMap {

    public static final float MAX_DISTANCE = 0.25f;
    public static final int DETECTOR_DELAY = 100;
    public static final int BOARD_WIDTH = 48;
    public static final int BOARD_LENGTH = 56;
    public static final float ROBOT_WIDTH = 6.5f;
    public static final float ROBOT_LENGTH = 9.5f;
    public static final float ULTRA_TO_CENTER_DIST = 3.0f; //UPDATE
    public static final float OBSTACLE_DIAMETER = 3.0f; //estimated based on project description
    public static final Rectangle BOUNDS = new Rectangle(0,0,BOARD_WIDTH,BOARD_LENGTH);
    public static final float DIST_BETWEEN_LINES = 6.0f; //VERIFY
    public static final Waypoint GOAL_ONE = new Waypoint(7, 32.5, 180);
    public static final Waypoint GOAL_TWO = new Waypoint(2, 15, 180); //unreachable with current setup
    
    private Wheel wheel1, wheel2;
    private EV3GyroSensor gyroSensor;
    private SampleProvider gyroProvider;
    private Chassis chassis;
    private final MovePilot pilot;
    private GyroscopeAdapter gyroAdapter;
    private GyroDirectionFinder gyroFinder;
    private PoseProvider opp;
    private EV3IRSensor irSensor;
    private EV3UltrasonicSensor ultraSensor;
    private EV3ColorSensor colorSensor;
    private RangeFeatureDetector detector;
    private final Navigator nav;
    private LineMap lineMap;
    private ShortestPathFinder pathFinder;
    
    private LineCheckThread lineCheckThread;
    
    private Waypoint home = new Waypoint(0.0, 0.0, 0);
    private Waypoint currentWaypoint;
    private boolean movingHome = false;
    
    private boolean detectingObstacle = false;
	
	public static void main(String[] args) {
		
		MoveAndMap program = null;
		try {
			program = new MoveAndMap();
			program.waitForButtonPress();
			program.test();
		} catch(Exception e) {
			System.out.println(e.getMessage());
			//e.printStackTrace();
			program.close();
			System.exit(0);
		}

	}
	
	public MoveAndMap() {
		
		irSensor = new EV3IRSensor(SensorPort.S1);
		colorSensor = new EV3ColorSensor(SensorPort.S2);
		gyroSensor = new EV3GyroSensor(SensorPort.S3);
		ultraSensor = new EV3UltrasonicSensor(SensorPort.S4);
		
		wheel1 = WheeledChassis.modelWheel(Motor.B, 3.2).offset(2.5).gearRatio(2.5).invert(true);
		wheel2 = WheeledChassis.modelWheel(Motor.C, 3.2).offset(-2.5).gearRatio(2.5).invert(true);
		
		gyroProvider = gyroSensor.getAngleMode();
		chassis = new WheeledChassis(new Wheel[] { wheel1, wheel2 }, WheeledChassis.TYPE_DIFFERENTIAL);
		
		 pilot = new MovePilot(chassis);
		 pilot.setAngularSpeed(45);
		 pilot.setLinearSpeed(5);
		 gyroAdapter = new GyroscopeAdapter(gyroProvider, 1);
		 gyroFinder = new GyroDirectionFinder(gyroAdapter);
		 opp = new OdometryPoseProvider(pilot);
		 //opp = new CompassPoseProvider(pilot, gyroFinder);
		 
	     detector = new RangeFeatureDetector(new RangeFinderAdapter(ultraSensor.getDistanceMode()), MAX_DISTANCE, DETECTOR_DELAY);
		 
		 lineMap = readLineMap();
		 pathFinder = new ShortestPathFinder(lineMap);
		 pathFinder.lengthenLines(3.0f);
		 
		 nav = new Navigator(pilot, opp);
		 
        detector.addListener(new FeatureListener() {
            public void featureDetected(Feature feature, FeatureDetector detector) {
                if (pilot.isMoving() && pilot.getMovement().getMoveType() != MoveType.ROTATE) {
                	System.out.println("Obstacle Detected!");
                	System.out.println(feature.getRangeReading().getRange() + "   " + feature.getRangeReading().getAngle());
                	detectingObstacle = true;
                    if (nav.isMoving()) {
                    	nav.stop();
                    	nav.clearPath();
                    }
                    float[] center = locateFeatureCenter(); //feature center in x,y coordinates
                    while(center == null) {
                    	Delay.msDelay(100);
                    }
                    addObstacleToMap(center);
                    reroute();     
                    nav.waitForStop();
                    detectingObstacle = false;
                }                   
            }       
	     });

		 detector.enableDetection(true);
		 detector.setPoseProvider(opp);
		 
		lineCheckThread = new LineCheckThread();
		lineCheckThread.setDirection(1);
		lineCheckThread.setColorSensor(colorSensor);
		lineCheckThread.start();
		
	}
	
	private void reroute() {
		try {
			Path newPath = pathFinder.findRoute(nav.getPoseProvider().getPose(), currentWaypoint, lineMap);
			nav.clearPath();
			nav.followPath(newPath);
			nav.waitForStop();
		} catch (DestinationUnreachableException e) {
			if(currentWaypoint == home) {
				System.out.println("We're Stuck!");
				close();
				System.exit(0);
			}
			currentWaypoint = home;
			reroute();
		}
	}
	
	// Assume we are already pointed somewhere at the feature, 
	// and the feature is roughly equal width and length
	// begin rotating to the left and right, record angles at which
	// detection stopped on both sides
	// assume object diameter is roughly 3" based on project description
	// use middle angle plus overall distance to locate center
	private float[] locateFeatureCenter() {
		
		System.out.println("CURRENT POSE");
		printPose();
		
		float[] center = new float[2];
		float threshold = 3.0f;
		float originalAngle = getAngle();
		
		float newDist = getUltraDistance();
		
		while(newDist < threshold) {
			preciseRelativeRotate(-1.0f);
			newDist = getUltraDistance();
		}
		float leftAngle = getAngle();
		
		preciseRotateTo(originalAngle);
		
		newDist = getUltraDistance();
		while(newDist < threshold) {
			preciseRelativeRotate(1.0f);
			newDist = getUltraDistance();
		}
		float rightAngle = getAngle();
		
		float centerAngle = (leftAngle + rightAngle) / 2.0f;
		preciseRotateTo(centerAngle);
		
		System.out.println("CENTER ANGLE " + centerAngle);
		
		float measuredDist = this.getDistanceAverage(10);
		
		System.out.println("MEASURED DIST " + measuredDist);
		
		float totalDist = measuredDist + ULTRA_TO_CENTER_DIST + (OBSTACLE_DIAMETER/2.0f);
		float x1 = nav.getPoseProvider().getPose().getX();
		float y1 = nav.getPoseProvider().getPose().getY();
		
		System.out.println("TOTAL DIST " + totalDist);
		
		//trig to figure out new x and y coords
		float xDist = (float) (totalDist * Math.sin(centerAngle));
		if(!movingHome)
			xDist = Math.abs(xDist);
		else
			xDist = -Math.abs(xDist);
		
		float yDist = (float) (totalDist * Math.cos(centerAngle));
		
		float x2 = x1 + xDist;
		float y2 = y1 + yDist;
		
		System.out.println("X DIST " + xDist + "   YDIST " + yDist);
		
		center[0] = x2;
		center[1] = y2;
		
		
		return center;
	}
	
	private void addObstacleToMap(float[] center) {
		float xCord = center[0];
		float yCord = center[1];
		float radius = OBSTACLE_DIAMETER / 2.0f;
		//add 4 lines for the feature to make a box
		Line top = new Line(xCord-radius, yCord-radius, xCord+radius, yCord-radius);
		Line right = new Line(xCord-radius, yCord-radius, xCord-radius, yCord+radius);
		Line left = new Line(xCord+radius, yCord-radius, xCord+radius, yCord+radius);
		Line bot = new Line(xCord-radius, yCord+radius, xCord+radius, yCord+radius);
		
		Line[] oldMap = lineMap.getLines();
		Line[] newMap = new Line[oldMap.length+4];
		for(int i = 0; i < oldMap.length; i++) {
			newMap[i] = oldMap[i];
		}
		int ind = oldMap.length;
		newMap[ind] = top;
		newMap[ind+1] = right;
		newMap[ind+2] = left;
		newMap[ind+3] = bot;
		
		lineMap = new LineMap(newMap, BOUNDS);
		printLineMap();
		
	}
	
	private void moveToFirstLine() {
		Path forwardPath = new Path();
		forwardPath.add(new Waypoint(10,0,0));
		nav.followPath(forwardPath);
		while(!lineCheckThread.newLineFound()) {
			Delay.msDelay(10);
		}
		nav.stop();
		nav.clearPath();
	}
	
	private void moveToNextLine() {
		if(!movingHome) {
			preciseRotateTo(0.0f);
			Waypoint nextLine = new Waypoint(
					nav.getPoseProvider().getPose().getX()+DIST_BETWEEN_LINES,
					nav.getPoseProvider().getPose().getY());
			Path p = new Path();
			p.add(nextLine);
			while(!lineCheckThread.newLineFound() || lineCheckThread.onSameLine()) {
				Delay.msDelay(10);
			}
			nav.stop();
			nav.clearPath();
			
		}
	}
	
	private void preciseRotateTo(float newAngle) {
		float angle = getAngle();
		//System.out.println(nav.getPoseProvider().getPose());
		//System.out.println(angle);
		
		while(angle >= newAngle + 1 || angle <= newAngle - 1) {
			nav.rotateTo(newAngle);
			nav.waitForStop();
			//Delay.msDelay(2000);
			angle = getAngle();
			//System.out.println(nav.getPoseProvider().getPose());
			//System.out.println(angle);
			opp.setPose(new Pose(opp.getPose().getX(), opp.getPose().getY(), angle));
			nav.setPoseProvider(opp);
		}
	}
	
	private void preciseRelativeRotate(float rotateAngle) {
		float angle = getAngle();
		angle += rotateAngle;
		preciseRotateTo(angle);
	}
	
	public void waitForButtonPress() {
		Thread buttonThread = new Thread() {
			@Override
			public void run() {
				Button.waitForAnyPress();
				close();
				System.exit(0);
			}		
		};
		buttonThread.start();
	}
	
	private void getBearings(int lineNum) {
		
		if(lineNum == 1)
			moveToFirstLine();
		else
			moveToNextLine();
		
		preciseRotateTo(90);
		float rightDist = getDistanceAverage(10);
		preciseRotateTo(-90);
		float leftDist = getDistanceAverage(10);
		
		System.out.println(rightDist);
		System.out.println(leftDist);
		
		if(leftDist + rightDist + ROBOT_LENGTH <= BOARD_WIDTH) {
			//obstacle is blocking one of the walls move to next line and try again
			
			
		}
		float offset = (BOARD_WIDTH - (rightDist + leftDist)) / 2;
		float y_position = leftDist + offset;
		float x_position = 8.5f; //distance from beginning to first line on either side of the board - wheel offset 
		x_position += (lineNum-1) * DIST_BETWEEN_LINES;
		
		opp.setPose(new Pose(x_position, y_position, -90));
		nav.setPoseProvider(opp);
		
		preciseRotateTo(0);
		
		System.out.println(nav.getPoseProvider().getPose());
	}
	
	public void test(){
		

		getBearings(1);
		getBearings(2);
/*
		try {
			Waypoint boardMiddle = new Waypoint(BOARD_LENGTH/2, BOARD_WIDTH/2, 0);
			System.out.println("GOING TO MIDDLE");
			nav.clearPath();
			
			nav.followPath(pathFinder.findRoute(nav.getPoseProvider().getPose(), boardMiddle, lineMap));
			currentWaypoint = boardMiddle;
			/*while(!nav.pathCompleted() || detectingObstacle) {}
			System.out.println("GOING TO GOAL");
			nav.clearPath();
			printPose();
			nav.followPath(pathFinder.findRoute(nav.getPoseProvider().getPose(), GOAL_ONE, lineMap));
			currentWaypoint = GOAL_ONE;
			while(!nav.pathCompleted() || detectingObstacle) {}
			System.out.println("GOING TO MIDDLE");
			nav.clearPath();
			printPose();
			nav.followPath(pathFinder.findRoute(nav.getPoseProvider().getPose(), boardMiddle, lineMap));
			currentWaypoint = boardMiddle;
			while(!nav.pathCompleted() || detectingObstacle) {}
			nav.clearPath();
			System.out.println("DONE");

		}catch(DestinationUnreachableException e) {
			
		}
		//close();
		//System.exit(0);*/
		
	}
	
	private void printPose() {
		System.out.println(nav.getPoseProvider().getPose());
	}
	
	public float getUltraDistance() {		
		SampleProvider sampleProvider = ultraSensor.getDistanceMode();
		float[] reading = new float[sampleProvider.sampleSize()];
		sampleProvider.fetchSample(reading, 0);
		
		float meterDist = reading[0];
		float inchDist = meterDist * 39.3701f;
		
		return inchDist;
	}
	
	public float getDistanceAverage(int total) {
		float average = 0.0f;
		for(int i = 0; i < total; i++)
			average += getUltraDistance();
		return average / (float)total;
	}
	
	public void close() {
		if(gyroSensor != null)
			gyroSensor.close();
		if(irSensor != null)
			irSensor.close();
		if(ultraSensor != null)
			ultraSensor.close();
		if(colorSensor != null)
			colorSensor.close();
	}
	
	private LineMap readLineMap() {
		FileInputStream fStream;
		try {
			fStream = new FileInputStream(new File(this.getClass().getResource("map.svg").getPath()));
			SVGMapLoader mapLoader = new SVGMapLoader(fStream);
			LineMap map = mapLoader.readLineMap();
			return map;
		} catch (FileNotFoundException e) {
			System.out.println(e.getMessage());
			//e.printStackTrace();
			close();
		} catch (XMLStreamException e) {
			System.out.println(e.getMessage());
			//e.printStackTrace();
			close();
		} catch (Exception e) {
			System.out.println(e.getMessage());
			close();
			System.exit(0);
		}
		return null;

	}
	
	private void printLineMap() {
		for(Line l : lineMap.getLines()) {
			System.out.println("(" + l.x1 + ", " + l.y1 + ") to (" + l.x2 + ", " + l.y2 + ")");
		}
	}
	
	public float getAngle() {
		float[] reading = new float[gyroProvider.sampleSize()];
		gyroProvider.fetchSample(reading, 0);
		return -reading[0];
	}
	
	public void printAngle(SampleProvider p) {
		float[] sample = new float[p.sampleSize()];
		p.fetchSample(sample, 0);
		System.out.println("Angle: " + sample[0]);
	}
	
}