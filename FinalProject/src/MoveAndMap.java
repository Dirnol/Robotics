import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Collection;

import javax.xml.stream.XMLStreamException;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RangeFinderAdapter;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.geometry.Rectangle;
import lejos.robotics.mapping.LineMap;
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
import lejos.robotics.pathfinding.AstarSearchAlgorithm;
import lejos.robotics.pathfinding.FourWayGridMesh;
import lejos.robotics.pathfinding.Node;
import lejos.robotics.pathfinding.NodePathFinder;
import lejos.robotics.pathfinding.Path;
import lejos.robotics.pathfinding.ShortestPathFinder;
import lejos.utility.Delay;

public class MoveAndMap {

    public static final float MAX_DISTANCE = 0.20f;
    public static final float BALL_DISTANCE = 15.5f;
    public static final int DETECTOR_DELAY = 100;
    public static final int BOARD_WIDTH = 48;
    public static final int BOARD_LENGTH = 56;
    public static final float ROBOT_WIDTH = 6.5f;
    public static final float ROBOT_LENGTH = 9.5f;
    public static final float ULTRA_TO_CENTER_DIST = 3.0f; //UPDATE
    public static final float OBSTACLE_DIAMETER = 3.0f; //estimated based on project description
    public static final Rectangle BOUNDS = new Rectangle(0,0,BOARD_WIDTH,BOARD_LENGTH);
    public static final float DIST_BETWEEN_LINES = 6.0f; //VERIFY
    public static final Waypoint GOAL_ONE = new Waypoint(50, 17, 0);
    public static final Waypoint GOAL_TWO = new Waypoint(2, 15, 180); //unreachable with current setup
    
    private EV3LargeRegulatedMotor motor1, motor2;
    private Wheel wheel1, wheel2;
    private EV3MediumRegulatedMotor claw;
    private EV3GyroSensor gyroSensor;
    private SampleProvider gyroProvider;
    private Chassis chassis;
    private final MovePilot pilot;
    private EV3IRSensor irSensor;
    private EV3UltrasonicSensor ultraSensor;
    private EV3ColorSensor colorSensor;
    private RangeFeatureDetector obstacleDetector, ballDetector;
    private Navigator nav;
    private LineMap lineMap;
    private ShortestPathFinder pathFinder;
    
    private FourWayGridMesh gridMesh;
    private AstarSearchAlgorithm astar;
    private NodePathFinder nodePathFinder;
    
    private LineCheckThread lineCheckThread;
    private boolean movingHome = false;
    
    private boolean detectingObstacle = false;
    private boolean foundBall = false;
    private boolean clawOpen = true;
    
    public float rx, ry;
    
    private int goals;
    private boolean gameOver;
	
	public static void main(String[] args) {
		
		MoveAndMap program = null;
		
		try {
			program = new MoveAndMap();
			new RemoteServer(program);
		} catch(Exception e) {
			System.out.println(e.getMessage());
			e.printStackTrace();
			program.close();
			System.exit(0);
		}

	}
	
	public MoveAndMap() {
		

		gyroSensor = new EV3GyroSensor(SensorPort.S3);

		
		motor1 = new EV3LargeRegulatedMotor(MotorPort.B);
		motor2 = new EV3LargeRegulatedMotor(MotorPort.C);
		claw = new EV3MediumRegulatedMotor(MotorPort.D);
		
		
		wheel1 = WheeledChassis.modelWheel(motor1, 3.2).offset(2.5).gearRatio(2.5).invert(true);
		wheel2 = WheeledChassis.modelWheel(motor2, 3.2).offset(-2.5).gearRatio(2.5).invert(true);
		
		gyroProvider = gyroSensor.getAngleMode();
		chassis = new WheeledChassis(new Wheel[] { wheel1, wheel2 }, WheeledChassis.TYPE_DIFFERENTIAL);
		
		 pilot = new MovePilot(chassis);
		 pilot.setAngularSpeed(60);
		 pilot.setLinearSpeed(4.0);
		 
		
	}
	
	public void startAuto() {
		initAuto();
		startGame();
	}

	public void initAuto() {
		irSensor = new EV3IRSensor(SensorPort.S1);
		colorSensor = new EV3ColorSensor(SensorPort.S2);
		ultraSensor = new EV3UltrasonicSensor(SensorPort.S4);
		
	     
		 
		lineMap = readLineMap();
		 //pathFinder = new ShortestPathFinder(lineMap);
		 //pathFinder.lengthenLines(3.0f);
		 
		astar = new AstarSearchAlgorithm();
		gridMesh = new FourWayGridMesh(lineMap, 1.0f, 3.0f);
		nodePathFinder = new NodePathFinder(astar, gridMesh);
		
		for(Node n : gridMesh.getMesh()) {
			if(n.x == 20)
			System.out.println(n.x + ", " + n.y);
		}
		 
		nav = new Navigator(pilot);
		 
		obstacleDetector = new RangeFeatureDetector(new RangeFinderAdapter(ultraSensor.getDistanceMode()), MAX_DISTANCE, DETECTOR_DELAY);
		obstacleDetector.addListener(new FeatureListener() {
			public void featureDetected(Feature feature, FeatureDetector detector) {
				if (pilot.isMoving() || pilot.getMovement().getMoveType() != MoveType.ROTATE || detectingObstacle) {
	           		Pose snapshot = new Pose(getPoseX(), getPoseY(), getPoseHeading());
	           		boolean known = checkIfKnownObstacle(
	           				toInches(feature.getRangeReading().getRange()),
	           				snapshot
	           				);
	           		
	           		if(known)
	           			return;
	           		else
	           			System.out.println("Unknown Obstacle!");
	           		
	           		if(!detectingObstacle) {
	           			detectingObstacle = true;
		                    /*if (nav.isMoving()) {
		                    	nav.stop();
		                    	nav.clearPath();
		                    }*/
		                    float[] center = locateFeatureCenter(
		                    		toInches(feature.getRangeReading().getRange()),
			           				snapshot); //feature center in x,y coordinates
		                	removeObstacleNodes(center);
		                	
		                	detectingObstacle = false;
	           		}
               }                   
           }       
	     });

		 obstacleDetector.enableDetection(false);
		 obstacleDetector.setPoseProvider(nav.getPoseProvider());
		 
		lineCheckThread = new LineCheckThread();
		lineCheckThread.setDirection(1);
		lineCheckThread.setColorSensor(colorSensor);
		lineCheckThread.start();
		
		ballDetector = new RangeFeatureDetector(new RangeFinderAdapter(irSensor.getDistanceMode()), BALL_DISTANCE, DETECTOR_DELAY);
		ballDetector.enableDetection(false);
		ballDetector.setPoseProvider(nav.getPoseProvider());
	}
	
	public void openClaw() {
		if(clawOpen)
			return;
		
		clawOpen = true;
		claw.forward();
		Delay.msDelay(2000);
		claw.stop();
	}
	
	public void closeClaw() {
		if(!clawOpen)
			return;

		clawOpen = false;
		claw.backward();
		Delay.msDelay(2000);
		claw.stop();
	}
        
    private boolean checkIfKnownObstacle(float sensorDist, Pose poseSnapshot) {

    	float centerAngle = poseSnapshot.getHeading();
    	float x1 = poseSnapshot.getX();
    	float y1 = poseSnapshot.getY();
		
		float totalDist = sensorDist + ULTRA_TO_CENTER_DIST + (OBSTACLE_DIAMETER/2.0f);
		
		//trig to figure out new x and y coords
		float xDist = (float) (totalDist * Math.cos(Math.toRadians(centerAngle)));
		float yDist = (float) (totalDist * Math.sin(Math.toRadians(centerAngle)));
		
		float x2 = x1 + xDist;
		float y2 = y1 + yDist;
		
		boolean foundOnMap = false;
		for(Node n : gridMesh.getMesh()) {
			if(n.x == (int)x2 && n.y == (int)y2) {
				foundOnMap = true;
				break;
			}	
		}
		
		if(foundOnMap)
			return false;
		return true;
    }
        
    private float toInches(float meters) {
    	return meters * 39.3701f;
    }
	
	// Assume we are already pointed somewhere at the feature, 
	// and the feature is roughly equal width and length
	// begin rotating to the left and right, record angles at which
	// detection stopped on both sides
	// assume object diameter is roughly 3" based on project description
	// use middle angle plus overall distance to locate center
	private float[] locateFeatureCenter(float sensorDist, Pose poseSnapshot) {
		
    	float centerAngle = poseSnapshot.getHeading();
    	float x1 = poseSnapshot.getX();
    	float y1 = poseSnapshot.getY();
		
		float totalDist = sensorDist + ULTRA_TO_CENTER_DIST + (OBSTACLE_DIAMETER/2.0f);
		
		//trig to figure out new x and y coords
		float xDist = (float) (totalDist * Math.cos(Math.toRadians(centerAngle)));
		float yDist = (float) (totalDist * Math.sin(Math.toRadians(centerAngle)));
		
		float x2 = x1 + xDist;
		float y2 = y1 + yDist;

		
		float[] center = new float[2];
		/*float threshold = 6.0f;
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
		
		//System.out.println("CENTER ANGLE " + centerAngle);
		
		float measuredDist = this.getDistanceAverage(10);
		
		//System.out.println("MEASURED DIST " + measuredDist);
		
		float totalDist = measuredDist + ULTRA_TO_CENTER_DIST + (OBSTACLE_DIAMETER/2.0f);
		float x1 = nav.getPoseProvider().getPose().getX();
		float y1 = nav.getPoseProvider().getPose().getY();
		
		//System.out.println("TOTAL DIST " + totalDist);
		
		//trig to figure out new x and y coords
		float xDist = (float) (totalDist * Math.cos(Math.toRadians(centerAngle)));
		float yDist = (float) (totalDist * Math.sin(Math.toRadians(centerAngle)));
		
		float x2 = x1 + xDist;
		float y2 = y1 + yDist;
		
		//System.out.println("X DIST " + xDist + "   YDIST " + yDist);
		*/
		center[0] = x2;
		center[1] = y2;
		
		
		return center;
	}
	
	private void removeObstacleNodes(float[] center) {
		int xCord = (int) center[0];
		int yCord = (int) center[1];
		int obstacleRadius = 5; //obstacle radius plus cushion to avoid collision
		
		Collection<Node> nodes = gridMesh.getMesh();
		Collection<Node> toRemove = new ArrayList<Node>();
		System.out.println(nodes.size());
		for(Node n : nodes) {
			if(xCord - obstacleRadius <= n.x && xCord + obstacleRadius >= n.x) {
				if(yCord - obstacleRadius <= n.y && yCord + obstacleRadius >= n.y) {
					toRemove.add(n);
				}
			}
		}
		
		System.out.println(toRemove.size());
		for(Node n : toRemove) {
			gridMesh.removeNode(n);
		}
	}
	
	private void moveToFirstLine() {
		lineCheckThread.resetNewLine();
		nav.getPoseProvider().setPose(new Pose(0,0,0));
		nav.goTo(10,0);
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
			while(!lineCheckThread.newLineFound()) {
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
		int tries = 0;
		while((angle >= newAngle + 1 || angle <= newAngle - 1) && tries <= 5) {
			if(newAngle == -180 && angle == 180)
				break;
			if(newAngle == 180 && angle == -180)
				break;
			
			nav.rotateTo(newAngle);
			nav.waitForStop();
			//Delay.msDelay(2000);
			angle = getAngle();
			//System.out.println(angle);
			//System.out.println(nav.getPoseProvider().getPose());
			nav.getPoseProvider().setPose(new Pose(getPoseX(), getPoseY(), angle));
			tries++;
		}
	}
	
	public void remoteRotate(float angle) {
		pilot.rotate(angle);
	}
	
	public void preciseRelativeRotate(float rotateAngle) {
		float angle = getAngle();
		angle += rotateAngle;
		preciseRotateTo(angle);
	}
	
	public void move(float distance) {
		pilot.travel(distance);
	}
	
	public void waitForButtonPress() {
		Thread buttonThread = new Thread() {
			@Override
			public void run() {
				int button = Button.waitForAnyPress();
				if(button == Button.ID_ESCAPE) {
					close();
					System.exit(0);
				}else {
					run();
				}
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
		
		if(leftDist + rightDist + ROBOT_LENGTH <= BOARD_WIDTH) {
			//obstacle is blocking one of the walls move to next line and try again
			
			
		}
		float offset = (BOARD_WIDTH - (rightDist + leftDist)) / 2;
		int y_position = (int) (leftDist + offset);
		int x_position = 8; //distance from beginning to first line on either side of the board - wheel offset 
		x_position += (lineNum-1) * DIST_BETWEEN_LINES;
		
		nav.getPoseProvider().setPose(new Pose(x_position, y_position, -90.0f));		
		preciseRotateTo(0);
	}
	
	private void searchForBall(float startAngle, float endAngle) {
		
		ballDetector.enableDetection(true);
		Feature ball = null;
		
		for(float i = startAngle; i < endAngle; i+=3) {
			nav.rotateTo(i);
			//preciseRotateTo(i);
			ball = ballDetector.scan();
			float[] reading = new float[1];
			irSensor.fetchSample(reading, 0);
			if(ball != null) {
				nav.stop();
				nav.clearPath();
				foundBall = true;
				System.out.println("Found The Ball!");
				return;
				
			}
			
		}
	}
	
	private float getPoseX() {
		return nav.getPoseProvider().getPose().getX();
	}
	
	private float getPoseY() {
		return nav.getPoseProvider().getPose().getY();
	}
	
	private float getPoseHeading() {
		return nav.getPoseProvider().getPose().getHeading();
	}
	
	private void captureBall() {
		preciseRelativeRotate(10);
		nav.getMoveController().travel(4.0f);
		nav.waitForStop();
		closeClaw();
	}
	
	private void deliverBall() {
		
		preciseRotateTo(0);
		try {
			System.out.println("Pathing..");
			Path toGoal = nodePathFinder.findRoute(nav.getPoseProvider().getPose(), GOAL_ONE);
			toGoal = simplifyPath(toGoal);
			printPath(toGoal);
			System.out.println("Pathing complete.");
			
			Waypoint prev = new Waypoint(getPoseX(), getPoseY());
			for(int i = 0; i < toGoal.size(); i++) {
				Waypoint point = toGoal.get(i);
				float angle = prev.angleTo(point);
				preciseRotateTo(angle);
				nav.goTo(point);
				nav.waitForStop();
				prev = point;
				printPose();
			}
			
			nav.waitForStop();
		} catch (DestinationUnreachableException e) {
			e.printStackTrace();
		}
		preciseRotateTo(0);
		openClaw();
		nav.getMoveController().travel(-5.0);
		nav.waitForStop();
		for(int i = 0; i < 4; i++) {
			Sound.beep();
			Delay.msDelay(1000);
		}
		Sound.beepSequenceUp();
		goals++;
	}
	
	private void printPath(Path path) {
		for(Waypoint wp : path) {
			System.out.println("(" + wp.x + ", " + wp.y + ")");
		}
	}
	
	private void initialState() {
		System.out.println("Ready!");
		Sound.beepSequence();
		Button.waitForAnyPress();
		Sound.beepSequenceUp();
		Delay.msDelay(1000);
	}
	
	private void goToStartingPosition() {
		printPose();
		Path toGoal = null;
		try {
			toGoal = nodePathFinder.findRoute(nav.getPoseProvider().getPose(), new Waypoint(7, 30));
			toGoal = simplifyPath(toGoal);
			nav.setPath(toGoal);
			nav.followPath();
			nav.waitForStop();
		} catch (DestinationUnreachableException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		printPath(toGoal);
		preciseRotateTo(0);
		getBearings(1);
		foundBall = false;
	}
	
	public void offensiveState() {
		
		preciseRotateTo(90);
		nav.goTo(8.0f, 40.0f);
		nav.waitForStop();
		preciseRotateTo(0);
		obstacleDetector.enableDetection(true);
		// ball could potentially be over here before the robot
		// gets all the way to the middle
		
		while(!foundBall) {
			for(float x = 16.0f; x < BOARD_LENGTH/2; x += 2.0f) {
				try {
					Path nextSpot = nodePathFinder.findRoute(
							nav.getPoseProvider().getPose(), new Waypoint(x, 40.0f));
					nextSpot = simplifyPath(nextSpot);
					nav.setPath(nextSpot);
					nav.followPath();
					nav.waitForStop();
					printPose();
					searchForBall(-20.0f, 20.0f);
					if(foundBall)
						break;
					preciseRotateTo(0);
				}catch(DestinationUnreachableException e) {
					
				}
				
			}
			if(foundBall)
				break;
			preciseRotateTo(-90);
			for(float y = 40.0f; y > 10; y -= 2.0f) {
				try {
					Path nextSpot = nodePathFinder.findRoute(
							nav.getPoseProvider().getPose(), new Waypoint(BOARD_LENGTH/2, y));
					nextSpot = simplifyPath(nextSpot);
					nav.setPath(nextSpot);
					nav.followPath();
					nav.waitForStop();
					printPose();
					searchForBall(-135.0f, -45.0f);
					if(foundBall)
						break;
					preciseRotateTo(-90);
				}catch(DestinationUnreachableException e) {
					
				}
			}
			break;
		}
		if(foundBall) {
			captureBall();
			deliverBall();
			goToStartingPosition();
			
		}else {
			//Ball not found, give up
			gameOver = true;
		}
		
	}
	
	public void startGame() {
		getBearings(1);
		
		goals = 0;
		while(goals < 4 && !gameOver) {
			initialState();
			offensiveState();
		}
		
		Sound.twoBeeps();
		System.out.println("Game Over");

		close();
		System.exit(0);
		
	}
	
	private Path simplifyPath(Path toSimplify) {
		Path simple = new Path();
		
		boolean xChanging = false;
		boolean yChanging = false;
		
		Waypoint old = null;
		for(Waypoint wp : toSimplify) {
			
			if(old != null) {
				if(old.x == wp.x)
					xChanging = true;
				if(old.y == wp.y)
					yChanging = true;
				
				if(old.x != wp.x && xChanging) {
					simple.add(old);
					xChanging = false;
				}
				
				if(old.y != wp.y && yChanging) {
					simple.add(old);
					yChanging = false;
				}
			}
			old = wp;
		}
		simple.add(toSimplify.get(toSimplify.size()-1));
		return simple;
	}
	
	public void test(){
		

		getBearings(1);

		try {
			Waypoint boardMiddle = new Waypoint(BOARD_LENGTH/2, nav.getPoseProvider().getPose().getY(), 0);
			System.out.println("GOING TO MIDDLE");
			//nav.clearPath();
			
			nav.followPath(pathFinder.findRoute(nav.getPoseProvider().getPose(), boardMiddle, lineMap));
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
			System.out.println("DONE");*/
			
			while(nav.isMoving() || detectingObstacle) {
				Delay.msDelay(100);
			}
			System.out.println("READY!");
			System.out.println("GOING TO MIDDLE");
			//nav.clearPath();
			
			nav.followPath(pathFinder.findRoute(nav.getPoseProvider().getPose(), boardMiddle, lineMap));

		}catch(DestinationUnreachableException e) {
			
		}
		//close();
		//System.exit(0);
		
	}
	
	private void printPose() {
		System.out.println(nav.getPoseProvider().getPose().getX() + ", " + nav.getPoseProvider().getPose().getY() + ", " + nav.getPoseProvider().getPose().getHeading());
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
		if(claw != null)
			claw.close();
		if(motor1 != null)
			motor1.close();
		if(motor2 != null)
			motor2.close();
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