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

    public static final float MAX_DISTANCE = 0.27f;
    public static final int DETECTOR_DELAY = 100;
    public static final int BOARD_WIDTH = 48;
    public static final int BOARD_LENGTH = 56;
    public static final float ROBOT_WIDTH = 6.5f;
    public static final float ROBOT_LENGTH = 9.5f;
    
    
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
    private boolean movingHome = false;
	
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
		 //pilot.setAngularSpeed(45);
		 //pilot.setLinearSpeed(10);
		 gyroAdapter = new GyroscopeAdapter(gyroProvider, 1);
		 gyroFinder = new GyroDirectionFinder(gyroAdapter);
		 opp = new OdometryPoseProvider(pilot);
		 //opp = new CompassPoseProvider(pilot, gyroFinder);
		 
	     detector = new RangeFeatureDetector(new RangeFinderAdapter(ultraSensor.getDistanceMode()), MAX_DISTANCE, DETECTOR_DELAY);
		 
		 lineMap = readLineMap();
		 pathFinder = new ShortestPathFinder(lineMap);
		 
		 
		 nav = new Navigator(pilot, opp);
		 
        detector.addListener(new FeatureListener() {
            public void featureDetected(Feature feature, FeatureDetector detector) {
                if (pilot.isMoving() && pilot.getMovement().getMoveType() != MoveType.ROTATE) {
                	System.out.println("Obstacle Detected!");
                    if (nav.isMoving()) 
                    	nav.stop();
                    
                    /*try {
						Path homePath = pathFinder.findRoute(opp.getPose(), home, lineMap);
						nav.followPath(homePath);
					} catch (DestinationUnreachableException e) {
						System.out.println(e.getMessage());
						//e.printStackTrace();
					}*/
                    
                }                   
            }       
	     });
		 
		 /*EV3NavigationModel model = new EV3NavigationModel();
		 model.addPoseProvider(opp);
		 model.addPilot(pilot);
		 model.addNavigator(nav);
		 model.addFeatureDetector(detector);*/
		 detector.enableDetection(true);
		 detector.setPoseProvider(opp);
		 
		lineCheckThread = new LineCheckThread();
		lineCheckThread.setDirection(1);
		lineCheckThread.setColorSensor(colorSensor);
		lineCheckThread.start();
		
	}
	
	private void moveToFirstLine() {
		Path forwardPath = new Path();
		forwardPath.add(new Waypoint(10,0,0));
		nav.followPath(forwardPath);
		while(lineCheckThread.getCurrentLine() == LineCheckThread.NO_LINE) {
			Delay.msDelay(10);
		}
		nav.stop();
		nav.clearPath();
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
	
	private void getBearings() {
		
		moveToFirstLine();
		
		preciseRotateTo(90);
		float rightDist = getDistanceAverage(10);
		preciseRotateTo(-90);
		float leftDist = getDistanceAverage(10);
		
		System.out.println(rightDist);
		System.out.println(leftDist);
		
		if(leftDist + rightDist + ROBOT_LENGTH >= BOARD_WIDTH) {
			//no obstacles!
			
		}
		float offset = (BOARD_WIDTH - (rightDist + leftDist)) / 2;
		float y_position = leftDist + offset;
		float x_position = 8.5f; //distance from beginning to first line on either side of the board - wheel offset 
		
		opp.setPose(new Pose(x_position, y_position, -90));
		nav.setPoseProvider(opp);
		
		preciseRotateTo(0);
		
		System.out.println(nav.getPoseProvider().getPose());
	}
	
	public void test(){
		
		
		getBearings();

		try {
			Path newPath = new Path();
			Waypoint boardMiddle = new Waypoint(BOARD_WIDTH/2, BOARD_LENGTH/2, 0);
			newPath.add(boardMiddle);
			nav.followPath(pathFinder.findRoute(nav.getPoseProvider().getPose(), boardMiddle));
			nav.waitForStop();
			System.out.println(nav.getPoseProvider().getPose());
		}catch(DestinationUnreachableException e) {
			
		}

		
		close();
		System.exit(0);
		
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