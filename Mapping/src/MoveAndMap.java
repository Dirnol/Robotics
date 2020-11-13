import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;

import javax.xml.stream.XMLStreamException;

import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.robotics.GyroscopeAdapter;
import lejos.robotics.RangeFinderAdapter;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.localization.CompassPoseProvider;
import lejos.robotics.localization.OdometryPoseProvider;
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
import lejos.utility.GyroDirectionFinder;

public class MoveAndMap {

    public static final float MAX_DISTANCE = 50f;
    public static final int DETECTOR_DELAY = 1000;
	
	public static void main(String[] args) {
		
		new MoveAndMap();

	}
	
	public MoveAndMap() {
		Wheel wheel1 = WheeledChassis.modelWheel(Motor.B, 3.0).offset(2.6).gearRatio(2.5).invert(true);
		Wheel wheel2 = WheeledChassis.modelWheel(Motor.C, 3.0).offset(-2.6).gearRatio(2.5).invert(true);
		EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S3);
		SampleProvider gyroProvider = gyroSensor.getAngleAndRateMode();
		Chassis chassis = new WheeledChassis(new Wheel[] { wheel1, wheel2 }, WheeledChassis.TYPE_DIFFERENTIAL);
		
		 final MovePilot pilot = new MovePilot(chassis);	
		 GyroscopeAdapter gyroAdapter = new GyroscopeAdapter(gyroProvider, 100);
		 GyroDirectionFinder gyroFinder = new GyroDirectionFinder(gyroAdapter);
		 CompassPoseProvider opp = new CompassPoseProvider(pilot, gyroFinder);
		 
		 EV3IRSensor ir = new EV3IRSensor(SensorPort.S1);
	     RangeFeatureDetector detector = new RangeFeatureDetector(new RangeFinderAdapter(ir.getDistanceMode()), MAX_DISTANCE, DETECTOR_DELAY);
		 
		 LineMap lineMap = readLineMap();
		 ShortestPathFinder pathFinder = new ShortestPathFinder(lineMap);
		 
		 
		 final Navigator nav = new Navigator(pilot, opp);
		 
        detector.addListener(new FeatureListener() {
            public void featureDetected(Feature feature, FeatureDetector detector) {
                if (pilot.isMoving() && pilot.getMovement().getMoveType() != MoveType.ROTATE) {
                	pilot.stop();
                    if (nav.isMoving()) nav.stop();
                }                   
            }       
	     });
		 
		 EV3NavigationModel model = new EV3NavigationModel();
		 model.addPoseProvider(opp);
		 model.addPilot(pilot);
		 model.addNavigator(nav);
		 model.addFeatureDetector(detector);
		 detector.enableDetection(true);
		 detector.setPoseProvider(opp);
		 
		 
		 Waypoint goal = new Waypoint(new Pose(10,10,180.0f));
		 
		 try {
			Path path = pathFinder.findRoute(opp.getPose(), goal);
			nav.followPath(path);
		} catch (DestinationUnreachableException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		
		
//		 System.out.println("Pose: " + opp.getPose());
//		 pilot.setLinearSpeed(3.0);  // in per second
//		 pilot.setAngularSpeed(120.0);
//		 pilot.rotate(-180);
//		 System.out.println("Pose: " + opp.getPose());
//		 pilot.travel(10,true);  //  move backward for 5 in
//		 System.out.println("Pose: " + opp.getPose());
//		 while(pilot.isMoving())Thread.yield();
//		 pilot.rotate(180);
//		 System.out.println("Pose: " + opp.getPose());
//		 pilot.stop();
		 
		 gyroSensor.close();
	}
	
	public void rectifyPoseHeadingWithGyro(OdometryPoseProvider opp, EV3GyroSensor gyro) {
		float gyroReading = getAngle(gyro);
		float poseHeading = opp.getPose().getHeading();
		
		float offset = gyroReading - poseHeading;
		opp.setPose(new Pose(opp.getPose().getX(), opp.getPose().getY(), opp.getPose().getHeading() + offset));
		
	}
	
	private LineMap readLineMap() {
		FileInputStream fStream;
		try {
			fStream = new FileInputStream(new File("map.svg"));
			SVGMapLoader mapLoader = new SVGMapLoader(fStream);
			LineMap map = mapLoader.readLineMap();
			return map;
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (XMLStreamException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;

	}
	
	public float getAngle(EV3GyroSensor gyro) {
		SampleProvider sampleProvider = gyro.getAngleMode();
		float[] reading = new float[sampleProvider.sampleSize()];
		sampleProvider.fetchSample(reading, 0);
		return reading[0];
	}
	
	public void printAngle(SampleProvider p) {
		float[] sample = new float[p.sampleSize()];
		p.fetchSample(sample, 0);
		System.out.println("Angle: " + sample[0]);
	}
	
}