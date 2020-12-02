import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Collection;

import javax.xml.stream.XMLStreamException;

import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.robotics.GyroscopeAdapter;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.mapping.LineMap;
import lejos.robotics.mapping.SVGMapLoader;
import lejos.robotics.navigation.DestinationUnreachableException;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.navigation.Navigator;
import lejos.robotics.navigation.Pose;
import lejos.robotics.navigation.Waypoint;
import lejos.robotics.pathfinding.AstarSearchAlgorithm;
import lejos.robotics.pathfinding.FourWayGridMesh;
import lejos.robotics.pathfinding.Node;
import lejos.robotics.pathfinding.NodePathFinder;
import lejos.robotics.pathfinding.Path;
import lejos.utility.Delay;
import lejos.utility.GyroDirectionFinder;

public class Test {
	
	private Wheel wheel1, wheel2;
	private Chassis chassis;
	private MovePilot pilot = null;
	private OdometryPoseProvider opp;
	private Navigator nav;
	private EV3IRSensor irSensor;
	private EV3MediumRegulatedMotor motor;
	
	public static void main(String[] args) {
		new Test();
	}
	
	public Test(){
		testClaw();
	}
	
	private void testAngle() {
		Waypoint p1 = new Waypoint(10,10);
		Waypoint p2 = new Waypoint(0, 10);
		double angle = p1.angleTo(p2);
		System.out.println("Angle: " + angle);
	}
	
	private void testClaw() {
		motor = new EV3MediumRegulatedMotor(MotorPort.D);
		
		//open
		motor.forward();
		Delay.msDelay(2100);
		motor.stop();
		
		//close
		//motor.backward();
		//Delay.msDelay(2300);
		//motor.stop();
		
		
		/*System.out.println("ROTATING!");
		motor.backward();
		Delay.msDelay(1000);
		motor.stop();
		System.out.println("WAITING!");
		Button.waitForAnyPress();
		System.out.println("UNROTATING!");
		motor.forward();
		Delay.msDelay(1000);
		motor.stop();
		System.out.println("WAITING!");
		Button.waitForAnyPress();*/
		
		motor.close();
	}
	
	public void testPath() {
		Path testPath = new Path();
		int sx = 8;
		int sy = 10;
		for(;sx < 15; sx++) {
			testPath.add(new Waypoint(sx, sy));
		}
		for(;sy < 20; sy++) {
			testPath.add(new Waypoint(sx, sy));
		}
		String path = "";
		for(Waypoint wp : testPath) {
			path += "(" + wp.x + ", " + wp.y + ", " + wp.getHeading() + ")\n";
		}
		System.out.println(path);
		
		testPath = simplifyPath(testPath);
		path = "";
		for(Waypoint wp : testPath) {
			path += "(" + wp.x + ", " + wp.y + ", " + wp.getHeading() + ")\n";
		}
		System.out.println(path);
	}
	
	private Path simplifyPath(Path toSimplify) {
		Path simple = new Path();
		
		boolean xChanging = false;
		boolean yChanging = false;
		
		Waypoint old = null;
		for(Waypoint wp : toSimplify) {
			
			if(old != null) {
				if(old.x == wp.x) {
					xChanging = true;
				}
				
				if(old.y == wp.y) {
					yChanging = true;
				}
				
				if(old.x != wp.x && xChanging) {
					simple.add(wp);
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

	
	public void testSensor() {
		EV3LargeRegulatedMotor motorB = null;
		EV3LargeRegulatedMotor motorC = null;
		
		try {
		motorB = new EV3LargeRegulatedMotor(MotorPort.B);
		motorC = new EV3LargeRegulatedMotor(MotorPort.C);
		
		wheel1 = WheeledChassis.modelWheel(motorB, 3.4).offset(2.5).gearRatio(2.5).invert(true);
		wheel2 = WheeledChassis.modelWheel(motorC, 3.4).offset(-2.5).gearRatio(2.5).invert(true);
		
		irSensor = new EV3IRSensor(SensorPort.S1);
		
		boolean scan = true;
		
		float[] data = new float[1];
		while(scan) {
			
			irSensor.getDistanceMode().fetchSample(data, 0);
			System.out.println(data[0]);
			Delay.msDelay(100);
		}
		
		chassis = new WheeledChassis(new Wheel[] { wheel1, wheel2 }, WheeledChassis.TYPE_DIFFERENTIAL);
		
		 pilot = new MovePilot(chassis);
		 pilot.setAngularSpeed(60);
		 pilot.setLinearSpeed(4.0);
		 opp = new OdometryPoseProvider(pilot);
		 
		 nav = new Navigator(pilot, opp);
		 
		 System.out.println(nav.getPoseProvider().getPose());
		 nav.goTo(10.0f, 0.0f);
		 nav.waitForStop();
		 
		 
		 motorB.close();
		 motorC.close();
		}catch(Exception e) {
			e.printStackTrace();
			motorB.close();
			motorC.close();
		}
	}
	
	
}
