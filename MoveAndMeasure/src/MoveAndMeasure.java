import lejos.hardware.*;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class MoveAndMeasure {

	public static void main(String[] args) {
		
		Sound.beepSequenceUp();
		Button.waitForAnyPress();
		
		UnregulatedMotor rightWheel = new UnregulatedMotor(MotorPort.B);
		UnregulatedMotor leftWheel = new UnregulatedMotor(MotorPort.C);
		
		EV3IRSensor irSensor = new EV3IRSensor(SensorPort.S4);
		SampleProvider sampleProvider = irSensor.getDistanceMode();
		
		int sampleRate = 100;
		int travelTime = 10000;
		int iterations = travelTime / sampleRate;
		
		/*rightWheel.setPower(50);
		leftWheel.setPower(50);
		
		for(int i = 0; i < iterations; i++) {
			float[] sample = new float[sampleProvider.sampleSize()];
			sampleProvider.fetchSample(sample, 0);
			
			float distance = sample[0];
			System.out.println(i + ": " + distance);
			
			Delay.msDelay(sampleRate);
		}
		*/
		rightWheel.stop();
		leftWheel.stop();
		
		rightWheel.close();
		leftWheel.close();
		irSensor.close();
		
		Sound.beepSequence();
		
	}
}
