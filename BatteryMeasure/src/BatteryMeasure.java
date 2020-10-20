import java.util.ArrayList;

import lejos.hardware.Battery;
import lejos.hardware.BrickFinder;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class BatteryMeasure {
	
	RegulatedMotor rightWheel;
	RegulatedMotor leftWheel;
	
	ArrayList<Double> motorSpeeds;
	
	EV3 ev3;
	TextLCD lcd;

	public static void main(String[] args) {
		new BatteryMeasure();
	}
	
	public BatteryMeasure() {
		
		ev3 = (EV3) BrickFinder.getLocal();
		lcd = ev3.getTextLCD();
		rightWheel = Motor.B;
		leftWheel = Motor.C;
		motorSpeeds = new ArrayList<>();
		
		float batteryCurrent = Battery.getBatteryCurrent();
		float motorCurrent = Battery.getMotorCurrent();
		float voltage = Battery.getVoltage();
		
		lcd.drawString("Battery: " + batteryCurrent, 1, 1);
		lcd.drawString("Motor: " + motorCurrent, 1, 2);
		lcd.drawString("Voltage: " + voltage, 1, 3);
		
		int travelTime = 5000;
		int sampleRate = 100;
		int iterations = travelTime / sampleRate;
		
		for(int i = 0; i < iterations; i++) {
			
			int rightSpeed = rightWheel.getRotationSpeed();
			int leftSpeed = leftWheel.getRotationSpeed();
			double average = (rightSpeed + leftSpeed) / 2.0;
			motorSpeeds.add(average);
			lcd.clear(4);
			lcd.drawString("Speed: " + average, 1, 4);
			Delay.msDelay(sampleRate);
		}
		
		double total = 0.0;
		for(double speed : motorSpeeds) {
			total += speed;
		}
		total /= motorSpeeds.size();
		
		lcd.drawString("Average Motor Speed: " + total, 1, 5);
		
		rightWheel.setSpeed(500);
		leftWheel.setSpeed(500);
		
		Delay.msDelay(travelTime);
		
		rightWheel.stop();
		leftWheel.stop();
		
		rightWheel.close();
		leftWheel.close();
	}
	
}
