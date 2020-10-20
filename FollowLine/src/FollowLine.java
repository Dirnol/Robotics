import java.text.DecimalFormat;

import lejos.hardware.*;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.Color;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class FollowLine {

	EV3 ev3;
	TextLCD lcd;
	RegulatedMotor rightWheel;
	RegulatedMotor leftWheel;
	
	public static void main(String[] args) {
		
		new FollowLine();
		
	}
	
	public FollowLine() {
		
		ev3 = (EV3) BrickFinder.getLocal();
		lcd = ev3.getTextLCD();
		
		rightWheel = Motor.B;
		leftWheel = Motor.C;
		
		Sound.beepSequenceUp();
		Button.waitForAnyPress();
		
		followLine();
		
		Sound.beepSequence();
	}
	
	private void followLine() {
		EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
		
		SensorMode rgbMode = colorSensor.getRGBMode();
		float[] rgbSample = new float[rgbMode.sampleSize()];
		float red = 0.0f;
		float green = 0.0f;
		float blue = 0.0f;
		
		float lowerBounds = 0.03f; //observed lowest acceptable value
		float upperBounds = 0.09f; //observed highest acceptable value
		float mid = 0.0f;
		
		int leftSpeed = 0;
		int rightSpeed = 0;
		int maxSpeed = 700;
		int normSpeed = 600;
		leftWheel.setSpeed(400);
		rightWheel.setSpeed(400);
		
		leftWheel.forward();
		rightWheel.forward();
		
		DecimalFormat decFormat = new DecimalFormat("#.###");
		
		while(true) {
			rgbMode.fetchSample(rgbSample, 0);
			red = rgbSample[0];
			green = rgbSample[1];
			blue = rgbSample[2];
			
			lcd.clear();
			lcd.drawString("Green: " + decFormat.format(green), 1, 1);
			
			mid = (lowerBounds + upperBounds) / 2;
			
			if (green > 0.060f && red < 0.060f && blue < 0.060f) {
				break;
			}
			else if (green <= mid - 0.005f) { // allow for some level of error
				leftSpeed = (int) (normSpeed + (maxSpeed-normSpeed)*(mid-green)/(mid-lowerBounds));
				leftSpeed = Math.min(Math.max(leftSpeed, 0), maxSpeed);
				rightSpeed = (int) (0 + normSpeed*(green-lowerBounds)/(mid-lowerBounds));
				rightSpeed = Math.min(Math.max(rightSpeed, 0), maxSpeed);
				
				// Another function for motor speed
//				leftSpeed = maxSpeed;
//				rightSpeed = (int) (maxSpeed*(green-lowerBounds)/(mid-lowerBounds));
//				rightSpeed = Math.min(Math.max(rightSpeed, 0), maxSpeed);
			}
			else if (mid + 0.005f <= green) {
				leftSpeed = (int) (0 + normSpeed*(upperBounds-green)/(upperBounds-mid));
				leftSpeed = Math.min(Math.max(leftSpeed, 0), maxSpeed);
				rightSpeed = (int) (normSpeed + (maxSpeed-normSpeed)*(green-mid)/(upperBounds-mid));
				rightSpeed = Math.min(Math.max(rightSpeed, 0), maxSpeed);
				
				// Another function for motor speed
//				rightSpeed = maxSpeed;
//				leftSpeed = (int) (maxSpeed*(upperBounds-green)/(upperBounds-mid));
//				leftSpeed = Math.min(Math.max(leftSpeed, 0), maxSpeed);
			}
			else {
				leftSpeed = normSpeed;
				rightSpeed = normSpeed;
				
				// Another function for motor speed
//				leftSpeed = maxSpeed;
//				rightSpeed = maxSpeed;
			}
			
			// Update speeds
			leftWheel.setSpeed(leftSpeed);
			rightWheel.setSpeed(rightSpeed);
			
			// Sometimes after setSpeed(0) motors would stop completely
			leftWheel.forward();
			rightWheel.forward();
			
			// Output data for debugging
			lcd.drawString(String.valueOf(leftWheel.isStalled()), 1, 2);
			lcd.drawString(String.valueOf(rightWheel.isStalled()), 1, 3);
			lcd.drawString(String.valueOf(leftSpeed), 1, 4);
			lcd.drawString(String.valueOf(rightSpeed), 1, 5);
		}
		
		rightWheel.stop();
		leftWheel.stop();
		
		colorSensor.close();
		rightWheel.stop();
		leftWheel.stop();
	}
	
	private void moveAndMeasure() {

		
		EV3IRSensor irSensor = new EV3IRSensor(SensorPort.S4);
		SampleProvider sampleProvider = irSensor.getDistanceMode();
		
		int sampleRate = 100;
		int travelTime = 10000;
		int iterations = travelTime / sampleRate;
		
		//rightWheel.setPower(50);
		//leftWheel.setPower(50);
		
		for(int i = 0; i < iterations; i++) {
			float[] sample = new float[sampleProvider.sampleSize()];
			sampleProvider.fetchSample(sample, 0);
			
			float distance = sample[0];
			System.out.println(i + ": " + distance);
			
			Delay.msDelay(sampleRate);
		}
		
		rightWheel.stop();
		leftWheel.stop();
		
		rightWheel.close();
		leftWheel.close();
		irSensor.close();
	}
}
