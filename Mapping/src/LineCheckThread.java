import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.utility.Delay;

public class LineCheckThread extends Thread{

	public static int NO_LINE = 10;
	
	int currentLine = NO_LINE; // 0 is the black line, +1 for each blue line, -1 for each green line
	int team = 0; //1 for blue, -1 for green
	boolean lineDetected = false;
	
	boolean newLineDetected = false;
	boolean onSameLine = false;
	
	private int direction = 1; //1 is going toward blue goal, //-1 going toward green goal
	
	private float colorThreshold = 0.060f;
	
	private boolean running = true;
	
	private EV3ColorSensor colorSensor;
	
	public int getCurrentLine() {
		return currentLine;
	}
	
	public void setDirection(int direction){
		this.direction = direction;
	}
	
	public void setColorSensor(EV3ColorSensor colorSensor) {
		this.colorSensor = colorSensor;
	}
	
	public void resetNewLine() {
		newLineDetected = false;
	}
	
	public boolean newLineFound() {
		return newLineDetected;
	}
	
	public boolean onSameLine() {
		return !newLineDetected;
	}
	
	public void run() {		
		SensorMode rgbMode = colorSensor.getRGBMode();
		float[] rgbSample = new float[rgbMode.sampleSize()];
		float red = 0.0f;
		float green = 0.0f;
		float blue = 0.0f;
		
		while(running) {
			rgbMode.fetchSample(rgbSample, 0);
			red = rgbSample[0];
			green = rgbSample[1];
			blue = rgbSample[2];
			
			if(green >= colorThreshold && red < colorThreshold && blue < colorThreshold && !newLineDetected) {
				newLineDetected = true;
				onSameLine = true;
				//System.out.println("Green detected!");
				//Green line detected!
				if(team == 0) { //First green line detected, update team and start at line -3
					team = -1;
					currentLine = -3;
				}else {
					if(direction == 1) {
						currentLine++;
					}else {
						currentLine--;
					}
				}
			}
			else if(blue >= colorThreshold && red < colorThreshold && green < colorThreshold && !newLineDetected) {
				newLineDetected = true;
				onSameLine = true;
				//System.out.println("Blue Detected!");
				//Blue line detected!
				if(team == 0) { //First blue line detected, update team and start at line 3
					team = 1;
					currentLine = 3;
				}else {
					if(direction == 1) {
						currentLine++;
					}else {
						currentLine--;
					}
				}
			}else if(blue < colorThreshold && red < colorThreshold && green < colorThreshold && !newLineDetected){
				newLineDetected = true;
				onSameLine = true;
			}
			else {
				newLineDetected = false;
				onSameLine = false;
				//System.out.println("Black Detected!");
				//Black line detected?
			}
			Delay.msDelay(10);
		}
	}
	
}
