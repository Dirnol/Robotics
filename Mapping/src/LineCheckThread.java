import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;

public class LineCheckThread extends Thread{

	int currentLine = 10; // 0 is the black line, +1 for each blue line, -1 for each green line
	int team = 0; //1 for blue, -1 for green
	boolean lineDetected = false;
	
	private int direction = 1; //1 is going toward blue goal, //-1 going toward green goal
	
	private float colorThreshold = 0.060f;
	
	private boolean running = true;
	
	public int getCurrentLine() {
		return currentLine;
	}
	
	public void setDirection(int direction){
		
	}
	public void run() {
		EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
		
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
			
			if(green >= colorThreshold && red < colorThreshold && blue < colorThreshold && !lineDetected) {
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
			else if(blue >= colorThreshold && red < colorThreshold && green < colorThreshold && !lineDetected) {
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
			}else {
				//Black line detected?
			}
		}
	}
	
}
