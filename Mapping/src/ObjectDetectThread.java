import lejos.utility.Delay;

public class ObjectDetectThread extends Thread{

	MoveAndMap program;
	float maxDistance;
	long delay;
	
	boolean detected = false;
	
	public ObjectDetectThread(MoveAndMap program, float maxDistance, long delay) {
		this.program = program;
		this.maxDistance = maxDistance;
		this.delay = delay;
	}
	
	@Override
	public void run() {
		
		float dist = program.getUltraDistance();
		if(dist < maxDistance) {
			detected = true;
		}else {
			detected = false;
		}
		Delay.msDelay(delay);
		
	}
	
	public boolean isDetected() {
		return detected;
	}
	
}
