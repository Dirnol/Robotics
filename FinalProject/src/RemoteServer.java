import java.io.BufferedInputStream;
import java.io.DataInputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;

public class RemoteServer {

	public static int ROBOT_PORT = 5000;
	
	private ServerSocket server;
	private Socket socket;
	private DataInputStream in;
	private String input = "";
	
	private MoveAndMap controller;
	
	
	public RemoteServer(MoveAndMap controller) {
		
		this.controller = controller;
		Thread serverThread = new Thread() {
			@Override
			public void run() {
				try {
					System.out.println("WAITING FOR\nCONTROLLER...");
					server = new ServerSocket(ROBOT_PORT);
					socket = server.accept();
					in = new DataInputStream(
							new BufferedInputStream(socket.getInputStream()));
					
					readInput();
					
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		};
		serverThread.start();

		
	}
	
	private void readInput() {
		
		while(!input.equals("QUIT")) {
			
			try {
				input = in.readUTF();

				if(input.contains("QUIT")){
					break;
				}
			
				if(input.contains("AUTO")){
					Thread autoThread = new Thread() {
						@Override
						public void run() {
							controller.startAuto();
						}
					};
					autoThread.start();
				}

				if(input.contains("TURN")){
					float angle = Float.parseFloat(input.split(" ")[1]);
					controller.remoteRotate(angle);
				}
				
				if(input.contains("FORWARD")){
					float distance = Float.parseFloat(input.split(" ")[1]);
					controller.move(distance);
				}
				
				if(input.contains("BACKWARD")){
					float distance = Float.parseFloat(input.split(" ")[1]);
					controller.move(-distance);
				}
				
				if(input.contains("OPEN")){
					controller.openClaw();
				}
				
				if(input.contains("CLOSE")){
					controller.closeClaw();
				}
				
				input = "";
			
			} catch (IOException e) {
				e.printStackTrace();
			}
			
		}
		
		controller.close();
		System.exit(0);
	}
	
}
