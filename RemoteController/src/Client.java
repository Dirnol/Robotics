import java.io.DataOutputStream;
import java.io.IOException;
import java.net.Socket;

public class Client {

	public static String ROBOT_HOST = "10.0.1.1";
	public static int ROBOT_PORT = 5000;
	
	private DataOutputStream out;
	private Socket socket;
	
	public Client() {

		Thread connectThread = new Thread() {
			@Override
			public void run() {
				try {
					socket = new Socket(ROBOT_HOST, ROBOT_PORT);
					out = new DataOutputStream(socket.getOutputStream());
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		};
		connectThread.start();
			
	}
	
	public void sendCommand(String command) {
		try {
			out.writeUTF(command);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	public void close() {
		try {
			out.close();
			socket.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
}
