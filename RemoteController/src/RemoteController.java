import javax.swing.JFrame;

public class RemoteController extends JFrame{

	private static final long serialVersionUID = 1L;
	private InputPanel inputPanel;
	private Client client;
	
	public static void main(String[] args) {
		new RemoteController();
	}
	
	public RemoteController() {
		this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		this.setTitle("Robot Controller");
		
		client = new Client();
		inputPanel = new InputPanel(client);
		this.setContentPane(inputPanel);
		this.setVisible(true);
		this.pack();
		this.setLocationRelativeTo(null);
	}
	
}
