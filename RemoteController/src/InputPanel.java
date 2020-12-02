import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;

public class InputPanel extends JPanel{

	private static final long serialVersionUID = 1L;
	private JTextField angleTextField;
	private JTextField forwardTextField;
	private JTextField backwardTextField;
	private JButton openClawButton;
	private JButton closeClawButton;
	private JButton autoModeButton;
	private JButton quitButton;
	private JButton turnButton;
	private JButton forwardButton;
	private JButton backwardButton;
	private JLabel turnLabel;
	private JLabel degreesLabel;
	private JLabel forwardLabel;
	private JLabel fInchesLabel;
	private JLabel backwardLabel;
	private JLabel bInchesLabel;
	
	private GridBagConstraints c;
	
	private Client client;
	
	
	public InputPanel(Client client) {
		
		this.client = client;
		
		turnLabel = new JLabel("Turn Robot (Positive Angle Turns Clockwise)");
		angleTextField = new JTextField(5);
		degreesLabel = new JLabel("°");
		turnButton = new JButton("Turn");
		turnButton.addActionListener(new TurnButtonListener());
		
		forwardLabel = new JLabel("Move Forward");
		forwardTextField = new JTextField(5);
		fInchesLabel = new JLabel("inches");
		forwardButton = new JButton("Forward");
		forwardButton.addActionListener(new ForwardButtonListener());
		
		backwardLabel = new JLabel("Move Backward");
		backwardTextField = new JTextField(5);
		bInchesLabel = new JLabel("inches");
		backwardButton = new JButton("Backward");
		backwardButton.addActionListener(new BackwardButtonListener());
		
		openClawButton = new JButton("Open Claw");
		openClawButton.addActionListener(new OpenClawButtonListener());
		
		closeClawButton = new JButton("Close Claw");
		closeClawButton.addActionListener(new CloseClawButtonListener());
		
		autoModeButton = new JButton("Autonomous Mode");
		autoModeButton.addActionListener(new AutoModeButtonListener());
		
		quitButton = new JButton("Quit");
		quitButton.addActionListener(new QuitButtonListener());
		
		this.setLayout(new GridBagLayout());
		c = new GridBagConstraints();
		
		organizeLayout();
	}
	
	private void organizeLayout() {
		
		setConstraints(GridBagConstraints.HORIZONTAL, 0, 0, 3, 1, 10, 10, 10, 10);
		this.add(autoModeButton, c);
		
		setConstraints(GridBagConstraints.HORIZONTAL, 0, 1, 3, 1, 10, 10, 0, 10);
		this.add(turnLabel, c);
		setConstraints(GridBagConstraints.HORIZONTAL, 0, 2, 1, 1, 0, 10, 0, 0);
		this.add(angleTextField, c);
		setConstraints(GridBagConstraints.HORIZONTAL, 1, 2, 1, 1, 0, 3, 0, 0);
		this.add(degreesLabel, c);
		setConstraints(GridBagConstraints.HORIZONTAL, 2, 2, 1, 1, 0, 10, 0, 10);
		this.add(turnButton, c);
		
		setConstraints(GridBagConstraints.HORIZONTAL, 0, 3, 1, 1, 10, 10, 0, 0);
		this.add(new JLabel(""), c);
		
		setConstraints(GridBagConstraints.HORIZONTAL, 0, 4, 1, 1, 10, 10, 0, 0);
		this.add(forwardLabel, c);
		setConstraints(GridBagConstraints.HORIZONTAL, 0, 5, 1, 1, 0, 10, 0, 0);
		this.add(forwardTextField, c);
		setConstraints(GridBagConstraints.HORIZONTAL, 1, 5, 1, 1, 0, 3, 0, 0);
		this.add(fInchesLabel, c);
		setConstraints(GridBagConstraints.HORIZONTAL, 2, 5, 1, 1, 0, 10, 0, 10);
		this.add(forwardButton, c);
		
		setConstraints(GridBagConstraints.HORIZONTAL, 0, 6, 1, 1, 10, 10, 0, 0);
		this.add(new JLabel(""), c);
		
		setConstraints(GridBagConstraints.HORIZONTAL, 0, 7, 1, 1, 10, 10, 0, 0);
		this.add(backwardLabel, c);
		setConstraints(GridBagConstraints.HORIZONTAL, 0, 8, 1, 1, 0, 10, 0, 0);
		this.add(backwardTextField, c);
		setConstraints(GridBagConstraints.HORIZONTAL, 1, 8, 1, 1, 0, 3, 0, 0);
		this.add(bInchesLabel, c);
		setConstraints(GridBagConstraints.HORIZONTAL, 2, 8, 1, 1, 0, 10, 0, 10);
		this.add(backwardButton, c);
		
		setConstraints(GridBagConstraints.HORIZONTAL, 0, 9, 1, 1, 20, 10, 0, 0);
		this.add(new JLabel(""), c);
		
		setConstraints(GridBagConstraints.HORIZONTAL, 0, 10, 3, 1, 10, 10, 0, 10);
		this.add(openClawButton, c);
		
		setConstraints(GridBagConstraints.HORIZONTAL, 0, 11, 3, 1, 10, 10, 10, 10);
		this.add(closeClawButton, c);
		

		
		setConstraints(GridBagConstraints.HORIZONTAL, 0, 12, 3, 1, 10, 10, 10, 10);
		this.add(quitButton, c);
		
	}
	
	private void setConstraints(int fill, int x, int y, int width,
			int height, int t, int l, int b, int r) {
		c.fill = fill;
		c.gridx = x;
		c.gridy = y;
		c.gridwidth = width;
		c.gridheight = height;
		c.insets = new Insets(t,l,b,r);
	}
	
	
	class TurnButtonListener implements ActionListener {
		@Override
		public void actionPerformed(ActionEvent e) {
			try {
				double angle = Double.parseDouble(angleTextField.getText());
				if(angle > 360 || angle < -360)
					return;
				String command = "TURN " + angle;
				client.sendCommand(command);
				System.out.println(command);
			}catch(NumberFormatException ex) {
				return;
			}
		}
	}
	
	class ForwardButtonListener implements ActionListener {
		@Override
		public void actionPerformed(ActionEvent e) {
			try {
				int distance = Integer.parseInt(forwardTextField.getText());
				if(distance > 60)
					return;
				String command = "FORWARD " + distance;
				client.sendCommand(command);
				System.out.println(command);
			}catch(NumberFormatException ex) {
				return;
			}
		}
	}
	
	class BackwardButtonListener implements ActionListener {
		@Override
		public void actionPerformed(ActionEvent e) {
			try {
				int distance = Integer.parseInt(backwardTextField.getText());
				if(distance < -60)
					return;
				String command = "BACKWARD " + distance;
				client.sendCommand(command);
				System.out.println(command);
			}catch(NumberFormatException ex) {
				return;
			}
		}
	}
	
	class OpenClawButtonListener implements ActionListener {
		@Override
		public void actionPerformed(ActionEvent e) {
			String command = "OPEN";
			client.sendCommand(command);
			System.out.println(command);
		}
	}
	
	class AutoModeButtonListener implements ActionListener {
		@Override
		public void actionPerformed(ActionEvent e) {
			String command = "AUTO";
			client.sendCommand(command);
			System.out.println(command);
		}
	}
	
	class CloseClawButtonListener implements ActionListener {
		@Override
		public void actionPerformed(ActionEvent e) {
			String command = "CLOSE";
			client.sendCommand(command);
			System.out.println(command);
		}
	}
	
	class QuitButtonListener implements ActionListener {
		@Override
		public void actionPerformed(ActionEvent e) {
			String command = "QUIT";
			client.sendCommand(command);
			System.out.println(command);
			System.exit(0);
		}
	}
}

