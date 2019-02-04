package lib.frc1747.controller;

import edu.wpi.first.wpilibj.Joystick;
import lib.frc1747.controller.button.JoystickButton;
import lib.frc1747.controller.button.POVButton;

public abstract class Controller {
	
	private Joystick stick;

	JoystickButton xButton, aButton, bButton, yButton, 
	leftBumper, rightBumper, leftTrigger, rightTrigger, start, back;
	
	JoystickButton [] buttons = new JoystickButton [] { null, 
			xButton, aButton, bButton, yButton, 
			leftBumper, rightBumper, leftTrigger, rightTrigger, start, back
	};
	
	public static final int LB = 5, RB = 6;
	
	public static final int UP = 0, RIGHT = 2, DOWN = 4, LEFT = 6,
			UP_RIGHT = 1, DOWN_RIGHT = 3, DOWN_LEFT = 5, UP_LEFT = 7;
	
	/*public enum dpad {
		UP, UP_RIGHT, RIGHT, DOWN_RIGHT, DOWN, DOWN_LEFT, LEFT, UP_LEFT
	}*/
	POVButton up, upRight, right, downRight, down, downLeft, left, upLeft;
	POVButton[] dpad = new POVButton[] {up, upRight, right, downRight, 
													down, downLeft, left, upLeft};
	
	public static final int LEFT_HORIZONTAL = 0, /*RIGHT_HORIZONTAL = 2,*/ 
		LEFT_VERTICAL = 1; /*RIGHT_VERTICAL = 3;*/
	
	public static final double DEADZONE = 0.075;
	
	public Controller(int port) {
	
		stick = new Joystick(port);
		
		buttons[LB] = new JoystickButton(getStick(), LB);
		buttons[RB] = new JoystickButton(getStick(), RB);
						
		for(int i = 0; i < 8; i++) {
			dpad[i] = new POVButton(getStick(), i * 45);
		}
	}
		
	public JoystickButton getButton(int buttonName) {
		return buttons[buttonName];
	}
	
	public POVButton getDPad(int dpadButton) {
		return dpad[dpadButton];
	}

	public Joystick getStick() {
		return stick;
	}
	
	public boolean getDPADButton(int Angle){
		return /*getStick().getPOV(0) == Angle;*/ getStick().getPOV() == Angle;
	}
}
