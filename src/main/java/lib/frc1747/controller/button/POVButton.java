package lib.frc1747.controller.button;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class POVButton extends JoystickButton {
	
	Joystick joystick;
	int buttonAngle;
	
	public POVButton(Joystick joystick, int buttonAngle) {
		super(joystick, buttonAngle);
		
		this.joystick = joystick;
		this.buttonAngle = buttonAngle;
	}

	@Override
	public boolean get(){
		return joystick.getPOV() == buttonAngle;
	}
}
