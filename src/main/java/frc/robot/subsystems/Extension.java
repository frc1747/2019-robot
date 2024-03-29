/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Extension extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  Solenoid solenoid;
  static Extension extend;

  public Extension() {
    solenoid = new Solenoid(RobotMap.EXTENSION_SOLENOID);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void setExtended(Boolean bool) {
    solenoid.set(bool);
  }

  public Boolean getPosition() {
    return solenoid.get();
  }

  public static Extension getInstance(){
    if(extend == null){
      extend = new Extension();
    }
    return extend;
  }
}
