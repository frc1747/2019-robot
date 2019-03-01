/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.HatchPanelIntake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Roller;

public class AutonHPIntake extends Command {
  Roller hpintake;
  long duration;
  long startTime;

  public AutonHPIntake(long duration) {
    this.duration = duration;
    hpintake = Roller.getInstance();
    requires(hpintake);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    hpintake.setRollerPower(0.5);
    startTime = System.currentTimeMillis();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return System.currentTimeMillis() - startTime >= duration;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    hpintake.setRollerPower(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
