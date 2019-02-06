/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.HatchPanelScoring;

import edu.wpi.first.wpilibj.command.Command;
// import frc.robot.subsystems.HatchPanelIntake;
import frc.robot.subsystems.HatchPanelScoring;

public class ActuateHPScoring extends Command {
  HatchPanelScoring hatch;
  long duration;
  long startTime;
  boolean state;
  public ActuateHPScoring(long duration, boolean state) {
    this.duration = duration;
    this.state = state;
    hatch = HatchPanelScoring.getInstance();
    requires(hatch);
    startTime = System.currentTimeMillis();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    hatch.setExtended(state);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (startTime - System.currentTimeMillis() >= duration);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    hatch.setExtended(false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
