/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.CargoIntake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.CargoScoring;

public class AutonExtend extends Command {
  CargoIntake intake;
  long duration;
  long startTime;
  public AutonExtend(long duration) {
    intake = CargoIntake.getInstance();
    requires(intake);
    this.duration = duration;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    intake.setExtended(true);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return System.currentTimeMillis() - startTime > duration;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    intake.setExtended(false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
