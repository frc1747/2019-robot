/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.CargoScoring;

import java.time.Duration;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.CargoScoring;

public class AutonOutake extends Command {
  CargoScoring scoring;
  long duration;
  long startTime;
  double power;

  public AutonOutake(long duration, double power) {
    scoring = CargoScoring.getInstance();
    requires(scoring);
    this.duration = duration;
    this.power = power;
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
    scoring.setPower(power);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (System.currentTimeMillis() - startTime > duration);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    scoring.setPower(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
