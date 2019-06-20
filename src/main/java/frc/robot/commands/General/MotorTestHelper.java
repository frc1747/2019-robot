/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.General;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Drivetrain;

public class MotorTestHelper extends Command {
  Drivetrain drivetrain;
  int pair;
  double powerL;
  double powerR;
  long duration;
  long startTime;

  public MotorTestHelper(int pair, double powerL, double powerR,  long duration) {
    drivetrain = Drivetrain.getInstance();
    requires(drivetrain);
    this.pair = pair;
    this.powerL = powerL;
    this.powerR = powerR;
    this.duration = duration;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startTime = System.currentTimeMillis();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    drivetrain.setLeftPower(0.5);

    // drivetrain.getLeftSide().PowerSetMotorGroup(pair, powerL);
    // drivetrain.getRightSide().PowerSetMotorGroup(pair, powerR);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return System.currentTimeMillis() - startTime >= duration;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    drivetrain.getLeftSide().PowerSetMotorGroup(pair, 0);
    drivetrain.getRightSide().PowerSetMotorGroup(pair, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
