/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.PIDDrive;

public class DriveDistance extends Command {
  

  int PIDPointValue;
  Drivetrain drive;

  public DriveDistance(int PIDPointValue) {
    drive =  Drivetrain.getInstance();
    this.PIDPointValue = PIDPointValue;
    requires(drive);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    drive.getRightSide().resetEncoder();
    drive.getLeftSide().resetEncoder();
    // drive.enable();
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putNumber("Distance", (drive.getLeftDistance() + drive.getRightDistance()) / 2);
    SmartDashboard.putNumber("Left Voltage", drive.getLeftSide().readValue());
    SmartDashboard.putNumber("Right Voltage", drive.getRightSide().readValue());
    // drive.setSetpoint(PIDPointValue);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    // drive.setSetpoint(0);
    // drive.disable();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
