/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import lib.frc1747.controller.Logitech;
import lib.frc1747.subsytems.HBRSubsystem;

public class DriveWithJoysticks extends Command {
  Drivetrain drivetrain;
  // boolean state = false;
  // boolean laststate = false;
  boolean shouldstop = true;

  public DriveWithJoysticks() {
    drivetrain = Drivetrain.getInstance();
    requires (drivetrain);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // distance PID
    drivetrain.setMode(Drivetrain.Follower.DISTANCE, HBRSubsystem.Mode.PID);
    drivetrain.setPIDMode(Drivetrain.Follower.DISTANCE, HBRSubsystem.PIDMode.VELOCITY);
    drivetrain.setILimit(Drivetrain.Follower.DISTANCE, 0);
    drivetrain.setFeedforward(Drivetrain.Follower.DISTANCE, 0, 1/RobotMap.S_V_MAX, 0);
    drivetrain.setFeedback(Drivetrain.Follower.DISTANCE, 0, 0, 0);
    drivetrain.resetIntegrator(Drivetrain.Follower.DISTANCE);

    // angle PID
    drivetrain.setMode(Drivetrain.Follower.ANGLE, HBRSubsystem.Mode.PID);
    drivetrain.setPIDMode(Drivetrain.Follower.ANGLE, HBRSubsystem.PIDMode.VELOCITY);
    drivetrain.setILimit(Drivetrain.Follower.ANGLE, 0);
    drivetrain.setFeedforward(Drivetrain.Follower.ANGLE, 0, 1 / (RobotMap.A_V_MAX * 2), 0);
    drivetrain.setFeedback(Drivetrain.Follower.ANGLE, 0.09, 0, 0);
    drivetrain.resetIntegrator(Drivetrain.Follower.ANGLE);
    drivetrain.setEnabled(true);
  //  drivetrain.setSetpoint(Drivetrain.Follower.ANGLE, drivetrain.getAngle());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      //   if (0.1 > Math.abs(OI.getInstance().getDriver().getAxis(Logitech.RIGHT_HORIZONTAL)) && !shouldstop){
      //   shouldstop = true;
      //   drivetrain.setSetpoint(DriveSub.Follower.ANGLE, drivetrain.getAngle());
      //   drivetrain.setLeftPower(0);
      //   drivetrain.setRightPower(0);
      // }
      
      // if (shouldstop && 0.1 <= Math.abs(OI.getInstance().getDriver().getAxis(Logitech.RIGHT_HORIZONTAL))){
      //   shouldstop = false;
      // }

      // else if (!shouldstop) {
      // drivetrain.setSetpoint(DriveSub.Follower.ANGLE, drivetrain.getAngle() + OI.getInstance().getDriver().getAxis(Logitech.RIGHT_HORIZONTAL) * Math.PI);
      // }
    

    drivetrain.setSetpoint(Drivetrain.Follower.DISTANCE, OI.getInstance().getDriver().getAxis(Logitech.LEFT_VERTICAL) * RobotMap.S_V_MAX);
    drivetrain.setSetpoint(Drivetrain.Follower.ANGLE, -OI.getInstance().getDriver().getAxis(Logitech.RIGHT_HORIZONTAL) * RobotMap.A_V_MAX);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    drivetrain.setSetpoint(Drivetrain.Follower.DISTANCE, 0);
    drivetrain.setSetpoint(Drivetrain.Follower.ANGLE, drivetrain.getAngle());
    drivetrain.setEnabled(false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
