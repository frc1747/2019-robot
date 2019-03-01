/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auton;

import com.tigerhuang.gambezi.Gambezi;
import com.tigerhuang.gambezi._Node;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import lib.frc1747.subsytems.HBRSubsystem;

public class AutoAlign extends Command {
  Gambezi gambezi;
  _Node dist;
  _Node ang;
  double distance;
  double angle;
  Drivetrain drive;
  public AutoAlign() {
    drive = Drivetrain.getInstance();
    requires(drive);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    gambezi = new Gambezi("10.17.47.18:5809");
    dist = gambezi.get_node("pi_vision/output/distance");
    ang = gambezi.get_node("pi_vision/output/angle to target");
    distance = dist.get_double();
    angle = ang.get_double();
    double[][][] profile = HBRSubsystem.generateSkidSteerPseudoProfile(distance, angle);
    drive.setMode(Drivetrain.Follower.DISTANCE, HBRSubsystem.Mode.FOLLOWER);
    drive.setPIDMode(Drivetrain.Follower.DISTANCE, HBRSubsystem.PIDMode.POSITION);
    drive.setILimit(Drivetrain.Follower.DISTANCE, 0);
    drive.setFeedforward(Drivetrain.Follower.DISTANCE, 0, 0, 0);
    drive.setFeedback(Drivetrain.Follower.DISTANCE, 0, 0, 0);
    drive.resetIntegrator(Drivetrain.Follower.DISTANCE);

    // angle PID
    drive.setMode(Drivetrain.Follower.ANGLE, HBRSubsystem.Mode.FOLLOWER);
    drive.setPIDMode(Drivetrain.Follower.ANGLE, HBRSubsystem.PIDMode.POSITION);
    drive.setILimit(Drivetrain.Follower.ANGLE, 0);
    drive.setFeedforward(Drivetrain.Follower.ANGLE, 0, 0, 0);
    drive.setFeedback(Drivetrain.Follower.ANGLE, 0, 0, 0);
    drive.resetIntegrator(Drivetrain.Follower.ANGLE);

    drive.setProfile(Drivetrain.Follower.DISTANCE,profile[0]);
    drive.setProfile(Drivetrain.Follower.ANGLE,profile[1]);

    drive.resume(Drivetrain.Follower.DISTANCE);
    drive.resume(Drivetrain.Follower.ANGLE);
    drive.setEnabled(true);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (!drive.isRunning(Drivetrain.Follower.DISTANCE) && !drive.isRunning(Drivetrain.Follower.ANGLE));

  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    drive.setEnabled(false);
    drive.setLeftPower(0);
    drive.setRightPower(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
