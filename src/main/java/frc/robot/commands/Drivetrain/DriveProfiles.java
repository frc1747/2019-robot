/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drivetrain;

import com.tigerhuang.gambezi.dashboard.GambeziDashboard;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Drivetrain;
import lib.frc1747.subsytems.HBRSubsystem;

public class DriveProfiles extends Command {
  Drivetrain drivetrain;
  String filename;
  public DriveProfiles(String filename) {
    this.filename = filename;
    drivetrain = Drivetrain.getInstance();
    requires (drivetrain);
    setInterruptible(true);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    double profile[][][] = HBRSubsystem.readProfilesFromFile(filename);
    double linearOffset = drivetrain.averageDistance();
    double angularOffset = drivetrain.getAngle();

    for (int i = 0; i < profile[0].length; i++){
      profile[0][i][0] += linearOffset;
      profile[1][i][0] += angularOffset;
    }

    // distance PID
    drivetrain.setMode(Drivetrain.Follower.DISTANCE, HBRSubsystem.Mode.FOLLOWER);
    drivetrain.setPIDMode(Drivetrain.Follower.DISTANCE, HBRSubsystem.PIDMode.POSITION);
    drivetrain.setILimit(Drivetrain.Follower.DISTANCE, 0);
    drivetrain.setFeedforward(Drivetrain.Follower.DISTANCE, 0, 0.07, 0.01);
    drivetrain.setFeedback(Drivetrain.Follower.DISTANCE, 0.64, 0, 0);
    drivetrain.resetIntegrator(Drivetrain.Follower.DISTANCE);

    // angle PID
    drivetrain.setMode(Drivetrain.Follower.ANGLE, HBRSubsystem.Mode.FOLLOWER);
    drivetrain.setPIDMode(Drivetrain.Follower.ANGLE, HBRSubsystem.PIDMode.POSITION);
    drivetrain.setILimit(Drivetrain.Follower.ANGLE, 0);
    drivetrain.setFeedforward(Drivetrain.Follower.ANGLE, 0, 0.11, 0.02);
    drivetrain.setFeedback(Drivetrain.Follower.ANGLE, 1.25, 0, 0.15);
    drivetrain.resetIntegrator(Drivetrain.Follower.ANGLE);

    drivetrain.setProfile(Drivetrain.Follower.DISTANCE,profile[0]);
    drivetrain.setProfile(Drivetrain.Follower.ANGLE,profile[1]);
    
    drivetrain.resume(Drivetrain.Follower.DISTANCE);
    drivetrain.resume(Drivetrain.Follower.ANGLE);
    drivetrain.setEnabled(true);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (!drivetrain.isRunning(Drivetrain.Follower.DISTANCE) && !drivetrain.isRunning(Drivetrain.Follower.ANGLE));
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    drivetrain.setEnabled(false);
    drivetrain.setLeftPower(0);
    drivetrain.setRightPower(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
